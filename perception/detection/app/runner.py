import argparse
import logging
import time
from cProfile import Profile
from pstats import SortKey, Stats
from queue import Queue
from threading import Event, Thread
from typing import Protocol, cast

import rospy
import tf2_ros
from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.camera_loader import load_camera
from app.config.config import Config
from app.config.config_loader import load_config
from app.container import Container
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.field_filter_loader import load_field_filter, load_global_field_transformer
from app.field_filter.field_request_handler import FieldRequestHandler
from app.field_filter.global_field_transformer import GlobalFieldTransformer
from app.keypoint.keypoint_interface import KeypointInterface
from app.keypoint.keypoint_loader import load_keypoint, load_keypoint_to_object_converter
from app.keypoint.keypoint_to_object_converter import KeypointToObjectConverter
from app.profiling.context_timer import ContextTimer
from app.robot_filter.robot_filter import RobotFilter
from app.robot_filter.robot_filter_loader import load_robot_filter
from app.segmentation.segmentation_interface import SegmentationInterface
from app.segmentation.segmentation_loader import load_segmentation
from bw_interfaces.msg import (
    EstimatedObjectArray,
    Heartbeat,
    KeypointInstanceArray,
    LabelMap,
    SegmentationInstanceArray,
)
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_map, get_robot
from bw_shared.messages.header import Header
from bw_shared.tick_regulator import regulate_tick
from perception_tools.initialize_logger import initialize
from perception_tools.json_logger import CustomJsonFormatter
from perception_tools.messages.camera_data import CameraData
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from perception_tools.rosbridge.transform_broadcaster_bridge import TransformBroadcasterBridge
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Empty


class Runner:
    def __init__(self, container: Container) -> None:
        self.container = container
        self.config = self.container.resolve(Config)
        self.heartbeat_publisher: RosPublisher[Heartbeat] = self.container.resolve_by_name("heartbeat_publisher")
        self.camera: CameraInterface = self.container.resolve_by_name("camera")

        self.field_filter: FieldFilterInterface = self.container.resolve_by_name("field_filter")
        self.field_request_handler = self.container.resolve(FieldRequestHandler)

        self.field_segmentation: SegmentationInterface = self.container.resolve_by_name("field_segmentation")
        self.field_debug_image_publisher: RosPublisher[Image] = self.container.resolve_by_name(
            "field_debug_image_publisher"
        )
        self.field_segmentation_publisher: RosPublisher[SegmentationInstanceArray] = self.container.resolve_by_name(
            "field_segmentation_publisher"
        )
        self.field_cloud_publisher: RosPublisher[PointCloud2] = self.container.resolve_by_name("field_cloud_publisher")
        self.point_cloud_publisher: RosPublisher[PointCloud2] = self.container.resolve_by_name("point_cloud_publisher")
        self.field_label_map_publisher: RosPublisher[LabelMap] = self.container.resolve_by_name(
            "field_label_map_publisher"
        )
        self.robot_label_map_publisher: RosPublisher[LabelMap] = self.container.resolve_by_name(
            "robot_label_map_publisher"
        )

        self.robot_keypoint: KeypointInterface = self.container.resolve_by_name("robot_keypoint")
        self.robot_debug_image_publisher: RosPublisher[Image] = self.container.resolve_by_name(
            "robot_debug_image_publisher"
        )
        self.robot_keypoint_publisher: RosPublisher[KeypointInstanceArray] = self.container.resolve_by_name(
            "robot_keypoint_publisher"
        )
        self.global_field_transformer = self.container.resolve(GlobalFieldTransformer)
        self.keypoint_to_object_converter: KeypointToObjectConverter = self.container.resolve_by_name(
            "keypoint_to_object_converter"
        )
        self.robot_filter = self.container.resolve(RobotFilter)
        self.transform_broadcaster = self.container.resolve(TransformBroadcasterBridge)

        self.robot_estimation_queue: Queue[EstimatedObjectArray] = Queue()
        self.robot_estimation_thread = Thread(target=self.robot_estimations_task)
        self.exit_event = Event()

        self.camera_data = CameraData()
        self.is_field_request_active = False
        self.prev_no_camera_warning_time = time.monotonic()
        self.logger = logging.getLogger(self.__class__.__name__)

    def start(self) -> None:
        self.logger.info("Runner started")
        self.field_label_map_publisher.publish(self.field_segmentation.get_model_to_system_labels())
        self.robot_label_map_publisher.publish(self.robot_keypoint.get_model_to_system_labels())
        self.robot_estimation_thread.start()

        if not self.camera.open():
            raise RuntimeError("Failed to open camera")

    def loop(self) -> None:
        if rospy.is_shutdown():
            raise KeyboardInterrupt("ROS is shutting down")
        self.heartbeat_publisher.publish(Heartbeat(header=Header.auto().to_msg(), node_name="perception"))
        if self.field_request_handler.has_request():
            self.is_field_request_active = True
            self.logger.debug("Field request received, processing field")
        if self.is_field_request_active:
            if self.perceive_field():
                self.is_field_request_active = False
                self.logger.debug("Field request processed successfully")
        else:
            self.perceive_robot()
        self.transform_broadcaster.send_pending_transforms()

    def robot_estimations_task(self) -> None:
        queue = self.robot_estimation_queue
        for dt in regulate_tick(self.config.target_tick_rate):
            if dt > self.config.loop_overrun_threshold:
                self.logger.warning(f"Estimations task overrun: {dt:.6f} seconds")
            if not queue.empty():
                robot_estimations = queue.get()
                if not (field := self.global_field_transformer.get_field()):
                    self.logger.warning("No map transform available. Skipping filter update.")
                    continue
                self.robot_filter.update_robot_estimations(robot_estimations, field)
            self.robot_filter.update()
            if self.exit_event.is_set():
                break
        self.logger.info("Robot estimation task stopped")

    def perceive_robot(self) -> None:
        if not self.camera.switch_mode(CameraMode.ROBOT_FINDER):
            self.logger.error("Failed to switch camera mode")
            return
        camera_data = self.camera.poll()
        if camera_data is None or camera_data.color_image.data.size == 0:
            self.logger.warning("No camera data received. Skipping robot perception.")
            return
        self.camera_data = camera_data
        if tfstamped_camera_from_world := self.camera_data.tfstamped_camera_from_world:
            self.global_field_transformer.update_camera_transform(tfstamped_camera_from_world)
        if not (field := self.global_field_transformer.get_field()):
            self.logger.debug("No field available for robot perception. Skipping.")
            return
        tf_camera_from_map = field.tfstamped_camera_from_map
        with ContextTimer("robot_keypoint.process_image"):
            robot_points, debug_image = self.robot_keypoint.process_image(
                camera_data.camera_info, camera_data.color_image, field
            )
        if robot_points:
            with ContextTimer("keypoint_to_object_converter.convert_to_objects"):
                robot_estimations = self.keypoint_to_object_converter.convert_to_objects(
                    camera_data.camera_info, robot_points, tf_camera_from_map
                )
            self.robot_keypoint_publisher.publish(robot_points)
            if robot_estimations:
                self.robot_estimation_queue.put(robot_estimations)

        if debug_image is not None:
            self.robot_debug_image_publisher.publish(debug_image.to_msg())

    def perceive_field(self) -> bool:
        self.logger.debug("Processing field request")

        if not self.camera.switch_mode(CameraMode.FIELD_FINDER):
            self.logger.error("Failed to switch camera mode")
            return True

        camera_data = self.camera.poll()
        if camera_data is None or camera_data.color_image.data.size == 0:
            now = time.monotonic()
            if now - self.prev_no_camera_warning_time > 1.0:
                self.logger.warning("No camera data. Ignoring field request.")
                self.prev_no_camera_warning_time = now
            return False
        self.camera_data = camera_data

        point_cloud = camera_data.point_cloud
        image = camera_data.color_image

        # for this single frame, camera and world are the same. camera -> world is computed after this.
        world_header = Header(
            stamp=camera_data.color_image.header.stamp,
            frame_id=camera_data.world_frame_id,
            seq=camera_data.color_image.header.seq,
        )
        point_cloud.header = world_header
        image.header = world_header
        self.point_cloud_publisher.publish(point_cloud.to_msg())

        with ContextTimer("field_segmentation.process_image"):
            field_seg, debug_image = self.field_segmentation.process_image(image)
        if not field_seg:
            self.logger.debug("No field detected")
            return False
        self.field_segmentation_publisher.publish(field_seg)
        with ContextTimer("field_filter.compute_field"):
            field_result, field_point_cloud = self.field_filter.compute_field(field_seg, point_cloud)
        if not field_result:
            self.logger.debug("No field result")
            return False
        aligned_field = self.global_field_transformer.process_field(field_result)
        if aligned_field:
            self.robot_filter.update_field(aligned_field)
        if debug_image:
            self.logger.info("Publishing debug image")
            self.field_debug_image_publisher.publish(debug_image.to_msg())
        else:
            self.logger.debug("No debug image to publish")
        if field_point_cloud:
            self.logger.info("Publishing field point cloud")
            self.field_cloud_publisher.publish(field_point_cloud.to_msg())
        else:
            self.logger.debug("No field point cloud to publish")
        return True

    def stop(self) -> None:
        self.camera.close()
        self.exit_event.set()
        self.robot_estimation_thread.join()
        rospy.signal_shutdown("Runner stopped")
        self.logger.info("Runner stopped")


class CommandLineArgs(Protocol):
    config: str
    profile: bool


def make_camera(container: Container) -> None:
    logger = logging.getLogger("make_camera")
    config = container.resolve(Config)
    camera = load_camera(config.camera, container)
    container.register(camera, "camera")

    logger.info(f"Camera: {camera}")


def make_ros_comms(container: Container) -> None:
    config = container.resolve(Config)

    RosPublisher.log = config.ros.log
    RosPublisher.exclude_filters = config.ros.exclude_filters
    RosPollSubscriber.log = config.ros.log
    RosPollSubscriber.exclude_filters = config.ros.exclude_filters

    heartbeat_publisher = RosPublisher("/perception/heartbeat", Heartbeat)
    container.register(heartbeat_publisher, "heartbeat_publisher")

    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    container.register(TransformBroadcasterBridge(static_broadcaster, tf_broadcaster))


def make_field_segmentation(container: Container) -> None:
    logger = logging.getLogger("make_field_segmentation")
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    field_segmentation = load_segmentation(container, config.field_segmentation)
    field_debug_image_publisher = RosPublisher(namespace + "/field/debug_image", Image, latch=True)
    field_segmentation_publisher = RosPublisher(
        namespace + "/field/segmentation", SegmentationInstanceArray, latch=True
    )
    field_cloud_publisher = RosPublisher(namespace + "/field/point_cloud", PointCloud2, latch=True)
    point_cloud_publisher = RosPublisher(namespace + "/point_cloud/cloud_registered", PointCloud2, latch=True)
    field_label_map_publisher = RosPublisher(namespace + "/field/label_map", LabelMap, latch=True)

    container.register(field_segmentation, "field_segmentation")
    container.register(field_segmentation_publisher, "field_segmentation_publisher")
    container.register(field_debug_image_publisher, "field_debug_image_publisher")
    container.register(field_cloud_publisher, "field_cloud_publisher")
    container.register(point_cloud_publisher, "point_cloud_publisher")
    container.register(field_label_map_publisher, "field_label_map_publisher")

    logger.info(f"Field segmentation: {type(field_segmentation)}")


def make_robot_keypoint(container: Container) -> None:
    logger = logging.getLogger("make_robot_keypoint")
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    robot_keypoint = load_keypoint(container, config.robot_keypoint)
    keypoint_to_object_converter = load_keypoint_to_object_converter(container, config.keypoint_to_object_converter)
    robot_debug_image_publisher = RosPublisher(namespace + "/robot/debug_image", Image)
    robot_keypoint_publisher = RosPublisher(namespace + "/robot/keypoints", KeypointInstanceArray)
    robot_label_map_publisher = RosPublisher(namespace + "/robot/label_map", LabelMap, latch=True)

    container.register(robot_keypoint, "robot_keypoint")
    container.register(keypoint_to_object_converter, "keypoint_to_object_converter")
    container.register(robot_keypoint_publisher, "robot_keypoint_publisher")
    container.register(robot_debug_image_publisher, "robot_debug_image_publisher")
    container.register(robot_label_map_publisher, "robot_label_map_publisher")

    logger.info(f"Robot segmentation: {type(robot_keypoint)}")


def make_field_interface(container: Container) -> None:
    logger = logging.getLogger("make_field_interface")
    config = container.resolve(Config)
    field_filter = load_field_filter(config.field_filter, container)
    container.register(field_filter, "field_filter")

    global_field_manager = load_global_field_transformer(config.global_field_manager, container)
    container.register(global_field_manager, GlobalFieldTransformer)

    logger.info(f"Field filter: {type(field_filter)}")


def init_ros_node(container: Container) -> None:
    config = container.resolve(Config)
    wait_for_ros_connection(connection_timeout=config.ros.connection_timeout)
    rospy.init_node("perception", log_level=rospy.DEBUG, disable_signals=True)


def make_field_request_handler(container: Container) -> None:
    config = container.resolve(Config)
    request_subscriber = RosPollSubscriber("/field_request", Empty)
    field_request_handler = FieldRequestHandler(config.field_request, request_subscriber)
    container.register(field_request_handler)


def make_robot_filter(container: Container) -> None:
    config = container.resolve(Config)
    shared_config = container.resolve(SharedConfig)
    robot_filter = load_robot_filter(container, config.robot_filter, shared_config)
    container.register(robot_filter)


def run_loop(app: Runner, config: Config) -> None:
    logger = logging.getLogger("run_loop")
    logger.info("Running perception")
    for dt in regulate_tick(config.target_tick_rate):
        if dt > config.loop_overrun_threshold:
            logger.warning(f"Loop overrun: {dt:.6f} seconds")
        app.loop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", action="store_true")
    args: CommandLineArgs = cast(CommandLineArgs, parser.parse_args())

    profile_app = args.profile

    shared_config = SharedConfig.from_files()
    config = load_config(get_robot())

    initialize(config.log_level, CustomJsonFormatter())
    print()  # Start log on a fresh line
    logger = logging.getLogger("main")
    logger.info("Initializing perception")
    if profile_app:
        logger.info("Profiling enabled")

    container = Container()
    container.register(config)
    container.register(shared_config)
    map_config = shared_config.get_map(FieldType(get_map()))
    container.register(map_config)

    init_ros_node(container)

    make_ros_comms(container)
    make_camera(container)
    make_field_segmentation(container)
    make_robot_keypoint(container)
    make_field_interface(container)
    make_field_request_handler(container)
    make_robot_filter(container)

    logger.info("Starting perception")

    app = Runner(container)
    app.start()

    profile = None

    try:
        if profile_app:
            with Profile() as profile:
                run_loop(app, config)
        else:
            run_loop(app, config)
    finally:
        logger.info("perception is stopping")
        app.stop()
        logger.info("perception stopped")
        if profile:
            Stats(profile).strip_dirs().sort_stats(SortKey.TIME).print_stats()
