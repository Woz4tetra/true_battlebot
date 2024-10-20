import argparse
import logging
import time
from cProfile import Profile
from pathlib import Path
from pstats import SortKey, Stats
from typing import Protocol, cast

import rospy
from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.camera_loader import load_camera
from app.config.config import Config
from app.config.config_loader import load_config
from app.config.list_configs import get_config_path
from app.container import Container
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.field_filter_loader import load_field_filter
from app.field_filter.field_request_handler import FieldRequestHandler
from app.keypoint.keypoint_interface import KeypointInterface
from app.keypoint.keypoint_loader import load_keypoint
from app.segmentation.segmentation_interface import SegmentationInterface
from app.segmentation.segmentation_loader import load_segmentation
from bw_interfaces.msg import EstimatedObject, Heartbeat, KeypointInstanceArray, LabelMap, SegmentationInstanceArray
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
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Empty


class Runner:
    def __init__(self, container: Container) -> None:
        self.container = container
        self.heartbeat_publisher: RosPublisher[Heartbeat] = self.container.resolve_by_name("heartbeat_publisher")
        self.camera = self.container.resolve(CameraInterface)

        self.field_filter = self.container.resolve(FieldFilterInterface)
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
        self.camera_data = CameraData()
        self.is_field_request_active = False
        self.prev_no_camera_warning_time = time.monotonic()
        self.logger = logging.getLogger("perception")

    def start(self) -> None:
        self.logger.info("Runner started")
        self.field_label_map_publisher.publish(self.field_segmentation.get_model_to_system_labels())
        self.robot_label_map_publisher.publish(self.robot_keypoint.get_model_to_system_labels())

        if not self.camera.open():
            raise RuntimeError("Failed to open camera")

    def loop(self) -> None:
        if rospy.is_shutdown():
            raise KeyboardInterrupt("ROS is shutting down")
        self.heartbeat_publisher.publish(Heartbeat(header=Header.auto().to_msg(), node_name="perception"))
        if self.field_request_handler.has_request():
            self.is_field_request_active = True
        if self.is_field_request_active:
            if self.perceive_field():
                self.is_field_request_active = False
        else:
            self.perceive_robot()

    def perceive_robot(self) -> None:
        if not self.camera.switch_mode(CameraMode.ROBOT_FINDER):
            self.logger.error("Failed to switch camera mode")
            return
        camera_data = self.camera.poll()
        if camera_data is None or camera_data.color_image.data.size == 0:
            return
        self.camera_data = camera_data
        robot_points, debug_image = self.robot_keypoint.process_image(camera_data.camera_info, camera_data.color_image)
        if debug_image:
            self.robot_debug_image_publisher.publish(debug_image.to_msg())
        if robot_points:
            self.robot_keypoint_publisher.publish(robot_points)

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

        self.point_cloud_publisher.publish(camera_data.point_cloud.to_msg())

        image = camera_data.color_image
        field_seg, debug_image = self.field_segmentation.process_image(image)
        if not field_seg:
            self.logger.debug("No field detected")
            return False
        self.field_segmentation_publisher.publish(field_seg)
        field_result, field_point_cloud = self.field_filter.compute_field(field_seg, camera_data.point_cloud)
        if not field_result:
            self.logger.debug("No field result")
            return False
        self.field_request_handler.send_response(field_result)
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
        rospy.signal_shutdown("Runner stopped")
        self.logger.info("Runner stopped")


class CommandLineArgs(Protocol):
    config: str
    profile: bool


def make_camera(container: Container) -> None:
    logger = logging.getLogger("perception")
    config = container.resolve(Config)
    camera = load_camera(config.camera, container)
    container.register(camera, CameraInterface)

    logger.info(f"Camera: {camera}")


def make_ros_comms(container: Container) -> None:
    config = container.resolve(Config)

    RosPublisher.log = config.ros.log
    RosPublisher.exclude_filters = config.ros.exclude_filters
    RosPollSubscriber.log = config.ros.log
    RosPollSubscriber.exclude_filters = config.ros.exclude_filters

    heartbeat_publisher = RosPublisher("/perception/heartbeat", Heartbeat)
    container.register(heartbeat_publisher, "heartbeat_publisher")


def make_field_segmentation(container: Container) -> None:
    logger = logging.getLogger("perception")
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
    logger = logging.getLogger("perception")
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    robot_keypoint = load_keypoint(container, config.robot_keypoint)
    robot_debug_image_publisher = RosPublisher(namespace + "/robot/debug_image", Image)
    robot_keypoint_publisher = RosPublisher(namespace + "/robot/keypoints", KeypointInstanceArray)
    robot_label_map_publisher = RosPublisher(namespace + "/robot/label_map", LabelMap, latch=True)

    container.register(robot_keypoint, "robot_keypoint")
    container.register(robot_keypoint_publisher, "robot_keypoint_publisher")
    container.register(robot_debug_image_publisher, "robot_debug_image_publisher")
    container.register(robot_label_map_publisher, "robot_label_map_publisher")

    logger.info(f"Robot segmentation: {type(robot_keypoint)}")


def make_field_interface(container: Container) -> None:
    logger = logging.getLogger("perception")
    config = container.resolve(Config)
    field_filter = load_field_filter(config.field_filter, container)
    container.register(field_filter, FieldFilterInterface)

    logger.info(f"Field filter: {type(field_filter)}")


def init_ros_node(container: Container) -> None:
    config = container.resolve(Config)
    wait_for_ros_connection(connection_timeout=config.ros.connection_timeout)
    rospy.init_node("perception", log_level=rospy.DEBUG, disable_signals=True)


def make_field_request_handler(container: Container) -> None:
    config = container.resolve(Config)
    request_subscriber = RosPollSubscriber("/perception/field/request", Empty)
    response_publisher = RosPublisher("/perception/field/response", EstimatedObject)
    field_request_handler = FieldRequestHandler(config.field_request, request_subscriber, response_publisher)
    container.register(field_request_handler)


def run_loop(app: Runner, config: Config) -> None:
    logger = logging.getLogger("perception")
    logger.info("Running perception")
    for dt in regulate_tick(config.target_tick_rate):
        if dt > config.loop_overrun_threshold:
            logger.warning(f"Loop overrun: {dt:.6f} seconds")
        app.loop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default=str(get_config_path()))
    parser.add_argument("--profile", action="store_true")
    args: CommandLineArgs = cast(CommandLineArgs, parser.parse_args())

    config_dir = Path(args.config)
    profile_app = args.profile

    shared_config = SharedConfig.from_files()
    config = load_config(config_dir, get_robot())

    initialize(CustomJsonFormatter())
    print()  # Start log on a fresh line
    logger = logging.getLogger("perception")
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
