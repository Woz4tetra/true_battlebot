import argparse
import logging
import time
from typing import Protocol, cast

import rospy
from app.camera.camera_interface import CameraInterface, CameraMode
from app.camera.camera_loader import load_camera
from app.config.config import Config
from app.config.config_loader import load_config
from app.container import Container
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.field_filter_loader import load_field_filter
from app.field_filter.field_request_handler import FieldRequestHandler
from app.json_logger import initialize
from app.keypoint.keypoint_interface import KeypointInterface
from app.keypoint.keypoint_loader import load_keypoint
from app.segmentation.segmentation_interface import SegmentationInterface
from app.segmentation.segmentation_loader import load_segmentation
from app.tick_regulator import regulate_tick
from bw_interfaces.msg import EstimatedObject, KeypointInstanceArray, SegmentationInstanceArray
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_map, get_robot
from bw_shared.messages.header import Header
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Empty
from std_msgs.msg import Header as RosHeader


class Runner:
    def __init__(self, container: Container) -> None:
        self.container = container
        self.heartbeat_publisher: RosPublisher[RosHeader] = self.container.resolve_by_name("heartbeat_publisher")
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
        self.point_cloud_publisher: RosPublisher[PointCloud2] = self.container.resolve_by_name("point_cloud_publisher")

        self.robot_keypoint: KeypointInterface = self.container.resolve_by_name("robot_keypoint")
        self.robot_debug_image_publisher: RosPublisher[Image] = self.container.resolve_by_name(
            "robot_debug_image_publisher"
        )
        self.robot_keypoint_publisher: RosPublisher[KeypointInstanceArray] = self.container.resolve_by_name(
            "robot_keypoint_publisher"
        )
        self.prev_image_time = time.time()
        self.logger = logging.getLogger("perception")

    def start(self) -> None:
        rospy.init_node("perception")
        self.logger.info("Runner started")

    def loop(self) -> None:
        if rospy.is_shutdown():
            raise KeyboardInterrupt("ROS is shutting down")
        self.heartbeat_publisher.publish(Header.auto().to_msg())
        self.perceive_robot()
        self.perceive_field()

    def perceive_robot(self) -> None:
        if not self.camera.open(CameraMode.ROBOT_FINDER):
            self.logger.error("Failed to open camera")
            return
        camera_data = self.camera.poll()
        if camera_data is None or camera_data.color_image.data.size == 0:
            return
        self.prev_image_time = time.time()
        robot_points, debug_image = self.robot_keypoint.process_image(camera_data.color_image)
        if debug_image:
            self.robot_debug_image_publisher.publish(debug_image.to_msg())
        self.robot_keypoint_publisher.publish(robot_points)

    def perceive_field(self) -> None:
        if not self.field_request_handler.has_request(self.prev_image_time):
            return
        self.logger.info("Processing field request")

        if not self.camera.open(CameraMode.FIELD_FINDER):
            self.logger.error("Failed to open camera")
            return

        camera_data = self.camera.poll()
        if camera_data is None or camera_data.color_image.data.size == 0:
            self.logger.warning("No camera data. Ignoring field request.")
            return
        self.prev_image_time = time.time()

        image = camera_data.color_image
        field_seg, debug_image = self.field_segmentation.process_image(image)
        self.field_segmentation_publisher.publish(field_seg)
        field_result, field_point_cloud = self.field_filter.compute_field(field_seg, camera_data.point_cloud)
        self.field_request_handler.send_response(field_result)
        if debug_image:
            self.logger.info("Publishing debug image")
            self.field_debug_image_publisher.publish(debug_image.to_msg())
        else:
            self.logger.debug("No debug image to publish")
        if field_point_cloud:
            self.logger.info("Publishing field point cloud")
            self.point_cloud_publisher.publish(field_point_cloud.to_msg())
        else:
            self.logger.debug("No field point cloud to publish")

    def stop(self) -> None:
        self.camera.close()
        rospy.signal_shutdown("Runner stopped")
        self.logger.info("Runner stopped")


class CommandLineArgs(Protocol):
    config_dir: str


logger = logging.getLogger("perception")


def make_camera(container: Container) -> None:
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

    heartbeat_publisher = RosPublisher("/perception/heartbeat", RosHeader)
    container.register(heartbeat_publisher, "heartbeat_publisher")


def make_field_segmentation(container: Container) -> None:
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    field_segmentation = load_segmentation(container, config.field_segmentation)
    field_debug_image_publisher = RosPublisher(namespace + "/field/debug_image", Image)
    field_segmentation_publisher = RosPublisher(namespace + "/field/segmentation", SegmentationInstanceArray)
    point_cloud_publisher = RosPublisher(namespace + "/field/point_cloud", PointCloud2)

    container.register(field_segmentation, "field_segmentation")
    container.register(field_segmentation_publisher, "field_segmentation_publisher")
    container.register(field_debug_image_publisher, "field_debug_image_publisher")
    container.register(point_cloud_publisher, "point_cloud_publisher")

    logger.info(f"Field segmentation: {field_segmentation}")


def make_robot_keypoint(container: Container) -> None:
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    robot_keypoint = load_keypoint(container, config.robot_keypoint)
    robot_debug_image_publisher = RosPublisher(namespace + "/robot/debug_image", Image)
    robot_keypoint_publisher = RosPublisher(namespace + "/robot/keypoints", KeypointInstanceArray)

    container.register(robot_keypoint, "robot_keypoint")
    container.register(robot_keypoint_publisher, "robot_keypoint_publisher")
    container.register(robot_debug_image_publisher, "robot_debug_image_publisher")

    logger.info(f"Robot segmentation: {robot_keypoint}")


def make_field_interface(container: Container) -> None:
    config = container.resolve(Config)
    field_filter = load_field_filter(config.field_filter, container)
    container.register(field_filter, FieldFilterInterface)

    logger.info(f"Field filter: {field_filter}")


def make_field_request_handler(container: Container) -> None:
    config = container.resolve(Config)
    request_subscriber = RosPollSubscriber("/perception/field/request", Empty)
    response_subscriber = RosPublisher("/perception/field/response", EstimatedObject)
    field_request_handler = FieldRequestHandler(config.field_request, request_subscriber, response_subscriber)
    container.register(field_request_handler)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config_dir", type=str)
    args: CommandLineArgs = cast(CommandLineArgs, parser.parse_args())

    config_dir = args.config_dir

    shared_config = SharedConfig.from_files()
    config = load_config(config_dir, get_robot())

    initialize()
    print()  # Start log on a fresh line
    logger.info("Initializing perception")

    container = Container()
    container.register(config)
    container.register(shared_config)
    map_config = shared_config.get_map(FieldType(get_map()))
    container.register(map_config)

    make_ros_comms(container)
    make_camera(container)
    make_field_segmentation(container)
    make_robot_keypoint(container)
    make_field_interface(container)
    make_field_request_handler(container)

    app = Runner(container)

    logger.info("Starting perception")
    app.start()
    logger.info("perception is running")
    try:
        for dt in regulate_tick(config.target_tick_rate):
            app.loop()
    finally:
        logger.info("perception is stopping")
        app.stop()
        logger.info("perception stopped")
