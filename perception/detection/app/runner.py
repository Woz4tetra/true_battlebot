import argparse
import logging
import time
from typing import Protocol, cast

import rospy
from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_map, get_robot
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Empty
from std_msgs.msg import Header as RosHeader

from app.camera.camera_interface import CameraInterface
from app.camera.camera_loader import load_camera
from app.config.config import Config
from app.config.config_loader import load_config
from app.container import Container
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.field_filter_loader import load_field_filter
from app.field_filter.field_request_handler import FieldRequestHandler
from app.json_logger import initialize
from app.segmentation.segmentation_interface import SegmentationInterface
from app.segmentation.segmentation_loader import load_segmentation


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

        self.robot_segmentation: SegmentationInterface = self.container.resolve_by_name("robot_segmentation")
        self.robot_debug_image_publisher: RosPublisher[Image] = self.container.resolve_by_name(
            "robot_debug_image_publisher"
        )
        self.robot_segmentation_publisher: RosPublisher[SegmentationInstanceArray] = self.container.resolve_by_name(
            "robot_segmentation_publisher"
        )
        self.camera_data: CameraData | None = None
        self.logger = logging.getLogger("perception")

    def start(self) -> None:
        rospy.init_node("perception")
        if not self.camera.open():
            raise RuntimeError("Failed to open camera")
        self.logger.info("Runner started")

    def loop(self) -> None:
        if rospy.is_shutdown():
            raise KeyboardInterrupt("ROS is shutting down")
        self.heartbeat_publisher.publish(Header.auto().to_msg())
        self.update_camera()
        self.update_field()

    def update_camera(self) -> None:
        if camera_data := self.camera.poll():
            if self.camera_data is None:
                self.logger.info("Received camera data")
            self.camera_data = camera_data
            robot_seg, debug_image = self.robot_segmentation.process_image(self.camera_data.color_image)
            if debug_image:
                self.robot_debug_image_publisher.publish(debug_image.to_msg())
            self.robot_segmentation_publisher.publish(robot_seg)

    def update_field(self) -> None:
        if self.camera_data is None:
            return
        header = Header.from_msg(self.camera_data.camera_info.header)
        if not self.field_request_handler.has_request(header.stamp):
            return

        self.logger.info("Processing field request")
        self.point_cloud_publisher.publish(self.camera_data.point_cloud.to_msg())
        image = self.camera_data.color_image
        field_seg, debug_image = self.field_segmentation.process_image(image)
        self.field_segmentation_publisher.publish(field_seg)
        field_result = self.field_filter.compute_field(field_seg, self.camera_data.point_cloud)
        self.field_request_handler.send_response(field_result)
        if debug_image:
            self.logger.info("Publishing debug image")
            self.field_debug_image_publisher.publish(debug_image.to_msg())
        else:
            self.logger.debug("No debug image to publish")

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


def make_robot_segmentation(container: Container) -> None:
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    robot_segmentation = load_segmentation(container, config.robot_segmentation)
    robot_debug_image_publisher = RosPublisher(namespace + "/robot/debug_image", Image)
    robot_segmentation_publisher = RosPublisher(namespace + "/robot/segmentation", SegmentationInstanceArray)

    container.register(robot_segmentation, "robot_segmentation")
    container.register(robot_segmentation_publisher, "robot_segmentation_publisher")
    container.register(robot_debug_image_publisher, "robot_debug_image_publisher")

    logger.info(f"Robot segmentation: {robot_segmentation}")


def make_field_interface(container: Container) -> None:
    config = container.resolve(Config)
    shared_config = container.resolve(SharedConfig)
    map_config = shared_config.get_map(FieldType(get_map()))
    field_filter = load_field_filter(map_config, config.field_filter, container)
    container.register(map_config)
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
    make_ros_comms(container)
    make_camera(container)
    make_field_segmentation(container)
    make_robot_segmentation(container)
    make_field_interface(container)
    make_field_request_handler(container)

    poll_delay = 1.0 / config.poll_rate
    app = Runner(container)

    logger.info("Starting perception")
    app.start()
    logger.info("perception is running")
    try:
        while True:
            time.sleep(poll_delay)
            app.loop()
    finally:
        logger.info("perception is stopping")
        app.stop()
        logger.info("perception stopped")
