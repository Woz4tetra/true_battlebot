import argparse
import logging
import time
from typing import Protocol, cast

from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_map, get_robot, get_ros_ip
from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.compressed_image import CompressedImage
from perception_tools.messages.field_result import FieldResult
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.empty import Empty
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from roslibpy import Ros

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
        self.ros = self.container.resolve(Ros)
        self.heartbeat_publisher: RosPublisher[Header] = self.container.resolve_by_name("heartbeat_publisher")
        self.camera = self.container.resolve(CameraInterface)

        self.field_filter = self.container.resolve(FieldFilterInterface)
        self.field_request_handler = self.container.resolve(FieldRequestHandler)

        self.field_segmentation: SegmentationInterface = self.container.resolve_by_name("field_segmentation")
        self.field_debug_image_publisher: RosPublisher[CompressedImage] = self.container.resolve_by_name(
            "field_debug_image_publisher"
        )
        self.field_segmentation_publisher: RosPublisher[SegmentationInstanceArray] = self.container.resolve_by_name(
            "field_segmentation_publisher"
        )

        self.robot_segmentation: SegmentationInterface = self.container.resolve_by_name("robot_segmentation")
        self.robot_debug_image_publisher: RosPublisher[CompressedImage] = self.container.resolve_by_name(
            "robot_debug_image_publisher"
        )
        self.robot_segmentation_publisher: RosPublisher[SegmentationInstanceArray] = self.container.resolve_by_name(
            "robot_segmentation_publisher"
        )
        self.camera_data: CameraData | None = None
        self.logger = logging.getLogger("perception")

    def start(self) -> None:
        self.ros.run()
        self.logger.info("Runner started")

    def loop(self) -> None:
        self.heartbeat_publisher.publish(Header.auto())

        if camera_data := self.camera.poll():
            if self.camera_data is None:
                self.logger.info("Received camera data")
            self.camera_data = camera_data
            robot_seg, debug_image = self.robot_segmentation.process_image(self.camera_data.color_image)
            if debug_image:
                self.robot_debug_image_publisher.publish(debug_image.to_compressed())
            self.robot_segmentation_publisher.publish(robot_seg)

        if self.camera_data is not None and self.field_request_handler.has_request(
            self.camera_data.camera_info.header.stamp
        ):
            self.logger.info("Processing field request")
            image = self.camera_data.color_image
            field_seg, debug_image = self.field_segmentation.process_image(image)
            self.field_segmentation_publisher.publish(field_seg)
            field_result = self.field_filter.compute_field(
                field_seg, self.camera_data.depth_image, self.camera_data.camera_info
            )
            self.field_request_handler.send_response(field_result)
            if debug_image:
                self.logger.info("Publishing debug image")
                self.field_debug_image_publisher.publish(debug_image.to_compressed())
            else:
                self.logger.debug("No debug image to publish")

    def stop(self) -> None:
        self.ros.close()
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
    host_ip = config.rosbridge.host if config.rosbridge.host else get_ros_ip()
    port = config.rosbridge.port
    ros = Ros(host_ip, port)
    container.register(ros)
    logger.info(f"ROS: {host_ip}:{port}")

    RosPublisher.log = config.rosbridge.log
    RosPublisher.exclude_filters = config.rosbridge.exclude_filters
    RosPollSubscriber.log = config.rosbridge.log
    RosPollSubscriber.exclude_filters = config.rosbridge.exclude_filters

    heartbeat_publisher = RosPublisher(ros, "/perception/heartbeat", Header)
    container.register(heartbeat_publisher, "heartbeat_publisher")


def make_field_segmentation(container: Container) -> None:
    config = container.resolve(Config)
    ros = container.resolve(Ros)
    namespace = config.camera_topic.namespace
    field_segmentation = load_segmentation(container, config.field_segmentation)
    field_debug_image_publisher = RosPublisher(ros, namespace + "/field/debug_image", CompressedImage)
    field_segmentation_publisher = RosPublisher(ros, namespace + "/field/segmentation", SegmentationInstanceArray)

    container.register(field_segmentation, "field_segmentation")
    container.register(field_segmentation_publisher, "field_segmentation_publisher")
    container.register(field_debug_image_publisher, "field_debug_image_publisher")

    logger.info(f"Field segmentation: {field_segmentation}")


def make_robot_segmentation(container: Container) -> None:
    config = container.resolve(Config)
    ros = container.resolve(Ros)
    namespace = config.camera_topic.namespace
    robot_segmentation = load_segmentation(container, config.robot_segmentation)
    robot_debug_image_publisher = RosPublisher(ros, namespace + "/robot/debug_image", CompressedImage)
    robot_segmentation_publisher = RosPublisher(ros, namespace + "/robot/segmentation", SegmentationInstanceArray)

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
    ros = container.resolve(Ros)
    request_subscriber = RosPollSubscriber(ros, "/perception/field/request", Empty)
    response_subscriber = RosPublisher(ros, "/perception/field/response", FieldResult)
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
