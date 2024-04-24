import argparse
import logging
import time
from typing import Protocol, cast

from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_map, get_robot
from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.image import Image
from perception_tools.messages.field_result import FieldResult
from perception_tools.rosbridge.ros_factory import RosFactory
from perception_tools.rosbridge.ros_publisher import RosPublisher

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
        self.ros_factory = self.container.resolve(RosFactory)
        self.camera = self.container.resolve(CameraInterface)
        self.field_segmentation: SegmentationInterface = self.container.resolve("field_segmentation")
        self.field_filter = self.container.resolve(FieldFilterInterface)
        self.field_request_handler = self.container.resolve(FieldRequestHandler)
        self.field_debug_image_publisher: RosPublisher = self.container.resolve("field_debug_image_publisher")
        self.camera_data: CameraData | None = None

    def start(self) -> None:
        self.ros_factory.connect()

    def loop(self) -> None:
        camera = self.camera
        field_segmentation = self.field_segmentation
        field_filter = self.field_filter
        field_request_handler = self.field_request_handler

        if camera_data := camera.poll():
            self.camera_data = camera_data
        if self.camera_data is not None and field_request_handler.has_request(
            self.camera_data.camera_info.header.stamp
        ):
            image = self.camera_data.color_image
            seg_result, debug_image = field_segmentation.process_image(image)
            field_result = field_filter.compute_field(
                seg_result, self.camera_data.depth_image, self.camera_data.camera_info
            )
            field_request_handler.send_response(field_result)
            if debug_image:
                self.field_debug_image_publisher.publish(debug_image.to_raw())

    def stop(self) -> None:
        self.ros_factory.disconnect()


class CommandLineArgs(Protocol):
    config_dir: str


logger = logging.getLogger("perception")


def make_ros_factory(config: Config, container: Container) -> None:
    ros_factory = RosFactory(config.rosbridge.host, config.rosbridge.port)
    container.register(ros_factory)


def make_field_segmentation(config: Config, container: Container) -> None:
    ros_factory = container.resolve(RosFactory)
    field_segmentation = load_segmentation(container, config.field_segmentation)
    field_debug_image_publisher = ros_factory.make_publisher("/perception/field/debug_image", Image, queue_size=1)

    container.register(field_segmentation, "field_segmentation")
    container.register(field_debug_image_publisher, "field_debug_image_publisher")

    logger.info(f"Field segmentation: {field_segmentation}")


def make_field_interface(config: Config, shared_config: SharedConfig, container: Container) -> None:
    map_config = shared_config.get_map(FieldType(get_map()))
    field_filter = load_field_filter(map_config, config.field_filter, container)
    container.register(map_config)
    container.register(field_filter, FieldFilterInterface)

    logger.info(f"Field filter: {field_filter}")


def make_field_request_handler(config: Config, container: Container) -> None:
    ros_factory = container.resolve(RosFactory)
    request_subscriber = ros_factory.make_raw_subscriber("/perception/field/request", "std_msgs/Empty", queue_size=1)
    response_subscriber = ros_factory.make_publisher("/perception/field/response", FieldResult, queue_size=1)
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

    camera = load_camera(config.camera)
    print()  # Start log on a fresh line
    logger.info(f"Camera: {camera}")

    container = Container()
    make_ros_factory(config, container)
    make_field_segmentation(config, container)
    make_field_interface(config, shared_config, container)
    make_field_request_handler(config, container)

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
