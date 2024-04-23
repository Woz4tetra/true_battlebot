import argparse
import logging
import time
from typing import Protocol, cast

from app import App
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_map, get_robot
from camera.camera_loader import load_camera
from config.config_loader import load_config
from container import Container
from field_filter.field_filter import FieldFilter
from field_filter.field_request_handler import FieldRequestHandler
from json_logger import initialize
from perception_tools.messages.camera.image import Image
from perception_tools.messages.field_result import FieldResult
from perception_tools.rosbridge.ros_factory import RosFactory
from segmentation.segmentation_loader import load_segmentation


class CommandLineArgs(Protocol):
    config_dir: str


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config_dir", type=str)
    args: CommandLineArgs = cast(CommandLineArgs, parser.parse_args())
    logger = logging.getLogger("perception")

    config_dir = args.config_dir

    shared_config = SharedConfig.from_files()
    config = load_config(config_dir, get_robot())

    initialize()

    camera = load_camera(config.camera)
    print()  # Start log on a fresh line
    logger.info(f"Camera: {camera}")

    container = Container()

    ros_factory = RosFactory(config.rosbridge.host, config.rosbridge.port)
    container.register(ros_factory)

    field_segmentation = load_segmentation(config.field_segmentation)
    container.register(field_segmentation, "field_segmentation")
    logger.info(f"Field segmentation: {field_segmentation}")

    map_config = shared_config.get_map(FieldType(get_map()))
    field_filter = FieldFilter(map_config, config.field_filter)
    container.register(field_filter)

    request_subscriber = ros_factory.make_raw_subscriber("/perception/field/request", "std_msgs/Empty", queue_size=1)
    response_subscriber = ros_factory.make_publisher("/perception/field/response", FieldResult, queue_size=1)
    field_request_handler = FieldRequestHandler(
        config.field_filter.stale_image_timeout, request_subscriber, response_subscriber
    )
    container.register(field_request_handler)
    field_debug_image_publisher = ros_factory.make_publisher("/perception/field/debug_image", Image, queue_size=1)
    container.register(field_debug_image_publisher, "field_debug_image_publisher")

    poll_delay = 1.0 / config.poll_rate
    app = App(container)

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


if __name__ == "__main__":
    main()
