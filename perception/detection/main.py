import argparse
import logging
import time
from dataclasses import dataclass
from typing import Protocol, cast

from bw_shared.configs.maps import MapConfig
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_map, get_robot
from camera.camera_interface import CameraInterface
from camera.camera_loader import load_camera
from config.config import Config
from config.config_loader import load_config
from field_filter.field_filter import FieldFilter
from field_filter.field_request_handler import FieldRequestHandler
from json_logger import initialize
from perception_tools.messages.camera.camera_data import CameraData
from rosbridge.ros_factory import RosFactory
from rosbridge.ros_publisher import RosPublisher
from segmentation.segmentation_interface import SegmentationInterface
from segmentation.segmentation_loader import load_segmentation


class CommandLineArgs(Protocol):
    config_dir: str


@dataclass
class App:
    shared_config: SharedConfig
    field_config: MapConfig
    config: Config
    camera: CameraInterface
    field_segmentation: SegmentationInterface
    field_filter: FieldFilter
    field_request_handler: FieldRequestHandler
    field_debug_image_publisher: RosPublisher
    ros_factory: RosFactory
    camera_data: CameraData | None = None

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
                self.field_debug_image_publisher.publish(debug_image.to_ros_image())

    def stop(self) -> None:
        self.ros_factory.disconnect


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

    ros_factory = RosFactory(config.rosbridge.host, config.rosbridge.port)

    field_segmentation = load_segmentation(config.field_segmentation)
    logger.info(f"Field segmentation: {field_segmentation}")

    map_config = shared_config.get_map(FieldType(get_map()))
    field_filter = FieldFilter(map_config, config.field_filter)

    request_subscriber = ros_factory.make_subscriber("/perception/field/request", "std_msgs/Empty", queue_size=1)
    response_subscriber = ros_factory.make_publisher(
        "/perception/field/response", "bw_interfaces/EstimatedObject", queue_size=1
    )
    field_request_handler = FieldRequestHandler(
        config.field_filter.stale_image_timeout, request_subscriber, response_subscriber
    )
    field_debug_image_publisher = ros_factory.make_publisher(
        "/perception/field/debug_image", "sensor_msgs/Image", queue_size=1
    )

    poll_delay = 1.0 / config.poll_rate
    app = App(
        shared_config=shared_config,
        field_config=map_config,
        config=config,
        camera=camera,
        field_segmentation=field_segmentation,
        field_filter=field_filter,
        field_request_handler=field_request_handler,
        field_debug_image_publisher=field_debug_image_publisher,
        ros_factory=ros_factory,
    )

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
