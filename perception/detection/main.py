import argparse
from typing import Protocol, cast

from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.field_type import FieldType
from bw_shared.environment import get_map, get_robot
from camera.camera_loader import load_camera
from config.config_loader import load_config
from field_filter.field_filter import FieldFilter
from perception_tools.messages.camera.camera_data import CameraData

from perception.detection.field_filter.field_request_handler import FieldRequestHandler
from perception.detection.segmentation.instance_segmentation import InstanceSegmentation


class CommandLineArgs(Protocol):
    config_dir: str


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config_dir", type=str)
    args: CommandLineArgs = cast(CommandLineArgs, parser.parse_args())

    config_dir = args.config_dir
    shared_config = SharedConfig.from_files()
    field_config = shared_config.get_map(FieldType(get_map()))
    config = load_config(config_dir, get_robot())
    camera = load_camera(config.camera)
    field_segmentation = InstanceSegmentation()
    field_filter = FieldFilter(field_config)
    field_request_handler = FieldRequestHandler()

    camera_data: CameraData | None = None
    while True:
        if camera_data := camera.poll():
            pass
        if camera_data is not None and field_request_handler.has_request():
            image = camera_data.color_image
            seg_result, debug_image = field_segmentation.process_image(image)
            field_result = field_filter.compute_field(seg_result, camera_data.depth_image, camera_data.camera_info)
            field_request_handler.send_response(field_result)


if __name__ == "__main__":
    main()
