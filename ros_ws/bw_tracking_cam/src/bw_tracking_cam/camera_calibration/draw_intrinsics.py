#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
import argparse
import os
from glob import glob

import argcomplete
import cv2
import toml
from bw_shared.camera_calibration.board_config import BoardConfig
from bw_shared.geometry.camera.camera_info_loader import CameraInfoData
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from bw_tools.tag_detection.draw_helpers import project_point_array_to_pixel
from sensor_msgs.msg import CameraInfo

from bw_tracking_cam.camera_calibration.load_images import load_images

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
os.chdir(SCRIPT_DIR)


def read_calibration(calibration_path: str) -> CameraInfo:
    with open(calibration_path, "r") as file:
        data = toml.load(file)
        return CameraInfoData.from_dict(data).to_msg()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", type=str, help="path to bag or video")
    parser.add_argument(
        "board_config",
        type=str,
        nargs="?",
        choices=glob("*.toml"),
        default="simulation_board.toml",
        help="path to board config",
    )
    parser.add_argument("calibration", type=str, help="path to calibration")
    parser.add_argument("-s", "--skip-seconds", type=float, default=0.0, help="skip seconds")

    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    calibration_path = args.calibration
    images_source = args.images
    board_config_path = args.board_config
    skip_seconds = args.skip_seconds

    info = read_calibration(calibration_path)
    rectifier = ImageRectifier(info)
    rectified_info = rectifier.get_rectified_info()

    config = BoardConfig.from_file(board_config_path)
    grid_in_tag = config.grid_points
    print(grid_in_tag)

    for image in load_images(images_source, skip_seconds):
        show_image = rectifier.rectify(image)
        # tag_pixels = project_point_array_to_pixel(grid_in_tag, rectified_info)
        # tag_pixels = tag_pixels.astype(int)
        # for tag_pixel in tag_pixels:
        #     cv2.circle(show_image, tag_pixel, 3, (255, 0, 0), -1)
        cv2.imshow("reproject", show_image)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            break


if __name__ == "__main__":
    main()
