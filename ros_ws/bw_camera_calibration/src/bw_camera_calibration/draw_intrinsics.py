#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
import argparse
import os
from glob import glob

import argcomplete
import cv2
import numpy as np
import toml
from bw_shared.camera_calibration.board_config import BoardConfig
from bw_shared.camera_calibration.detector.load_detector import load_detector
from bw_shared.geometry.camera.camera_info_loader import CameraInfoData
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.projection_math.project_point_array_to_pixel import project_point_array_to_pixel
from sensor_msgs.msg import CameraInfo

from bw_camera_calibration.compute_board_pose import compute_board_pose
from bw_camera_calibration.load_images import load_images

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
os.chdir(SCRIPT_DIR)


def read_calibration(calibration_path: str) -> CameraInfo:
    with open(calibration_path, "r") as file:
        data = toml.load(file)
        return CameraInfoData.from_dict(data).to_msg()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", type=str, help="path to bag or video")
    parser.add_argument("calibration", type=str, help="path to calibration")
    parser.add_argument(
        "board_config",
        type=str,
        nargs="?",
        choices=glob("*.toml"),
        default="simulation_board.toml",
        help="path to board config",
    )
    parser.add_argument(
        "-f",
        "--filters",
        type=str,
        nargs="*",
        default="",
        help="Filters to apply to images. For bags, this is the topics to use.",
    )
    parser.add_argument(
        "-p",
        "--parameters",
        type=str,
        default="calibration_parameters.json",
        help="calibration parameter file",
    )
    parser.add_argument("-s", "--skip-seconds", type=float, default=0.0, help="skip seconds")

    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    calibration_path = args.calibration
    images_source = args.images
    board_config_path = args.board_config
    skip_seconds = args.skip_seconds
    filters = args.filters
    parameter_path = args.parameters

    info = read_calibration(calibration_path)
    print("Loaded camera info:\n", info)
    rectifier = ImageRectifier(info)
    rectified_info = rectifier.get_rectified_info()
    print("Rectified camera info:\n", rectified_info)

    config = BoardConfig.from_file(board_config_path)
    detector = load_detector(config, parameter_path)
    grid_in_tag = detector.board.get_grid_points()

    for image in load_images(images_source, skip_seconds, filters):
        rectified_image = rectifier.rectify(image)
        board_pose = compute_board_pose(rectified_image, rectified_info, detector)
        if board_pose is not None:
            grid_in_camera = points_transform_by(grid_in_tag, board_pose.tfmat)
            tag_pixels = project_point_array_to_pixel(grid_in_camera, rectified_info)
            tag_pixels = tag_pixels.astype(np.int32)
            for tag_pixel in tag_pixels:
                cv2.circle(rectified_image, tag_pixel, 3, (0, 255, 0), -1)
        cv2.imshow("reproject", rectified_image)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            break


if __name__ == "__main__":
    main()
