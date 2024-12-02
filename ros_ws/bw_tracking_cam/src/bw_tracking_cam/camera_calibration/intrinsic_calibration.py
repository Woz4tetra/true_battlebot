#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
import argparse
import os
from glob import glob
from typing import Generator, Optional

import argcomplete
import cv2
import numpy as np
import toml
from bw_shared.camera_calibration.board_config import BoardConfig
from bw_shared.camera_calibration.load_detector import load_detector
from bw_shared.geometry.camera.camera_info_loader import CameraInfoData
from cv2 import aruco
from sensor_msgs.msg import CameraInfo

from bw_tracking_cam.camera_calibration.load_images import load_images

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
os.chdir(SCRIPT_DIR)


def write_calibration(info: CameraInfo, calibration_path: str) -> None:
    with open(calibration_path, "w") as file:
        toml.dump(CameraInfoData.from_msg(info).to_dict(), file)


def compute_camera_info(
    configs: list[BoardConfig], parameter_path: str, images: Generator[np.ndarray, None, None], debug: bool = False
) -> CameraInfo:
    detectors = [load_detector(config, parameter_path) for config in configs]

    for detector, config in zip(detectors, configs):
        board_image = config.generate_image()
        if isinstance(detector, aruco.CharucoDetector):
            corners, tag_ids, marker_corners, marker_ids = detector.detectBoard(board_image)
        elif isinstance(detector, aruco.ArucoDetector):
            corners, tag_ids, rejected = detector.detectMarkers(board_image)
            marker_ids = np.array([tag_ids])
        else:
            raise ValueError("Unsupported detector type")
    marker_ids = np.sort(marker_ids.flatten())
    assert np.all(
        np.sort(config.ids) == np.sort(marker_ids)
    ), f"IDs do not match. Expected: {config.ids}, got: {marker_ids}"

    all_object_points = []
    all_image_points = []
    image_size: Optional[tuple[int, int]] = None

    window_name = "aruco"
    if debug:
        cv2.namedWindow(window_name)
    max_height = 1080

    def draw_image(image_to_draw: np.ndarray) -> None:
        if not debug:
            return
        # resize image if it is too large
        if image_to_draw.shape[1] > max_height:
            scale = max_height / image_to_draw.shape[1]
            image_to_draw = cv2.resize(image_to_draw, (0, 0), fx=scale, fy=scale)
        cv2.imshow(window_name, image_to_draw)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            quit()

    for image in images:
        if image_size is None:
            image_size = (image.shape[1], image.shape[0])
        else:
            assert image_size == (image.shape[1], image.shape[0]), "All images must have the same size"

        for detector in detectors:
            if isinstance(detector, aruco.CharucoDetector):
                corners, tag_ids, marker_corners, marker_ids = detector.detectBoard(image)
            elif isinstance(detector, aruco.ArucoDetector):
                corners, tag_ids, rejected = detector.detectMarkers(image)
            else:
                raise ValueError("Unsupported detector type")

            if corners is None or tag_ids is None:
                draw_image(image)
                continue
            object_points, image_points = config.board.matchImagePoints(corners, tag_ids)  # type: ignore
            if object_points is None or image_points is None:
                draw_image(image)
                continue
            all_image_points.append(image_points)
            all_object_points.append(object_points)

        debug_image = np.copy(image)
        debug_corners = np.array(corners, dtype=np.float64)
        # resize image if it is too large
        if debug_image.shape[1] > max_height:
            scale = max_height / debug_image.shape[1]
            debug_image = cv2.resize(debug_image, (0, 0), fx=scale, fy=scale)
            debug_corners = debug_corners * scale

        if isinstance(detector, aruco.CharucoDetector):
            debug_image = aruco.drawDetectedCornersCharuco(debug_image, debug_corners, tag_ids, (0, 255, 0))
        elif isinstance(detector, aruco.ArucoDetector):
            debug_image = aruco.drawDetectedMarkers(debug_image, debug_corners, tag_ids, (0, 255, 0))  # type: ignore
        else:
            raise ValueError("Unsupported detector type")
        draw_image(debug_image)

    print(f"Object points: {len(all_object_points)}")
    print(f"Image points: {len(all_image_points)}")
    print(f"Image size: {image_size}")
    assert image_size is not None, "No images found."
    ret, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        all_object_points,
        all_image_points,
        image_size,
        None,  # type: ignore
        None,  # type: ignore
    )

    mean_error = 0.0
    for index in range(len(all_object_points)):
        imgpoints2, _ = cv2.projectPoints(
            all_object_points[index], rvecs[index], tvecs[index], camera_matrix, distortion
        )
        error = cv2.norm(all_image_points[index], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    print(f"Total error: {mean_error}")

    rectification_matrix = np.eye(3)
    projection = np.zeros((3, 4))
    projection[0:3, 0:3] = camera_matrix
    projection_matrix = projection

    info = CameraInfo(
        width=image_size[0],
        height=image_size[1],
        distortion_model="plumb_bob",
        D=distortion.flatten().tolist(),
        K=camera_matrix.flatten().tolist(),
        R=rectification_matrix.flatten().tolist(),
        P=projection_matrix.flatten().tolist(),
    )
    return info


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", type=str, help="path to bag or video")
    parser.add_argument(
        "board_configs",
        type=str,
        nargs="+",
        choices=glob("*.toml"),
        default="simulation_board.toml",
        help="path to board config",
    )
    parser.add_argument(
        "-p",
        "--parameters",
        type=str,
        choices=glob("*.json"),
        default="calibration_parameters.json",
        help="calibration parameter file",
    )
    parser.add_argument("-c", "--calibration-path", type=str, default="", help="output directory")
    parser.add_argument("-s", "--skip-seconds", type=float, default=0.0, help="skip seconds")
    parser.add_argument("--debug", action="store_true", help="show debug image")
    argcomplete.autocomplete(parser)

    args = parser.parse_args()
    image_source = args.images
    parameter_path = args.parameters
    calibration_path = args.calibration_path
    board_config_paths = args.board_configs
    skip_interval = args.skip_seconds
    debug = args.debug

    if len(calibration_path) == 0:
        calibration_path = os.path.splitext(image_source)[0] + ".toml"

    configs = [BoardConfig.from_file(path) for path in board_config_paths]

    info = compute_camera_info(configs, parameter_path, load_images(image_source, skip_interval), debug)
    write_calibration(info, calibration_path)
    print(info)
    print(f"Calibration saved to {calibration_path}")


if __name__ == "__main__":
    main()
