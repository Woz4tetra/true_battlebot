import argparse
from typing import Optional, Tuple

import cv2
import numpy as np
from board_config import BoardConfig
from cv2 import aruco
from tqdm import tqdm


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", type=str, nargs="+", help="path to input image file(s)")
    parser.add_argument(
        "-p", "--parameters", type=str, default="calibration_parameters.json", help="calibration parameter file"
    )
    parser.add_argument("-o", "--output", type=str, default="", help="output directory")
    args = parser.parse_args()
    image_paths = args.images
    parameter_path = args.parameters
    output_dir = args.output

    acceptance_threshold = 0.9
    config = BoardConfig()

    detector_params = aruco.DetectorParameters()
    charuco_params = aruco.CharucoParameters()
    refine_params = aruco.RefineParameters()

    fs = cv2.FileStorage(parameter_path, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_FORMAT_JSON)
    fnode: cv2.FileNode = fs.root()
    detector_params.readDetectorParameters(fnode)

    detector_params.markerBorderBits = config.border_bits

    detector = aruco.CharucoDetector(config.board, charuco_params, detector_params, refine_params)

    board_image = config.generate_image()
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(board_image)
    marker_ids = np.sort(marker_ids.flatten())
    assert np.all(
        np.sort(config.ids) == np.sort(marker_ids)
    ), f"IDs do not match. Expected: {config.ids}, got: {marker_ids}"

    all_object_points = []
    all_image_points = []
    image_size: Optional[Tuple[int, int]] = None
    with tqdm(image_paths) as pbar:
        for index, image_path in enumerate(image_paths):
            image = cv2.imread(image_path)
            if image_size is None:
                image_size = (image.shape[1], image.shape[0])
            else:
                assert image_size == (image.shape[1], image.shape[0]), "All images must have the same size"

            charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

            if charuco_corners is None or charuco_ids is None:
                print(f"No charuco corners found in {image_path}")
                continue
            object_points, image_points = config.board.matchImagePoints(charuco_corners, charuco_ids)
            all_image_points.append(image_points)
            all_object_points.append(object_points)
            aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids, (0, 255, 0))
            # cv2.imshow("aruco", image)
            # key = chr(cv2.waitKey(-1) & 0xFF)
            # if key == "q":
            #     break

            pbar.update(1)

    ret, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        all_object_points, all_image_points, image_size, None, None
    )
    print(camera_matrix)
    print(distortion)


if __name__ == "__main__":
    main()
