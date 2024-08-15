import argparse

import cv2
import numpy as np
from cv2 import aruco


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", type=str, nargs="+", help="path to input image file(s)")
    parser.add_argument(
        "-p", "--parameters", type=str, default="calibration_parameters.json", help="calibration parameter file"
    )
    parser.add_argument("-o", "--output", type=str, default="", help="output directory")
    parser.add_argument("--show-board", action="store_true", help="show generated board")
    args = parser.parse_args()
    image_paths = args.images
    parameter_path = args.parameters
    output_dir = args.output
    show_board = args.show_board

    id_grid = np.arange(0, 64).reshape((8, 8))
    id_grid = np.flip(id_grid, axis=1)
    ids = id_grid.flatten()
    marker_size = 0.15
    marker_separation = 0.05
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    board = aruco.GridBoard((8, 8), marker_size, marker_separation, aruco_dict, ids=ids)

    img = board.generateImage((800, 800), borderBits=2, marginSize=10)
    if show_board:
        img = np.rot90(img, k=2)
        img = np.array(img, dtype=np.uint8)
        cv2.imshow("aruco", img)
        cv2.waitKey(-1)

    aruco_params = aruco.DetectorParameters()

    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    fs = cv2.FileStorage(parameter_path, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_FORMAT_JSON)
    fnode: cv2.FileNode = fs.root()
    aruco_params.readDetectorParameters(fnode)

    detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    # Make sure detector works in perfect conditions
    corners, detected_ids, rejected = detector.detectMarkers(img)
    ids_array = np.array(detected_ids).flatten()
    assert np.all(np.sort(ids_array) == np.sort(ids)), f"IDs do not match. Expected: {ids}, got: {ids_array}"

    for image_path in image_paths:
        image = cv2.imread(image_path)
        corners, detected_ids, rejected = detector.detectMarkers(image)
        if len(corners) == 0:
            print(f"No markers found in {image_path}")
            continue
        aruco.drawDetectedMarkers(image, corners, detected_ids)
        cv2.imshow("aruco", image)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            break


if __name__ == "__main__":
    main()
