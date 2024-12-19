import cv2
from cv2 import aruco

from bw_shared.camera_calibration.board_config import BoardConfig


def load_detector(config: BoardConfig, parameter_path: str) -> cv2.Algorithm:
    if config.board_type == "charuco":
        assert isinstance(config.board, aruco.CharucoBoard)
        detector = aruco.CharucoDetector(config.board, charuco_params, detector_params, refine_params)
    elif config.board_type == "grid":
        detector = aruco.ArucoDetector(config.aruco_dict, detector_params, refine_params)
    else:
        raise ValueError(f"Unsupported board type: {config.board_type}")
    return detector
