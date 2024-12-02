import cv2
from cv2 import aruco

from bw_shared.camera_calibration.board_config import BoardConfig


def load_detector(config: BoardConfig, parameter_path: str) -> cv2.Algorithm:
    detector_params = aruco.DetectorParameters()
    charuco_params = aruco.CharucoParameters()
    refine_params = aruco.RefineParameters()

    fs = cv2.FileStorage(parameter_path, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_FORMAT_JSON)
    fnode: cv2.FileNode = fs.root()
    detector_params.readDetectorParameters(fnode)

    detector_params.markerBorderBits = config.border_bits

    if config.board_type == "charuco":
        assert isinstance(config.board, aruco.CharucoBoard)
        detector = aruco.CharucoDetector(config.board, charuco_params, detector_params, refine_params)
    elif config.board_type == "grid":
        detector = aruco.ArucoDetector(config.aruco_dict, detector_params, refine_params)
    else:
        raise ValueError(f"Unsupported board type: {config.board_type}")
    return detector
