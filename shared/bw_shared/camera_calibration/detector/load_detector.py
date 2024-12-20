from bw_shared.camera_calibration.board.load_board import load_board
from bw_shared.camera_calibration.board_config import BoardConfig, BoardType
from bw_shared.camera_calibration.detector.aruco_detector import ArucoGridDetector, CharucoDetector
from bw_shared.camera_calibration.detector.detector import Detector


def load_detector(config: BoardConfig, parameter_path: str) -> Detector:
    return {
        BoardType.CHARUCO: CharucoDetector,
        BoardType.ARUCO_GRID: ArucoGridDetector,
    }[config.board_type](load_board(config), parameter_path)
