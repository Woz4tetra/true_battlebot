from bw_shared.camera_calibration.board.aruco_board import ArucoBoard
from bw_shared.camera_calibration.board.load_board import load_board
from bw_shared.camera_calibration.board_config import BoardConfig, BoardType
from bw_shared.camera_calibration.detector.aruco_detector import ArucoGridDetector, CharucoDetector
from bw_shared.camera_calibration.detector.detector import Detector


def load_detector(config: BoardConfig, parameter_path: str) -> Detector:
    board = load_board(config)
    if not isinstance(board, ArucoBoard):
        raise ValueError(f"Board is not of type ArucoBoard: {type(board)}")
    return {
        BoardType.CHARUCO: CharucoDetector,
        BoardType.ARUCO_GRID: ArucoGridDetector,
    }[config.board_type](board, parameter_path)
