from bw_shared.camera_calibration.board.april_grid_board import AprilGridBoard
from bw_shared.camera_calibration.board.aruco_board import ArucoBoard
from bw_shared.camera_calibration.board.board import Board
from bw_shared.camera_calibration.board.load_board import load_board
from bw_shared.camera_calibration.board_config import BoardConfig, BoardType
from bw_shared.camera_calibration.detector.april_grid_detector import AprilGridDetector
from bw_shared.camera_calibration.detector.aruco_detector import ArucoDetector, ArucoGridDetector, CharucoDetector
from bw_shared.camera_calibration.detector.detector import Detector


def load_aruco_detector(config: BoardConfig, board: Board, parameter_path: str) -> ArucoDetector:
    if not isinstance(board, ArucoBoard):
        raise ValueError(f"Board is not of type ArucoBoard: {type(board)}")
    return {
        BoardType.CHARUCO: CharucoDetector,
        BoardType.ARUCO_GRID: ArucoGridDetector,
    }[config.board_type](board, parameter_path)


def load_detector(config: BoardConfig, parameter_path: str) -> Detector:
    board = load_board(config)
    if config.board_type in (BoardType.CHARUCO, BoardType.ARUCO_GRID):
        return load_aruco_detector(config, board, parameter_path)
    elif config.board_type == BoardType.APRIL_GRID:
        if not isinstance(board, AprilGridBoard):
            raise ValueError(f"Board is not of type AprilGridBoard: {type(board)}")
        return AprilGridDetector(board)
    else:
        raise ValueError(f"Unknown detector type: {config.board_type}")
