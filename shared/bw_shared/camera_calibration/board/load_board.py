from bw_shared.camera_calibration.board import AprilGridBoard, ArucoGridBoard, CharucoBoard
from bw_shared.camera_calibration.board.board import Board
from bw_shared.camera_calibration.board_config import BoardConfig, BoardType


def load_board(config: BoardConfig) -> Board:
    return {
        BoardType.CHARUCO: CharucoBoard,
        BoardType.ARUCO_GRID: ArucoGridBoard,
        BoardType.APRIL_GRID: AprilGridBoard,
    }[config.board_type](config)
