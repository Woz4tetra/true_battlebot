from bw_shared.camera_calibration.board.aruco_board import ArucoGridBoard, CharucoBoard
from bw_shared.camera_calibration.board.board import Board
from bw_shared.camera_calibration.board_config import BoardConfig, BoardType


def load_board(config: BoardConfig) -> Board:
    return {
        BoardType.CHARUCO: CharucoBoard,
        BoardType.ARUCO_GRID: ArucoGridBoard,
    }[config.board_type](config)
