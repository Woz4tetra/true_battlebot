import numpy as np
from cv2 import aruco

from bw_shared.camera_calibration.board.board import Board
from bw_shared.camera_calibration.board_config import BoardConfig

DICT_MAPPING = {
    "4x4_50": aruco.DICT_4X4_50,
    "4x4_100": aruco.DICT_4X4_100,
    "4x4_250": aruco.DICT_4X4_250,
    "4x4_1000": aruco.DICT_4X4_1000,
    "5x5_50": aruco.DICT_5X5_50,
    "5x5_100": aruco.DICT_5X5_100,
    "5x5_250": aruco.DICT_5X5_250,
    "5x5_1000": aruco.DICT_5X5_1000,
    "6x6_50": aruco.DICT_6X6_50,
    "6x6_100": aruco.DICT_6X6_100,
    "6x6_250": aruco.DICT_6X6_250,
    "6x6_1000": aruco.DICT_6X6_1000,
    "7x7_50": aruco.DICT_7X7_50,
    "7x7_100": aruco.DICT_7X7_100,
    "7x7_250": aruco.DICT_7X7_250,
    "7x7_1000": aruco.DICT_7X7_1000,
    "aruco_original": aruco.DICT_ARUCO_ORIGINAL,
    "apriltag_16h5": aruco.DICT_APRILTAG_16h5,
    "apriltag_25h9": aruco.DICT_APRILTAG_25h9,
    "apriltag_36h10": aruco.DICT_APRILTAG_36h10,
    "apriltag_36h11": aruco.DICT_APRILTAG_36h11,
    "aruco_mip_36h12": aruco.DICT_ARUCO_MIP_36h12,
}


class ArucoBoard(Board):
    def __init__(self, config: BoardConfig, aruco_board: aruco.Board, aruco_dict: aruco.Dictionary) -> None:
        self.aruco_board = aruco_board
        self.aruco_dict = aruco_dict
        super().__init__(config)

    def generate_image(self) -> np.ndarray:
        all_marker_size_px = (
            self.config.num_rows * max(self.config.square_size, self.config.marker_size) * self.config.px_per_meter
        )
        margin_size = int(self.config.texture_size - all_marker_size_px) // 2
        if margin_size < 0:
            raise ValueError("Texture size is too small for the given board size")
        image = self.aruco_board.generateImage(
            (self.config.texture_size, self.config.texture_size),
            borderBits=self.config.border_bits,
            marginSize=margin_size,
        )
        image = np.rot90(image, self.config.num_90_rotations)
        return image


class CharucoBoard(ArucoBoard):
    def __init__(self, config: BoardConfig) -> None:
        aruco_dict = aruco.getPredefinedDictionary(DICT_MAPPING[config.tag_family])
        ids = config.start_id + np.arange(0, config.num_rows * config.num_rows // 2)
        aruco_board = aruco.CharucoBoard(
            (config.num_rows, config.num_rows),
            config.square_size,
            config.marker_size,
            aruco_dict,
            ids=ids,
        )
        self.ids = ids.tolist()
        super().__init__(config, aruco_board, aruco_dict)

    def get_ids(self) -> list[int]:
        return self.ids


class ArucoGridBoard(ArucoBoard):
    def __init__(self, config: BoardConfig) -> None:
        aruco_dict = aruco.getPredefinedDictionary(DICT_MAPPING[config.tag_family])
        ids = config.start_id + np.arange(0, config.num_rows * config.num_rows)
        config.square_size = 0.0
        marker_separation = (config.board_size / config.num_rows - config.marker_size) / 2
        aruco_board = aruco.GridBoard(
            (config.num_rows, config.num_rows),
            config.marker_size,
            marker_separation,
            aruco_dict,
            ids=ids,
        )
        self.ids = ids.tolist()
        super().__init__(config, aruco_board, aruco_dict)

    def get_ids(self) -> list[int]:
        return self.ids
