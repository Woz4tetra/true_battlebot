import numpy as np
from cv2 import aruco

from bw_shared.camera_calibration.board.aruco_board import DICT_MAPPING, ArucoBoard
from bw_shared.camera_calibration.board_config import BoardConfig


class ArucoGridBoard(ArucoBoard):
    def __init__(self, config: BoardConfig) -> None:
        aruco_dict = aruco.getPredefinedDictionary(DICT_MAPPING[config.tag_family])
        ids = config.start_id + np.arange(0, config.num_rows * config.num_columns)
        config.square_size = config.marker_size
        marker_separation = (config.board_width / config.num_columns - config.marker_size) / 2
        aruco_board = aruco.GridBoard(
            (config.num_rows, config.num_columns),
            config.marker_size,
            marker_separation,
            aruco_dict,
            ids=ids,
        )
        self.ids = ids.tolist()
        super().__init__(config, aruco_board, aruco_dict)

    def get_ids(self) -> list[int]:
        return self.ids
