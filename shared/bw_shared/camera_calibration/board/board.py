from abc import ABC, abstractmethod

import numpy as np

from bw_shared.camera_calibration.board_config import BoardConfig


class Board(ABC):
    def __init__(self, config: BoardConfig) -> None:
        self.config = config

    @abstractmethod
    def generate_image(self) -> np.ndarray: ...

    @abstractmethod
    def get_width(self) -> float: ...

    @abstractmethod
    def get_height(self) -> float: ...

    def get_grid_points(self, anchor: tuple[int, int] = (0, 0)) -> np.ndarray:
        grid_size = self.config.num_rows + 1
        width = self.get_width()
        height = self.get_height()
        x_range = ((anchor[0]) * (width / 2), (anchor[0] + 2) * (width / 2))
        y_range = ((anchor[1]) * (height / 2), (anchor[1] + 2) * (height / 2))
        num_90_rotations = self.config.num_90_rotations % 4
        if num_90_rotations == 1:
            x_range, y_range = y_range, x_range
        elif num_90_rotations == 2:
            x_range = (-x_range[1], -x_range[0])
            y_range = (-y_range[1], -y_range[0])
        elif num_90_rotations == 3:
            x_range, y_range = y_range, x_range
            x_range = (-x_range[1], -x_range[0])
            y_range = (-y_range[1], -y_range[0])
        mesh_x, mesh_y = np.meshgrid(
            np.linspace(x_range[0], x_range[1], grid_size), np.linspace(y_range[0], y_range[1], grid_size)
        )
        grid_in_tag = np.vstack([mesh_x.ravel(), mesh_y.ravel()]).reshape(2, -1).T
        zeros = np.zeros((grid_size * grid_size, 1))
        return np.concatenate((grid_in_tag, zeros), axis=1)

    @abstractmethod
    def get_ids(self) -> list[int]: ...
