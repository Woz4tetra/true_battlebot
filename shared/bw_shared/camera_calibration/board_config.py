from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import toml
from cv2 import aruco

from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class BoardConfig:
    texture_size: int = 4096
    board_size: float = 2.0  # meters
    num_rows: int = 8
    start_id: int = 1
    marker_size: float = 0.16
    square_size: float = 0.24
    border_bits: int = 1

    def __post_init__(self) -> None:
        self.ids = self.start_id + np.arange(0, self.num_rows * self.num_rows // 2)
        self.px_per_meter = self.texture_size / self.board_size
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.board = aruco.CharucoBoard(
            (self.num_rows, self.num_rows), self.square_size, self.marker_size, self.aruco_dict, ids=self.ids
        )
        self.all_tag_width = self.num_rows * self.square_size
        self.grid_points = self.get_grid_points()

    def generate_image(self) -> np.ndarray:
        all_marker_size_px = self.num_rows * self.square_size * self.px_per_meter
        margin_size = int(self.texture_size - all_marker_size_px) // 2
        if margin_size < 0:
            raise ValueError("Texture size is too small for the given board size")
        return self.board.generateImage(
            (self.texture_size, self.texture_size), borderBits=self.border_bits, marginSize=margin_size
        )

    def get_grid_points(self, anchor: tuple[int, int] = (0, 0)) -> np.ndarray:
        grid_size = self.num_rows + 1
        length = self.all_tag_width
        x_range = ((anchor[0]) * (length / 2), (anchor[0] + 2) * (length / 2))
        y_range = ((anchor[1]) * (length / 2), (anchor[1] + 2) * (length / 2))
        mesh_x, mesh_y = np.meshgrid(
            np.linspace(x_range[0], x_range[1], grid_size), np.linspace(y_range[0], y_range[1], grid_size)
        )
        grid_in_tag = np.vstack([mesh_x.ravel(), mesh_y.ravel()]).reshape(2, -1).T
        zeros = np.zeros((grid_size * grid_size, 1))
        return np.concatenate((grid_in_tag, zeros), axis=1)

    @classmethod
    def from_dict(cls, data: dict) -> BoardConfig:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)

    @classmethod
    def from_file(cls, path: str) -> BoardConfig:
        with open(path, "r") as file:
            return cls.from_dict(toml.load(file))

    def to_file(self, path: str) -> None:
        with open(path, "w") as file:
            toml.dump(self.to_dict(), file)
