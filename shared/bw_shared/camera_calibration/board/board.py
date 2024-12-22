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

    @abstractmethod
    def get_grid_points(self, anchor: tuple[int, int] = (0, 0)) -> np.ndarray: ...

    @abstractmethod
    def get_ids(self) -> list[int]: ...
