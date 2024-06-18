from abc import abstractmethod

import numpy as np


class BasePlaneSolver:
    @abstractmethod
    def solve(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        pass
