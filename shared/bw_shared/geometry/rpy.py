from __future__ import annotations

from typing import Iterable, Tuple, Union, cast

import numpy as np
from geometry_msgs.msg import Quaternion

from bw_shared.geometry.rotation_transforms_core import (
    euler_from_matrix_core,
    euler_from_quaternion_core,
    euler_matrix_core,
    quaternion_from_euler_core,
)


class RPY(Tuple[float, float, float]):
    def __new__(cls, iterable: Iterable[float]) -> RPY:
        vals = tuple(iterable)
        if (num := len(vals)) != 3:
            raise TypeError(f"Expected 3 elements, got {num} from {iterable}")
        return tuple.__new__(cls, vals)

    @classmethod
    def from_array(cls, np_array: np.ndarray) -> RPY:
        """
        Return a correctly typed instance of RPY from a numpy float array of length 3.
        """
        assert np_array.shape == (3,)

        return cls(cast(Tuple[float, float, float], tuple(np_array)))

    def to_array(self) -> np.ndarray:
        """
        Return a size 3 numpy array representing the coordinate
        """
        return np.array(self)

    @classmethod
    def zeros(cls) -> RPY:
        return RPY((0.0, 0.0, 0.0))

    @property
    def roll(self) -> float:
        return self[0]

    @property
    def pitch(self) -> float:
        return self[1]

    @property
    def yaw(self) -> float:
        return self[2]

    def to_matrix(self) -> np.ndarray:
        return euler_matrix_core(self.roll, self.pitch, self.yaw)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> RPY:
        angles = euler_from_matrix_core(matrix)
        return cls(angles)

    def to_quaternion(self) -> Quaternion:
        return Quaternion(*quaternion_from_euler_core(self.roll, self.pitch, self.yaw))

    @classmethod
    def from_degrees(cls, iterable: Iterable[float]) -> RPY:
        return cls(np.deg2rad(np.array(iterable, dtype=np.float64)))

    @classmethod
    def from_quaternion(cls, quat: Quaternion) -> RPY:
        angles = euler_from_quaternion_core((quat.x, quat.y, quat.z, quat.w))
        return cls(angles)

    def __add__(self, other: Union[RPY, Tuple]) -> RPY:
        return RPY((self[0] + other[0], self[1] + other[1], self[2] + other[2]))

    def __sub__(self, other: RPY) -> RPY:
        return RPY((self[0] - other[0], self[1] - other[1], self[2] - other[2]))

    def __neg__(self) -> RPY:
        return RPY((-1 * self[0], -1 * self[1], -1 * self[2]))

    def __str__(self) -> str:
        return f"({self[0]:6.3f}, {self[1]:6.3f}, {self[2]:6.3f})"
