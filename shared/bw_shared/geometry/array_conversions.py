from typing import Union

import numpy as np
from geometry_msgs.msg import Point, Vector3

PointOrVector = Union[Point, Vector3]


def xyz_to_array(xyz: PointOrVector) -> np.ndarray:
    """Convert a Point or Vector3 to a numpy array."""
    return np.array([xyz.x, xyz.y, xyz.z])


def array_to_vector3(array: np.ndarray) -> Vector3:
    """Convert a numpy array to a Vector3."""
    return Vector3(x=array[0], y=array[1], z=array[2])
