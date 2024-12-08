import numpy as np
from geometry_msgs.msg import Quaternion

from bw_shared.geometry import rotation_transforms_core
from bw_shared.geometry.rotation_transforms_core import AXIS_KEY
from bw_shared.geometry.rpy import RPY


def euler_matrix(rpy: RPY, axes: AXIS_KEY = "sxyz") -> np.ndarray:
    return rotation_transforms_core.euler_matrix_core(rpy[0], rpy[1], rpy[2], axes)


def euler_from_matrix(matrix: np.ndarray, axes: AXIS_KEY = "sxyz") -> RPY:
    return RPY(rotation_transforms_core.euler_from_matrix_core(matrix, axes))


def quaternion_matrix(quaternion: Quaternion) -> np.ndarray:
    return rotation_transforms_core.quaternion_matrix_core((quaternion.x, quaternion.y, quaternion.z, quaternion.w))


def quaternion_from_matrix(matrix: np.ndarray) -> Quaternion:
    return Quaternion(*rotation_transforms_core.quaternion_from_matrix_core(matrix))


def euler_from_quaternion(quaternion: Quaternion, axes: AXIS_KEY = "sxyz") -> RPY:
    return RPY(
        rotation_transforms_core.euler_from_quaternion_core(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w), axes
        )
    )


def quaternion_from_euler(roll: float, pitch: float, yaw: float, axes: AXIS_KEY = "sxyz") -> Quaternion:
    return Quaternion(*rotation_transforms_core.quaternion_from_euler_core(roll, pitch, yaw, axes))
