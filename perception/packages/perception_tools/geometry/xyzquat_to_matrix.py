import numpy as np
from bw_shared.geometry.rotation_transforms_core import quaternion_matrix_core


def xyzquat_to_matrix(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    homogeneous_matrix = quaternion_matrix_core((qx, qy, qz, qw))
    homogeneous_matrix[0, 3] = x
    homogeneous_matrix[1, 3] = y
    homogeneous_matrix[2, 3] = z
    return homogeneous_matrix
