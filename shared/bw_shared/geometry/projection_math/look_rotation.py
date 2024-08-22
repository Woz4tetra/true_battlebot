from typing import Optional

import numpy as np
from geometry_msgs.msg import Quaternion, Vector3

from bw_shared.geometry.rotation_transforms import quaternion_from_matrix

EPSILON = np.finfo(float).eps


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm <= EPSILON:
        return vec
    return vec / norm


def look_rotation(forward: np.ndarray, up: Optional[np.ndarray] = None) -> Quaternion:
    """
    Create a quaternion representing an observer at the origin looking in the direction of the forward vector.
    Up vector is used to determine the orientation of the observer.
    :param forward: The direction to look
    :return: The quaternion representing the rotation
    """
    if np.linalg.norm(forward) <= EPSILON:
        return Quaternion(w=1.0)
    if up is None:
        up = np.array([0.0, 0.0, 1.0])
    if np.linalg.norm(up) <= EPSILON:
        return Quaternion(w=1.0)

    # compute rotation matrix
    new_forward = normalize(forward)
    new_left = normalize(np.cross(up, new_forward))
    new_up = normalize(np.cross(new_forward, new_left))
    rotation_matrix = np.column_stack([new_forward, new_left, new_up])

    homogeneous_rotation = np.eye(4)
    homogeneous_rotation[:3, :3] = rotation_matrix
    return quaternion_from_matrix(homogeneous_rotation)
