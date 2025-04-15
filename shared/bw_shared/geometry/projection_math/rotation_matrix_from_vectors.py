from typing import Optional

import numpy as np

from bw_shared.epsilon import EPSILON
from bw_shared.geometry.transform3d import Transform3D


def rotation_matrix_from_vectors(vec1: np.ndarray, vec2: np.ndarray, epsilon: float = EPSILON) -> np.ndarray:
    """
    Find the rotation matrix that aligns vec1 to vec2
    From https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    vec1_mag = np.linalg.norm(vec1)
    vec2_mag = np.linalg.norm(vec2)
    if vec1_mag < epsilon or vec2_mag < epsilon or np.allclose(vec1, vec2, atol=epsilon, rtol=0):
        return np.eye(3)
    a = (vec1 / vec1_mag).reshape(3)
    b = (vec2 / vec2_mag).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    if s < epsilon:
        return np.eye(3)
    kmat = np.array(
        [
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0],
        ]
    )
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2))
    return rotation_matrix


def transform_matrix_from_vectors(
    plane_center: np.ndarray, plane_normal: np.ndarray, up_vec: Optional[np.ndarray] = None, epsilon: float = EPSILON
) -> Transform3D:
    up_vec = np.array([0.0, 0.0, 1.0]) if up_vec is None else up_vec
    plane_tfmat = np.eye(4)
    plane_tfmat[:3, :3] = rotation_matrix_from_vectors(up_vec, plane_normal, epsilon)
    plane_tfmat[:3, 3] = plane_center
    return Transform3D(plane_tfmat)
