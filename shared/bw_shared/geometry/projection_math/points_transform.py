import numpy as np
from numba import njit


def points_transform_by(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    # transform is a 4x4 matrix
    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    padded_points = np.concatenate((points, ones), axis=1)
    return np.tensordot(padded_points, transform, axes=(1, 1))[:, 0:3]


@njit
def points_forward_by(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    # transform is a 4x4 matrix
    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    padded_points = np.concatenate((points, ones), axis=1)
    for i in range(padded_points.shape[0]):
        padded_points[i] = np.dot(padded_points[i], transform)
    return padded_points[:, 0:3]
