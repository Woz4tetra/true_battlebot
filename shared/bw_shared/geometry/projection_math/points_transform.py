import numpy as np


def points_transform_by(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    # transform is a 4x4 matrix
    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    padded_points = np.concatenate((points, ones), axis=1)
    return np.tensordot(padded_points, transform, axes=(1, 1))[:, 0:3]
