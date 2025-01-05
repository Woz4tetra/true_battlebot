import numpy as np


def plane_from_3_points(point1: np.ndarray, point2: np.ndarray, point3: np.ndarray) -> np.ndarray:
    """
    Calculate the plane defined by 3 points
    :param point1: 3D point
    :param point2: 3D point
    :param point3: 3D point
    :return: normal vector of the plane
    """
    normal = np.cross(point2 - point1, point3 - point1)
    magnitude = np.linalg.norm(normal)
    if magnitude < 1e-6:
        return np.array([0.0, 0.0, 1.0])
    return normal / magnitude
