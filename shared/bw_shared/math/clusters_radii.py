import numpy as np
from scipy.spatial import KDTree


def clusters_radii(points_array: np.ndarray, min_cluster_size: int = 2) -> np.ndarray:
    """
    Returns the distances between each cluster of points.
    """
    if min_cluster_size < 1:
        raise ValueError("min_cluster_size must be >= 1")
    if min_cluster_size == 1:
        return np.zeros(points_array.shape[1])
    distances, _ = KDTree(points_array).query(points_array, k=min_cluster_size)
    return np.array(distances[:, min_cluster_size - 1])
