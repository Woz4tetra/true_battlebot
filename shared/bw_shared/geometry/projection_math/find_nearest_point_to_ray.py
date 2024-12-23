from typing import Optional

import numpy as np


def find_nearest_point_to_ray(
    points: np.ndarray, ray: np.ndarray, ray_origin: Optional[np.ndarray] = None
) -> np.ndarray:
    """
    Find the nearest point in a set of points to a ray.

    Args:
        points: An Nx3 array of points.
        ray: A 3 element array representing a ray projected from (0, 0, 0)

    Returns:
        The point in the set of points that is nearest to the ray.
    """

    if len(points.shape) <= 1:
        raise ValueError("points must be at least a 2D array")
    if len(points) == 1:
        return points[0]

    ray_origin = ray_origin if ray_origin is not None else np.zeros(3)
    ray = ray - ray_origin
    points = points - ray_origin

    dot_products = np.tensordot(points, ray, axes=1)
    dot_products = np.maximum(dot_products, 0.0)  # if negative, the closest point is in the opposite direction to ray.
    closest_points = ray * dot_products[:, None]
    distances = np.linalg.norm(points - closest_points, axis=1)
    min_index = np.argmin(distances)
    nearest_point = points[min_index]
    return nearest_point
