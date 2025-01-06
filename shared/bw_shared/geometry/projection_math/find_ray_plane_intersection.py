import numpy as np

EPSILON = np.finfo(float).eps


def find_ray_plane_intersection(ray: np.ndarray, plane_point: np.ndarray, plane_normal: np.ndarray) -> np.ndarray:
    """
    Find the intersection of a ray with a plane

    Args:
        ray: The direction of the ray. Assumes the ray starts at the origin.
        plane_point: A point on the plane relative to the origin.
        plane_normal: The normal vector of the plane.
    Returns:
        The point where the ray intersects the plane. numpy array of shape (3,)
    """

    dot_with_normal = np.dot(ray, plane_normal)

    if np.abs(dot_with_normal) < EPSILON:
        # Ray is parallel to the plane
        return np.zeros(3)

    fac = -1 * np.dot(plane_point, -1 * plane_normal) / dot_with_normal
    return ray * fac
