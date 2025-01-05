import numpy as np


def find_ray_plane_insection(ray: np.ndarray, plane_normal: np.ndarray, plane_point: np.ndarray) -> np.ndarray:
    """
    Find the intersection of a ray with a plane
    :param ray: 3D ray
    :param plane_normal: normal vector of the plane
    :param plane_point: point on the plane
    :return: intersection point
    """
    t = np.dot(plane_normal, plane_point - ray) / np.dot(plane_normal, ray)
    return ray + t * ray
