import numpy as np
from bw_shared.geometry.projection_math.plane_from_3_points import plane_from_3_points


def points_to_plane(plane_points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    point1, point2, point3 = plane_points[0:3]
    plane_tri_points = np.array([point1, point2, point3])
    plane_normal = plane_from_3_points(point1, point2, point3)
    plane_center = np.mean(plane_tri_points, axis=0)
    return plane_center, plane_normal
