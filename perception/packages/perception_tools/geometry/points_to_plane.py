import numpy as np
from bw_shared.geometry.plane import Plane


def points_to_plane(plane_points: np.ndarray) -> Plane:
    point1, point2, point3 = plane_points[0:3]
    return Plane.from_3_point_arrays(point1, point2, point3)
