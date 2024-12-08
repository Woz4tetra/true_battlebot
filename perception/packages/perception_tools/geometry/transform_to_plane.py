import numpy as np
from bw_shared.geometry.projection_math.plane_from_3_points import plane_from_3_points


def transform_to_plane(transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    point_0 = np.array([0, 0, 0, 1])
    point_1 = np.array([1, 0, 0, 1])
    point_2 = np.array([0, 1, 0, 1])

    transformed_point_0 = np.dot(transform, point_0)[0:3]
    transformed_point_1 = np.dot(transform, point_1)[0:3]
    transformed_point_2 = np.dot(transform, point_2)[0:3]

    plane_normal = plane_from_3_points(transformed_point_0, transformed_point_1, transformed_point_2)
    return transformed_point_0, plane_normal
