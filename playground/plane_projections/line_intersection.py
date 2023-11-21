from typing import Optional

import numpy as np
from bw_object_filter.field_math.find_minimum_rectangle import find_minimum_rectangle
from bw_tools.structs.rpy import RPY
from bw_tools.structs.transform3d import Transform3D
from geometry_msgs.msg import Vector3


def line_plane_intersection(
    line_point0: np.ndarray,
    line_point1: np.ndarray,
    plane_center: np.ndarray,
    plane_normal: np.ndarray,
    epsilon: float = 1e-6,
) -> Optional[np.ndarray]:
    """
    Calculate the intersection of a line and a plane
    :param line: 3D line in the form of a point and a vector
    :param plane: 3D plane in the form of a point and a normal vector
    :return: 3D point of intersection
    """
    root_vector = line_point1 - line_point0
    dot = plane_normal @ root_vector
    if abs(dot) > epsilon:
        # The factor of the point between p0 -> p1 (0 - 1)
        # If 'fac' is between (0 - 1) the point intersects with the segment.
        # Otherwise:
        #  < 0.0: behind p0.
        #  > 1.0: infront of p1.
        w = line_point0 - plane_center
        fac = -plane_normal @ w / dot
        u = root_vector * fac
        return line_point0 + u
    return None


def main():
    plane_transform = Transform3D.from_position_and_rpy(Vector3(1, 3, 5), RPY((np.pi / 4, 0, np.pi / 4)))
    plane_normal = plane_transform.rotation_matrix @ np.array([0, 0, 1])
    print(plane_normal)
    line_point0 = np.array([0, 0, 0])
    line_point1 = np.array([1, 1, 1])
    plane_center = np.array([1, 3, 5])
    print(line_plane_intersection(line_point0, line_point1, plane_center, plane_normal[0:3]))

    points = np.array(
        [
            [6.6131123, 46.5124914],
            [6.6129421, 46.5125385],
            [6.6127998, 46.5125783],
            [6.6126291, 46.512626],
            [6.6125308, 46.5124593],
            [6.6127016, 46.5124121],
            [6.6128452, 46.5123724],
            [6.6130153, 46.5123244],
            [6.6131123, 46.5124914],
        ]
    )
    print(find_minimum_rectangle(points))


if __name__ == "__main__":
    main()
