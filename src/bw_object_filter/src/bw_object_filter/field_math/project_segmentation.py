from typing import Optional

import numpy as np
from bw_interfaces.msg import Contour, SegmentationInstance, UVKeypoint
from bw_tools.structs.transform3d import Transform3D
from image_geometry import PinholeCameraModel


def line_plane_intersection(
    line_point0: np.ndarray,
    line_point1: np.ndarray,
    plane_center: np.ndarray,
    plane_normal: np.ndarray,
    epsilon: float = 1e-6,
) -> Optional[np.ndarray]:
    """
    Calculate the intersection of a line and a plane

    Credit: https://stackoverflow.com/questions/5666222/3d-line-plane-intersection
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


def project_segmentation(
    camera_model: PinholeCameraModel, plane_transform: Transform3D, segmentation: SegmentationInstance
) -> np.ndarray:
    plane_normal = plane_transform.rotation_matrix @ np.array([0, 0, 1])
    plane_center = plane_transform.position_array
    line_point0 = np.array([0, 0, 0])
    points = []
    for contour in segmentation.contours:  # type: ignore
        contour: Contour
        for point in contour.points:  # type: ignore
            point: UVKeypoint
            ray = camera_model.projectPixelTo3dRay((point.x, point.y))
            intersection = line_plane_intersection(line_point0, ray, plane_center, plane_normal)
            if intersection is None:
                continue
            points.append(intersection)
    return np.array(points)
