import numpy as np
from bw_interfaces.msg import SegmentationInstance
from image_geometry import PinholeCameraModel

from bw_shared.geometry.array_conversions import array_to_vector3, xyz_to_array
from bw_shared.geometry.plane import Plane


def raycast_segmentation(camera_model: PinholeCameraModel, segmentation: SegmentationInstance) -> np.ndarray:
    rays = []
    for contour in segmentation.contours:
        for point in contour.points:
            ray = np.array(camera_model.projectPixelTo3dRay((point.x, point.y)))
            rays.append(ray)
    return np.array(rays)


def project_segmentation(rays: np.ndarray, plane: Plane) -> np.ndarray:
    points = []
    for ray in rays:
        intersection = plane.ray_intersection(array_to_vector3(ray))
        if intersection is None:
            continue
        points.append(xyz_to_array(intersection))
    return np.array(points)
