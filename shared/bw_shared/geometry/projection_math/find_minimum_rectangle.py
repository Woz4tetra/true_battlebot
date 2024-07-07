import numpy as np
from scipy.spatial import ConvexHull

from bw_shared.geometry.xy import XY


def find_minimum_rectangle(points: np.ndarray) -> np.ndarray:
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.
    Credit: https://gis.stackexchange.com/questions/22895/finding-minimum-area-rectangle-for-given-points/169633#169633

    :param points: an nx2 matrix of coordinates
    :rval: an nx2 matrix of coordinates
    """

    pi2 = np.pi / 2.0

    # get the convex hull for the points
    hull_points = points[ConvexHull(points).vertices]

    # calculate edge angles
    edges = np.zeros((len(hull_points) - 1, 2))
    edges = hull_points[1:] - hull_points[:-1]

    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi2))
    angles = np.unique(angles)

    # find rotation matrices
    rotations = np.vstack([np.cos(angles), np.cos(angles - pi2), np.cos(angles + pi2), np.cos(angles)]).T
    rotations = rotations.reshape((-1, 2, 2))

    # apply rotations to the hull
    rot_points = np.dot(rotations, hull_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    rotate = rotations[best_idx]

    box_vector = np.array([[x1, y2], [x2, y2], [x2, y1], [x1, y1]])

    # apply rotation dot product row by row
    rval = np.tensordot(box_vector, rotate, axes=(1, 0))

    return rval


def get_rectangle_extents(rectangle: np.ndarray) -> XY:
    """rectangle is a 4x2 matrix of points"""
    width = float(np.linalg.norm(rectangle[0] - rectangle[1]))
    height = float(np.linalg.norm(rectangle[1] - rectangle[2]))
    return XY(width, height)


def get_rectangle_angle(rectangle: np.ndarray) -> float:
    """
    Finds the angle of the rectangle with respect to the x-axis
    rectangle is a 4x2 matrix of points
    """
    angles = []
    for index in range(len(rectangle) - 1):
        p0 = rectangle[index]
        p1 = rectangle[index + 1]
        delta = p1 - p0
        angle = np.arctan2(delta[1], delta[0])
        angles.append(angle)
    angles_array = np.array(angles)
    return angles_array[np.argmin(np.abs(angles_array))]
