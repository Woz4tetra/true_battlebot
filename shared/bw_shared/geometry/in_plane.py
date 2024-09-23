"""
Helper math functions for 2D (in-plane) geometry.
"""

from typing import Optional

import numpy as np

from bw_shared.geometry.field_bounds import FieldBounds2D


def triangle_area(p_1: np.ndarray, p_2: np.ndarray, p_3: np.ndarray) -> float:
    """
    Calculates the area of triangle defined by points 1->2->3->1.
    """
    dx_12, dy_12 = p_1 - p_2
    dx_32, dy_32 = p_3 - p_2

    return abs(dx_32 * dy_12 - dx_12 * dy_32) / 2


def dist_from_point_to_line(point: np.ndarray, line: np.ndarray) -> float:
    """
    Calculates the perpendicular distance from point to an infinite line defined by segment. Ref:
    https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points

    If segment has zero length, return distance between point and degenerate line segment
    """

    seg_len = float(np.linalg.norm(line[0] - line[1]))
    if seg_len == 0.0:
        return float(np.linalg.norm(point - line[0]))

    area = abs(triangle_area(point, line[0], line[1]))
    return 2 * area / seg_len


def nearest_point_on_line(point: np.ndarray, line: np.ndarray) -> np.ndarray:
    """
    Calculates the point on an infinite line closest to the given point.

    Parameters:
    point (np.ndarray): The point from which the nearest point on the line is calculated.
    line (np.ndarray): A 2xN array where the first row is a point on the line and the second row is the direction
        vector of the line.

    Returns:
        np.ndarray: The nearest point on the line.
    """
    # Extract a point on the line and the direction vector of the line
    line_point = line[0]
    line_dir = line[1]

    # Calculate the vector from the line point to the given point
    point_vector = point - line_point

    # Project the point_vector onto the line direction vector
    projection_length = np.dot(point_vector, line_dir) / np.dot(line_dir, line_dir)
    projection_vector = projection_length * line_dir

    # Calculate the nearest point on the line
    nearest_point = line_point + projection_vector

    return nearest_point


def interpolate_on_segment(segment: np.ndarray, t_param: float) -> np.ndarray:
    """
    Interpolates between the start and end of the segment using the parameter t.
    """
    t_compliment = 1 - t_param
    return t_compliment * segment[0] + t_param * segment[1]


def line_line_intersection_parameterized(
    reference_line: np.ndarray, other_line: np.ndarray
) -> Optional[tuple[float, float]]:
    """
    Calculates the point intersection between (infinite lines defined by) ref/other_segment.

    Reference:
    https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment

    Args:
        reference_line: 2x2 array where the first row is the first point on the reference line and the second row is the
            second point on the reference line.
        other_line: 2x2 array where the first row is the first point on the other line and the second row is the
            second point on the other line.

    Returns:
        float: parameterization t where t = 0,1 corresponds to
            reference_line[0], reference_line[1]
    """

    p_1 = reference_line[0]
    p_2 = reference_line[1]
    p_3 = other_line[0]
    p_4 = other_line[1]

    dx_12, dy_12 = p_1 - p_2
    dx_34, dy_34 = p_3 - p_4
    dx_13, dy_13 = p_1 - p_3

    denom = dx_12 * dy_34 - dy_12 * dx_34

    if denom == 0.0:
        # Lines are parallel, no solution
        return None

    t_num = dx_13 * dy_34 - dy_13 * dx_34
    u_num = -1 * (dx_12 * dy_13 - dy_12 * dx_13)

    return t_num / denom, u_num / denom


def line_line_intersection(reference_line: np.ndarray, other_line: np.ndarray) -> Optional[np.ndarray]:
    """
    Calculates the point intersection between (infinite lines defined by) ref/other_segment.

    Uses line_line_intersection_parameterized and interpolate_on_segment.
    """
    params = line_line_intersection_parameterized(reference_line, other_line)
    if params is None:
        return None
    return interpolate_on_segment(reference_line, params[0])


def line_bounds_intersection(segment: np.ndarray, bounds: FieldBounds2D) -> list[np.ndarray]:
    """
    Calculates the intersection point between the segment and the field bounds.
    """
    bound_segments = [
        np.array([[bounds[0].x, bounds[0].y], [bounds[1].x, bounds[0].y]]),
        np.array([[bounds[1].x, bounds[0].y], [bounds[1].x, bounds[1].y]]),
        np.array([[bounds[1].x, bounds[1].y], [bounds[0].x, bounds[1].y]]),
        np.array([[bounds[0].x, bounds[1].y], [bounds[0].x, bounds[0].y]]),
    ]
    intersections = []
    for bound_segment in bound_segments:
        params = line_line_intersection_parameterized(segment, bound_segment)
        if params is None:
            continue
        t_param, u_param = params
        if 0 <= t_param <= 1 and 0 <= u_param <= 1:
            intersection = interpolate_on_segment(segment, t_param)
            intersections.append(intersection)
    return intersections
