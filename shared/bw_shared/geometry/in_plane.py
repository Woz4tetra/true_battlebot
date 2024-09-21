"""
Helper math functions for 2D (in-plane) geometry.
"""

import numpy as np

from bw_shared.geometry.field_bounds import FieldBounds2D


def triangle_area(p_1: np.ndarray, p_2: np.ndarray, p_3: np.ndarray) -> float:
    """
    Calculates the signed area of triangle defined by points 1->2->3->1.
    """
    dx_12, dy_12 = p_1 - p_2
    dx_32, dy_32 = p_3 - p_2

    return (dx_32 * dy_12 - dx_12 * dy_32) / 2


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


def nearest_point_on_line(point: np.ndarray, line: np.ndarray):
    """
    Calculates the point on the infinite line defined by segment closest to the given point.
    """
    (x1, y1), (x2, y2), (x3, y3) = point, line[0], line[1]
    dx, dy = x2 - x1, y2 - y1
    det = dx * dx + dy * dy
    a = (dy * (y3 - y1) + dx * (x3 - x1)) / det
    return x1 + a * dx, y1 + a * dy


def interpolate_on_segment(segment: np.ndarray, t_param: float) -> np.ndarray:
    """
    Interpolates between the start and end of the segment using the parameter t.
    """
    t_compliment = 1 - t_param
    return t_compliment * segment[0] + t_param * segment[1]


def line_line_intersection(reference_line: np.ndarray, other_line: np.ndarray) -> np.ndarray:
    """
    Calculates the point intersection between (infinite lines defined by) ref/other_segment.

    Reference:
    https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
    """

    p_1 = reference_line[0]
    p_2 = reference_line[1]
    p_3 = other_line[0]
    p_4 = other_line[1]

    dx_12, dy_12 = p_1 - p_2
    dx_34, dy_34 = p_3 - p_4
    dx_13, dy_13 = p_1 - p_3

    denom = dx_12 * dy_34 - dy_12 * dx_34
    t_num = dx_13 * dy_34 - dy_13 * dx_34

    if denom == 0.0:
        # Lines are parallel, no solution
        return np.array([np.nan, np.nan])

    t_param = t_num / denom

    return interpolate_on_segment(reference_line, t_param)


def line_bounds_intersection(segment: np.ndarray, bounds: FieldBounds2D) -> list[np.ndarray]:
    """
    Calculates the intersection point between the segment and the field bounds.
    """
    bound_segments = np.array(
        [
            [bounds[0].x, bounds[0].y],
            [bounds[1].x, bounds[0].y],
            [bounds[1].x, bounds[1].y],
            [bounds[0].x, bounds[1].y],
        ]
    )
    intersections = []
    for bound_segment in bound_segments:
        intersection = line_line_intersection(segment, bound_segment)
        if np.any(np.isnan(intersection)):
            continue
        intersections.append(intersection)
    return intersections
