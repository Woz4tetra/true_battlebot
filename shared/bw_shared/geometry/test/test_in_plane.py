from typing import Optional

import numpy as np
import pytest
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.in_plane import (
    dist_from_point_to_line,
    interpolate_on_segment,
    line_bounds_intersection,
    line_line_intersection,
    nearest_point_on_line,
    triangle_area,
)
from bw_shared.geometry.xy import XY


@pytest.mark.parametrize(
    ("triangle", "area"),
    (
        (np.array(((0.0, 1.0), (-1.0, 0.0), (1.0, 0.0))), 1.0),
        (np.array(((0.0, 1.0), (1.0, 0.0), (-1.0, 0.0))), 1.0),
    ),
)
def test_triangle_area(triangle: np.ndarray, area: float) -> None:
    assert triangle_area(*triangle) == area


@pytest.mark.parametrize(
    ("point", "line", "expected_distance"),
    [
        (np.array([0.0, 1.0]), np.array([(-1.0, 0.0), (1.0, 0.0)]), 1.0),
        (np.array([0.0, 1.0]), np.array([(1.0, 0.0), (-1.0, 0.0)]), 1.0),
        (np.array([0.0, 0.0]), np.array([(-1.0, -1.0), (1.0, 1.0)]), 0.0),
        (np.array([10.0, 1.0]), np.array([(-1.0, 0.0), (1.0, 0.0)]), 1.0),
    ],
)
def test_dist_from_point_to_line(point: np.ndarray, line: np.ndarray, expected_distance: float) -> None:
    assert np.isclose(dist_from_point_to_line(point, line), expected_distance)


@pytest.mark.parametrize(
    ("point", "line", "expected_point"),
    [
        (np.array([0.0, 1.0]), np.array([(-1.0, 0.0), (1.0, 0.0)]), np.array([0.0, 0.0])),
        (np.array([-1.0, 1.0]), np.array([(-1.0, -1.0), (1.0, 1.0)]), np.array([0.0, 0.0])),
        (np.array([10.0, 1.0]), np.array([(-1.0, 0.0), (1.0, 0.0)]), np.array([10.0, 0.0])),
        (np.array([0.5, -1.0]), np.array([(-2.0, 1.0), (2.0, 2.0)]), np.array([-1.75, 1.25])),
    ],
)
def test_nearest_point_on_line(point: np.ndarray, line: np.ndarray, expected_point: np.ndarray) -> None:
    assert np.all(nearest_point_on_line(point, line) == expected_point)


@pytest.mark.parametrize(
    ("segment", "t_param", "expected_point"),
    [
        (np.array([[0.0, 0.0], [1.0, 0.0]]), 0.0, np.array([0.0, 0.0])),
        (np.array([[0.0, 0.0], [1.0, 0.0]]), 0.5, np.array([0.5, 0.0])),
        (np.array([[0.0, 0.0], [1.0, 0.0]]), 1.0, np.array([1.0, 0.0])),
        (np.array([[0.0, 0.0], [1.0, 1.0]]), 0.5, np.array([0.5, 0.5])),
    ],
)
def test_interpolate_on_segment(segment: np.ndarray, t_param: float, expected_point: np.ndarray) -> None:
    assert np.all(interpolate_on_segment(segment, t_param) == expected_point)


@pytest.mark.parametrize(
    ("line1", "line2", "expected_intersection"),
    [
        (np.array([[0.0, 0.0], [1.0, 0.0]]), np.array([[0.5, -0.5], [0.5, 0.5]]), np.array([0.5, 0.0])),
        (np.array([[0.0, 0.0], [1.0, 0.0]]), np.array([[0.5, -0.5], [0.5, -0.1]]), np.array([0.5, 0.0])),
        (np.array([[0.0, 0.0], [1.0, 0.0]]), np.array([[0.0, 0.0], [1.0, 0.0]]), None),
        (np.array([[0.0, 0.0], [1.0, 0.0]]), np.array([[0.0, 1.0], [1.0, 1.0]]), None),
        (np.array([[0.0, 0.0], [1.0, 0.0]]), np.array([[0.5, -0.5], [0.5, 0.0]]), np.array([0.5, 0.0])),
        (np.array([[0.0, 0.0], [1.0, 0.0]]), np.array([[0.5, -0.5], [0.5, 0.1]]), np.array([0.5, 0.0])),
        (np.array([[-1.0, -1.0], [1.0, 1.0]]), np.array([[1.0, -1.0], [-1.0, 1.0]]), np.array([0.0, 0.0])),
        (np.array([[0.0, 2.0], [2.0, 2.0]]), np.array([[1.0, -1.0], [1.0, 1.0]]), np.array([1.0, 2.0])),
    ],
)
def test_line_line_intersection(
    line1: np.ndarray, line2: np.ndarray, expected_intersection: Optional[np.ndarray]
) -> None:
    intersection = line_line_intersection(line1, line2)
    if expected_intersection is None:
        assert intersection is None
    else:
        assert np.all(intersection == expected_intersection)


@pytest.mark.parametrize(
    ("segment", "bounds", "expected_intersection"),
    (
        (np.array([[0.0, 0.0], [2.0, 0.0]]), (XY(-1.0, -1.0), XY(1.0, 1.0)), np.array([1.0, 0.0])),
        (np.array([[0.0, 0.0], [0.01, 0.0]]), (XY(-1.0, -1.0), XY(1.0, 1.0)), None),
        (np.array([[0.0, 0.0], [-2.0, 0.0]]), (XY(-1.0, -1.0), XY(1.0, 1.0)), np.array([-1.0, 0.0])),
        (np.array([[0.0, 2.0], [2.0, 2.0]]), (XY(-1.0, -1.0), XY(1.0, 1.0)), None),
    ),
)
def test_line_bounds_intersection(
    segment: np.ndarray, bounds: FieldBounds2D, expected_intersection: Optional[np.ndarray]
) -> None:
    intersections = line_bounds_intersection(segment, bounds)
    if expected_intersection is None:
        assert intersections == []
    else:
        assert len(intersections) == 1
        assert np.all(intersections[0] == expected_intersection)
