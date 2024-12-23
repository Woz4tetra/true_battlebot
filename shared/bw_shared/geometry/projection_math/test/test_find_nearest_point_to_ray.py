import numpy as np
import pytest
from bw_shared.geometry.projection_math.find_nearest_point_to_ray import find_nearest_point_to_ray


def test_bad_shape() -> None:
    with pytest.raises(ValueError):
        find_nearest_point_to_ray(np.array([1, 2, 3]), np.array([1, 0, 0]))
    with pytest.raises(ValueError):
        find_nearest_point_to_ray(np.array([]), np.array([1, 0, 0]))

    find_nearest_point_to_ray(np.array([[1, 2, 3]]), np.array([1, 0, 0]))
    find_nearest_point_to_ray(np.array([[1], [2], [3]]), np.array([1, 0, 0]))


@pytest.mark.parametrize(
    ("points", "ray", "expected_nearest_point"),
    [
        (np.array([[2, 0, 0]]), np.array([1, 0, 0]), np.array([2, 0, 0])),
        (np.array([[2, 0, 0], [4, 0, 0]]), np.array([1, 0, 0]), np.array([2, 0, 0])),
        (
            np.array(
                [
                    [2, 0, 0],
                    [4, 0, 0],
                    [0, 2, 0],
                    [2, 10, 0],
                    [2, 10, 10],
                ]
            ),
            np.array([1, 0, 0]),
            np.array([2, 0, 0]),
        ),
        (
            np.array(
                [
                    [2, 0, 0],
                    [4, 0, 0],
                    [0, 2, 0],
                    [2, 10, 0],
                    [2, 10, 10],
                ]
            ),
            np.array([0, 1, 0]),
            np.array([0, 2, 0]),
        ),
        (np.array([[2, 0, 0], [0.5, 0.5, 0]]), np.array([1, 1, 0]), np.array([0.5, 0.5, 0])),
    ],
)
def test_find_nearest_point_to_ray(points: np.ndarray, ray: np.ndarray, expected_nearest_point: np.ndarray) -> None:
    assert pytest.approx(find_nearest_point_to_ray(points, ray)) == expected_nearest_point
