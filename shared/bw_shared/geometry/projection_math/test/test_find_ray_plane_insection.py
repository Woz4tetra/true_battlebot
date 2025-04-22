import numpy as np
import pytest
from bw_shared.geometry.projection_math.find_ray_plane_intersection import find_ray_plane_intersection


@pytest.mark.parametrize(
    ("ray", "plane_normal", "plane_point", "expected_intersection"),
    [
        (np.array([1, 0, 0]), np.array([-1, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])),
        (np.array([1, 0, 0]), np.array([-1, 0, 0]), np.array([1, 0, 0]), np.array([-1, 0, 0])),
        (np.array([1, 0, 0]), np.array([-1, 0, 0]), np.array([0, 0, 1]), np.array([0, 0, 0])),
    ],
)
def test_find_ray_plane_insection(
    ray: np.ndarray, plane_normal: np.ndarray, plane_point: np.ndarray, expected_intersection: np.ndarray
) -> None:
    result = find_ray_plane_intersection(ray, plane_normal, plane_point)
    print(result, expected_intersection)
    assert pytest.approx(result) == expected_intersection
