import numpy as np
import pytest
from perception_tools.messages.point_cloud import PointCloud


@pytest.mark.parametrize(
    ("point_cloud", "input_mask", "filtered_points"),
    (
        (
            PointCloud(
                points=np.array(
                    [
                        [1.0, 2.0, 3.0],
                        [4.0, 5.0, 6.0],
                        [7.0, 8.0, 9.0],
                    ]
                ),
            ),
            np.array([True, False, True]),
            np.array(
                [
                    [1.0, 2.0, 3.0],
                    [7.0, 8.0, 9.0],
                ]
            ),
        ),
        (
            PointCloud(
                points=np.array(
                    [
                        [1.0, 2.0, 3.0],
                        [4.0, 5.0, 6.0],
                        [7.0, 8.0, 9.0],
                    ]
                ),
            ),
            np.array([False, False, False]),
            np.zeros((0, 3)),
        ),
    ),
)
def test_filtered_points(point_cloud: PointCloud, input_mask: np.ndarray, filtered_points: np.ndarray) -> None:
    result = point_cloud.filtered_points(input_mask)
    assert np.allclose(result, filtered_points), result
