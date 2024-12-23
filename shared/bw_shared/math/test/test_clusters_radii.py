import numpy as np
import pytest
from bw_shared.math.clusters_radii import clusters_radii


@pytest.mark.parametrize(
    ("test_input", "expected"),
    [
        (np.array([[1, 1], [1 + np.sqrt(2), 1 + np.sqrt(2)]], dtype=np.float64), [2, 2]),
        (np.array([[1, 0], [2, 0], [2, 1], [2, 2]], dtype=np.float64), [1, 1, 1, 1]),
        (np.array([[1, 0], [2, 0], [4, 0], [2, 1], [2, 2]], dtype=np.float64), [1, 1, 2, 1, 1]),
        (np.array([[0, 0], [4, 0], [10, 0]], dtype=np.float64), [4, 4, 6]),
        (np.array([[0, 0], [7, 0], [10, 0]], dtype=np.float64), [7, 3, 3]),
        (np.array([[0, 0, 0], [7, 0, 0], [10, 0, 0]], dtype=np.float64), [7, 3, 3]),
        (np.array([[0, 0, 0, 0], [7, 0, 0, 0], [10, 0, 0, 0]], dtype=np.float64), [7, 3, 3]),
        (
            np.array(
                [[1, 1, 1], [1 + np.sqrt(4 / 3), 1 + np.sqrt(4 / 3), 1 + np.sqrt(4 / 3)]],
                dtype=np.float64,
            ),
            [2, 2],
        ),
    ],
)
def test_min_distances_in_tensor(test_input: np.ndarray, expected: np.ndarray) -> None:
    kd_distances = clusters_radii(test_input)
    assert np.allclose(kd_distances, expected)
