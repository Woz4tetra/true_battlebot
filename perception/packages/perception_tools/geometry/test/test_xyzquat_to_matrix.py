import numpy as np
import pytest
from perception_tools.geometry.xyzquat_to_matrix import xyzquat_to_matrix


@pytest.mark.parametrize(
    ("x", "y", "z", "qx", "qy", "qz", "qw", "expected_tf"),
    [
        (
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        ),
        (
            1.0,
            2.0,
            3.0,
            0.0,
            0.0,
            0.0,
            1.0,
            np.array(
                [
                    [1.0, 0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [0.0, 0.0, 1.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        ),
        (
            3.0,
            1.0,
            2.0,
            # 45 roll, 45 pitch, 45 yaw
            0.191342,
            0.46194,
            0.191342,
            0.844623,
            np.array(
                [
                    [1 / 2, (-1 / 2 + np.sqrt(2) / 4), (1 / 2 + np.sqrt(2) / 4), 3.0],
                    [1 / 2, (1 / 2 + np.sqrt(2) / 4), (-1 / 2 + np.sqrt(2) / 4), 1.0],
                    [-np.sqrt(2) / 2, 1 / 2, 1 / 2, 2.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        ),
    ],
)
def test_xyzquat_to_matrix(
    x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float, expected_tf: np.ndarray
) -> None:
    tf = xyzquat_to_matrix(x, y, z, qx, qy, qz, qw)
    print(tf)
    print(expected_tf)
    assert np.allclose(tf, expected_tf)
