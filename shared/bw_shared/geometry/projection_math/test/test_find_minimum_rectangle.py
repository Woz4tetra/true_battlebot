import numpy as np
import pytest
from bw_shared.geometry.projection_math.find_minimum_rectangle import get_rectangle_angle


def rotated_rectangle(points: np.ndarray, rotation: float) -> np.ndarray:
    rotation_2d = np.array(
        [
            [np.cos(rotation), -np.sin(rotation), 0],
            [np.sin(rotation), np.cos(rotation), 0],
            [0, 0, 1],
        ]
    )
    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    padded_points = np.concatenate((points, ones), axis=1)
    return np.tensordot(padded_points, rotation_2d, axes=(1, 1))[:, 0:2]


@pytest.mark.parametrize(
    ("rectangle", "rotation_angle", "expected_angle"),
    [
        (np.array([[0, 0], [1, 0], [1, 1], [0, 1]]), 0.0, 0.0),
        (np.array([[0, 0], [1, 0]]), 0.0, 0.0),
        (np.array([[0, 0], [1, 0], [1, 1], [0, 1]]), 0.1, 0.1),
        (np.array([[0, 0], [1, 0], [1, 1], [0, 1]]), -0.1, -0.1),
        (np.array([[0, 0], [1, 0], [1, 1], [0, 1]]), np.pi, 0.0),
        (np.array([[0, 0], [1, 0], [1, 1], [0, 1]]), np.pi - 0.01, -0.01),
        (np.array([[0, 1], [1, 1], [1, 0], [0, 0]]), 0.1, 0.1),
        (np.array([[0, 0], [0, 1], [1, 1], [1, 0]]), 0.1, 0.1),
        (np.array([[0, 0], [0, 1], [1, 1], [1, 0]]), np.pi / 2, 0.0),
        (
            np.array(
                [
                    [1.124998359491549, -0.6099353447578799],
                    [-1.138044250443917, -0.6148212538319505],
                    [-1.1429457651313446, 1.6554495233589384],
                    [1.1200968448041213, 1.6603354324330089],
                ]
            ),
            0.0,
            0.0021589966804465496,
        ),
        (
            np.array(
                [
                    [0.5065838778349051, 0.740648375512913],
                    [0.4813407615999007, -0.3304424802773759],
                    [-0.6439521651508171, -0.30392194750012824],
                    [-0.6187090489158127, 0.7671689082901607],
                ]
            ),
            0.0,
            -0.02356330839536369,
        ),
    ],
)
def test_get_rectangle_angle(rectangle: np.ndarray, rotation_angle: float, expected_angle: float) -> None:
    if rotation_angle != 0.0:
        rectangle = rotated_rectangle(rectangle, rotation_angle)
    assert pytest.approx(expected_angle, 1e-6) == get_rectangle_angle(rectangle)
