import numpy as np
import pytest
from bw_interfaces.msg import Contour, UVKeypoint
from perception_tools.inference.common import contour_to_msg, msg_to_mask


@pytest.mark.parametrize(
    ("input_contours", "expected_msg"),
    (
        (
            [np.array([[0, 0], [0, 1], [1, 1], [1, 0]])],
            [
                Contour(
                    points=[UVKeypoint(x=0, y=0), UVKeypoint(x=0, y=1), UVKeypoint(x=1, y=1), UVKeypoint(x=1, y=0)],
                    area=1.0,
                )
            ],
        ),
        (
            [np.array([[0, 0], [0, 1], [1, 1], [1, 0]]), np.array([[0, 0], [0, 1], [1, 1], [1, 0]])],
            [
                Contour(
                    points=[UVKeypoint(x=0, y=0), UVKeypoint(x=0, y=1), UVKeypoint(x=1, y=1), UVKeypoint(x=1, y=0)],
                    area=1.0,
                ),
                Contour(
                    points=[UVKeypoint(x=0, y=0), UVKeypoint(x=0, y=1), UVKeypoint(x=1, y=1), UVKeypoint(x=1, y=0)],
                    area=1.0,
                ),
            ],
        ),
    ),
)
def test_contour_to_msg(input_contours: list[np.ndarray], expected_msg: list[Contour]) -> None:
    assert contour_to_msg(input_contours) == expected_msg


@pytest.mark.parametrize(
    ("contour_msgs", "width", "height", "expected_mask"),
    (
        (
            [
                Contour(
                    points=[UVKeypoint(x=0, y=0), UVKeypoint(x=0, y=1), UVKeypoint(x=1, y=1), UVKeypoint(x=1, y=0)],
                    area=1.0,
                )
            ],
            2,
            2,
            np.array(
                [
                    [1, 1],
                    [1, 1],
                ]
            ),
        ),
        (
            [
                Contour(
                    points=[UVKeypoint(x=0, y=0), UVKeypoint(x=0, y=1), UVKeypoint(x=1, y=1), UVKeypoint(x=1, y=0)],
                    area=1.0,
                ),
                Contour(
                    points=[UVKeypoint(x=2, y=2), UVKeypoint(x=2, y=3), UVKeypoint(x=3, y=3), UVKeypoint(x=3, y=2)],
                    area=1.0,
                ),
            ],
            4,
            4,
            np.array(
                [
                    [1, 1, 0, 0],
                    [1, 1, 0, 0],
                    [0, 0, 1, 1],
                    [0, 0, 1, 1],
                ]
            ),
        ),
    ),
)
def test_msg_to_mask(contour_msgs: list[Contour], width: int, height: int, expected_mask: np.ndarray) -> None:
    assert np.array_equal(msg_to_mask(contour_msgs, width, height), expected_mask)
