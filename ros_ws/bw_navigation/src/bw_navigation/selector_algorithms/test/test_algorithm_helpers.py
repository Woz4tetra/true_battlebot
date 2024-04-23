import math

import pytest
from bw_interfaces.msg import EstimatedObject
from bw_navigation.selector_algorithms.algorithm_helpers import (
    compute_nearest_pose_to_opponent,
    compute_pose_behind_opponent,
    compute_push_target,
    interpolate,
    is_behind_opponent,
    is_on_target,
    is_point_in_bounds,
)
from bw_navigation.selector_algorithms.test.helpers import make_match_state
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.xy import XY
from geometry_msgs.msg import Vector3


def test_is_on_target() -> None:
    on_target_lateral_threshold = 0.1
    assert is_on_target(
        make_match_state(
            control=Pose2D(10.0, 0.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 2.0),
            opponent=Pose2D(1.0, 0.0, 3.0),
            size=XY(0.0, 0.0),
        ),
        on_target_lateral_threshold=on_target_lateral_threshold,
    )
    assert not is_on_target(
        make_match_state(
            control=Pose2D(10.0, 0.1, 0.0),
            guidance=Pose2D(0.0, 0.0, 2.0),
            opponent=Pose2D(1.0, 0.0, 3.0),
            size=XY(0.0, 0.0),
        ),
        on_target_lateral_threshold=0.1,
    )
    assert is_on_target(
        make_match_state(
            control=Pose2D(-1.0, -1.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 2.0),
            opponent=Pose2D(1.0, 1.0, 3.0),
            size=XY(0.0, 0.0),
        ),
        on_target_lateral_threshold=0.1,
    )


def test_compute_push_target() -> None:
    keep_back_buffer = 0.1
    assert compute_push_target(
        make_match_state(
            control=Pose2D(10.0, 10.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 0.0),
            opponent=Pose2D(1.0, 0.0, 0.0),
            size=XY(0.0, 0.0),
        ),
        keep_back_buffer=keep_back_buffer,
    ) == pytest.approx(Pose2D(0.1, 0.0, math.pi))


def test_compute_pose_behind_opponent() -> None:
    keep_back_buffer = 0.1
    assert compute_pose_behind_opponent(
        make_match_state(
            control=Pose2D(10.0, 10.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 0.0),
            opponent=Pose2D(1.0, 0.0, 0.0),
            size=XY(0.0, 0.0),
        ),
        keep_back_buffer=keep_back_buffer,
    ) == pytest.approx(Pose2D(1.1, 0.0, math.pi))


def test_compute_nearest_pose_to_opponent() -> None:
    keep_back_buffer = 0.1
    assert compute_nearest_pose_to_opponent(
        make_match_state(
            control=Pose2D(0.0, 0.0, 0.0),
            guidance=Pose2D(10.0, 10.0, 0.0),
            opponent=Pose2D(1.0, 0.0, 0.0),
            size=XY(0.0, 0.0),
        ),
        keep_back_buffer=keep_back_buffer,
    ) == pytest.approx(Pose2D(0.9, 0.0, 0.0))


@pytest.mark.parametrize(
    ("point", "size", "expected"),
    [
        (XY(0.0, 0.0), XY(1.0, 1.0), True),
        (XY(0.0, 0.0), XY(0.5, 0.5), True),
        (XY(1.0, 1.0), XY(1.0, 1.0), False),
        (XY(1.0, 0.0), XY(1.0, 1.0), False),
        (XY(0.5, 0.5), XY(1.0, 1.0), True),
        (XY(-0.5, -0.5), XY(1.0, 1.0), True),
        (XY(-0.5, -0.5001), XY(1.0, 1.0), False),
    ],
)
def test_is_point_in_bounds(point: XY, size: XY, expected: bool) -> None:
    field = EstimatedObject(size=Vector3(x=size.x, y=size.y, z=0.0))
    robot = EstimatedObject()
    border_buffer = 0.0
    assert is_point_in_bounds(point, robot, field, border_buffer=border_buffer) == expected


def test_interpolate() -> None:
    point1 = XY(0.0, 0.0)
    point2 = XY(1.0, 1.0)
    assert interpolate(point1, point2, 0.5) == pytest.approx(XY(0.5, 0.5))
    assert interpolate(point1, point2, 0.0) == pytest.approx(XY(0.0, 0.0))
    assert interpolate(point1, point2, 1.0) == pytest.approx(XY(1.0, 1.0))


def test_is_behind_opponent() -> None:
    assert is_behind_opponent(
        make_match_state(
            control=Pose2D(10.0, 0.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 2.0),
            opponent=Pose2D(1.0, 0.0, 3.0),
            size=XY(1.0, 1.0),
        )
    )

    assert not is_behind_opponent(
        make_match_state(
            control=Pose2D(0.0, 0.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 2.0),
            opponent=Pose2D(1.0, 0.0, 3.0),
            size=XY(1.0, 1.0),
        )
    )

    assert is_behind_opponent(
        make_match_state(
            control=Pose2D(-0.1, 0.0, 0.0),
            guidance=Pose2D(1.0, 0.0, 2.0),
            opponent=Pose2D(0.0, 0.0, 3.0),
            size=XY(1.0, 1.0),
        )
    )

    assert not is_behind_opponent(
        make_match_state(
            control=Pose2D(0.1, 0.0, 0.0),
            guidance=Pose2D(1.0, 0.0, 2.0),
            opponent=Pose2D(0.0, 0.0, 3.0),
            size=XY(1.0, 1.0),
        )
    )
