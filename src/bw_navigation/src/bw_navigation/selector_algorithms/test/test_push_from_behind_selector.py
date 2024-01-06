import math

import pytest
from bw_interfaces.msg import EstimatedObject
from bw_navigation.selector_algorithms.match_state import MatchState
from bw_navigation.selector_algorithms.push_from_behind_selector import PushFromBehindSelector, PushFromBehindState
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.xy import XY
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry


@pytest.fixture
def push_from_behind_selector() -> PushFromBehindSelector:
    selector = PushFromBehindSelector()
    selector.keep_back_distance = 0.1
    selector.on_target_lateral_threshold = 0.075
    return selector


def pose2d_to_odom(pose: Pose2D) -> Odometry:
    msg = Odometry()
    msg.pose.pose = pose.to_msg()
    return msg


def make_match_state(control: Pose2D, guidance: Pose2D, opponent: Pose2D, size: XY) -> MatchState:
    return MatchState(
        frame_id="map",
        controlled_bot=EstimatedObject(state=pose2d_to_odom(control)),
        guidance_bot=EstimatedObject(state=pose2d_to_odom(guidance)),
        opponent_bot=EstimatedObject(state=pose2d_to_odom(opponent)),
        field=EstimatedObject(size=Vector3(x=size.x, y=size.y, z=0.0)),
    )


@pytest.mark.parametrize(
    ("is_on_target", "in_bounds", "state"),
    [
        (True, False, PushFromBehindState.PUSH),
        (True, True, PushFromBehindState.PUSH),
        (False, True, PushFromBehindState.GET_BEHIND),
        (False, False, PushFromBehindState.GET_NEAR),
    ],
)
def test_get_target(
    push_from_behind_selector: PushFromBehindSelector, is_on_target: bool, in_bounds: bool, state: PushFromBehindState
) -> None:
    def target_override(match_state: MatchState) -> bool:
        return is_on_target

    def in_bounds_override(point: XY, field: EstimatedObject) -> bool:
        return in_bounds

    push_from_behind_selector.is_on_target = target_override
    push_from_behind_selector.is_point_in_bounds = in_bounds_override

    match_state = make_match_state(
        control=Pose2D(10.0, 0.0, 0.0),
        guidance=Pose2D(0.0, 0.0, 2.0),
        opponent=Pose2D(1.0, 0.0, 3.0),
        size=XY(1.0, 1.0),
    )

    push_from_behind_selector.get_target(match_state)
    assert push_from_behind_selector.state == state


def test_is_on_target(push_from_behind_selector: PushFromBehindSelector) -> None:
    assert push_from_behind_selector.is_on_target(
        make_match_state(
            control=Pose2D(10.0, 0.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 2.0),
            opponent=Pose2D(1.0, 0.0, 3.0),
            size=XY(0.0, 0.0),
        )
    )
    assert not push_from_behind_selector.is_on_target(
        make_match_state(
            control=Pose2D(10.0, 0.1, 0.0),
            guidance=Pose2D(0.0, 0.0, 2.0),
            opponent=Pose2D(1.0, 0.0, 3.0),
            size=XY(0.0, 0.0),
        )
    )
    assert push_from_behind_selector.is_on_target(
        make_match_state(
            control=Pose2D(-1.0, -1.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 2.0),
            opponent=Pose2D(1.0, 1.0, 3.0),
            size=XY(0.0, 0.0),
        )
    )


def test_compute_push_target(push_from_behind_selector: PushFromBehindSelector) -> None:
    assert push_from_behind_selector.compute_push_target(
        make_match_state(
            control=Pose2D(10.0, 10.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 0.0),
            opponent=Pose2D(1.0, 0.0, 0.0),
            size=XY(0.0, 0.0),
        )
    ) == pytest.approx(Pose2D(0.1, 0.0, math.pi))


def test_compute_pose_behind_opponent(push_from_behind_selector: PushFromBehindSelector) -> None:
    assert push_from_behind_selector.compute_pose_behind_opponent(
        make_match_state(
            control=Pose2D(10.0, 10.0, 0.0),
            guidance=Pose2D(0.0, 0.0, 0.0),
            opponent=Pose2D(1.0, 0.0, 0.0),
            size=XY(0.0, 0.0),
        )
    ) == pytest.approx(Pose2D(1.1, 0.0, math.pi))


def test_compute_nearest_pose_to_opponent(push_from_behind_selector: PushFromBehindSelector) -> None:
    assert push_from_behind_selector.compute_nearest_pose_to_opponent(
        make_match_state(
            control=Pose2D(0.0, 0.0, 0.0),
            guidance=Pose2D(10.0, 10.0, 0.0),
            opponent=Pose2D(1.0, 0.0, 0.0),
            size=XY(0.0, 0.0),
        )
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
def test_is_point_in_bounds(
    push_from_behind_selector: PushFromBehindSelector, point: XY, size: XY, expected: bool
) -> None:
    field = EstimatedObject(size=Vector3(x=size.x, y=size.y, z=0.0))
    assert push_from_behind_selector.is_point_in_bounds(point, field) == expected


def test_interpolate(push_from_behind_selector: PushFromBehindSelector) -> None:
    point1 = XY(0.0, 0.0)
    point2 = XY(1.0, 1.0)
    assert push_from_behind_selector.interpolate(point1, point2, 0.5) == pytest.approx(XY(0.5, 0.5))
    assert push_from_behind_selector.interpolate(point1, point2, 0.0) == pytest.approx(XY(0.0, 0.0))
    assert push_from_behind_selector.interpolate(point1, point2, 1.0) == pytest.approx(XY(1.0, 1.0))
