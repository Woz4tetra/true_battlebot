import pytest
from bw_navigation.selector_algorithms.push_from_behind_selector import PushFromBehindSelector, PushFromBehindState
from bw_navigation.selector_algorithms.test.helpers import make_match_state
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.xy import XY


@pytest.fixture
def push_from_behind_selector() -> PushFromBehindSelector:
    selector = PushFromBehindSelector()
    selector.keep_back_buffer = 0.1
    selector.on_target_lateral_threshold = 0.075
    return selector


@pytest.mark.parametrize(
    ("is_on_target", "in_bounds", "state", "ignore_opponents"),
    [
        (True, False, PushFromBehindState.PUSH, True),
        (True, True, PushFromBehindState.PUSH, True),
        (False, True, PushFromBehindState.GET_BEHIND, False),
        (False, False, PushFromBehindState.GET_NEAR, False),
    ],
)
def test_get_target(
    push_from_behind_selector: PushFromBehindSelector,
    is_on_target: bool,
    in_bounds: bool,
    state: PushFromBehindState,
    ignore_opponents: bool,
) -> None:
    push_from_behind_selector.is_on_target = lambda match_state: is_on_target  # type: ignore
    push_from_behind_selector.is_point_in_bounds = lambda point, field: in_bounds  # type: ignore

    match_state = make_match_state(
        control=Pose2D(10.0, 0.0, 0.0),
        guidance=Pose2D(0.0, 0.0, 2.0),
        opponent=Pose2D(1.0, 0.0, 3.0),
        size=XY(1.0, 1.0),
    )

    result = push_from_behind_selector.get_target(match_state)
    assert push_from_behind_selector.state == state
    assert result.ignore_opponent_obstacles == ignore_opponents
