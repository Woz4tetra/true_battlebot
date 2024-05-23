from enum import Enum, auto

import rospy
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from bw_navigation.selector_algorithms.algorithm_helpers import (
    compute_nearest_pose_to_opponent,
    compute_pose_behind_opponent,
    compute_push_target,
    is_on_target,
    is_point_in_bounds,
)
from bw_navigation.selector_algorithms.match_state import MatchState, SelectionResult

from .base_selector import BaseSelector


class PushFromBehindState(Enum):
    IDLE = auto()
    PUSH = auto()
    GET_BEHIND = auto()
    GET_NEAR = auto()


class PushFromBehindSelector(BaseSelector):
    """
    If the controlled robot is not in line with the guidance bot and opponent, select goal that's behind the
    opponent pointing towards the guidance bot.
    Otherwise, if the controlled bot is in position, select goal that's just in front the guidance bot
    (push the opponent to the guidance bot).
    """

    def __init__(self) -> None:
        self.keep_back_buffer = 0.05
        self.on_target_lateral_threshold = 0.1
        self.border_buffer = 0.1
        self.state = PushFromBehindState.IDLE

    def is_on_target(self, match_state: MatchState) -> bool:
        return is_on_target(match_state, self.on_target_lateral_threshold)

    def is_point_in_bounds(self, pose_behind_opponent: Pose2D, match_state: MatchState) -> bool:
        return is_point_in_bounds(
            pose_behind_opponent.to_point(), match_state.controlled_bot, match_state.field, self.border_buffer
        )

    def get_target(self, match_state: MatchState) -> SelectionResult:
        """
        Get the target pose for the controlled bot.
        If the controlled bot is in line with the guidance bot and opponent, select goal that's just in front the
        guidance bot (push the opponent to the guidance bot).
        Otherwise, select a goal that's behind the opponent pointing towards the guidance bot.
        If the goal is outside the field, select a goal close to the opponent that is still inside the field.
        """
        if self.is_on_target(match_state):
            push_target = compute_push_target(match_state, self.keep_back_buffer)
            self.set_state(PushFromBehindState.PUSH)
            return SelectionResult(
                goal=PoseStamped(
                    header=Header(frame_id=match_state.frame_id),
                    pose=push_target.to_msg(),
                ),
                ignore_opponent_obstacles=True,
            )
        pose_behind_opponent = compute_pose_behind_opponent(match_state, self.keep_back_buffer)
        if self.is_point_in_bounds(pose_behind_opponent, match_state):
            self.set_state(PushFromBehindState.GET_BEHIND)
            return SelectionResult(
                goal=PoseStamped(
                    header=Header(frame_id=match_state.frame_id),
                    pose=pose_behind_opponent.to_msg(),
                ),
                ignore_opponent_obstacles=False,
            )

        nearest_pose_to_opponent = compute_nearest_pose_to_opponent(match_state, self.keep_back_buffer)
        self.set_state(PushFromBehindState.GET_NEAR)
        return SelectionResult(
            goal=PoseStamped(
                header=Header(frame_id=match_state.frame_id),
                pose=nearest_pose_to_opponent.to_msg(),
            ),
            ignore_opponent_obstacles=False,
        )

    def set_state(self, state: PushFromBehindState) -> None:
        if self.state != state:
            self.state = state
            rospy.logdebug(f"PushFromBehindSelector state changed to {state}")
