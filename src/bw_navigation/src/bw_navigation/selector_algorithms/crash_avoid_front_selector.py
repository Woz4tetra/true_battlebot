from bw_tools.structs.pose2d import Pose2D
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from bw_navigation.selector_algorithms.algorithm_helpers import (
    compute_pose_behind_opponent,
    is_behind_opponent,
    is_point_in_bounds,
)
from bw_navigation.selector_algorithms.match_state import MatchState, SelectionResult

from .base_selector import BaseSelector


class CrashAvoidFrontSelector(BaseSelector):
    def __init__(self) -> None:
        self.keep_back_buffer = 0.05
        self.border_buffer = 0.1

    def is_point_in_bounds(self, pose_behind_opponent: Pose2D, match_state: MatchState) -> bool:
        return is_point_in_bounds(
            pose_behind_opponent.to_point(), match_state.controlled_bot, match_state.field, self.border_buffer
        )

    def get_target(self, match_state: MatchState) -> SelectionResult:
        behind_opponent = is_behind_opponent(match_state)
        target_pose = match_state.opponent_pose
        if not behind_opponent:
            pose_behind_opponent = compute_pose_behind_opponent(match_state, self.keep_back_buffer)
            is_in_bounds = self.is_point_in_bounds(pose_behind_opponent, match_state)
            if is_in_bounds:
                target_pose = pose_behind_opponent

        return SelectionResult(
            goal=PoseStamped(
                header=Header(frame_id=match_state.frame_id),
                pose=target_pose.to_msg(),
            ),
            ignore_opponent_obstacles=behind_opponent,
        )
