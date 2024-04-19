from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from bw_navigation.selector_algorithms.match_state import MatchState, SelectionResult

from .base_selector import BaseSelector


class CrashSelector(BaseSelector):
    def __init__(self) -> None:
        pass

    def get_target(self, match_state: MatchState) -> SelectionResult:
        target_pose = match_state.opponent_pose
        return SelectionResult(
            goal=PoseStamped(
                header=Header(frame_id=match_state.frame_id),
                pose=target_pose.to_msg(),
            ),
            ignore_opponent_obstacles=True,
        )
