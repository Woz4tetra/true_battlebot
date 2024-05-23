from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import PoseStamped

from bw_navigation.selector_algorithms.match_state import MatchState, SelectionResult

from .base_selector import BaseSelector


class SacrificialSelector(BaseSelector):
    """
    Put the mini bot between the guidance bot and opponent bot.
    """

    def __init__(self) -> None:
        pass

    def get_target(self, match_state: MatchState) -> SelectionResult:
        control_pose = Pose2D.from_msg(match_state.guidance_bot.pose.pose)
        opponent_pose = Pose2D.from_msg(match_state.opponent_bot.pose.pose)

        mid_pose = Pose2D(
            (opponent_pose.x + control_pose.x) / 2,
            (opponent_pose.y + control_pose.y) / 2,
            control_pose.heading(opponent_pose),
        )

        midpoint = PoseStamped()
        midpoint.header.frame_id = match_state.frame_id
        midpoint.pose = mid_pose.to_msg()

        return SelectionResult(
            goal=midpoint,
            ignore_opponent_obstacles=False,
        )
