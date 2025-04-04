import math
from typing import Optional

from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import Twist

from bw_navigation.planners.engines.config.recover_from_danger_config import RecoverFromDangerConfig
from bw_navigation.planners.shared.match_state import MatchState


class RecoverFromDangerEngine:
    def __init__(self, config: RecoverFromDangerConfig) -> None:
        self.config = config

    def compute_recovery_command(self, match_state: MatchState) -> Optional[Twist]:
        if not self.is_controlled_robot_in_danger(match_state):
            return None
        target_relative_to_controlled_bot = match_state.goal_pose.relative_to(match_state.controlled_robot_pose)
        twist = Twist()
        twist.linear.x = self.config.linear_magnitude
        twist.angular.z = math.copysign(
            self.config.angular_magnitude,
            -1 * target_relative_to_controlled_bot.y,
        )
        return twist

    def is_controlled_robot_in_danger(self, match_state: MatchState) -> bool:
        for opponent_state in match_state.opponent_robot_states:
            opponent_pose = Pose2D.from_msg(opponent_state.pose.pose)
            controlled_bot_relative_to_opponent = match_state.controlled_robot_pose.relative_to(opponent_pose)
            opponent_width = max(opponent_state.size.x, opponent_state.size.y)
            combined_width = match_state.controlled_robot_width + opponent_width
            magnitude_lower_bound = combined_width * self.config.size_multiplier
            if (
                abs(controlled_bot_relative_to_opponent.heading()) < self.config.angle_tolerance
                and (
                    magnitude_lower_bound
                    < controlled_bot_relative_to_opponent.magnitude()
                    < self.config.linear_tolerance
                )
                and opponent_state.twist.twist.linear.x > self.config.velocity_threshold
            ):
                return True
        return False
