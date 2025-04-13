from enum import Enum, auto

from bw_shared.geometry.pose2d import Pose2D
from visualization_msgs.msg import Marker

from bw_navigation.planners.engines.config.main_bot_intent_config import MainBotIntentConfig
from bw_navigation.planners.shared.marker_utils import make_text_marker
from bw_navigation.planners.shared.match_state import MatchState


class MainBotIntentState(Enum):
    ATTACKING = auto()
    EVADING = auto()


class MainBotIntentEngine:
    def __init__(self, config: MainBotIntentConfig) -> None:
        self.config = config
        self.markers: list[Marker] = []

    def get_markers(self) -> list[Marker]:
        return self.markers

    def compute_intent(self, match_state: MatchState) -> MainBotIntentState:
        attack_fan_angle = self.config.attack_fan_angle_rad
        friendly_robot_speed = abs(match_state.friendly_robot.twist.twist.linear.x)
        self.markers = []
        for other_state in match_state.opponent_robot_states:
            opponent_pose = Pose2D.from_msg(other_state.pose.pose)
            controlled_bot_relative_to_opponent = opponent_pose.relative_to(match_state.friendly_robot_pose)
            fan_angle = abs(controlled_bot_relative_to_opponent.heading())
            is_in_attach_range = (
                fan_angle / 2 < attack_fan_angle and friendly_robot_speed > self.config.velocity_threshold
            )
            if is_in_attach_range:
                self.markers.append(
                    make_text_marker(
                        "ATTACKING",
                        match_state.friendly_robot.pose.pose,
                        ns="main_bot_intent",
                        marker_id=0,
                        color=(1.0, 0.2, 0.1),
                    )
                )
                return MainBotIntentState.ATTACKING
        self.markers.append(
            make_text_marker(
                "EVADING",
                match_state.friendly_robot.pose.pose,
                ns="main_bot_intent",
                marker_id=0,
                color=(1.0, 0.2, 0.1),
            )
        )
        return MainBotIntentState.EVADING
