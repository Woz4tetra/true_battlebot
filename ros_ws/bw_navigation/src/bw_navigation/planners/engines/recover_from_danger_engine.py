import math
from enum import Enum, auto
from typing import Optional

from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker

from bw_navigation.planners.engines.config.recover_from_danger_config import RecoverFromDangerConfig
from bw_navigation.planners.shared.match_state import MatchState


class DangerState(Enum):
    SAFE = auto()
    LEFT_DANGER = auto()
    RIGHT_DANGER = auto()


class RecoverFromDangerEngine:
    def __init__(self, config: RecoverFromDangerConfig) -> None:
        self.config = config
        self.markers: list[Marker] = []

    def compute_recovery_command(self, match_state: MatchState) -> Optional[Twist]:
        self.markers.clear()
        danger_state = self.in_danger_state(match_state)
        if danger_state == DangerState.SAFE:
            return None
        twist = Twist()
        twist.linear.x = self.config.linear_magnitude
        twist.angular.z = self.config.angular_magnitude * (1 if danger_state == DangerState.LEFT_DANGER else -1)
        return twist

    def in_danger_state(self, match_state: MatchState) -> DangerState:
        states = match_state.opponent_robot_states + [match_state.friendly_robot]
        danger_fan_angle = self.config.danger_fan_angle_rad
        for other_state in states:
            opponent_pose = Pose2D.from_msg(other_state.pose.pose)
            controlled_bot_relative_to_opponent = match_state.controlled_robot_pose.relative_to(opponent_pose)
            fan_angle = abs(controlled_bot_relative_to_opponent.heading()) * 2
            opponent_speed = other_state.twist.twist.linear.x
            projected_distance = opponent_speed * self.config.reaction_time
            distance_to_opponent = controlled_bot_relative_to_opponent.magnitude()
            self.markers.append(self.make_arc_marker(opponent_pose, danger_fan_angle, projected_distance))
            if fan_angle < danger_fan_angle and projected_distance > distance_to_opponent:
                return (
                    DangerState.LEFT_DANGER if controlled_bot_relative_to_opponent.y > 0 else DangerState.RIGHT_DANGER
                )
        return DangerState.SAFE

    def get_markers(self) -> list[Marker]:
        return self.markers

    def make_arc_marker(self, pose: Pose2D, angle: float, distance: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = "recover_from_danger"
        marker.id = len(self.markers)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.scale.x = 0.05
        num_points = max(1, int(distance * 10))
        marker.points.append(Point(pose.x, pose.y, 0.0))
        for index in range(num_points + 1):
            theta = pose.theta + angle * (index / num_points) - angle / 2
            x = pose.x + distance * math.cos(theta)
            y = pose.y + distance * math.sin(theta)
            point = Point(x, y, 0.0)
            marker.points.append(point)
        marker.points.append(Point(pose.x, pose.y, 0.0))
        return marker
