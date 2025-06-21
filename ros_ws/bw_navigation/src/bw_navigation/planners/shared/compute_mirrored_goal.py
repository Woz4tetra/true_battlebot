import math

from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.polar import Polar
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import Twist

from bw_navigation.planners.engines.config.overlay_velocity_config import OverlayVelocityConfig
from bw_navigation.planners.shared.goal_target_from_pose import goal_target_from_pose
from bw_navigation.planners.shared.match_state import MatchState


def compute_mirrored_goal(match_state: MatchState, angle_offset: float) -> EstimatedObject:
    goal_point = match_state.goal_point
    friendly_point = match_state.friendly_robot_point
    relative_friendly_point = friendly_point - goal_point
    relative_friendly_polar = Polar.from_xy(relative_friendly_point)
    mirrored_polar = Polar(relative_friendly_polar.radius, relative_friendly_polar.theta + angle_offset)
    mirrored_point = mirrored_polar.to_xy() + goal_point
    mirrored_theta = match_state.friendly_robot_pose.theta - angle_offset
    mirrored_pose = Pose2D(mirrored_point.x, mirrored_point.y, mirrored_theta)
    return goal_target_from_pose(mirrored_pose, match_state)


def compute_mirrored_state(match_state: MatchState, angle_offset: float = math.pi) -> MatchState:
    return MatchState(
        goal_target=compute_mirrored_goal(match_state, angle_offset),
        robot_states=match_state.robot_states,
        field_bounds=match_state.field_bounds,
        controlled_robot_name=match_state.controlled_robot_name,
        friendly_robot_name=match_state.friendly_robot_name,
        avoid_robot_names=match_state.avoid_robot_names,
    )


def overlay_friendly_twist(
    twist: Twist, match_state: MatchState, config: OverlayVelocityConfig
) -> tuple[Twist, float, float]:
    magnify = config.friendly_mirror_magnify
    start_overlay_distance = config.friendly_mirror_proximity
    relative_goal = match_state.relative_goal
    if abs(relative_goal.theta) > config.angle_threshold_rad:
        return twist, 0.0, 0.0

    friendly_twist = match_state.friendly_robot.twist.twist
    distance_to_goal = match_state.distance_to_goal
    ratio = max(0.0, min(1.0, 1 - distance_to_goal / start_overlay_distance))
    input_factor = 1 - ratio
    friendly_factor = ratio * magnify
    new_twist = Twist()
    new_twist.linear.x = twist.linear.x * input_factor + friendly_twist.linear.x * friendly_factor
    new_twist.angular.z = twist.angular.z * input_factor + friendly_twist.angular.z * friendly_factor
    return new_twist, input_factor, friendly_factor
