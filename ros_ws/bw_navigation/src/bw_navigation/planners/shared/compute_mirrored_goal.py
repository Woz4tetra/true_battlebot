import math

from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.polar import Polar
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import PoseWithCovariance, Twist

from bw_navigation.planners.shared.match_state import MatchState


def compute_mirrored_goal(match_state: MatchState) -> EstimatedObject:
    goal_point = match_state.goal_point
    friendly_point = match_state.friendly_robot_point
    goal_target = match_state.goal_target
    relative_friendly_point = friendly_point - goal_point
    relative_friendly_polar = Polar.from_xy(relative_friendly_point)
    mirrored_polar = Polar(-1 * relative_friendly_polar.radius, relative_friendly_polar.theta)
    mirrored_point = mirrored_polar.to_xy() + goal_point
    mirrored_theta = match_state.friendly_robot_pose.theta + math.pi
    mirrored_pose = Pose2D(mirrored_point.x, mirrored_point.y, mirrored_theta)
    return EstimatedObject(
        header=goal_target.header,
        child_frame_id=goal_target.child_frame_id,
        pose=PoseWithCovariance(pose=mirrored_pose.to_msg(), covariance=goal_target.pose.covariance),
        twist=goal_target.twist,
        size=goal_target.size,
        label=goal_target.label,
        keypoints=goal_target.keypoints,
        keypoint_names=goal_target.keypoint_names,
        score=goal_target.score,
    )


def compute_mirrored_state(match_state: MatchState) -> MatchState:
    return MatchState(
        goal_target=compute_mirrored_goal(match_state),
        robot_states=match_state.robot_states,
        field_bounds=match_state.field_bounds,
        controlled_robot_name=match_state.controlled_robot_name,
        friendly_robot_name=match_state.friendly_robot_name,
        avoid_robot_names=match_state.avoid_robot_names,
    )


def overlay_friendly_twist(
    twist: Twist, match_state: MatchState, magnify: float, start_overlay_distance: float
) -> Twist:
    friendly_twist = match_state.friendly_robot.twist.twist
    distance_to_goal = match_state.distance_to_goal
    ratio = 1 - distance_to_goal / start_overlay_distance
    new_twist = Twist()
    new_twist.linear.x = twist.linear.x * (1 - ratio) + friendly_twist.linear.x * ratio * magnify
    new_twist.angular.z = twist.angular.z * (1 - ratio) + friendly_twist.angular.z * ratio * magnify
    return new_twist
