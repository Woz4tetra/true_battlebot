import math

from bw_shared.geometry.in_plane import nearest_projected_point
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY

from bw_navigation.planners.shared.goal_target_from_pose import goal_target_from_pose
from bw_navigation.planners.shared.is_in_bounds import get_inset_field
from bw_navigation.planners.shared.match_state import MatchState


def get_bounded_match_state(buffer_xy: XY, match_state: MatchState) -> MatchState:
    inset_field = get_inset_field(buffer_xy, match_state)
    away_from_opponent_heading = match_state.relative_goal.heading()
    directed_controlled_pose = Pose2D(
        match_state.controlled_robot_point.x,
        match_state.controlled_robot_point.y,
        away_from_opponent_heading,
    )
    nearest_point = nearest_projected_point(directed_controlled_pose, inset_field)
    nearest_pose = Pose2D(nearest_point.x, nearest_point.y, match_state.controlled_robot_pose.theta)
    goal_target = goal_target_from_pose(nearest_pose, match_state)
    return MatchState(
        goal_target=goal_target,
        robot_states=match_state.robot_states,
        field_bounds=match_state.field_bounds,
        controlled_robot_name=match_state.controlled_robot_name,
        friendly_robot_name=match_state.friendly_robot_name,
        avoid_robot_names=match_state.avoid_robot_names,
    )
