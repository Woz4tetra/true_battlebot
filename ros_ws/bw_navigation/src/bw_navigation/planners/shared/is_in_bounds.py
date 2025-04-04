from bw_shared.geometry.xy import XY

from bw_navigation.planners.shared.match_state import MatchState


def get_inset_field(buffer_xy: XY, match_state: MatchState) -> tuple[XY, XY]:
    field = match_state.field_bounds
    controlled_robot_size = max(match_state.controlled_robot.size.x, match_state.controlled_robot.size.y)
    controlled_robot_size_half = controlled_robot_size / 2
    buffer = buffer_xy + XY(controlled_robot_size_half, controlled_robot_size_half)
    inset_field = (field[0] + buffer, field[1] - buffer)
    return inset_field


def is_controlled_bot_in_bounds(buffer_xy: XY, angle_tolerance: float, match_state: MatchState) -> bool:
    controlled_robot_point = match_state.controlled_robot_point
    goal_relative_to_controlled_bot = match_state.relative_goal
    goal_heading = goal_relative_to_controlled_bot.heading()
    inset_field = get_inset_field(buffer_xy, match_state)
    return inset_field[0] <= controlled_robot_point <= inset_field[1] or (abs(goal_heading) < angle_tolerance)


def is_goal_in_bounds(buffer_xy: XY, match_state: MatchState) -> bool:
    goal_point = match_state.goal_point
    inset_field = get_inset_field(buffer_xy, match_state)
    return inset_field[0] <= goal_point <= inset_field[1]
