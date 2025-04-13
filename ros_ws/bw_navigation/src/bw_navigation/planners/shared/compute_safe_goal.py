from bw_interfaces.msg import EstimatedObject

from bw_navigation.planners.shared.match_state import MatchState


def compute_safe_goal(match_state: MatchState) -> EstimatedObject:
    for robot_name, robot_state in match_state.robot_states.items():
        if robot_name == match_state.controlled_robot_name:
            continue


def compute_safe_state(match_state: MatchState) -> MatchState:
    return MatchState(
        goal_target=compute_safe_goal(match_state),
        robot_states=match_state.robot_states,
        field_bounds=match_state.field_bounds,
        controlled_robot_name=match_state.controlled_robot_name,
        friendly_robot_name=match_state.friendly_robot_name,
        avoid_robot_names=match_state.avoid_robot_names,
    )
