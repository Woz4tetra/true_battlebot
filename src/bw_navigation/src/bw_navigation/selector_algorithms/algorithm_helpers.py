import math

from bw_interfaces.msg import EstimatedObject
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.xy import XY

from bw_navigation.selector_algorithms.match_state import MatchState


def is_on_target(match_state: MatchState, on_target_lateral_threshold: float) -> bool:
    """
    Check if the controlled bot is in line with the guidance bot and opponent
    """
    guidance_pose = match_state.guidance_pose
    opponent_pose = match_state.opponent_pose
    controlled_pose = match_state.controlled_pose
    guidance_pointed_to_opponent = Pose2D(guidance_pose.x, guidance_pose.y, match_state.guidance_to_opponent_heading)
    guidance_to_opponent = opponent_pose.relative_to(guidance_pointed_to_opponent)
    guidance_to_controlled = controlled_pose.relative_to(guidance_pointed_to_opponent)
    return (
        abs(guidance_to_opponent.y) < on_target_lateral_threshold
        and abs(guidance_to_controlled.y) < on_target_lateral_threshold
    )


def compute_push_target(match_state: MatchState, keep_back_buffer: float) -> Pose2D:
    """
    Compute a pose just in front of the guidance bot, pointing at the opponent, using the keep_back_distance
    """
    guidance_pose = match_state.guidance_pose
    opponent_pose = match_state.opponent_pose
    distance_to_opponent = match_state.guidance_to_opponent_magnitude
    if distance_to_opponent <= 0.0:
        return match_state.controlled_pose
    interpolation_ratio = keep_back_distance(match_state, keep_back_buffer) / distance_to_opponent
    target_point = interpolate(guidance_pose.to_point(), opponent_pose.to_point(), interpolation_ratio)
    target_pose = Pose2D(target_point.x, target_point.y, match_state.guidance_to_opponent_heading + math.pi)
    return target_pose


def compute_pose_behind_opponent(match_state: MatchState, keep_back_buffer: float) -> Pose2D:
    """
    Compute a pose behind the opponent, pointing at the guidance bot, using the keep_back_distance
    """
    guidance_pose = match_state.guidance_pose
    opponent_pose = match_state.opponent_pose
    distance_to_opponent = match_state.guidance_to_opponent_magnitude
    distance_behind_opponent = distance_to_opponent + keep_back_distance(match_state, keep_back_buffer)
    if distance_to_opponent <= 0.0:
        return match_state.controlled_pose
    interpolation_ratio = distance_behind_opponent / distance_to_opponent
    point_behind_opponent = interpolate(guidance_pose.to_point(), opponent_pose.to_point(), interpolation_ratio)
    pose_behind_opponent = Pose2D(
        point_behind_opponent.x, point_behind_opponent.y, match_state.guidance_to_opponent_heading + math.pi
    )
    return pose_behind_opponent


def compute_nearest_pose_to_opponent(match_state: MatchState, keep_back_buffer: float) -> Pose2D:
    """
    Compute a pose close to the opponent that is still inside the field
    """
    opponent_pose = match_state.opponent_pose
    controlled_pose = match_state.controlled_pose
    distance_to_opponent = opponent_pose.magnitude(controlled_pose)
    goal_distance = keep_back_distance(match_state, keep_back_buffer)
    if distance_to_opponent < goal_distance:
        return controlled_pose
    distance_near_opponent = distance_to_opponent - goal_distance
    interpolation_ratio = distance_near_opponent / distance_to_opponent
    point_near_opponent = interpolate(controlled_pose.to_point(), opponent_pose.to_point(), interpolation_ratio)
    pose_near_opponent = Pose2D(point_near_opponent.x, point_near_opponent.y, controlled_pose.heading(opponent_pose))
    return pose_near_opponent


def is_point_in_bounds(point: XY, robot: EstimatedObject, field: EstimatedObject, border_buffer: float) -> bool:
    robot_dim = max(robot.size.x, robot.size.y)
    half_x = max(0.0, field.size.x / 2 - robot_dim - border_buffer)
    half_y = max(0.0, field.size.y / 2 - robot_dim - border_buffer)
    return -half_x <= point.x <= half_x and -half_y <= point.y <= half_y


def keep_back_distance(match_state: MatchState, keep_back_buffer: float) -> float:
    """
    Compute the distance to keep back from the opponent
    """
    return match_state.controlled_diameter / 2 + match_state.opponent_diameter / 2 + keep_back_buffer


def interpolate(obj1: XY, obj2: XY, t: float) -> XY:
    """
    Interpolate between two points
    """
    t_complement = 1 - t
    return XY(t_complement * obj1.x + t * obj2.x, t_complement * obj1.y + t * obj2.y)


def is_behind_opponent(match_state: MatchState) -> bool:
    controlled_pose = match_state.controlled_pose
    opponent_pose = match_state.opponent_pose
    guidance_pointed_to_opponent = Pose2D(opponent_pose.x, opponent_pose.y, match_state.guidance_to_opponent_heading)
    controlled_to_opponent = controlled_pose.relative_to(guidance_pointed_to_opponent)

    return controlled_to_opponent.x >= 0
