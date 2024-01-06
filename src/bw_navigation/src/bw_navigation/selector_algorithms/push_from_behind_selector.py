import math
from enum import Enum, auto

import rospy
from bw_interfaces.msg import EstimatedObject
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.xy import XY
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from bw_navigation.selector_algorithms.match_state import MatchState, SelectionResult

from .base_selector import BaseSelector


class PushFromBehindState(Enum):
    IDLE = auto()
    PUSH = auto()
    GET_BEHIND = auto()
    GET_NEAR = auto()


class PushFromBehindSelector(BaseSelector):
    """
    If the controlled robot is not in line with the guidance bot and opponent, select goal that's behind the
    opponent pointing towards the guidance bot.
    Otherwise, if the controlled bot is in position, select goal that's just in front the guidance bot
    (push the opponent to the guidance bot).
    """

    def __init__(self) -> None:
        self.keep_back_distance = 0.1
        self.on_target_lateral_threshold = 0.075
        self.state = PushFromBehindState.IDLE

    def get_target(self, match_state: MatchState) -> SelectionResult:
        """
        Get the target pose for the controlled bot.
        If the controlled bot is in line with the guidance bot and opponent, select goal that's just in front the
        guidance bot (push the opponent to the guidance bot).
        Otherwise, select a goal that's behind the opponent pointing towards the guidance bot.
        If the goal is outside the field, select a goal close to the opponent that is still inside the field.
        """
        if self.is_on_target(match_state):
            push_target = self.compute_push_target(match_state)
            self.set_state(PushFromBehindState.PUSH)
            return SelectionResult(
                goal=PoseStamped(
                    header=Header(frame_id=match_state.frame_id),
                    pose=push_target.to_msg(),
                ),
                ignore_opponent_obstacles=True,
            )
        pose_behind_opponent = self.compute_pose_behind_opponent(match_state)
        if self.is_point_in_bounds(pose_behind_opponent.to_point(), match_state.field):
            self.set_state(PushFromBehindState.GET_BEHIND)
            return SelectionResult(
                goal=PoseStamped(
                    header=Header(frame_id=match_state.frame_id),
                    pose=pose_behind_opponent.to_msg(),
                ),
                ignore_opponent_obstacles=False,
            )

        nearest_pose_to_opponent = self.compute_nearest_pose_to_opponent(match_state)
        self.set_state(PushFromBehindState.GET_NEAR)
        return SelectionResult(
            goal=PoseStamped(
                header=Header(frame_id=match_state.frame_id),
                pose=nearest_pose_to_opponent.to_msg(),
            ),
            ignore_opponent_obstacles=False,
        )

    def set_state(self, state: PushFromBehindState) -> None:
        if self.state != state:
            self.state = state
            rospy.loginfo(f"PushFromBehindSelector state changed to {state}")

    def is_on_target(self, match_state: MatchState) -> bool:
        """
        Check if the controlled bot is in line with the guidance bot and opponent
        """
        guidance_pose = match_state.guidance_pose
        opponent_pose = match_state.opponent_pose
        controlled_pose = match_state.controlled_pose
        guidance_pointed_to_opponent = Pose2D(
            guidance_pose.x, guidance_pose.y, match_state.guidance_to_opponent_heading
        )
        guidance_to_opponent = opponent_pose.relative_to(guidance_pointed_to_opponent)
        guidance_to_controlled = controlled_pose.relative_to(guidance_pointed_to_opponent)
        return (
            abs(guidance_to_opponent.y) < self.on_target_lateral_threshold
            and abs(guidance_to_controlled.y) < self.on_target_lateral_threshold
        )

    def compute_push_target(self, match_state: MatchState) -> Pose2D:
        """
        Compute a pose just in front of the guidance bot, pointing at the opponent, using the keep_back_distance
        """
        guidance_pose = match_state.guidance_pose
        opponent_pose = match_state.opponent_pose
        distance_to_opponent = match_state.guidance_to_opponent_magnitude
        interpolation_ratio = self.keep_back_distance / distance_to_opponent
        target_point = self.interpolate(guidance_pose.to_point(), opponent_pose.to_point(), interpolation_ratio)
        target_pose = Pose2D(target_point.x, target_point.y, match_state.guidance_to_opponent_heading + math.pi)
        return target_pose

    def compute_pose_behind_opponent(self, match_state: MatchState) -> Pose2D:
        """
        Compute a pose behind the opponent, pointing at the guidance bot, using the keep_back_distance
        """
        guidance_pose = match_state.guidance_pose
        opponent_pose = match_state.opponent_pose
        distance_to_opponent = match_state.guidance_to_opponent_magnitude
        distance_behind_opponent = distance_to_opponent + self.keep_back_distance
        interpolation_ratio = distance_behind_opponent / distance_to_opponent
        point_behind_opponent = self.interpolate(
            guidance_pose.to_point(), opponent_pose.to_point(), interpolation_ratio
        )
        pose_behind_opponent = Pose2D(
            point_behind_opponent.x, point_behind_opponent.y, match_state.guidance_to_opponent_heading + math.pi
        )
        return pose_behind_opponent

    def compute_nearest_pose_to_opponent(self, match_state: MatchState) -> Pose2D:
        """
        Compute a pose close to the opponent that is still inside the field
        """
        opponent_pose = match_state.opponent_pose
        controlled_pose = match_state.controlled_pose
        distance_to_opponent = opponent_pose.magnitude(controlled_pose)
        if distance_to_opponent < self.keep_back_distance:
            return controlled_pose
        distance_near_opponent = distance_to_opponent - self.keep_back_distance
        interpolation_ratio = distance_near_opponent / distance_to_opponent
        point_near_opponent = self.interpolate(
            controlled_pose.to_point(), opponent_pose.to_point(), interpolation_ratio
        )
        pose_near_opponent = Pose2D(
            point_near_opponent.x, point_near_opponent.y, controlled_pose.heading(opponent_pose)
        )
        return pose_near_opponent

    def is_point_in_bounds(self, point: XY, field: EstimatedObject) -> bool:
        half_x = field.size.x / 2
        half_y = field.size.y / 2
        return -half_x <= point.x <= half_x and -half_y <= point.y <= half_y

    def interpolate(self, obj1: XY, obj2: XY, t: float) -> XY:
        """
        Interpolate between two points
        """
        t_complement = 1 - t
        return XY(t_complement * obj1.x + t * obj2.x, t_complement * obj1.y + t * obj2.y)
