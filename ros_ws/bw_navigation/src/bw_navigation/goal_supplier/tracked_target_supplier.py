from typing import List, Optional

from bw_interfaces.msg import EstimatedObject, GoToGoalGoal
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xyz import XYZ
from bw_tools.messages.target_type import TargetType

from bw_navigation.goal_supplier.goal_supplier_interface import GoalSupplierInterface


class TrackedTargetSupplier(GoalSupplierInterface):
    def __init__(self, controlled_robot: str, opponent_names: List[str]) -> None:
        self.opponent_names = opponent_names
        self.controlled_robot = controlled_robot
        self.target_selectors = {
            TargetType.NEAREST_OPPONENT: self.nearest_target,
            TargetType.LARGEST_OPPONENT: self.largest_target,
        }

    def initialize(self, goal: GoToGoalGoal, field: EstimatedObject) -> None:
        self.target_type = TargetType(goal.target_type)

    def nearest_target(self, robot_states: dict[str, EstimatedObject]) -> Optional[Pose2D]:
        if self.controlled_robot not in robot_states:
            return None
        robot_pose = Pose2D.from_msg(robot_states[self.controlled_robot].pose.pose)
        nearest_target = None
        nearest_distance = float("inf")
        for name, state in robot_states.items():
            if name not in self.opponent_names:
                continue
            target_pose = Pose2D.from_msg(state.pose.pose)
            distance = robot_pose.magnitude(target_pose)
            if distance < nearest_distance:
                nearest_target = target_pose
                nearest_distance = distance
        return nearest_target

    def largest_target(self, robot_states: dict[str, EstimatedObject]) -> Optional[Pose2D]:
        largest_target = None
        largest_size = 0.0
        for name, state in robot_states.items():
            if name not in self.opponent_names:
                continue
            target_pose = Pose2D.from_msg(state.pose.pose)
            size = XYZ.from_msg(state.size).magnitude()
            if size > largest_size:
                largest_target = target_pose
                largest_size = size
        return largest_target

    def get_goal(self, robot_states: dict[str, EstimatedObject], field: EstimatedObject) -> Optional[Pose2D]:
        return self.target_selectors[self.target_type](robot_states)
