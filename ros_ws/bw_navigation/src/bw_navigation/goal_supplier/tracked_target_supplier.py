from typing import Callable, List, Optional

import rospy
from bw_interfaces.msg import EstimatedObject, GoToGoalGoal
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xyz import XYZ
from bw_shared.messages.field import Field
from bw_tools.messages.target_type import TargetType

from bw_navigation.goal_supplier.goal_supplier_interface import GoalSupplierInterface


class TrackedTargetSupplier(GoalSupplierInterface):
    def __init__(self, controlled_robot: str, opponent_names: List[str]) -> None:
        self.opponent_names = opponent_names
        self.controlled_robot = controlled_robot
        self.target_selectors: dict[TargetType, Callable[[dict[str, EstimatedObject]], Optional[EstimatedObject]]] = {
            TargetType.NEAREST_OPPONENT: self.nearest_target,
            TargetType.LARGEST_OPPONENT: self.largest_target,
            TargetType.SMALLEST_OPPONENT: self.smallest_target,
        }
        self.continuously_select_goal = True
        self.last_target_name = None

    def initialize(self, goal: GoToGoalGoal, field: Field, continuously_select_goal: bool) -> None:
        self.target_type = TargetType(goal.target_type)
        self.continuously_select_goal = continuously_select_goal
        self.last_target_name = None

    def nearest_target(self, robot_states: dict[str, EstimatedObject]) -> Optional[EstimatedObject]:
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
                nearest_target = state
                nearest_distance = distance
        if nearest_target is None:
            rospy.logwarn(f"No nearest target found. Opponent names: {self.opponent_names}")
        return nearest_target

    def largest_target(self, robot_states: dict[str, EstimatedObject]) -> Optional[EstimatedObject]:
        largest_target: Optional[EstimatedObject] = None
        largest_size = 0.0
        for name, state in robot_states.items():
            if name not in self.opponent_names:
                continue
            size = XYZ.from_msg(state.size).magnitude()
            if size > largest_size:
                largest_target = state
                largest_size = size
        if largest_target is None:
            rospy.logwarn(f"No largest target found. Opponent names: {self.opponent_names}")
        return largest_target

    def smallest_target(self, robot_states: dict[str, EstimatedObject]) -> Optional[EstimatedObject]:
        smallest_target: Optional[EstimatedObject] = None
        smallest_size = float("inf")
        for name, state in robot_states.items():
            if name not in self.opponent_names:
                continue
            size = XYZ.from_msg(state.size).magnitude()
            if size < smallest_size:
                smallest_target = state
                smallest_size = size
        if smallest_target is None:
            rospy.logwarn(f"No smallest target found. Opponent names: {self.opponent_names}")
        return smallest_target

    def get_goal(self, robot_states: dict[str, EstimatedObject], field: FieldBounds2D) -> Optional[EstimatedObject]:
        if not self.continuously_select_goal and self.last_target_name is not None:
            return robot_states.get(self.last_target_name)
        target_robot = self.target_selectors[self.target_type](robot_states)
        if target_robot is not None and self.last_target_name is None:
            self.last_target_name = target_robot.label
        return target_robot
