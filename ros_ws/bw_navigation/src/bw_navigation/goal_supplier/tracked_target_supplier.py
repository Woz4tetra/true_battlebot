from typing import Optional

from bw_interfaces.msg import EstimatedObject, GoToGoalGoal
from bw_shared.geometry.pose2d import Pose2D

from bw_navigation.goal_supplier.goal_supplier_interface import GoalSupplierInterface


class TrackedTargetSupplier(GoalSupplierInterface):
    def __init__(self) -> None:
        self.target_name = ""

    def initialize(self, goal: GoToGoalGoal, field: EstimatedObject) -> None:
        self.target_name = goal.target_name

    def get_goal(self, robot_states: dict[str, EstimatedObject], field: EstimatedObject) -> Optional[Pose2D]:
        if self.target_name not in robot_states:
            return None
        return Pose2D.from_msg(robot_states[self.target_name].pose.pose)
