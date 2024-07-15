from typing import Optional

from bw_interfaces.msg import EstimatedObject, GoToGoalGoal
from bw_shared.geometry.pose2d import Pose2D

from bw_navigation.exceptions import FrameIdMismatchError
from bw_navigation.goal_supplier.goal_supplier_interface import GoalSupplierInterface


class FixedPoseSupplier(GoalSupplierInterface):
    def __init__(self) -> None:
        self.goal = Pose2D(0.0, 0.0, 0.0)

    def initialize(self, goal: GoToGoalGoal, field: EstimatedObject) -> None:
        if goal.goal.header.frame_id != field.header.frame_id:
            raise FrameIdMismatchError(field.header.frame_id, goal.goal.header.frame_id)
        self.goal = Pose2D.from_msg(goal.goal.pose)

    def get_goal(self, robot_states: dict[str, EstimatedObject], field: EstimatedObject) -> Optional[Pose2D]:
        return self.goal
