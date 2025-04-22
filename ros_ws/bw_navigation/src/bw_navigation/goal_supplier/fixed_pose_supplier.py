from typing import Optional

from bw_interfaces.msg import EstimatedObject, GoToGoalGoal
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.messages.field import Field

from bw_navigation.exceptions import FrameIdMismatchError
from bw_navigation.goal_supplier.goal_supplier_interface import GoalSupplierInterface


class FixedPoseSupplier(GoalSupplierInterface):
    def __init__(self) -> None:
        self.goal = EstimatedObject()
        self.goal.pose.pose = Pose2D(0.0, 0.0, 0.0).to_msg()

    def initialize(self, goal: GoToGoalGoal, field: Field, continuously_select_goal: bool) -> None:
        if goal.goal.header.frame_id != field.header.frame_id:
            raise FrameIdMismatchError(field.header.frame_id, goal.goal.header.frame_id)
        self.goal.pose.pose = goal.goal.pose

    def get_goal(self, robot_states: dict[str, EstimatedObject], field: FieldBounds2D) -> Optional[EstimatedObject]:
        return self.goal
