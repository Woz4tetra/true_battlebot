from abc import ABC, abstractmethod
from typing import Optional

from bw_interfaces.msg import EstimatedObject, GoToGoalGoal
from bw_shared.geometry.pose2d import Pose2D


class GoalSupplierInterface(ABC):
    @abstractmethod
    def initialize(self, goal: GoToGoalGoal, field: EstimatedObject) -> None: ...

    @abstractmethod
    def get_goal(self, robot_states: dict[str, EstimatedObject], field: EstimatedObject) -> Optional[Pose2D]: ...
