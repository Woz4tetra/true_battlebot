from abc import ABC, abstractmethod
from typing import Optional

from bw_interfaces.msg import EstimatedObject, GoToGoalGoal
from bw_shared.geometry.field_bounds import FieldBounds2D


class GoalSupplierInterface(ABC):
    @abstractmethod
    def initialize(self, goal: GoToGoalGoal, field: EstimatedObject, continuously_select_goal: bool) -> None: ...

    @abstractmethod
    def get_goal(
        self, robot_states: dict[str, EstimatedObject], field_bounds: FieldBounds2D
    ) -> Optional[EstimatedObject]: ...
