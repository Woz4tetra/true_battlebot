from abc import ABC, abstractmethod
from typing import Optional, Tuple

from bw_interfaces.msg import EstimatedObject, GoalEngineConfig
from bw_shared.geometry.field_bounds import FieldBounds2D
from geometry_msgs.msg import Twist

from bw_navigation.planners.goal_progress import GoalProgress


class PlannerInterface(ABC):
    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def go_to_goal(
        self,
        dt: float,
        goal_target: EstimatedObject,
        robot_states: dict[str, EstimatedObject],
        field: FieldBounds2D,
        engine_config: Optional[GoalEngineConfig],
        xy_tolerance: float,
    ) -> Tuple[Twist, GoalProgress]: ...
