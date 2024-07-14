from abc import ABC, abstractmethod
from typing import Tuple

from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import Twist


class PlannerInterface(ABC):
    @abstractmethod
    def go_to_goal(
        self, dt: float, goal_pose: Pose2D, robot_states: dict[str, EstimatedObject], field: EstimatedObject
    ) -> Tuple[Twist, bool]: ...
