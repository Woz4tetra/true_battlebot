from typing import Tuple

from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import Twist

from ros_ws.bw_navigation.src.bw_navigation.planners.planner_interface import PlannerInterface


class CrashOpponentAvoidFriendly(PlannerInterface):
    def go_to_goal(
        self, dt: float, goal_pose: Pose2D, robot_states: dict[str, EstimatedObject], field: EstimatedObject
    ) -> Tuple[Twist, bool]:
        return Twist(), False
