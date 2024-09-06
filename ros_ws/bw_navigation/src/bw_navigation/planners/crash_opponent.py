from typing import Tuple

import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.pid.config import PidConfig
from geometry_msgs.msg import Twist

from bw_navigation.planners.engines.pid_follower_engine import PidFollowerEngine
from bw_navigation.planners.planner_interface import PlannerInterface


class CrashOpponent(PlannerInterface):
    def __init__(self, controlled_robot: str) -> None:
        self.controlled_robot = controlled_robot
        linear_pid = PidConfig(kp=0.5, ki=0.0, kd=0.01, kf=1.0)
        angular_pid = PidConfig(kp=1.0, ki=0.01, kd=0.0, kf=0.2)
        self.pid_follower = PidFollowerEngine(linear_pid, angular_pid)

    def go_to_goal(
        self, dt: float, goal_pose: Pose2D, robot_states: dict[str, EstimatedObject], field: EstimatedObject
    ) -> Tuple[Twist, bool]:
        if self.controlled_robot not in robot_states:
            rospy.logwarn_throttle(1, f"Robot {self.controlled_robot} not found in robot states")
            return Twist(), False
        controlled_robot_pose = Pose2D.from_msg(robot_states[self.controlled_robot].pose.pose)
        twist = self.pid_follower.compute(dt, controlled_robot_pose, goal_pose)
        return twist, False
