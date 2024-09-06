import math

from bw_shared.geometry.input_modulus import normalize_angle
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.pid.config import PidConfig
from bw_shared.pid.pid import PID
from geometry_msgs.msg import Twist, Vector3


class PidFollowerEngine:
    def __init__(self, linear_pid: PidConfig, angular_pid: PidConfig) -> None:
        self.linear_pid = PID(linear_pid)
        self.angular_pid = PID(angular_pid)

    def compute(self, dt: float, current_pose: Pose2D, goal_pose: Pose2D) -> Twist:
        relative_goal = goal_pose.relative_to(current_pose)
        heading = relative_goal.heading()
        linear_velocity = self.linear_pid.update(relative_goal.x, 0.0, dt)
        if linear_velocity < 0.0:
            heading = normalize_angle(heading + math.pi)
        angular_velocity = self.angular_pid.update(heading, 0.0, dt)
        return Twist(linear=Vector3(x=linear_velocity), angular=Vector3(z=angular_velocity))
