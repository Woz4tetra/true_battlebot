import math
from typing import Optional

from bw_shared.geometry.input_modulus import normalize_angle
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.pid.config import PidConfig
from bw_shared.pid.pid import PID
from geometry_msgs.msg import Twist, Vector3


class PidFollowerEngine:
    def __init__(
        self,
        linear_pid: PidConfig,
        angular_pid: PidConfig,
        always_face_forward: bool,
        clamp_linear: Optional[tuple[float, float]] = None,
        clamp_angular: Optional[tuple[float, float]] = None,
    ) -> None:
        self.linear_pid = PID(linear_pid)
        self.angular_pid = PID(angular_pid)
        self.clamp_linear = clamp_linear
        self.clamp_angular = clamp_angular
        self.always_face_forward = always_face_forward

    def compute(self, dt: float, current_pose: Pose2D, goal_pose: Pose2D) -> Twist:
        relative_goal = goal_pose.relative_to(current_pose)
        heading = relative_goal.heading()
        linear_velocity = self.linear_pid.update(relative_goal.x, 0.0, dt)
        if relative_goal.x < 0.0 and not self.always_face_forward:
            heading = normalize_angle(heading + math.pi)
        angular_velocity = self.angular_pid.update(heading, 0.0, dt)
        if self.clamp_linear is not None:
            linear_velocity = max(self.clamp_linear[0], min(self.clamp_linear[1], linear_velocity))
        if self.clamp_angular is not None:
            angular_velocity = max(self.clamp_angular[0], min(self.clamp_angular[1], angular_velocity))
        return Twist(linear=Vector3(x=linear_velocity), angular=Vector3(z=angular_velocity))
