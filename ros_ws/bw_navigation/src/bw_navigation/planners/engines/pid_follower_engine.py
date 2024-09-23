import math

from bw_shared.geometry.input_modulus import normalize_angle
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.pid.pid import PID
from geometry_msgs.msg import Twist, Vector3

from bw_navigation.planners.engines.trajectory_planner_engine_config import PidFollowerEngineConfig


class PidFollowerEngine:
    def __init__(self, config: PidFollowerEngineConfig) -> None:
        self.linear_pid = PID(config.linear_pid)
        self.angular_pid = PID(config.angular_pid)
        self.clamp_linear = config.clamp_linear
        self.clamp_angular = config.clamp_angular
        self.always_face_forward = config.always_face_forward

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
