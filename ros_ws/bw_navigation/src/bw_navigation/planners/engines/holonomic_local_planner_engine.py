import math
from typing import Optional

from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from wpimath import geometry
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians

from bw_navigation.planners.engines.config.holonomic_local_planner_engine_config import (
    HolonomicLocalPlannerEngineConfig,
)
from bw_navigation.planners.engines.config.holonomic_trajectory_global_planner_config import (
    HolonomicTrajectoryGlobalPlannerConfig,
)


class HolonomicLocalPlannerEngine:
    def __init__(
        self, config: HolonomicLocalPlannerEngineConfig, plan_config: HolonomicTrajectoryGlobalPlannerConfig
    ) -> None:
        self.config = config
        profile_constraints = TrapezoidProfileRadians.Constraints(
            plan_config.max_angular_velocity, plan_config.max_acceleration_angular
        )
        self.x_pid = PIDController(
            self.config.linear_x_pid.kp, self.config.linear_x_pid.ki, self.config.linear_x_pid.kd
        )
        self.y_pid = PIDController(
            self.config.linear_y_pid.kp, self.config.linear_y_pid.ki, self.config.linear_y_pid.kd
        )
        self.angular_pid = ProfiledPIDControllerRadians(
            self.config.angular_pid.kp, self.config.angular_pid.ki, self.config.angular_pid.kd, profile_constraints
        )
        self.angular_pid.enableContinuousInput(-math.pi, math.pi)
        self.controller = HolonomicDriveController(self.x_pid, self.y_pid, self.angular_pid)

        self.desired_pose = Pose2D(0.0, 0.0, 0.0)
        self.obstacles: list[Optional[EstimatedObject]] = []

    def reset(self) -> None:
        self.desired_pose = Pose2D(0.0, 0.0, 0.0)
        self.obstacles = []
        self.x_pid.reset()
        self.y_pid.reset()
        self.angular_pid.reset()

    def make_desired_pose_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "desired_pose"
        marker.id = 0
        marker.pose = self.desired_pose.to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.1
        return marker

    def make_obstacle_markers(self) -> list[Marker]:
        markers = []
        for index, obstacle in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "obstacle"
            marker.id = index
            if obstacle is None:
                marker.action = Marker.DELETE
                markers.append(marker)
                continue
            marker.pose = obstacle.pose.pose
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            friendly_robot_size = max(obstacle.size.x, obstacle.size.y) + self.config.obstacle_buffer
            marker.scale.x = friendly_robot_size
            marker.scale.y = friendly_robot_size
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            markers.append(marker)
        return markers

    def visualize_local_plan(self) -> list[Marker]:
        markers = [self.make_desired_pose_marker()]
        markers.extend(self.make_obstacle_markers())
        return markers

    def compute(self, desired_state: Trajectory.State, robot_pose: Pose2D) -> Twist:
        twist = Twist()
        desired_angle = desired_state.pose.rotation()
        current_pose = geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta)
        chassis_speeds: ChassisSpeeds = self.controller.calculate(current_pose, desired_state, desired_angle)
        twist.linear.x = chassis_speeds.vx
        twist.linear.y = chassis_speeds.vy
        twist.angular.z = chassis_speeds.omega
        return twist
