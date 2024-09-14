import copy
from typing import Optional

import numpy as np
import rospy
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from wpimath import geometry
from wpimath.controller import RamseteController
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator


class TrajectoryPlannerEngine:
    def __init__(self) -> None:
        self.config = TrajectoryConfig(maxVelocity=2.5, maxAcceleration=2.0)
        self.controller = RamseteController(b=2.0, zeta=0.7)
        self.start_time = rospy.Time.now()
        self.generator: Optional[Trajectory] = None

    def generate_trajectory(self, robot_pose: Pose2D, goal_pose: Pose2D) -> None:
        self.generator = TrajectoryGenerator.generateTrajectory(
            waypoints=[
                geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta),
                geometry.Pose2d(goal_pose.x, goal_pose.y, goal_pose.theta),
            ],
            config=self.config,
        )
        self.start_time = rospy.Time.now()

    def visualize_trajectory(self, num_samples: int = 10) -> MarkerArray:
        if self.generator is None:
            rospy.logwarn("Trajectory not generated")
            return MarkerArray()
        markers = MarkerArray()
        for index, time in enumerate(np.linspace(0, self.generator.totalTime(), num_samples)):
            traj_pose = self.generator.sample(time)
            marker = Marker()
            translation = traj_pose.pose.translation()
            rotation = traj_pose.pose.rotation()
            angle: float = rotation.radians()  # type: ignore
            pose = Pose2D(translation.x, translation.y, angle)
            marker.header.frame_id = "map"
            marker.ns = "trajectory"
            marker.id = index
            marker.pose = pose.to_msg()
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            markers.markers.append(marker)
        return markers

    def should_replan(self) -> bool:
        if self.generator is None:
            return True
        trajectory_duration = rospy.Time.now() - self.start_time
        return trajectory_duration.to_sec() > self.generator.totalTime()

    def compute(self, robot_pose: Pose2D) -> Twist:
        if self.generator is None:
            rospy.logwarn("Trajectory not generated")
            return Twist()
        current_time = rospy.Time.now()
        desired_pose = self.generator.sample((current_time - self.start_time).to_sec())
        chassis_speeds = self.controller.calculate(
            geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta),
            desired_pose,
        )
        twist = Twist()
        twist.linear.x = chassis_speeds.vx
        twist.angular.z = chassis_speeds.omega
        return twist
