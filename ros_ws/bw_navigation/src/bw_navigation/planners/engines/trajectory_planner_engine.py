from queue import Queue
from threading import Thread
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
    def __init__(
        self, max_velocity: float, max_acceleration: float, ramsete_b: float = 2.0, ramsete_zeta: float = 0.7
    ) -> None:
        self.config = TrajectoryConfig(maxVelocity=max_velocity, maxAcceleration=max_acceleration)
        self.config.setStartVelocity(max_velocity)
        self.config.setEndVelocity(max_velocity)
        self.controller = RamseteController(b=ramsete_b, zeta=ramsete_zeta)
        self.start_time = rospy.Time.now()
        self.planning_thread = Thread(daemon=True, target=self.planning_task)
        self.goal_in_queue: Queue[tuple[Pose2D, Pose2D]] = Queue()
        self.plan_out_queue: Queue[Trajectory] = Queue()
        self.is_planning = False
        self.active_trajectory: Optional[Trajectory] = None
        self.planning_thread.start()

    def planning_task(self) -> None:
        while True:
            robot_pose, goal_pose = self.goal_in_queue.get()
            self.is_planning = True
            trajectory = TrajectoryGenerator.generateTrajectory(
                waypoints=[
                    geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta),
                    geometry.Pose2d(goal_pose.x, goal_pose.y, goal_pose.theta),
                ],
                config=self.config,
            )
            self.plan_out_queue.put(trajectory)
            self.is_planning = False

    def generate_trajectory(self, robot_pose: Pose2D, goal_pose: Pose2D) -> None:
        if not self.is_planning:
            self.goal_in_queue.put((robot_pose, goal_pose))
        else:
            rospy.logdebug("Trajectory generation already in progress")

    def visualize_trajectory(self, num_samples: int = 10) -> MarkerArray:
        if self.active_trajectory is None:
            rospy.logwarn("Trajectory not generated")
            return MarkerArray()
        markers = MarkerArray()
        for index, time in enumerate(np.linspace(0, self.active_trajectory.totalTime(), num_samples)):
            traj_pose = self.active_trajectory.sample(time)
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
        if self.active_trajectory is None:
            return True
        trajectory_duration = rospy.Time.now() - self.start_time
        return trajectory_duration.to_sec() > self.active_trajectory.totalTime()

    def compute(self, robot_pose: Pose2D) -> Twist:
        if not self.plan_out_queue.empty():
            while not self.plan_out_queue.empty():
                self.active_trajectory = self.plan_out_queue.get()
            self.start_time = rospy.Time.now()
        if self.active_trajectory is None:
            rospy.logwarn("Trajectory not generated")
            return Twist()
        current_time = rospy.Time.now()
        desired_pose = self.active_trajectory.sample((current_time - self.start_time).to_sec())
        chassis_speeds = self.controller.calculate(
            geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta),
            desired_pose,
        )
        twist = Twist()
        twist.linear.x = chassis_speeds.vx
        twist.angular.z = chassis_speeds.omega
        return twist
