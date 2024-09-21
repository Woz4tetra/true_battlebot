from dataclasses import dataclass
from queue import Queue
from threading import Thread
from typing import Optional

import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.in_plane import line_bounds_intersection
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.twist2d import Twist2D
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from wpimath import geometry
from wpimath.controller import RamseteController
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.trajectory.constraint import CentripetalAccelerationConstraint, DifferentialDriveKinematicsConstraint

from bw_navigation.planners.engines.trajectory_planner_engine_config import TrajectoryPlannerEngineConfig


@dataclass
class PlanningTicket:
    robot_pose: Pose2D
    robot_velocity: Twist2D
    goal_pose: Pose2D
    goal_velocity: Twist2D
    field: FieldBounds2D


class TrajectoryPlannerEngine:
    def __init__(self, config: TrajectoryPlannerEngineConfig) -> None:
        self.config = config
        self.traj_config = TrajectoryConfig(
            maxVelocity=self.config.max_velocity, maxAcceleration=self.config.max_acceleration
        )
        self.kinematics = DifferentialDriveKinematics(self.config.track_width)

        self.traj_config.setStartVelocity(self.config.max_velocity)
        self.traj_config.setEndVelocity(self.config.max_velocity)
        if self.config.max_centripetal_acceleration is not None:
            self.traj_config.addConstraint(
                CentripetalAccelerationConstraint(maxCentripetalAcceleration=self.config.max_centripetal_acceleration),
            )
        self.traj_config.addConstraint(
            DifferentialDriveKinematicsConstraint(self.kinematics, self.config.max_velocity),
        )

        self.controller = RamseteController(b=self.config.ramsete_b, zeta=self.config.ramsete_zeta)

        self.start_time = rospy.Time.now()

        self.planning_thread = Thread(daemon=True, target=self.planning_task)
        self.goal_in_queue: Queue[PlanningTicket] = Queue()
        self.plan_out_queue: Queue[Trajectory] = Queue()
        self.is_planning = False
        self.active_trajectory: Optional[Trajectory] = None

        self.planning_thread.start()

    def _bound_pose_along_line(self, pose: Pose2D, projected_pose: Pose2D, bounds: FieldBounds2D) -> Pose2D:
        segment = np.array(
            [
                [pose.x, pose.y],
                [projected_pose.x, projected_pose.y],
            ]
        )
        intersections = line_bounds_intersection(segment, bounds)
        if len(intersections) == 0:
            return projected_pose
        intersection_poses = [
            Pose2D(intersection[0], intersection[1], projected_pose.theta) for intersection in intersections
        ]
        closest_intersection = min(
            intersection_poses, key=lambda intersection: intersection.relative_to(projected_pose).magnitude()
        )
        return closest_intersection

    def _plan_with_goal_velocity(self, ticket: PlanningTicket) -> Trajectory:
        robot_pose = ticket.robot_pose
        goal_pose = ticket.goal_pose
        trajectory = self._plan_once(robot_pose, goal_pose)
        for _ in range(self.config.forward_project_max_iters):
            total_time = trajectory.totalTime()
            travel_distance = ticket.goal_velocity.x * total_time
            projected_goal = Pose2D(travel_distance, 0.0, 0.0).transform_by(goal_pose)
            projected_goal = self._bound_pose_along_line(goal_pose, projected_goal, ticket.field)
            if projected_goal.relative_to(goal_pose).magnitude() < self.config.forward_project_converge_threshold:
                break
            trajectory = self._plan_once(robot_pose, goal_pose)
        return trajectory

    def _plan_once(self, robot_pose: Pose2D, goal_pose: Pose2D) -> Trajectory:
        return TrajectoryGenerator.generateTrajectory(
            waypoints=[
                geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta),
                geometry.Pose2d(goal_pose.x, goal_pose.y, goal_pose.theta),
            ],
            config=self.traj_config,
        )

    def planning_task(self) -> None:
        while True:
            try:
                ticket = self.goal_in_queue.get()

                if self.config.used_measured_velocity:
                    robot_velocity = ticket.robot_velocity
                    self.traj_config.setStartVelocity(robot_velocity.x)
                self.is_planning = True
                if self.config.forward_project_goal_velocity:
                    trajectory = self._plan_with_goal_velocity(ticket)
                else:
                    trajectory = self._plan_once(ticket.robot_pose, ticket.goal_pose)
                self.plan_out_queue.put(trajectory)
                self.is_planning = False
            except Exception as e:
                rospy.logerr(f"Trajectory planning failed: {e}", exc_info=True)
                self.is_planning = False

    def generate_trajectory(
        self, robot_state: EstimatedObject, goal_target: EstimatedObject, field: FieldBounds2D
    ) -> None:
        if not self.is_planning:
            robot_pose = Pose2D.from_msg(robot_state.pose.pose)
            robot_velocity = Twist2D.from_msg(robot_state.twist.twist)
            goal_pose = Pose2D.from_msg(goal_target.pose.pose)
            goal_velocity = Twist2D.from_msg(goal_target.twist.twist)
            self.goal_in_queue.put(PlanningTicket(robot_pose, robot_velocity, goal_pose, goal_velocity, field))
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
