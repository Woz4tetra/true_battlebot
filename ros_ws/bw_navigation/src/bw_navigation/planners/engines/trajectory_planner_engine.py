import random
from queue import Queue
from threading import Thread
from typing import Optional

import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObject, VelocityProfile
from bw_interfaces.msg import Trajectory as TrajectoryMsg
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.in_plane import line_bounds_intersection
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.twist2d import Twist2D
from visualization_msgs.msg import Marker
from wpimath import geometry
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.trajectory.constraint import CentripetalAccelerationConstraint, DifferentialDriveKinematicsConstraint

from bw_navigation.planners.engines.trajectory_helpers import PlanningResult, PlanningTicket, get_theta
from bw_navigation.planners.engines.trajectory_planner_engine_config import TrajectoryPlannerEngineConfig


class TrajectoryPlannerEngine:
    def __init__(self, config: TrajectoryPlannerEngineConfig) -> None:
        self.plan_config = config
        self.kinematics = DifferentialDriveKinematics(self.plan_config.track_width)
        self.replan_interval = rospy.Duration.from_sec(self.plan_config.replan_interval)
        self.start_time = rospy.Time()

        self.planning_thread = Thread(daemon=True, target=self._planning_task)
        self.goal_in_queue: Queue[PlanningTicket] = Queue(maxsize=1)
        self.plan_out_queue: Queue[PlanningResult] = Queue()
        self.is_planning = False
        self.active_trajectory: Optional[Trajectory] = None
        self.active_trajectory_msg = TrajectoryMsg()

        self.planning_thread.start()

    def reset(self) -> None:
        self.active_trajectory = None
        self.start_time = rospy.Time()
        while self.is_planning:
            rospy.loginfo("Waiting for planning to finish before resetting")
            rospy.sleep(0.1)
        while not self.goal_in_queue.empty():
            self.goal_in_queue.get()

    def make_trajectory_config_from_velocity_profile(
        self, velocity_profile: Optional[VelocityProfile]
    ) -> TrajectoryConfig:
        if velocity_profile:
            return self.make_trajectory_config(
                velocity_profile.max_velocity,
                velocity_profile.max_acceleration,
                None
                if np.isnan(velocity_profile.max_centripetal_acceleration)
                else velocity_profile.max_centripetal_acceleration,
            )
        else:
            return self.make_trajectory_config(
                self.plan_config.max_velocity,
                self.plan_config.max_acceleration,
                self.plan_config.max_centripetal_acceleration,
            )

    def make_trajectory_config(
        self, max_velocity: float, max_acceleration: float, max_centripetal_acceleration: Optional[float] = None
    ) -> TrajectoryConfig:
        traj_config = TrajectoryConfig(maxVelocity=max_velocity, maxAcceleration=max_acceleration)
        traj_config.setStartVelocity(max_velocity)
        traj_config.setEndVelocity(max_velocity)
        if max_centripetal_acceleration is not None:
            traj_config.addConstraint(
                CentripetalAccelerationConstraint(maxCentripetalAcceleration=max_centripetal_acceleration)
            )
        traj_config.addConstraint(
            DifferentialDriveKinematicsConstraint(self.kinematics, max_velocity),
        )
        return traj_config

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
        distance_to_goal = goal_pose.relative_to(robot_pose).magnitude() * self.plan_config.prediction_magnification
        total_time = distance_to_goal / self.plan_config.max_velocity
        travel_distance = ticket.goal_velocity.x * total_time
        projected_goal = Pose2D(travel_distance, 0.0, 0.0).transform_by(goal_pose)
        projected_goal = self._bound_pose_along_line(goal_pose, projected_goal, ticket.field)
        trajectory = self._plan_once(robot_pose, projected_goal, ticket.trajectory_config)
        return trajectory

    def _plan_once(self, robot_pose: Pose2D, goal_pose: Pose2D, trajectory_config: TrajectoryConfig) -> Trajectory:
        trajectory = None
        original_goal_pose = goal_pose
        noise = self.plan_config.planning_failure_random_noise
        while trajectory is None:
            try:
                trajectory = TrajectoryGenerator.generateTrajectory(
                    waypoints=[
                        geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta),
                        geometry.Pose2d(goal_pose.x, goal_pose.y, goal_pose.theta),
                    ],
                    config=trajectory_config,
                )
            except RuntimeError as e:
                rospy.logerr(f"Trajectory generation failed: {e}")
                # randomize the goal pose to avoid getting stuck in the same failure
                goal_pose = Pose2D(
                    original_goal_pose.x + random.uniform(-noise, noise),
                    original_goal_pose.y + random.uniform(-noise, noise),
                    original_goal_pose.theta + random.uniform(-noise, noise),
                )
        return trajectory

    def _planning_task(self) -> None:
        while True:
            try:
                ticket = self.goal_in_queue.get()

                self.is_planning = True
                planning_start_time = rospy.Time.now()
                if self.plan_config.forward_project_goal_velocity:
                    trajectory = self._plan_with_goal_velocity(ticket)
                else:
                    trajectory = self._plan_once(ticket.robot_pose, ticket.goal_pose, ticket.trajectory_config)
                planning_finish_time = rospy.Time.now()
                self.plan_out_queue.put(PlanningResult(trajectory, planning_start_time, planning_finish_time))
                self.is_planning = False
            except Exception as e:
                rospy.logerr(f"Trajectory planning failed: {e}", exc_info=True)
                self.is_planning = False

    def make_trajectory_markers(self, num_samples: int) -> list[Marker]:
        markers: list[Marker] = []
        if self.active_trajectory is None:
            rospy.logwarn("Trajectory not generated")
            return markers
        for index, time in enumerate(np.linspace(0, self.active_trajectory.totalTime(), num_samples)):
            traj_pose = self.active_trajectory.sample(time)
            marker = Marker()
            translation = traj_pose.pose.translation()
            rotation = traj_pose.pose.rotation()
            angle = get_theta(rotation)
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
            markers.append(marker)
        return markers

    def visualize_trajectory(self, num_samples: int = 10) -> list[Marker]:
        return self.make_trajectory_markers(num_samples)

    def should_replan(self) -> bool:
        if self.active_trajectory is None:
            return True
        now = rospy.Time.now()
        trajectory_duration = now - self.start_time
        return (
            trajectory_duration.to_sec() > self.active_trajectory.totalTime()
            or trajectory_duration > self.replan_interval
        )

    def generate_trajectory(
        self,
        robot_state: EstimatedObject,
        goal_target: EstimatedObject,
        field: FieldBounds2D,
        velocity_profile: Optional[VelocityProfile],
    ) -> None:
        if not self.is_planning:
            robot_pose = Pose2D.from_msg(robot_state.pose.pose)
            robot_velocity = Twist2D.from_msg(robot_state.twist.twist)
            goal_pose = Pose2D.from_msg(goal_target.pose.pose)
            goal_velocity = Twist2D.from_msg(goal_target.twist.twist)

            trajectory_config = self.make_trajectory_config_from_velocity_profile(velocity_profile)
            if self.plan_config.used_measured_velocity:
                trajectory_config.setStartVelocity(robot_velocity.x)
            self.goal_in_queue.put(
                PlanningTicket(robot_pose, robot_velocity, goal_pose, goal_velocity, field, trajectory_config)
            )
        else:
            rospy.logdebug("Trajectory generation already in progress")

    def compute(
        self,
        robot_state: EstimatedObject,
        goal_target: EstimatedObject,
        field: FieldBounds2D,
        velocity_profile: Optional[VelocityProfile],
    ) -> tuple[Optional[Trajectory], bool, rospy.Time]:
        did_replan = False
        now = rospy.Time.now()
        if self.should_replan():
            self.generate_trajectory(robot_state, goal_target, field, velocity_profile)
        if not self.plan_out_queue.empty():
            while not self.plan_out_queue.empty():
                result = self.plan_out_queue.get()
                self.active_trajectory = result.trajectory
                did_replan = True
                planning_time = (result.planning_finish_time - result.planning_start_time).to_sec()
                plan_age = (now - result.planning_start_time).to_sec()
                rospy.loginfo(f"Trajectory planning took {planning_time:0.6f} seconds")
                rospy.loginfo(f"Trajectory is {plan_age:0.6f} seconds old")
            self.start_time = now - rospy.Duration.from_sec(self.plan_config.trajectory_lookahead)
        if self.active_trajectory is None:
            rospy.logwarn("Trajectory not generated")
        return self.active_trajectory, did_replan, self.start_time
