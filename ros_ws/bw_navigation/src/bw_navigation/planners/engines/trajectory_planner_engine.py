import random
from dataclasses import dataclass
from queue import Queue
from threading import Thread
from typing import Optional

import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObject
from bw_interfaces.msg import Trajectory as TrajectoryMsg
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.in_plane import does_circle_collide_with_path, line_bounds_intersection, pose_to_line
from bw_shared.geometry.polar import Polar
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.twist2d import Twist2D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from visualization_msgs.msg import Marker
from wpimath import geometry
from wpimath.controller import RamseteController
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.trajectory.constraint import CentripetalAccelerationConstraint, DifferentialDriveKinematicsConstraint

from bw_navigation.planners.engines.trajectory_planner_engine_config import PathPlannerConfig
from bw_navigation.planners.goal_progress import GoalProgress


@dataclass
class PlanningTicket:
    robot_pose: Pose2D
    robot_velocity: Twist2D
    goal_pose: Pose2D
    goal_velocity: Twist2D
    field: FieldBounds2D


@dataclass
class PlanningResult:
    trajectory: Trajectory
    planning_start_time: rospy.Time
    planning_finish_time: rospy.Time


def get_theta(rotation: geometry.Rotation2d) -> float:
    return float(rotation.radians())  # type: ignore


def trajectory_to_msg(start_time: rospy.Time, trajectory: Trajectory) -> TrajectoryMsg:
    msg = TrajectoryMsg()
    msg.header.stamp = start_time
    msg.header.frame_id = "map"
    for state in trajectory.states():
        pose = PoseStamped()
        pose.header.stamp = start_time + rospy.Duration.from_sec(state.t)
        pose.pose = Pose2D(x=state.pose.X(), y=state.pose.Y(), theta=get_theta(state.pose.rotation())).to_msg()
        msg.poses.append(pose)

        twist = TwistStamped()
        twist.header.stamp = start_time + rospy.Duration.from_sec(state.t)
        twist.twist = Twist2D(x=state.velocity, y=0.0, theta=state.velocity * state.curvature).to_msg()
        msg.twists.append(twist)
    return msg


class TrajectoryPlannerEngine:
    def __init__(self, config: PathPlannerConfig) -> None:
        self.plan_config = config
        self.engine_config = config.trajectory_planner_engine
        self.traj_config = TrajectoryConfig(
            maxVelocity=self.plan_config.max_velocity, maxAcceleration=self.plan_config.max_acceleration
        )
        self.kinematics = DifferentialDriveKinematics(self.plan_config.track_width)

        self.traj_config.setStartVelocity(self.plan_config.max_velocity)
        self.traj_config.setEndVelocity(self.plan_config.max_velocity)
        if self.plan_config.max_centripetal_acceleration is not None:
            self.traj_config.addConstraint(
                CentripetalAccelerationConstraint(
                    maxCentripetalAcceleration=self.plan_config.max_centripetal_acceleration
                ),
            )
        self.traj_config.addConstraint(
            DifferentialDriveKinematicsConstraint(self.kinematics, self.plan_config.max_velocity),
        )

        self.controller = RamseteController(b=self.plan_config.ramsete.b, zeta=self.plan_config.ramsete.zeta)

        self.start_time = rospy.Time.now()

        self.planning_thread = Thread(daemon=True, target=self.planning_task)
        self.goal_in_queue: Queue[PlanningTicket] = Queue()
        self.plan_out_queue: Queue[PlanningResult] = Queue()
        self.is_planning = False
        self.active_trajectory: Optional[Trajectory] = None
        self.active_trajectory_msg = TrajectoryMsg()
        self.desired_pose = Pose2D(0.0, 0.0, 0.0)
        self.obstacles: list[Optional[EstimatedObject]] = []

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
        distance_to_goal = goal_pose.relative_to(robot_pose).magnitude() * self.plan_config.prediction_magnification
        total_time = distance_to_goal / self.plan_config.max_velocity
        travel_distance = ticket.goal_velocity.x * total_time
        projected_goal = Pose2D(travel_distance, 0.0, 0.0).transform_by(goal_pose)
        projected_goal = self._bound_pose_along_line(goal_pose, projected_goal, ticket.field)
        trajectory = self._plan_once(robot_pose, projected_goal)
        return trajectory

    def _plan_once(self, robot_pose: Pose2D, goal_pose: Pose2D) -> Trajectory:
        trajectory = None
        original_goal_pose = goal_pose
        noise = self.engine_config.planning_failure_random_noise
        while trajectory is None:
            try:
                trajectory = TrajectoryGenerator.generateTrajectory(
                    waypoints=[
                        geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta),
                        geometry.Pose2d(goal_pose.x, goal_pose.y, goal_pose.theta),
                    ],
                    config=self.traj_config,
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

    def planning_task(self) -> None:
        while True:
            try:
                ticket = self.goal_in_queue.get()

                if self.engine_config.used_measured_velocity:
                    robot_velocity = ticket.robot_velocity
                    self.traj_config.setStartVelocity(robot_velocity.x)
                self.is_planning = True
                planning_start_time = rospy.Time.now()
                if self.engine_config.forward_project_goal_velocity:
                    trajectory = self._plan_with_goal_velocity(ticket)
                else:
                    trajectory = self._plan_once(ticket.robot_pose, ticket.goal_pose)
                planning_finish_time = rospy.Time.now()
                self.plan_out_queue.put(PlanningResult(trajectory, planning_start_time, planning_finish_time))
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

    def make_trajectory_markers(self, num_samples: int) -> list[Marker]:
        markers = []
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
            friendly_robot_size = max(obstacle.size.x, obstacle.size.y) + self.plan_config.obstacle_buffer
            marker.scale.x = friendly_robot_size
            marker.scale.y = friendly_robot_size
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            markers.append(marker)
        return markers

    def visualize_trajectory(self, num_samples: int = 10) -> list[Marker]:
        return self.make_trajectory_markers(num_samples)

    def visualize_local_plan(self) -> list[Marker]:
        markers = [self.make_desired_pose_marker()]
        markers.extend(self.make_obstacle_markers())
        return markers

    def should_replan(self) -> bool:
        if self.active_trajectory is None:
            return True
        trajectory_duration = rospy.Time.now() - self.start_time
        return trajectory_duration.to_sec() > self.active_trajectory.totalTime()

    def get_robot_collisions(
        self,
        controlled_robot_state: EstimatedObject,
        desired_pose: Pose2D,
        friendly_robot_states: list[EstimatedObject],
    ) -> list[EstimatedObject]:
        controlled_robot_position = XY(
            controlled_robot_state.pose.pose.position.x, controlled_robot_state.pose.pose.position.y
        )
        controlled_robot_size = max(controlled_robot_state.size.x, controlled_robot_state.size.y)
        desired_position = XY(desired_pose.x, desired_pose.y)
        collision_states = []
        for robot_state in friendly_robot_states:
            friendly_robot_position = XY(robot_state.pose.pose.position.x, robot_state.pose.pose.position.y)
            friendly_robot_size = max(robot_state.size.x, robot_state.size.y) + self.plan_config.obstacle_buffer
            if does_circle_collide_with_path(
                friendly_robot_position,
                friendly_robot_size + self.plan_config.obstacle_buffer,
                controlled_robot_position,
                desired_position,
                controlled_robot_size,
            ):
                collision_states.append(robot_state)
        return collision_states

    def set_obstacles(self, obstacles: list[EstimatedObject]) -> None:
        if len(obstacles) > len(self.obstacles):
            self.obstacles.extend([None] * (len(obstacles) - len(self.obstacles)))
        for index in range(len(self.obstacles)):
            if index >= len(obstacles):
                self.obstacles[index] = None
            else:
                self.obstacles[index] = obstacles[index]

    def rescale_radii(self, min_radius: float, radii: np.ndarray) -> np.ndarray:
        return 1.0 / (self.plan_config.obstacle_lookahead + min_radius - min_radius) * (radii - min_radius)

    def select_polar_goal(self, polar_values: list[Polar]) -> Polar:
        angles = np.array([polar.theta for polar in polar_values])
        radii = np.array([polar.radius for polar in polar_values])
        min_radius = np.min(radii)
        weights = self.rescale_radii(min_radius, radii)
        if np.sum(weights) == 0:
            weights = np.ones_like(weights)
        average_angle = float(np.average(angles, weights=weights))
        return Polar(min_radius, average_angle)

    def route_around_obstacles(
        self,
        controlled_robot_state: EstimatedObject,
        desired_pose: Pose2D,
        friendly_robot_states: list[EstimatedObject],
    ) -> Pose2D:
        collision_states = self.get_robot_collisions(controlled_robot_state, desired_pose, friendly_robot_states)
        self.set_obstacles(collision_states)
        if len(collision_states) == 0:
            return desired_pose
        collision_poses = [Pose2D.from_msg(robot.pose.pose) for robot in collision_states]
        controlled_robot_pose = Pose2D.from_msg(controlled_robot_state.pose.pose)
        collisions_polar = [
            Polar.from_xy(collision_pose.relative_to(controlled_robot_pose)) for collision_pose in collision_poses
        ]
        if len(collisions_polar) == 1:
            only_collision = collisions_polar[0]
            angle_adjustment = np.pi / 2 if only_collision.theta < 0 else np.pi / 2
            collisions_polar.append(Polar(only_collision.radius, only_collision.theta + angle_adjustment))
        polar_goal = self.select_polar_goal(collisions_polar)
        relative_xy_goal = polar_goal.to_xy()
        rerouted_goal_pose = Pose2D(relative_xy_goal.x, relative_xy_goal.y, 0.0).transform_by(controlled_robot_pose)
        return rerouted_goal_pose

    def compute(
        self, controlled_robot_state: EstimatedObject, friendly_robot_states: list[EstimatedObject]
    ) -> tuple[Twist, GoalProgress]:
        if not self.plan_out_queue.empty():
            while not self.plan_out_queue.empty():
                result = self.plan_out_queue.get()
                self.active_trajectory = result.trajectory
                planning_time = (result.planning_finish_time - result.planning_start_time).to_sec()
                plan_age = (rospy.Time.now() - result.planning_start_time).to_sec()
                rospy.loginfo(f"Trajectory planning took {planning_time:0.6f} seconds")
                rospy.loginfo(f"Trajectory is {plan_age:0.6f} seconds old")
            if self.active_trajectory is not None:
                self.active_trajectory_msg = trajectory_to_msg(self.start_time, self.active_trajectory)
            self.start_time = rospy.Time.now() - rospy.Duration.from_sec(self.engine_config.trajectory_lookahead)
        if self.active_trajectory is None:
            rospy.logwarn("Trajectory not generated")
            return Twist(), GoalProgress(is_done=False)
        current_time = rospy.Time.now()
        time_from_start = (current_time - self.start_time).to_sec()
        desired_state = self.active_trajectory.sample(time_from_start)
        desired_pose = Pose2D(desired_state.pose.X(), desired_state.pose.Y(), get_theta(desired_state.pose.rotation()))
        rerouted_pose = self.route_around_obstacles(controlled_robot_state, desired_pose, friendly_robot_states)
        self.desired_pose = rerouted_pose

        desired_state.pose = geometry.Pose2d(rerouted_pose.x, rerouted_pose.y, rerouted_pose.theta)
        robot_pose = Pose2D.from_msg(controlled_robot_state.pose.pose)
        current_pose = geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta)
        chassis_speeds = self.controller.calculate(current_pose, desired_state)

        twist = Twist()
        twist.linear.x = chassis_speeds.vx
        twist.angular.z = chassis_speeds.omega

        total_time = self.active_trajectory.totalTime()
        is_done = time_from_start > total_time
        return twist, GoalProgress(
            is_done=is_done,
            total_time=total_time,
            time_left=total_time - time_from_start,
            trajectory=self.active_trajectory_msg,
        )
