from typing import Optional

import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.in_plane import does_circle_collide_with_path
from bw_shared.geometry.polar import Polar
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from wpimath import geometry
from wpimath.controller import RamseteController
from wpimath.trajectory import Trajectory

from bw_navigation.planners.engines.trajectory_helpers import get_theta, trajectory_to_msg
from bw_navigation.planners.engines.trajectory_planner_engine_config import LocalPlannerEngineConfig, RamseteConfig
from bw_navigation.planners.goal_progress import GoalProgress


class LocalPlannerEngine:
    def __init__(self, config: LocalPlannerEngineConfig, ramsete: RamseteConfig) -> None:
        self.config = config
        self.ramsete_config = ramsete
        self.controller = RamseteController(b=self.ramsete_config.b, zeta=self.ramsete_config.zeta)

        self.start_time = rospy.Time.now()

        self.desired_pose = Pose2D(0.0, 0.0, 0.0)
        self.obstacles: list[Optional[EstimatedObject]] = []

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
            friendly_robot_size = max(robot_state.size.x, robot_state.size.y) + self.config.obstacle_buffer
            if does_circle_collide_with_path(
                friendly_robot_position,
                friendly_robot_size + self.config.obstacle_buffer,
                controlled_robot_position,
                desired_position,
                controlled_robot_size,
            ):
                collision_states.append(robot_state)
        return collision_states

    def _set_obstacles(self, obstacles: list[EstimatedObject]) -> None:
        if len(obstacles) > len(self.obstacles):
            self.obstacles.extend([None] * (len(obstacles) - len(self.obstacles)))
        for index in range(len(self.obstacles)):
            if index >= len(obstacles):
                self.obstacles[index] = None
            else:
                self.obstacles[index] = obstacles[index]

    def _rescale_radii(self, min_radius: float, radii: np.ndarray) -> np.ndarray:
        return 1.0 / (self.config.obstacle_lookahead + min_radius - min_radius) * (radii - min_radius)

    def _select_polar_goal(self, polar_values: list[Polar]) -> Polar:
        angles = np.array([polar.theta for polar in polar_values])
        radii = np.array([polar.radius for polar in polar_values])
        min_radius = np.min(radii)
        weights = self._rescale_radii(min_radius, radii)
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
        self._set_obstacles(collision_states)
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
        polar_goal = self._select_polar_goal(collisions_polar)
        relative_xy_goal = polar_goal.to_xy()
        rerouted_goal_pose = Pose2D(relative_xy_goal.x, relative_xy_goal.y, 0.0).transform_by(controlled_robot_pose)
        return rerouted_goal_pose

    def compute(
        self,
        trajectory: Trajectory,
        controlled_robot_state: EstimatedObject,
        friendly_robot_states: list[EstimatedObject],
    ) -> tuple[Twist, GoalProgress]:
        current_time = rospy.Time.now()
        time_from_start = (current_time - self.start_time).to_sec()
        desired_state = trajectory.sample(time_from_start)
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

        total_time = trajectory.totalTime()
        is_done = time_from_start > total_time

        trajectory_msg = trajectory_to_msg(self.start_time, trajectory)
        return twist, GoalProgress(
            is_done=is_done,
            total_time=total_time,
            time_left=total_time - time_from_start,
            trajectory=trajectory_msg,
        )
