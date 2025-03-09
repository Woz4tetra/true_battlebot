from typing import Optional

import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.in_plane import does_circle_collide_with_path
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from wpimath import geometry
from wpimath.controller import RamseteController
from wpimath.trajectory import Trajectory

from bw_navigation.planners.engines.backaway_from_wall_engine import BackawayFromWallEngine
from bw_navigation.planners.engines.trajectory_helpers import get_theta, trajectory_to_msg
from bw_navigation.planners.engines.trajectory_planner_engine_config import LocalPlannerEngineConfig, RamseteConfig
from bw_navigation.planners.goal_progress import GoalProgress


class LocalPlannerEngine:
    def __init__(
        self, config: LocalPlannerEngineConfig, ramsete: RamseteConfig, backaway_engine: BackawayFromWallEngine
    ) -> None:
        self.config = config
        self.ramsete_config = ramsete
        self.controller = RamseteController(b=self.ramsete_config.b, zeta=self.ramsete_config.zeta)
        self.backaway_engine = backaway_engine

        self.desired_pose = Pose2D(0.0, 0.0, 0.0)
        self.obstacles: list[Optional[EstimatedObject]] = []

    def reset(self) -> None:
        self.desired_pose = Pose2D(0.0, 0.0, 0.0)
        self.obstacles = []

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

    def respond_to_obstacles(
        self,
        controlled_robot_state: EstimatedObject,
        desired_pose: Pose2D,
        friendly_robot_states: list[EstimatedObject],
    ) -> tuple[Pose2D, bool]:
        collision_states = self.get_robot_collisions(controlled_robot_state, desired_pose, friendly_robot_states)
        self._set_obstacles(collision_states)
        if len(collision_states) == 0:
            return desired_pose, False
        else:
            return Pose2D.from_msg(controlled_robot_state.pose.pose), True

    def scale_down_near_goal(self, error: Pose2D, chassis_speeds: Twist) -> Twist:
        dist_magnitude = error.magnitude()
        new_chassis_speeds = Twist()
        if dist_magnitude < self.config.slowdown_distance:
            scale_factor = dist_magnitude / self.config.slowdown_distance
            new_chassis_speeds.linear.x = chassis_speeds.linear.x * scale_factor
        else:
            new_chassis_speeds.linear.x = chassis_speeds.linear.x

        angle_magnitude = abs(error.theta)
        if angle_magnitude < self.config.slowdown_angle:
            scale_factor = angle_magnitude / self.config.slowdown_angle
            rospy.loginfo(f"scale_factor: {scale_factor}")
            new_chassis_speeds.angular.z = chassis_speeds.angular.z * scale_factor
        else:
            new_chassis_speeds.angular.z = chassis_speeds.angular.z

        return new_chassis_speeds

    def compute(
        self,
        trajectory: Trajectory,
        start_time: rospy.Time,
        controlled_robot_state: EstimatedObject,
        friendly_robot_states: list[EstimatedObject],
    ) -> tuple[Twist, GoalProgress]:
        time_from_start = (rospy.Time.now() - start_time).to_sec()
        desired_state = trajectory.sample(time_from_start)
        desired_pose = Pose2D(desired_state.pose.X(), desired_state.pose.Y(), get_theta(desired_state.pose.rotation()))
        robot_pose = Pose2D.from_msg(controlled_robot_state.pose.pose)
        error = desired_pose.relative_to(robot_pose)
        rerouted_pose, was_colliding = self.respond_to_obstacles(
            controlled_robot_state, desired_pose, friendly_robot_states
        )
        self.desired_pose = rerouted_pose

        twist = Twist()
        if not was_colliding:
            desired_state.pose = geometry.Pose2d(rerouted_pose.x, rerouted_pose.y, rerouted_pose.theta)
            current_pose = geometry.Pose2d(robot_pose.x, robot_pose.y, robot_pose.theta)
            chassis_speeds = self.controller.calculate(current_pose, desired_state)
            twist.linear.x = chassis_speeds.vx
            twist.angular.z = chassis_speeds.omega
            twist = self.scale_down_near_goal(error, twist)
        else:
            rospy.logdebug("Backing away from obstacle.")
            twist = self.backaway_engine.compute(desired_pose, robot_pose)

        total_time = trajectory.totalTime()
        is_done = time_from_start > total_time

        trajectory_msg = trajectory_to_msg(start_time, trajectory)
        return twist, GoalProgress(
            is_done=is_done,
            total_time=total_time,
            time_left=total_time - time_from_start,
            trajectory=trajectory_msg,
        )
