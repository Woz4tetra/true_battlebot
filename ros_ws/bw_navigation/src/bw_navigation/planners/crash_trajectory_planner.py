from typing import Tuple

import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.in_plane import nearest_projected_point
from bw_shared.geometry.input_modulus import normalize_angle
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

from bw_navigation.planners.engines.local_planner_engine import LocalPlannerEngine
from bw_navigation.planners.engines.thrash_engine import ThrashEngine
from bw_navigation.planners.engines.trajectory_planner_engine import TrajectoryPlannerEngine
from bw_navigation.planners.engines.trajectory_planner_engine_config import PlannerConfig
from bw_navigation.planners.goal_progress import GoalProgress, compute_feedback_distance
from bw_navigation.planners.planner_interface import PlannerInterface


class CrashTrajectoryPlanner(PlannerInterface):
    def __init__(self, controlled_robot: str, opponent_names: list[str], config: PlannerConfig) -> None:
        self.config = config
        self.traj_config = config.global_planner
        self.controlled_robot = controlled_robot
        self.opponent_names = opponent_names
        self.rotate_180_buffer = self.config.rotate_180_buffer
        self.buffer_xy = XY(self.config.rotate_180_buffer, self.config.rotate_180_buffer)
        self.angle_tolerance = self.config.angle_tolerance
        self.global_planner = TrajectoryPlannerEngine(self.traj_config)
        self.local_planner = LocalPlannerEngine(self.config.local_planner, self.config.ramsete)
        self.thrash_recover_engine = ThrashEngine(self.config.thrash_recovery)
        self.visualization_publisher = rospy.Publisher(
            "trajectory_visualization", MarkerArray, queue_size=1, latch=True
        )
        self.prev_move_time = rospy.Time.now()
        self.prev_move_pose = Pose2D(0.0, 0.0, 0.0)

    def go_to_goal(
        self, dt: float, goal_target: EstimatedObject, robot_states: dict[str, EstimatedObject], field: FieldBounds2D
    ) -> Tuple[Twist, GoalProgress]:
        now = rospy.Time.now()
        if self.controlled_robot not in robot_states:
            rospy.logwarn_throttle(1, f"Robot {self.controlled_robot} not found in robot states")
            return Twist(), GoalProgress(is_done=False)

        markers: list[Marker] = []

        controlled_robot_state = robot_states[self.controlled_robot]
        controlled_robot_pose = Pose2D.from_msg(controlled_robot_state.pose.pose)
        controlled_robot_point = XY(controlled_robot_pose.x, controlled_robot_pose.y)

        goal_pose = Pose2D.from_msg(goal_target.pose.pose)
        goal_point = XY(goal_pose.x, goal_pose.y)

        friendly_robot_states = [
            state
            for name, state in robot_states.items()
            if (name not in self.opponent_names and name != self.controlled_robot)
        ]

        if controlled_robot_pose.relative_to(self.prev_move_pose).magnitude() > self.config.move_threshold:
            self.prev_move_time = now
            self.prev_move_pose = controlled_robot_pose
        did_controlled_robot_move = (now - self.prev_move_time).to_sec() < self.config.move_timeout

        controlled_robot_size = max(
            robot_states[self.controlled_robot].size.x, robot_states[self.controlled_robot].size.y
        )
        controlled_robot_size_half = controlled_robot_size / 2
        buffer = self.buffer_xy + XY(controlled_robot_size_half, controlled_robot_size_half)
        inset_field = (field[0] + buffer, field[1] - buffer)
        nearest_point_on_bounds = nearest_projected_point(controlled_robot_pose, field)
        is_in_bounds = inset_field[0] <= controlled_robot_point <= inset_field[1] or (
            nearest_point_on_bounds is None
            or nearest_point_on_bounds.magnitude(controlled_robot_point)
            > controlled_robot_size_half + self.rotate_180_buffer
        )

        goal_progress = GoalProgress(is_done=False)
        if not did_controlled_robot_move:
            rospy.logdebug(f"Thrashing to recover. {controlled_robot_pose} -> {goal_pose}")
            twist = self.thrash_recover_engine.compute(dt)
        elif is_in_bounds:
            markers.extend(self.local_planner.visualize_local_plan())
            trajectory, did_replan = self.global_planner.compute(controlled_robot_state, goal_target, field)
            if trajectory is None:
                rospy.logwarn("No active trajectory")
                return Twist(), goal_progress
            if did_replan:
                rospy.logdebug(f"Replanned trajectory. {controlled_robot_pose} -> {goal_pose}")
                markers.extend(self.global_planner.visualize_trajectory())
            twist, goal_progress = self.local_planner.compute(trajectory, controlled_robot_state, friendly_robot_states)
        else:
            rospy.logdebug(f"Backing away from wall. {controlled_robot_pose} -> {goal_pose}")
            goal_heading = (goal_point - controlled_robot_point).heading()
            angle_error = normalize_angle(goal_heading - controlled_robot_pose.theta)
            angle_sign = 1 if angle_error > 0 else -1
            linear_sign = 1 if abs(angle_error) < self.angle_tolerance else -1
            twist = Twist()
            twist.linear.x = linear_sign * self.config.backaway_recover.linear_velocity
            twist.angular.z = angle_sign * self.config.backaway_recover.rotate_velocity

        twist.linear.x = max(
            -1 * self.traj_config.max_velocity,
            min(self.traj_config.max_velocity, twist.linear.x),
        )
        twist.angular.z = max(
            -1 * self.traj_config.max_angular_velocity,
            min(self.traj_config.max_angular_velocity, twist.angular.z),
        )

        goal_progress.distance_to_goal = compute_feedback_distance(controlled_robot_state, goal_target)

        self.visualization_publisher.publish(MarkerArray(markers=markers))
        return twist, goal_progress
