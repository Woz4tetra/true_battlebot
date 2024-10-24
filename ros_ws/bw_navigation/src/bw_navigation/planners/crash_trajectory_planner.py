import math
from typing import Optional, Tuple

import rospy
from bw_interfaces.msg import EstimatedObject, GoalEngineConfig
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

from bw_navigation.planners.engines.backaway_from_wall_engine import BackawayFromWallEngine
from bw_navigation.planners.engines.local_planner_engine import LocalPlannerEngine
from bw_navigation.planners.engines.rotate_to_angle_engine import RotateToAngleEngine
from bw_navigation.planners.engines.thrash_engine import ThrashEngine
from bw_navigation.planners.engines.trajectory_planner_engine import TrajectoryPlannerEngine
from bw_navigation.planners.engines.trajectory_planner_engine_config import PlannerConfig
from bw_navigation.planners.goal_progress import GoalProgress, compute_feedback_distance
from bw_navigation.planners.match_state import MatchState
from bw_navigation.planners.planner_interface import PlannerInterface


class CrashTrajectoryPlanner(PlannerInterface):
    def __init__(self, controlled_robot: str, avoid_robot_names: list[str], config: PlannerConfig) -> None:
        self.config = config
        self.traj_config = config.global_planner
        self.controlled_robot = controlled_robot
        self.avoid_robot_names = avoid_robot_names
        self.rotate_180_buffer = self.config.rotate_180_buffer
        self.buffer_xy = XY(self.config.rotate_180_buffer, self.config.rotate_180_buffer)
        self.backaway_engine = BackawayFromWallEngine(self.config.backaway_recover)
        self.global_planner = TrajectoryPlannerEngine(self.traj_config)
        self.local_planner = LocalPlannerEngine(self.config.local_planner, self.config.ramsete, self.backaway_engine)
        self.rotate_to_angle_engine = RotateToAngleEngine(self.config.rotate_to_angle.pid)
        self.thrash_recover_engine = ThrashEngine(self.config.thrash_recovery)
        self.visualization_publisher = rospy.Publisher(
            "trajectory_visualization", MarkerArray, queue_size=1, latch=True
        )
        self.trajectory_complete_time = None
        self.prev_move_time = rospy.Time.now()
        self.prev_positions: list[tuple[float, XY]] = []
        self.prev_position_time_window = rospy.Duration.from_sec(self.config.move_timeout)

    def reset(self) -> None:
        self.global_planner.reset()
        self.local_planner.reset()
        self.thrash_recover_engine.reset()
        self.prev_move_time = rospy.Time.now()
        self.prev_positions = []
        self.trajectory_complete_time = None

    def go_to_goal(
        self,
        dt: float,
        goal_target: EstimatedObject,
        robot_states: dict[str, EstimatedObject],
        field: FieldBounds2D,
        engine_config: Optional[GoalEngineConfig],
        xy_tolerance: float,
    ) -> Tuple[Twist, GoalProgress]:
        now = rospy.Time.now()
        if self.controlled_robot not in robot_states:
            rospy.logwarn_throttle(1, f"Robot {self.controlled_robot} not found in robot states")
            return Twist(), GoalProgress(is_done=False)

        match_state = MatchState(
            goal_target=goal_target,
            robot_states=robot_states,
            field_bounds=field,
            controlled_robot_name=self.controlled_robot,
            avoid_robot_names=self.avoid_robot_names,
        )

        markers: list[Marker] = []
        controlled_robot = match_state.controlled_robot
        rotate_at_end = False if engine_config is None else engine_config.rotate_at_end
        goal_progress = GoalProgress(is_done=False)
        twist = Twist()

        if self.trajectory_complete_time is not None and rotate_at_end:
            rospy.logdebug("Rotating at end.")
            if now - self.trajectory_complete_time < rospy.Duration.from_sec(self.config.rotate_to_angle.timeout):
                twist = self.rotate_to_angle_engine.compute(
                    dt, match_state.controlled_robot_pose.theta, match_state.goal_pose.theta
                )
            else:
                goal_progress.is_done = True
        elif not self.did_controlled_robot_move(now, match_state):
            rospy.logdebug("Thrashing to recover")
            twist = self.thrash_recover_engine.compute(dt)
        elif self.is_controlled_robot_in_danger(match_state):
            rospy.logdebug("In danger recovery.")
            target_relative_to_controlled_bot = match_state.goal_pose.relative_to(match_state.controlled_robot_pose)
            twist = Twist()
            twist.linear.x = self.config.in_danger_recovery.linear_magnitude
            twist.angular.z = math.copysign(
                self.config.in_danger_recovery.angular_magnitude,
                -1 * target_relative_to_controlled_bot.y,
            )
        elif self.is_in_bounds(match_state):
            markers.extend(self.local_planner.visualize_local_plan())
            trajectory, did_replan, start_time = self.global_planner.compute(
                controlled_robot, goal_target, field, engine_config
            )
            if trajectory is None:
                rospy.logwarn("No active trajectory")
                return Twist(), goal_progress
            if did_replan:
                rospy.logdebug("Replanned trajectory.")
                markers.extend(self.global_planner.visualize_trajectory())
            twist, goal_progress = self.local_planner.compute(
                trajectory, start_time, controlled_robot, match_state.friendly_robot_states
            )
            if goal_progress.is_done and match_state.distance_to_goal < xy_tolerance:
                self.trajectory_complete_time = now
                goal_progress.is_done = not rotate_at_end
        else:
            rospy.logdebug("Backing away from wall.")
            twist = self.backaway_engine.compute(match_state.goal_pose, match_state.controlled_robot_pose)

        if engine_config:
            max_velocity = engine_config.max_velocity
            max_angular_velocity = engine_config.max_angular_velocity
        else:
            max_velocity = self.traj_config.max_velocity
            max_angular_velocity = self.traj_config.max_angular_velocity
        twist.linear.x = max(-1 * max_velocity, min(max_velocity, twist.linear.x))
        twist.angular.z = max(-1 * max_angular_velocity, min(max_angular_velocity, twist.angular.z))

        goal_progress.distance_to_goal = compute_feedback_distance(controlled_robot, goal_target)

        self.visualization_publisher.publish(MarkerArray(markers=markers))
        return twist, goal_progress

    def did_controlled_robot_move(self, now: rospy.Time, match_state: MatchState) -> bool:
        self.prev_positions.append((now, match_state.controlled_robot_point))
        while self.prev_positions[-1][0] - self.prev_positions[0][0] > self.prev_position_time_window:
            self.prev_positions.pop(0)
        earliest_position = self.prev_positions[0][1]
        sum_position = XY(0.0, 0.0)
        for timestamp, position in self.prev_positions[1:]:
            sum_position += position - earliest_position
        average_position = XY(sum_position.x / len(self.prev_positions), sum_position.y / len(self.prev_positions))
        if average_position.magnitude() > self.config.move_threshold:
            self.prev_move_time = now
        return (now - self.prev_move_time).to_sec() < self.config.move_timeout

    def is_in_bounds(self, match_state: MatchState) -> bool:
        field = match_state.field_bounds
        controlled_robot_point = match_state.controlled_robot_point
        controlled_robot_size = max(match_state.controlled_robot.size.x, match_state.controlled_robot.size.y)
        controlled_robot_size_half = controlled_robot_size / 2
        buffer = self.buffer_xy + XY(controlled_robot_size_half, controlled_robot_size_half)
        inset_field = (field[0] + buffer, field[1] - buffer)
        goal_relative_to_controlled_bot = match_state.goal_pose.relative_to(match_state.controlled_robot_pose)
        goal_heading = goal_relative_to_controlled_bot.heading()
        return inset_field[0] <= controlled_robot_point <= inset_field[1] or (
            abs(goal_heading) < self.config.backaway_recover.angle_tolerance
        )

    def is_controlled_robot_in_danger(self, match_state: MatchState) -> bool:
        for opponent_state in match_state.opponent_robot_states:
            opponent_pose = Pose2D.from_msg(opponent_state.pose.pose)
            controlled_bot_relative_to_opponent = match_state.controlled_robot_pose.relative_to(opponent_pose)
            opponent_width = max(opponent_state.size.x, opponent_state.size.y)
            combined_width = match_state.controlled_robot_width + opponent_width
            magnitude_lower_bound = combined_width * self.config.in_danger_recovery.size_multiplier
            if (
                abs(controlled_bot_relative_to_opponent.heading()) < self.config.in_danger_recovery.angle_tolerance
                and (
                    magnitude_lower_bound
                    < controlled_bot_relative_to_opponent.magnitude()
                    < self.config.in_danger_recovery.linear_tolerance
                )
                and opponent_state.twist.twist.linear.x > self.config.in_danger_recovery.velocity_threshold
            ):
                return True
        return False
