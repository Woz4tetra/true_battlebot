from typing import Optional, Tuple

import rospy
from bw_interfaces.msg import EstimatedObject, GoalEngineConfig
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.xy import XY
from bw_shared.pid.pid import PID
from bw_tools.messages.goal_strategy import GoalStrategy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

from bw_navigation.planners.engines.backaway_from_wall_engine import BackawayFromWallEngine
from bw_navigation.planners.engines.config.trajectory_planner_config import (
    TrajectoryPlannerConfig,
)
from bw_navigation.planners.engines.did_robot_move_engine import DidRobotMoveEngine
from bw_navigation.planners.engines.local_planner_engine import LocalPlannerEngine
from bw_navigation.planners.engines.recover_from_danger_engine import RecoverFromDangerEngine
from bw_navigation.planners.engines.rotate_to_angle_engine import RotateToAngleEngine
from bw_navigation.planners.engines.thrash_engine import ThrashEngine
from bw_navigation.planners.engines.trajectory_planner_engine import TrajectoryPlannerEngine
from bw_navigation.planners.planner_interface import PlannerInterface
from bw_navigation.planners.shared.clamp_twist_with_config import clamp_twist_with_config
from bw_navigation.planners.shared.compute_mirrored_goal import (
    compute_mirrored_state,
    overlay_friendly_twist,
)
from bw_navigation.planners.shared.goal_progress import GoalProgress, compute_feedback_distance
from bw_navigation.planners.shared.is_in_bounds import is_controlled_bot_in_bounds
from bw_navigation.planners.shared.match_state import MatchState


class TrajectoryPlanner(PlannerInterface):
    def __init__(
        self,
        controlled_robot_name: str,
        friendly_robot_name: str,
        avoid_robot_names: list[str],
        config: TrajectoryPlannerConfig,
    ) -> None:
        self.config = config
        self.traj_config = config.global_planner
        self.traj_velocity_limits = GoalEngineConfig(
            max_velocity=self.traj_config.max_velocity,
            max_angular_velocity=self.traj_config.max_angular_velocity,
        )
        self.controlled_robot_name = controlled_robot_name
        self.friendly_robot_name = friendly_robot_name
        self.avoid_robot_names = avoid_robot_names
        self.buffer_xy = XY(self.config.in_bounds_buffer, self.config.in_bounds_buffer)
        self.backaway_engine = BackawayFromWallEngine(self.config.backaway_recover)
        self.global_planner = TrajectoryPlannerEngine(self.traj_config)
        self.local_planner = LocalPlannerEngine(self.config.local_planner, self.config.ramsete, self.backaway_engine)
        self.rotate_to_angle_engine = RotateToAngleEngine(self.config.rotate_to_angle.pid)
        self.thrash_recover_engine = ThrashEngine(self.config.thrash_recovery)
        self.did_robot_move_engine = DidRobotMoveEngine(self.config.did_move)
        self.recover_from_engine = RecoverFromDangerEngine(self.config.recover_from_danger)
        self.near_goal_linear_pid = PID(self.config.near_goal.linear_pid)
        self.near_goal_angular_pid = PID(self.config.near_goal.angular_pid)
        self.trajectory_complete_time = None

    def reset(self) -> None:
        self.global_planner.reset()
        self.local_planner.reset()
        self.thrash_recover_engine.reset()
        self.did_robot_move_engine.reset()
        self.trajectory_complete_time = None

    def go_to_goal(
        self,
        dt: float,
        goal_target: EstimatedObject,
        robot_states: dict[str, EstimatedObject],
        field: FieldBounds2D,
        engine_config: Optional[GoalEngineConfig],
        xy_tolerance: float,
        goal_strategy: GoalStrategy,
    ) -> Tuple[Twist, GoalProgress, MarkerArray]:
        now = rospy.Time.now()
        if self.controlled_robot_name not in robot_states:
            rospy.logwarn_throttle(1, f"Robot {self.controlled_robot_name} not found in robot states")
            return Twist(), GoalProgress(is_done=False), MarkerArray()

        match_state = MatchState(
            goal_target=goal_target,
            robot_states=robot_states,
            field_bounds=field,
            controlled_robot_name=self.controlled_robot_name,
            friendly_robot_name=self.friendly_robot_name,
            avoid_robot_names=self.avoid_robot_names,
        )
        twist = Twist()
        goal_progress = GoalProgress(is_done=False)

        if goal_strategy == GoalStrategy.MIRROR_FRIENDLY:
            match_state = compute_mirrored_state(match_state)

            if match_state.distance_to_goal < self.config.overlay_velocity.friendly_mirror_proximity:
                relative_goal = match_state.goal_pose.relative_to(match_state.controlled_robot_pose)
                twist.linear.x = self.near_goal_linear_pid.update(relative_goal.x, 0.0, dt)
                twist.angular.z = self.near_goal_angular_pid.update(relative_goal.theta, 0.0, dt)
                twist, input_factor, friendly_factor = overlay_friendly_twist(
                    twist, match_state, self.config.overlay_velocity
                )
                return twist, goal_progress, MarkerArray()

        markers: list[Marker] = []
        controlled_robot = match_state.controlled_robot
        goal_target = match_state.goal_target
        rotate_at_end = False if engine_config is None else engine_config.rotate_at_end

        if self.trajectory_complete_time is not None and rotate_at_end:
            rospy.logdebug("Rotating at end.")
            if now - self.trajectory_complete_time < rospy.Duration.from_sec(self.config.rotate_to_angle.timeout):
                twist = self.rotate_to_angle_engine.compute(
                    dt, match_state.controlled_robot_pose.theta, match_state.goal_pose.theta
                )
            else:
                goal_progress.is_done = True
        # elif not self.did_robot_move_engine.did_move(match_state.controlled_robot_pose_stamped):
        #     rospy.logdebug("Thrashing to recover")
        #     twist = self.thrash_recover_engine.compute(dt)
        # elif avoid_danger_twist := self.recover_from_engine.compute_recovery_command(match_state):
        #     rospy.logdebug("In danger recovery.")
        #     twist = avoid_danger_twist
        elif is_controlled_bot_in_bounds(self.buffer_xy, self.config.backaway_recover.angle_tolerance, match_state):
            markers.extend(self.local_planner.visualize_local_plan())
            trajectory, did_replan, start_time = self.global_planner.compute(
                controlled_robot, goal_target, field, engine_config
            )
            if trajectory is None:
                rospy.logwarn("No active trajectory")
                return Twist(), goal_progress, MarkerArray(markers=markers)
            if did_replan:
                rospy.logdebug("Replanned trajectory.")
                markers.extend(self.global_planner.visualize_trajectory())
            twist, goal_progress = self.local_planner.compute(
                trajectory, start_time, controlled_robot, match_state.avoid_robot_states
            )
            if goal_progress.is_done and match_state.distance_to_goal < xy_tolerance:
                self.trajectory_complete_time = now
                goal_progress.is_done = not rotate_at_end
        else:
            rospy.logdebug("Backing away from wall.")
            twist = self.backaway_engine.compute(match_state.goal_pose, match_state.controlled_robot_pose)

        twist = clamp_twist_with_config(twist, engine_config, self.traj_velocity_limits)
        goal_progress.distance_to_goal = compute_feedback_distance(controlled_robot, goal_target)

        return twist, goal_progress, MarkerArray(markers=markers)
