import math
from typing import Optional, Tuple

import rospy
from bw_interfaces.msg import EstimatedObject, GoalEngineConfig
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.input_modulus import normalize_angle
from bw_shared.geometry.xy import XY
from bw_shared.pid.pid import PID
from bw_tools.messages.goal_strategy import GoalStrategy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

from bw_navigation.planners.engines.backaway_from_wall_engine import BackawayFromWallEngine
from bw_navigation.planners.engines.config.pid_planner_config import PidPlannerConfig
from bw_navigation.planners.engines.did_robot_move_engine import DidRobotMoveEngine
from bw_navigation.planners.engines.recover_from_danger_engine import RecoverFromDangerEngine
from bw_navigation.planners.engines.thrash_engine import ThrashEngine
from bw_navigation.planners.planner_interface import PlannerInterface
from bw_navigation.planners.shared.clamp_twist_with_config import clamp_twist_with_config
from bw_navigation.planners.shared.compute_mirrored_goal import (
    compute_mirrored_state,
    overlay_friendly_twist,
)
from bw_navigation.planners.shared.goal_progress import GoalProgress, compute_feedback_distance
from bw_navigation.planners.shared.is_in_bounds import is_controlled_bot_in_bounds, is_goal_in_bounds
from bw_navigation.planners.shared.marker_utils import make_pose_marker, make_text_marker
from bw_navigation.planners.shared.match_state import MatchState


class PidPlanner(PlannerInterface):
    def __init__(
        self,
        controlled_robot_name: str,
        friendly_robot_name: str,
        avoid_robot_names: list[str],
        config: PidPlannerConfig,
    ) -> None:
        self.config = config
        self.default_velocity_limits = GoalEngineConfig(
            max_velocity=self.config.max_velocity,
            max_angular_velocity=self.config.max_angular_velocity,
        )
        self.controlled_robot_name = controlled_robot_name
        self.friendly_robot_name = friendly_robot_name
        self.avoid_robot_names = avoid_robot_names
        self.buffer_xy = XY(self.config.in_bounds_buffer, self.config.in_bounds_buffer)
        self.backaway_engine = BackawayFromWallEngine(self.config.backaway_recover)
        self.thrash_recover_engine = ThrashEngine(self.config.thrash_recovery)
        self.did_robot_move_engine = DidRobotMoveEngine(self.config.did_move)
        self.recover_from_engine = RecoverFromDangerEngine(self.config.recover_from_danger)
        self.linear_pid = PID(self.config.linear_pid)
        self.angular_pid = PID(self.config.angular_pid)

    def reset(self) -> None:
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.thrash_recover_engine.reset()
        self.did_robot_move_engine.reset()

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
        if goal_strategy == GoalStrategy.MIRROR_FRIENDLY:
            mirrored_match_state = compute_mirrored_state(match_state)
            if is_goal_in_bounds(self.buffer_xy, mirrored_match_state):
                match_state = mirrored_match_state

        goal_progress = GoalProgress(is_done=False)
        twist = Twist()
        markers: list[Marker] = []

        is_in_tolerance = match_state.distance_to_goal < xy_tolerance
        did_move = self.did_robot_move_engine.did_move(match_state.controlled_robot_pose_stamped)

        if not did_move and not is_in_tolerance:
            twist = self.thrash_recover_engine.compute(dt)
            markers.append(make_text_marker("Thrash recovery", match_state.controlled_robot_pose.to_msg(), "status"))
        elif avoid_danger_twist := self.recover_from_engine.compute_recovery_command(match_state):
            twist = avoid_danger_twist
            markers.append(make_text_marker("Danger recovery", match_state.controlled_robot_pose.to_msg(), "status"))
        elif is_controlled_bot_in_bounds(self.buffer_xy, self.config.backaway_recover.angle_tolerance, match_state):
            twist = self.compute_twist(dt, match_state)
            twist, input_factor, friendly_factor = overlay_friendly_twist(
                twist, match_state, self.config.overlay_velocity
            )
            status_msg = f"Go to goal. in:{int(input_factor * 100)} fr:{int(friendly_factor * 100)}"
            markers.append(make_pose_marker(match_state.goal_pose.to_msg(), "goal"))
            markers.append(make_text_marker(status_msg, match_state.controlled_robot_pose.to_msg(), "status"))
        else:
            twist = self.backaway_engine.compute(match_state.goal_pose, match_state.controlled_robot_pose)
            markers.append(make_text_marker("Backaway recovery", match_state.controlled_robot_pose.to_msg(), "status"))

        markers.extend(self.recover_from_engine.get_markers())

        twist = clamp_twist_with_config(twist, engine_config, self.default_velocity_limits)
        goal_progress.distance_to_goal = compute_feedback_distance(match_state.controlled_robot, goal_target)

        return twist, goal_progress, MarkerArray(markers=markers)

    def compute_twist(self, dt: float, match_state: MatchState) -> Twist:
        relative_goal = match_state.relative_goal
        relative_xy = XY(relative_goal.x, relative_goal.y)
        distance_to_goal = relative_xy.magnitude()
        is_goal_behind = relative_xy.x < 0.0
        if distance_to_goal > self.config.match_angle_distance_threshold:
            heading = relative_xy.heading()
            if is_goal_behind:
                heading += math.pi
            heading = normalize_angle(heading)
            angular_velocity = self.angular_pid.update(heading, 0.0, dt)
        else:
            angular_velocity = self.angular_pid.update(relative_goal.theta, 0.0, dt)
        linear_velocity = self.linear_pid.update(relative_goal.x, 0.0, dt)

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        return twist
