from typing import Optional, Tuple

import rospy
from bw_interfaces.msg import EstimatedObject, GoalEngineConfig
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.xy import XY
from bw_tools.messages.goal_strategy import GoalStrategy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from wpimath import geometry
from wpimath.trajectory import Trajectory

from bw_navigation.planners.engines.config.holonomic_trajectory_planner_config import HolonomicTrajectoryPlannerConfig
from bw_navigation.planners.engines.holonomic_local_planner_engine import HolonomicLocalPlannerEngine
from bw_navigation.planners.engines.holonomic_trajectory_planner_engine import HolonomicTrajectoryPlannerEngine
from bw_navigation.planners.engines.trajectory_helpers import trajectory_to_msg
from bw_navigation.planners.planner_interface import PlannerInterface
from bw_navigation.planners.shared.compute_mirrored_goal import compute_mirrored_state
from bw_navigation.planners.shared.goal_progress import GoalProgress
from bw_navigation.planners.shared.is_in_bounds import is_goal_in_bounds
from bw_navigation.planners.shared.match_state import MatchState


class HolonomicTrajectoryPlanner(PlannerInterface):
    def __init__(
        self,
        controlled_robot_name: str,
        friendly_robot_name: str,
        avoid_robot_names: list[str],
        config: HolonomicTrajectoryPlannerConfig,
    ) -> None:
        self.config = config
        self.traj_config = config.global_planner
        self.default_velocity_limits = GoalEngineConfig(
            max_velocity=self.traj_config.max_velocity,
            max_angular_velocity=self.traj_config.max_angular_velocity,
        )
        self.controlled_robot_name = controlled_robot_name
        self.friendly_robot_name = friendly_robot_name
        self.avoid_robot_names = avoid_robot_names

        self.buffer_xy = XY(self.config.in_bounds_buffer, self.config.in_bounds_buffer)
        self.global_planner = HolonomicTrajectoryPlannerEngine(self.traj_config)
        self.local_planner = HolonomicLocalPlannerEngine(self.config.local_planner, self.traj_config)

    def reset(self) -> None:
        pass

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

        twist = Twist()
        goal_progress = GoalProgress(is_done=False)
        markers: list[Marker] = []
        if match_state.distance_to_goal < xy_tolerance:
            rospy.loginfo("Goal is within xy tolerance, stopping robot.")
            goal_progress.is_done = True
            return twist, goal_progress, MarkerArray(markers=markers)

        controlled_robot = match_state.controlled_robot
        goal_target = match_state.goal_target
        trajectory, did_replan, start_time = self.global_planner.compute(
            controlled_robot, goal_target, field, engine_config
        )
        if trajectory is None:
            rospy.logwarn("No active trajectory")
            return Twist(), goal_progress, MarkerArray(markers=markers)
        if did_replan:
            rospy.logdebug("Replanned trajectory.")
            markers.extend(self.global_planner.visualize_trajectory())

        time_from_start = (rospy.Time.now() - start_time).to_sec()
        trajectory_msg = trajectory_to_msg(start_time, trajectory)
        total_time = trajectory.totalTime()
        goal_progress.is_done = time_from_start > total_time
        goal_progress.time_left = total_time - time_from_start
        goal_progress.total_time = total_time
        goal_progress.trajectory = trajectory_msg

        # desired_state = trajectory.sample(time_from_start)
        desired_state = Trajectory.State(
            pose=geometry.Pose2d(
                match_state.goal_pose.x,
                match_state.goal_pose.y,
                match_state.goal_pose.theta,
            )
        )
        twist = self.local_planner.compute(desired_state, match_state.controlled_robot_pose)

        return twist, goal_progress, MarkerArray(markers=markers)
