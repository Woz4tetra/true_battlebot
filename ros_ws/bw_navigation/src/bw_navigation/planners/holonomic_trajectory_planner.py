from typing import Optional, Tuple

import rospy
from bw_interfaces.msg import EstimatedObject, GoalEngineConfig
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_tools.messages.goal_strategy import GoalStrategy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray

from bw_navigation.planners.engines.config.holonomic_trajectory_planner_config import HolonomicTrajectoryPlannerConfig
from bw_navigation.planners.planner_interface import PlannerInterface
from bw_navigation.planners.shared.goal_progress import GoalProgress
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
        self.controlled_robot_name = controlled_robot_name
        self.friendly_robot_name = friendly_robot_name
        self.avoid_robot_names = avoid_robot_names

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
        markers = []

        return twist, goal_progress, MarkerArray(markers=markers)
