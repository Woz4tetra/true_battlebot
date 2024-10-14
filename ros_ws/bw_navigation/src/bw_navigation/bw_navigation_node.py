#!/usr/bin/env python

from typing import Dict

import rospy
from actionlib import SimpleActionServer
from bw_interfaces.msg import (
    EstimatedObject,
    EstimatedObjectArray,
    GoToGoalAction,
    GoToGoalFeedback,
    GoToGoalGoal,
    GoToGoalResult,
    RobotFleetConfigMsg,
)
from bw_shared.configs.robot_fleet_config import RobotConfig, RobotFleetConfig
from bw_shared.enums.robot_team import RobotTeam
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.xy import XY
from bw_shared.geometry.xyz import XYZ
from bw_shared.tick_regulator import regulate_tick
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param
from bw_tools.messages.goal_strategy import GoalStrategy
from bw_tools.messages.goal_type import GoalType
from geometry_msgs.msg import PoseStamped, Twist

from bw_navigation.exceptions import NavigationError
from bw_navigation.goal_supplier import FixedPoseSupplier, GoalSupplierInterface, TrackedTargetSupplier
from bw_navigation.planners import CrashOpponent, CrashTrajectoryPlanner, PlannerInterface
from bw_navigation.planners.engines.trajectory_planner_engine_config import PlannerConfig


class BwNavigationNode:
    def __init__(self) -> None:
        shared_config = get_shared_config()

        self.controlled_robot = get_param("~controlled_robot", "mini_bot")
        self.goal_tolerance = get_param("~goal_tolerance", 0.1)
        self.tick_rate = get_param("~tick_rate", 100.0)
        self.friendly_fire = get_param("~friendly_fire", False)
        self.try_again_if_not_in_tolerance = get_param("~try_again_if_not_in_tolerance", True)

        robot_configs = {robot.name: robot for robot in shared_config.robots.robots}
        if self.controlled_robot not in robot_configs:
            raise ValueError(f"Controlled robot {self.controlled_robot} not found in shared config")

        self.non_opponent_robot_configs = [
            robot for robot in shared_config.robots.robots if robot.team != RobotTeam.THEIR_TEAM
        ]

        if not robot_configs[self.controlled_robot].is_controlled:
            raise ValueError(f"Controlled robot {self.controlled_robot} is not marked as controlled in shared config")

        self.goal_server = SimpleActionServer(
            "go_to_goal", GoToGoalAction, execute_cb=self.go_to_goal_callback, auto_start=False
        )
        self.field = EstimatedObject()
        self.robots: dict[str, EstimatedObject] = {}
        self.field_bounds_2d: FieldBounds2D = (XY(0.0, 0.0), XY(0.0, 0.0))

        self.initialize_planners(shared_config.robots.robots)

        self.twist_pub = rospy.Publisher(f"{self.controlled_robot}/cmd_vel/navigation", Twist, queue_size=1)
        self.goal_pose_pub = rospy.Publisher(f"{self.controlled_robot}/goal_pose", PoseStamped, queue_size=1)

        self.estimated_field_sub = rospy.Subscriber(
            "filter/field", EstimatedObject, queue_size=1, callback=self.estimated_field_callback
        )
        self.robot_states_sub = rospy.Subscriber(
            "filtered_states", EstimatedObjectArray, queue_size=1, callback=self.robot_states_callback
        )
        self.opponent_fleet_sub = rospy.Subscriber(
            "opponent_fleet", RobotFleetConfigMsg, self.opponent_fleet_callback, queue_size=1
        )
        self.goal_server.start()
        rospy.loginfo("Navigation is ready")

    def opponent_fleet_callback(self, msg: RobotFleetConfigMsg) -> None:
        opponent_fleet = RobotFleetConfig.from_msg(msg)
        rospy.loginfo(
            f"Received opponent fleet. Resetting planners. "
            f"{len(self.non_opponent_robot_configs)} friendly robots. "
            f"{len(opponent_fleet.robots)} opponents."
        )
        robot_fleet = self.non_opponent_robot_configs + opponent_fleet.robots
        self.initialize_planners(robot_fleet)

    def initialize_planners(self, robots: list[RobotConfig]) -> None:
        if self.friendly_fire:
            rospy.logwarn("!!! Friendly fire is enabled !!!")
            opponent_names = [
                robot.name
                for robot in robots
                if (robot.team == RobotTeam.OUR_TEAM and robot.name != self.controlled_robot)
            ]
        else:
            opponent_names = [robot.name for robot in robots if robot.team == RobotTeam.THEIR_TEAM]

        avoid_robot_names = [robot.name for robot in robots if robot.team == RobotTeam.REFEREE]

        self.goal_suppliers: Dict[GoalType, GoalSupplierInterface] = {
            GoalType.FIXED_POSE: FixedPoseSupplier(),
            GoalType.TRACKED_TARGET: TrackedTargetSupplier(self.controlled_robot, opponent_names),
        }
        self.planners: Dict[GoalStrategy, PlannerInterface] = {
            GoalStrategy.CRASH_OPPONENT: CrashOpponent(self.controlled_robot),
            GoalStrategy.CRASH_TRAJECTORY_PLANNER: CrashTrajectoryPlanner(
                self.controlled_robot, avoid_robot_names, PlannerConfig()
            ),
        }
        rospy.loginfo(f"Initialized {len(self.goal_suppliers)} goal suppliers and {len(self.planners)} planners")

    def estimated_field_callback(self, estimated_field: EstimatedObject) -> None:
        if not self.field.header.frame_id:
            rospy.loginfo("Field received")
        self.field = estimated_field
        half_x = estimated_field.size.x / 2
        half_y = estimated_field.size.y / 2
        self.field_bounds = (
            XYZ(-half_x, -half_y, 0.0),
            XYZ(half_x, half_y, estimated_field.size.z),
        )
        self.field_bounds_2d = (
            XY(self.field_bounds[0].x, self.field_bounds[0].y),
            XY(self.field_bounds[1].x, self.field_bounds[1].y),
        )

    def robot_states_callback(self, robot_states: EstimatedObjectArray) -> None:
        self.robots = {robot.label: robot for robot in robot_states.robots}

    def go_to_goal_callback(self, goal: GoToGoalGoal) -> None:
        rospy.loginfo("Received goal")
        if not self.field.header.frame_id:
            rospy.logwarn("No field found, cannot go to goal")
            self.goal_server.set_aborted()
            return

        goal_strategy = GoalStrategy(goal.strategy)
        goal_type = GoalType(goal.goal_type)

        goal_supplier = self.goal_suppliers[goal_type]
        try:
            goal_supplier.initialize(goal, self.field)
        except NavigationError as e:
            rospy.logerr(f"Error initializing: {e}")
            self.goal_server.set_aborted()
            return

        planner = self.planners[goal_strategy]

        goal_feedback = PoseStamped(header=self.field.header)

        for dt in regulate_tick(self.tick_rate):
            if rospy.is_shutdown():
                self.goal_server.set_aborted()
                break

            if self.goal_server.is_preempt_requested():
                rospy.loginfo("Preempted")
                self.goal_server.set_preempted()
                break

            if len(self.robots) == 0:
                rospy.logwarn_throttle(1, "No robots found")
                continue

            try:
                goal_target = goal_supplier.get_goal(self.robots, self.field_bounds_2d)
            except NavigationError as e:
                rospy.logerr(f"Error getting goal: {e}")
                self.goal_server.set_aborted()
                break

            if goal_target is None:
                rospy.logwarn(f"No goal found. Available robots: {self.robots.keys()}")
                self.goal_server.set_succeeded(GoToGoalResult(success=False))
                break

            goal_feedback.pose = goal_target.pose.pose
            self.goal_pose_pub.publish(goal_feedback)

            try:
                twist, goal_progress = planner.go_to_goal(dt, goal_target, self.robots, self.field_bounds_2d)
            except NavigationError as e:
                rospy.logerr(f"Error going to goal: {e}")
                self.goal_server.set_aborted()
                break

            self.twist_pub.publish(twist)

            feedback = GoToGoalFeedback()
            goal_progress.fill_feedback(feedback)
            self.goal_server.publish_feedback(feedback)

            if goal_progress.is_done:
                is_success = goal_progress.distance_to_goal < self.goal_tolerance
                if not is_success and self.try_again_if_not_in_tolerance:
                    rospy.loginfo("Goal not reached, trying again")
                    continue
                rospy.loginfo(f"Planner finished successfully: {is_success}")
                self.goal_server.set_succeeded(GoToGoalResult(success=is_success))
                break
        self.twist_pub.publish(Twist())

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("bw_navigation_node", log_level=rospy.DEBUG)
    BwNavigationNode().run()
