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
)
from bw_shared.enums.robot_team import RobotTeam
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY
from bw_shared.geometry.xyz import XYZ
from bw_shared.tick_regulator import regulate_tick
from bw_tools.configs.rosparam_client import get_shared_config
from bw_tools.get_param import get_param
from bw_tools.messages.goal_strategy import GoalStrategy
from bw_tools.messages.goal_type import GoalType
from geometry_msgs.msg import PoseStamped, Twist

from bw_navigation.exceptions import NavigationError
from bw_navigation.goal_supplier import (
    FixedPoseSupplier,
    GoalSupplierInterface,
    TrackedTargetSupplier,
)
from bw_navigation.planners import CrashOpponent, CrashTrajectoryPlanner, PlannerInterface


class BwNavigationNode:
    def __init__(self) -> None:
        shared_config = get_shared_config()

        self.controlled_robot = get_param("controlled_robot", "mini_bot")
        self.goal_tolerance = get_param("goal_tolerance", 0.1)
        self.tick_rate = get_param("tick_rate", 100.0)

        robot_configs = {robot.name: robot for robot in shared_config.robots.robots}
        if self.controlled_robot not in robot_configs:
            raise ValueError(f"Controlled robot {self.controlled_robot} not found in shared config")

        if not robot_configs[self.controlled_robot].is_controlled:
            raise ValueError(f"Controlled robot {self.controlled_robot} is not marked as controlled in shared config")

        self.goal_server = SimpleActionServer(
            "go_to_goal", GoToGoalAction, execute_cb=self.go_to_goal_callback, auto_start=False
        )
        self.field = EstimatedObject()
        self.robots: dict[str, EstimatedObject] = {}

        opponent_names = [robot.name for robot in shared_config.robots.robots if robot.team == RobotTeam.THEIR_TEAM]
        # TODO: remove before competition
        # opponent_names = [
        #     robot.name
        #     for robot in shared_config.robots.robots
        #     if (robot.team == RobotTeam.OUR_TEAM and robot.name != self.controlled_robot)
        # ]

        self.goal_suppliers: Dict[GoalType, GoalSupplierInterface] = {
            GoalType.FIXED_POSE: FixedPoseSupplier(),
            GoalType.TRACKED_TARGET: TrackedTargetSupplier(self.controlled_robot, opponent_names),
        }
        self.planners: Dict[GoalStrategy, PlannerInterface] = {
            GoalStrategy.CRASH_OPPONENT: CrashOpponent(self.controlled_robot),
            GoalStrategy.CRASH_TRAJECTORY_PLANNER: CrashTrajectoryPlanner(self.controlled_robot),
        }

        self.twist_pub = rospy.Publisher(f"{self.controlled_robot}/cmd_vel/navigation", Twist, queue_size=1)
        self.goal_pose_pub = rospy.Publisher(f"{self.controlled_robot}/goal_pose", PoseStamped, queue_size=1)

        self.estimated_field_sub = rospy.Subscriber(
            "filter/field", EstimatedObject, queue_size=1, callback=self.estimated_field_callback
        )
        self.robot_states_sub = rospy.Subscriber(
            "filtered_states", EstimatedObjectArray, queue_size=1, callback=self.robot_states_callback
        )
        self.goal_server.start()
        rospy.loginfo("Navigation is ready")

    def estimated_field_callback(self, estimated_field: EstimatedObject) -> None:
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

    def compute_feedback_distance(self, goal_pose: Pose2D) -> float:
        robot_pose = Pose2D.from_msg(self.robots[self.controlled_robot].pose.pose)
        return goal_pose.relative_to(robot_pose).magnitude()

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
            if self.goal_server.is_preempt_requested():
                rospy.loginfo("Preempted")
                self.goal_server.set_preempted()
                break

            if len(self.robots) == 0:
                rospy.logwarn_throttle(1, "No robots found")
                continue

            try:
                goal_pose = goal_supplier.get_goal(self.robots, self.field_bounds_2d)
            except NavigationError as e:
                rospy.logerr(f"Error getting goal: {e}")
                self.goal_server.set_aborted()
                break

            if goal_pose is None:
                rospy.logwarn_throttle(1, f"No goal found. Available robots: {self.robots.keys()}")
                continue

            goal_feedback.pose = goal_pose.to_msg()
            self.goal_pose_pub.publish(goal_feedback)

            try:
                twist, is_done = planner.go_to_goal(dt, goal_pose, self.robots, self.field_bounds_2d)
            except NavigationError as e:
                rospy.logerr(f"Error going to goal: {e}")
                self.goal_server.set_aborted()
                break

            self.twist_pub.publish(twist)

            distance_to_goal = self.compute_feedback_distance(goal_pose)
            self.goal_server.publish_feedback(
                GoToGoalFeedback(
                    distance_to_goal=distance_to_goal,
                )
            )

            if is_done:
                is_success = distance_to_goal < self.goal_tolerance
                rospy.loginfo(f"Planner finished successfully: {is_success}")
                self.goal_server.set_succeeded(
                    GoToGoalResult(
                        success=is_success,
                    )
                )
                break
        self.twist_pub.publish(Twist())

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("bw_navigation_node", log_level=rospy.DEBUG)
    BwNavigationNode().run()
