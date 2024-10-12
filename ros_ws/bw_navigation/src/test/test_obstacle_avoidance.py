import time
from unittest import mock

from bw_interfaces.msg import EstimatedObject
from bw_navigation.planners.engines.trajectory_planner_engine import TrajectoryPlannerEngine
from bw_navigation.planners.engines.trajectory_planner_engine_config import PathPlannerConfig
from bw_shared.geometry.pose2d import Pose2D


def make_robot(pose: Pose2D, size: float, label: str) -> EstimatedObject:
    robot = EstimatedObject()
    robot.header.frame_id = "map"
    robot.child_frame_id = label
    robot.pose.pose = pose.to_msg()
    robot.size.x = size
    robot.size.y = size
    robot.label = label
    return robot


def test_basic(planner_config: PathPlannerConfig) -> None:
    start_pose = Pose2D(0.0, 0.0, 0.0)
    goal_pose = Pose2D(1.0, 0.0, 0.0)

    controlled_robot = make_robot(start_pose, 0.2, "mini_bot")
    friendly_robot = make_robot(Pose2D(0.5, 0.0, 0.0), 0.2, "main_bot")
    friendly_robots = [friendly_robot]

    with mock.patch("rospy.Time.now", side_effect=lambda: time.time()):
        planner_engine = TrajectoryPlannerEngine(planner_config)

    assert planner_engine.get_robot_collisions(controlled_robot, goal_pose, friendly_robots) == friendly_robots

    new_goal = planner_engine.route_around_obstacles(controlled_robot, goal_pose, friendly_robots)

    assert not planner_engine.get_robot_collisions(controlled_robot, new_goal, friendly_robots)
