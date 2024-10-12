import time
from unittest import mock

import pytest
from bw_navigation.planners.engines.trajectory_planner_engine import TrajectoryPlannerEngine
from bw_navigation.planners.engines.trajectory_planner_engine_config import PathPlannerConfig
from bw_shared.geometry.pose2d import Pose2D


@pytest.mark.timeout(10)
def test_straight_line(planner_config: PathPlannerConfig) -> None:
    goal_distance = 1.0
    start_pose = Pose2D(0.0, 0.0, 0.0)
    goal_pose = Pose2D(goal_distance, 0.0, 0.0)

    with mock.patch("rospy.Time.now", side_effect=lambda: time.time()):
        planner_engine = TrajectoryPlannerEngine(planner_config)
        trajectory = planner_engine._plan_once(start_pose, goal_pose)

    ramp_time = planner_config.max_velocity / planner_config.max_acceleration
    ramp_distance = planner_config.max_velocity**2 / planner_config.max_acceleration
    linear_time = (goal_distance - 2 * ramp_distance) / planner_config.max_velocity
    total_time = 2 * ramp_time + linear_time

    assert trajectory.totalTime() == pytest.approx(total_time, rel=0.01)

    for state in trajectory.states():
        assert state.pose.Y() == pytest.approx(0.0, rel=0.01)
        assert -0.01 <= state.pose.X() <= goal_distance + 0.01
