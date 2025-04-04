import pytest
from bw_navigation.planners.engines.config.trajectory_planner_config import (
    TrajectoryPlannerConfig,
)
from bw_navigation.planners.engines.trajectory_planner_engine import TrajectoryPlannerEngine
from bw_shared.geometry.pose2d import Pose2D


@pytest.fixture
def global_planner_config() -> TrajectoryPlannerConfig:
    return TrajectoryPlannerConfig(
        max_velocity=1.0,
        max_acceleration=1.0,
        track_width=0.2,
    )


@pytest.mark.timeout(10)
def test_straight_line(global_planner_config: TrajectoryPlannerConfig) -> None:
    goal_distance = 1.0
    start_pose = Pose2D(0.0, 0.0, 0.0)
    goal_pose = Pose2D(goal_distance, 0.0, 0.0)

    planner_engine = TrajectoryPlannerEngine(global_planner_config)
    trajectory_config = planner_engine.make_trajectory_config_from_velocity_profile(None)
    trajectory = planner_engine._plan_once(start_pose, goal_pose, trajectory_config)

    ramp_time = global_planner_config.max_velocity / global_planner_config.max_acceleration
    ramp_distance = global_planner_config.max_velocity**2 / global_planner_config.max_acceleration
    linear_time = (goal_distance - 2 * ramp_distance) / global_planner_config.max_velocity
    total_time = 2 * ramp_time + linear_time

    assert trajectory.totalTime() == pytest.approx(total_time, rel=0.01)

    for state in trajectory.states():
        assert state.pose.Y() == pytest.approx(0.0, rel=0.01)
        assert -0.01 <= state.pose.X() <= goal_distance + 0.01
