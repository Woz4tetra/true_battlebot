import pytest
from bw_navigation.planners.engines.trajectory_planner_engine_config import PathPlannerConfig


@pytest.fixture
def planner_config() -> PathPlannerConfig:
    return PathPlannerConfig(
        max_velocity=1.0,
        max_acceleration=1.0,
        track_width=0.2,
    )
