import pytest
from bw_interfaces.msg import EstimatedObject
from bw_navigation.planners.engines.backaway_from_wall_engine import BackawayFromWallEngine
from bw_navigation.planners.engines.config.backaway_recover_config import (
    BackawayRecoverConfig,
)
from bw_navigation.planners.engines.config.local_planner_engine_config import (
    LocalPlannerEngineConfig,
)
from bw_navigation.planners.engines.config.ramsete_config import RamseteConfig
from bw_navigation.planners.engines.local_planner_engine import LocalPlannerEngine
from bw_shared.geometry.pose2d import Pose2D
from matplotlib import pyplot as plt
from plot_helpers import draw_arrow, plot_robots


@pytest.fixture
def local_planner_engine() -> LocalPlannerEngine:
    config = LocalPlannerEngineConfig()
    ramsete = RamseteConfig()
    backaway_engine = BackawayFromWallEngine(BackawayRecoverConfig())
    return LocalPlannerEngine(config, ramsete, backaway_engine)


def make_robot(pose: Pose2D, size: float, label: str) -> EstimatedObject:
    robot = EstimatedObject()
    robot.header.frame_id = "map"
    robot.child_frame_id = label
    robot.pose.pose = pose.to_msg()
    robot.size.x = size
    robot.size.y = size
    robot.label = label
    return robot


def plot_planner_state(
    controlled_robot: EstimatedObject, goal_pose: Pose2D, new_goal: Pose2D, friendly_robots: list[EstimatedObject]
) -> None:
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    window = 4.0
    ax.set_xlim(-window, window)
    ax.set_ylim(-window, window)

    plot_robots(ax, controlled_robot, friendly_robots)
    draw_arrow(ax, new_goal, label="new_goal", color="red")
    draw_arrow(ax, goal_pose, label="goal_pose", color="green")
    plt.legend()
    plt.show()


def test_basic(local_planner_engine: LocalPlannerEngine) -> None:
    start_pose = Pose2D(0.0, 0.0, 0.0)
    goal_pose = Pose2D(1.0, 0.0, 0.0)

    controlled_robot = make_robot(start_pose, 0.2, "mini_bot")
    friendly_robot = make_robot(Pose2D(0.5, 0.0, 0.0), 0.2, "main_bot")
    friendly_robots = [friendly_robot]

    assert local_planner_engine.get_robot_collisions(controlled_robot, goal_pose, friendly_robots) == friendly_robots

    new_goal, was_colliding = local_planner_engine.respond_to_obstacles(controlled_robot, goal_pose, friendly_robots)
    assert was_colliding
    # plot_planner_state(controlled_robot, goal_pose, new_goal, friendly_robots)

    assert not local_planner_engine.get_robot_collisions(controlled_robot, new_goal, friendly_robots)


def test_multi_bot(local_planner_engine: LocalPlannerEngine) -> None:
    start_pose = Pose2D(0.0, 0.0, 0.0)
    goal_pose = Pose2D(1.0, 0.0, 0.0)
    controlled_robot = make_robot(start_pose, 0.2, "mini_bot")
    friendly_robots = [
        make_robot(Pose2D(0.5, -0.2, 0.0), 0.2, "bot1"),
        make_robot(Pose2D(0.5, 0.2, 0.0), 0.2, "bot2"),
    ]

    assert local_planner_engine.get_robot_collisions(controlled_robot, goal_pose, friendly_robots)

    new_goal, was_colliding = local_planner_engine.respond_to_obstacles(controlled_robot, goal_pose, friendly_robots)
    assert was_colliding
    # plot_planner_state(controlled_robot, goal_pose, new_goal, friendly_robots)

    assert not local_planner_engine.get_robot_collisions(controlled_robot, new_goal, friendly_robots)
