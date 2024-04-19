from bw_shared.configs.robots import RobotConfig
from bw_tools.configs.rosparam_client import get_shared_config


def load_rosparam_robot_config(robot_name: str) -> RobotConfig:
    shared_config = get_shared_config()
    robots = shared_config.robots

    robot_config = None
    for robot in robots.robots:
        if robot.name == robot_name:
            robot_config = robot
            break
    if robot_config is None:
        raise ValueError(f"Could not find robot with name {robot_name}")
    return robot_config
