from bw_tools.configs.robot_config import RobotConfig, RobotFleetConfig
from bw_tools.typing import get_param


def load_rosparam_robot_config(robot_name: str) -> RobotConfig:
    robot_config = get_param("/robots", None)
    if robot_config is None:
        raise ValueError("Must specify robot_config in the parameter server")
    robots = RobotFleetConfig.from_dict(robot_config)

    robot_config = None
    for robot in robots.robots:
        if robot.name == robot_name:
            robot_config = robot
            break
    if robot_config is None:
        raise ValueError(f"Could not find robot with name {robot_name}")
    return robot_config
