from app.config.config import Config
from perception_tools.directories.config_directory import load_config_as_dict


def load_config(robot_name: str) -> Config:
    config_raw = load_config_as_dict(robot_name)
    return Config.from_dict(config_raw)
