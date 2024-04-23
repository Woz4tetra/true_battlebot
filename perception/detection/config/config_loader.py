import os

import toml
from config.config import Config


def load_config(config_dir: str, robot_name: str) -> Config:
    path = os.path.join(config_dir, robot_name + ".toml")
    with open(path, "r") as file:
        config_raw = toml.load(file)
    return Config.from_dict(config_raw)
