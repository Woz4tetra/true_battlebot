import os

import tomllib
from config.config import Config


def load_config(config_dir: str, robot_name: str) -> Config:
    path = os.path.join(config_dir, robot_name + ".toml")
    with open(path, "rb") as file:
        config_raw = tomllib.load(file)
    return Config.from_dict(config_raw)
