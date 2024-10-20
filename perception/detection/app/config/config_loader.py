from pathlib import Path

import toml
from app.config.config import Config


def load_config(config_dir: Path, robot_name: str) -> Config:
    path = config_dir / (robot_name + ".toml")
    with open(path, "r") as file:
        config_raw = toml.load(file)
    return Config.from_dict(config_raw)
