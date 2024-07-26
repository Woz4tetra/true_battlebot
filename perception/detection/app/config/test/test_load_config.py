import os

import pytest
from app.config.config import Config
from app.config.config_loader import load_config

CONFIG_DIR = "../configs"


@pytest.mark.parametrize(
    "robot_name",
    [os.path.splitext(name)[0] for name in os.listdir(CONFIG_DIR) if name.endswith(".toml")],
)
def test_load_config(robot_name: str) -> None:
    assert type(load_config("../configs", robot_name)) == Config
