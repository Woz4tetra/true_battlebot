import os

import dacite.exceptions
import pytest
from app.config.config import Config
from app.config.config_loader import load_config
from app.config.ros_config import RosConfig
from bw_shared.messages.dataclass_utils import from_dict

CONFIG_DIR = "../configs"


@pytest.mark.parametrize(
    "robot_name",
    [os.path.splitext(name)[0] for name in os.listdir(CONFIG_DIR) if name.endswith(".toml")],
)
def test_load_config(robot_name: str) -> None:
    assert isinstance(load_config("../configs", robot_name), Config)


def test_extra_field() -> None:
    with pytest.raises(dacite.exceptions.UnexpectedDataError):
        from_dict(RosConfig, {"extra_field": "value"})
