import dacite.exceptions
import pytest
from app.config.config import Config
from app.config.config_loader import load_config
from app.config.list_configs import get_config_path, list_configs, list_robots
from app.config.ros_config import RosConfig
from bw_shared.messages.dataclass_utils import from_dict


def test_get_config_path() -> None:
    assert get_config_path().exists()
    assert get_config_path().is_dir()


def test_list_configs() -> None:
    assert len(list_configs()) > 0
    for config in list_configs():
        assert config.suffix == ".toml"
        assert config.exists()
        assert config.is_file()


@pytest.mark.parametrize("robot_name", list_robots())
def test_load_config(robot_name: str) -> None:
    assert isinstance(load_config(get_config_path(), robot_name), Config)


def test_extra_field() -> None:
    with pytest.raises(dacite.exceptions.UnexpectedDataError):
        from_dict(RosConfig, {"extra_field": "value"})
