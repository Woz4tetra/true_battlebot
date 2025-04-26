from enum import Enum
from pathlib import Path

import toml


class ConfigType(Enum):
    ROBOT = ""
    FIELD_LABEL_TOOL = "field_label_tool"
    AUTO_LABEL = "auto_label"


def get_config_path() -> Path:
    script_dir = Path(__file__).parent
    config_path = script_dir / ".." / ".." / ".." / "configs"
    return config_path


def get_path_in_config(config_type: ConfigType = ConfigType.ROBOT) -> Path:
    return get_config_path() / config_type.value


def list_configs(config_type: ConfigType = ConfigType.ROBOT) -> list[Path]:
    return [config for config in get_path_in_config(config_type).iterdir() if config.suffix == ".toml"]


def list_robots() -> list[str]:
    return [config.stem for config in list_configs()]


def load_config_as_dict(config_name: str, config_type: ConfigType = ConfigType.ROBOT) -> dict:
    path = get_path_in_config(config_type) / (config_name + ".toml")
    with open(path, "r") as file:
        config_dict = toml.load(file)
    return config_dict
