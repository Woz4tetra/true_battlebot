from pathlib import Path

import toml
from app.config.metrics_tool_config.metrics_tool_config import MetricsToolConfig


def load_config(config_path: Path) -> MetricsToolConfig:
    with open(config_path, "r") as file:
        config_raw = toml.load(file)
    return MetricsToolConfig.from_dict(config_raw)
