from pathlib import Path


def get_config_path() -> Path:
    script_dir = Path(__file__).parent
    config_path = script_dir / ".." / ".." / ".." / "configs"
    return config_path


def list_configs() -> list[Path]:
    return [config for config in get_config_path().iterdir() if config.suffix == ".toml"]


def list_robots() -> list[str]:
    return [config.stem for config in list_configs()]
