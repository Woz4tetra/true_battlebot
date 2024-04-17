import os

from bw_interfaces.msg import SystemSummary


def _get_env(name: str) -> str:
    value = os.environ.get(name, "")
    if len(value) == 0:
        raise ValueError(f"Environment variable {name} is not set")
    return value


def get_robot() -> str:
    return _get_env("ROBOT")


def get_map() -> str:
    return _get_env("MAP_NAME")


def get_image_version() -> str:
    return _get_env("IMAGE_VERSION")


def get_system_info() -> SystemSummary:
    return SystemSummary(
        robot=get_robot(),
        map=get_map(),
        version=get_image_version(),
    )
