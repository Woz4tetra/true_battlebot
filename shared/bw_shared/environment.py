import os


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


def get_ros_ip() -> str:
    return os.environ.get("ROS_IP", "")

def get_project_dir() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
