from typing import Type, TypeVar

import rospy
from bw_shared.configs.shared_config import SharedConfig

T = TypeVar("T")


def wait_for_param(key: str, value_type: Type[T]) -> T:
    while not rospy.has_param(key):
        rospy.loginfo_throttle(3, f"Waiting for {key} to be set")
        rospy.sleep(0.1)
    value = rospy.get_param(key, "")
    assert isinstance(value, value_type), f"{type(value)} is not {value_type}"
    return value


def get_shared_config() -> SharedConfig:
    shared_config = wait_for_param("/shared_config", dict)
    return SharedConfig.from_dict(shared_config)


def get_map() -> str:
    return wait_for_param("/map", str)


def get_robot() -> str:
    return wait_for_param("/robot", str)
