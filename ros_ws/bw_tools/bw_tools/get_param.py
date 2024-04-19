from typing import TypeVar, cast

import rospy

T = TypeVar("T")


def get_param(path: str, default: T) -> T:
    value = rospy.get_param(path, default)
    if value is None:
        return default
    else:
        return cast(T, value)
