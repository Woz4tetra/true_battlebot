from typing import Optional

from bw_interfaces.msg import GoalEngineConfig
from geometry_msgs.msg import Twist


def clamp_twist_with_config(
    twist: Twist,
    supplied_engine_config: Optional[GoalEngineConfig],
    default_engine_config: Optional[GoalEngineConfig] = None,
) -> Twist:
    if supplied_engine_config:
        max_velocity = supplied_engine_config.max_velocity
        max_angular_velocity = supplied_engine_config.max_angular_velocity
    elif default_engine_config:
        max_velocity = default_engine_config.max_velocity
        max_angular_velocity = default_engine_config.max_angular_velocity
    else:
        return twist

    linear_x = max(-1 * max_velocity, min(max_velocity, twist.linear.x))
    angular_z = max(-1 * max_angular_velocity, min(max_angular_velocity, twist.angular.z))

    clamped_twist = Twist()
    clamped_twist.linear.x = linear_x
    clamped_twist.angular.z = angular_z
    return clamped_twist
