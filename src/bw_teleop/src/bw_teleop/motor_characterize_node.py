#!/usr/bin/env python
import time

import rospy
from bw_tools.structs.teleop_bridge.ping_info import PingInfo
from bw_tools.typing import get_param

from bw_teleop.bridge_interface import BridgeInterface
from bw_teleop.parameters import load_rosparam_robot_config


class MotorCharacterizeNode:
    def __init__(self) -> None:
        self.mini_bot_config = load_rosparam_robot_config(get_param("~robot_name", "mini_bot"))

        self.rate = get_param("~rate", 1000)
        port = get_param("~port", 4176)
        device_id = self.mini_bot_config.bridge_id
        broadcast_address = get_param("~broadcast_address", "192.168.8.255")

        self.ping_timeout = get_param("~ping_timeout", 0.5)

        self.left_direction = 1 if get_param("~flip_left", False) else -1
        self.right_direction = 1 if get_param("~flip_right", True) else -1

        self.prev_ping_time = time.perf_counter()

        self.bridge = BridgeInterface(broadcast_address, port, device_id, {PingInfo: self.ping_callback})

        self.ping_timer = rospy.Timer(rospy.Duration.from_sec(0.2), self.ping_timer_callback)

    def ping_timer_callback(self, event) -> None:
        self.bridge.send_ping()

    def ping_callback(self, ping_info: PingInfo) -> None:
        latency = self.bridge.compute_latency(ping_info)
        self.prev_ping_time = time.perf_counter()

    def check_ping(self) -> None:
        ping_delay = time.perf_counter() - self.prev_ping_time
        if ping_delay > self.ping_timeout:
            rospy.logwarn_throttle(1.0, f"No ping received for {ping_delay:0.4f} seconds")

    def run(self) -> None:
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.check_ping()
            self.bridge.receive()
            rate.sleep()


def main() -> None:
    log_level = rospy.DEBUG
    rospy.init_node("motor_characterization", log_level=log_level)
    MotorCharacterizeNode().run()


if __name__ == "__main__":
    main()
