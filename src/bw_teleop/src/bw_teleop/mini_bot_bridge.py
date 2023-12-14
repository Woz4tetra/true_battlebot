#!/usr/bin/env python
import socket

import rospy
from bw_tools.typing import get_param
from geometry_msgs.msg import Twist

from bw_teleop.structs import MotorCommand, MotorDescription


class MiniBotBridge:
    def __init__(self) -> None:
        self.rate = get_param("~rate", 50)
        self.port = get_param("~port", 4176)
        self.device_id = get_param("~device_id", 1)
        self.destination = get_param("~destination", "192.168.8.187")
        self.base_width = get_param("~base_width", 0.1315)
        self.speed_to_command = get_param("~speed_to_command", 255 / 0.5)
        self.max_speed = get_param("~max_speed", 1.0)
        self.deadzone = get_param("~deadzone", 0.05)

        self.packet = b""

        self.socket = socket.socket(type=socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.bind(("0.0.0.0", self.port))

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=1)

    def send_packet(self, packet: bytes) -> None:
        self.socket.sendto(packet, (self.destination, self.port))

    def velocity_to_command(self, velocity: float) -> MotorCommand:
        if abs(velocity) < self.deadzone:
            return MotorCommand(0, 0)
        direction = 1 if velocity > 0 else -1
        speed = min(255, int(abs(velocity) * self.speed_to_command))
        return MotorCommand(direction, speed)

    def twist_callback(self, msg: Twist) -> None:
        left_velocity = msg.linear.x - msg.angular.z * self.base_width / 2
        right_velocity = msg.linear.x + msg.angular.z * self.base_width / 2
        max_speed = max(abs(left_velocity), abs(right_velocity))
        if max_speed > self.max_speed:
            left_velocity *= self.max_speed / max_speed
            right_velocity *= self.max_speed / max_speed
        commands = [self.velocity_to_command(left_velocity), self.velocity_to_command(right_velocity)]
        self.packet = MotorDescription(self.device_id, commands).as_bytes()

    def run(self) -> None:
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if len(self.packet) > 0:
                self.send_packet(self.packet)
            rate.sleep()


def main() -> None:
    rospy.init_node("mini_bot_bridge")
    MiniBotBridge().run()


if __name__ == "__main__":
    main()
