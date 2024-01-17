#!/usr/bin/env python
import socket
import time

import rospy
from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.header_type import HeaderType
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.motor_description import MotorDescription
from bw_tools.structs.teleop_bridge.ping_info import PingInfo
from bw_tools.typing import get_param, seconds_to_duration
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

BUFFER_SIZE = 255


class MiniBotBridge:
    def __init__(self) -> None:
        self.rate = get_param("~rate", 50)
        self.port = get_param("~port", 4176)
        self.device_id = get_param("~device_id", 1)
        self.destination = get_param("~destination", "192.168.8.187")
        self.base_width = get_param("~base_width", 0.1315)
        self.max_ground_speed = get_param("~max_ground_speed", 0.5)
        self.speed_to_command = 255 / self.max_ground_speed
        self.max_speed = get_param("~max_speed", 1.0)
        self.deadzone = get_param("~deadzone", 0.05)
        self.command_timeout = seconds_to_duration(get_param("~command_timeout", 0.5))
        self.ping_timeout = get_param("~ping_timeout", 0.5)

        self.packet = b""
        self.last_command_time = rospy.Time.now()

        self.socket = socket.socket(type=socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.port))
        self.socket.setblocking(False)

        self.start_time = time.perf_counter()
        self.prev_ping_time = time.perf_counter()

        self.ping_pub = rospy.Publisher("ping", Float64, queue_size=1)
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=1)
        self.ping_timer = rospy.Timer(seconds_to_duration(0.25), self.ping_timer_callback)
        self.send_timer = rospy.Timer(seconds_to_duration(1.0 / self.rate), self.send_timer_callback)

    def send_packet(self, packet: bytes) -> None:
        rospy.logdebug(f"Sending packet to {self.destination}:{self.port}. {len(packet)} bytes. {packet}")
        self.socket.sendto(packet, (self.destination, self.port))

    def velocity_to_command(self, velocity: float) -> MotorCommand:
        if abs(velocity) < self.deadzone:
            return MotorCommand(0, 0)
        direction = 1 if velocity > 0 else -1
        speed = min(255, int(abs(velocity) * self.speed_to_command))
        return MotorCommand(direction, speed)

    def twist_callback(self, msg: Twist) -> None:
        self.last_command_time = rospy.Time.now()
        left_velocity = msg.linear.x - msg.angular.z * self.base_width / 2
        right_velocity = msg.linear.x + msg.angular.z * self.base_width / 2
        max_speed = max(abs(left_velocity), abs(right_velocity))
        if max_speed > self.max_speed:
            left_velocity *= self.max_speed / max_speed
            right_velocity *= self.max_speed / max_speed
        self.set_velocities(left_velocity, right_velocity)

    def set_velocities(self, left_velocity: float, right_velocity: float) -> None:
        commands = [self.velocity_to_command(left_velocity), self.velocity_to_command(right_velocity)]
        self.packet = MotorDescription(self.device_id, commands).to_bytes()

    def send_timer_callback(self, event) -> None:
        if rospy.Time.now() - self.last_command_time > self.command_timeout:
            self.set_velocities(0, 0)
        if len(self.packet) > 0:
            self.send_packet(self.packet)

    def get_ping_time(self) -> float:
        return time.perf_counter() - self.start_time

    def ping_timer_callback(self, event) -> None:
        timestamp = self.get_ping_time()
        # Timer will loop at ~2.38 hours
        microseconds = int(timestamp * 1e6) & ((2 << 31) - 1)
        self.send_packet(PingInfo(Header.from_id(self.device_id), microseconds).to_bytes())

    def ping_callback(self, ping_info: PingInfo) -> None:
        self.prev_ping_time = time.perf_counter()
        timestamp = self.get_ping_time()
        latency = timestamp - ping_info.timestamp * 1e-6
        self.ping_pub.publish(latency)

    def receive(self) -> None:
        try:
            (packet, (address, port)) = self.socket.recvfrom(BUFFER_SIZE)
        except BlockingIOError:
            return
        rospy.logdebug(f"Received packet from {address}:{port}. {len(packet)} bytes. {packet}")
        if address != self.destination or port != self.port:
            rospy.logwarn(f"Received packet from an unknown address: {address}:{port}")
            return
        read_size = len(packet)
        if read_size == 0:
            return
        if read_size < Header.sizeof():
            rospy.logwarn("Received packet is too small to be valid")
            return
        header = Header.from_bytes(packet)
        if header.device_id != self.device_id:
            rospy.logwarn("Received packet for a different device")
            return
        if header.size != read_size:
            rospy.logwarn(f"Received packet size does not match header. Expected: {header.size}. Read: {read_size}")
            return
        try:
            if header.type == HeaderType.PING:
                self.ping_callback(PingInfo.from_bytes(packet))
        except ValueError as e:
            rospy.logwarn("Failed to parse packet. %s: %s" % (packet, e))

    def check_ping(self) -> None:
        ping_delay = time.perf_counter() - self.prev_ping_time
        if ping_delay > self.ping_timeout:
            rospy.logwarn_throttle(1.0, f"No ping received for {ping_delay:0.4f} seconds")

    def run(self) -> None:
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            self.check_ping()
            self.receive()
            rate.sleep()


def main() -> None:
    rospy.init_node("mini_bot_bridge", log_level=rospy.DEBUG)
    MiniBotBridge().run()


if __name__ == "__main__":
    main()
