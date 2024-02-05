#!/usr/bin/env python
import math
import socket
import struct
import time
from typing import Optional

import rospy
from bw_interfaces.msg import MotorVelocities
from bw_tools.configs.robot_config import RobotFleetConfig
from bw_tools.structs.teleop_bridge.header import Header
from bw_tools.structs.teleop_bridge.header_type import HeaderType
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.motor_description import MotorDescription
from bw_tools.structs.teleop_bridge.ping_info import PingInfo
from bw_tools.typing import get_param
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

BUFFER_SIZE = 512


def is_loopback(host: str):
    loopback_checker = {
        socket.AF_INET: lambda x: struct.unpack("!I", socket.inet_aton(x))[0] >> (32 - 8) == 127,
        socket.AF_INET6: lambda x: x == "::1",
    }
    for family in (socket.AF_INET, socket.AF_INET6):
        try:
            r = socket.getaddrinfo(host, None, family, socket.SOCK_STREAM)
        except socket.gaierror:
            return False
        for family, _, _, _, sockaddr in r:
            if not loopback_checker[family](sockaddr[0]):
                return False
    return True


class MacroPwm:
    def __init__(self, cycle_time: float, min_value: float, max_value: float) -> None:
        self.cycle_time = cycle_time
        self.min_value = min_value
        self.max_value = max_value
        self.prev_time = time.perf_counter()

    def update(self, value: float) -> float:
        abs_value = abs(value)
        if abs_value > self.max_value or abs_value < self.min_value:
            return value
        now = time.perf_counter()
        percent_cycle = abs_value / (self.max_value - self.min_value)
        switch_over_time = self.cycle_time * percent_cycle + self.prev_time
        over_cycle_time = self.cycle_time + self.prev_time
        if now > over_cycle_time:
            self.prev_time = now
        if now > switch_over_time:
            return math.copysign(self.min_value, value)
        return math.copysign(self.max_value, value)


class MiniBotBridge:
    def __init__(self) -> None:
        robot_config = get_param("/robots", None)
        if robot_config is None:
            raise ValueError("Must specify robot_config in the parameter server")
        self.robots = RobotFleetConfig.from_dict(robot_config)

        self.robot_name = get_param("~robot_name", "mini_bot")

        mini_bot_config = None
        for robot in self.robots.robots:
            if robot.name == self.robot_name:
                mini_bot_config = robot
                break
        if mini_bot_config is None:
            raise ValueError(f"Could not find robot with name {self.robot_name}")
        self.mini_bot_config = mini_bot_config

        self.rate = get_param("~rate", 1000)
        self.port = get_param("~port", 4176)
        self.device_id = self.mini_bot_config.bridge_id
        self.base_radius = self.mini_bot_config.base_width / 2
        self.broadcast_address = get_param("~broadcast_address", "192.168.8.255")
        self.deadzone = get_param("~deadzone", 0.0)
        self.min_speed = get_param("~min_speed", 0.2)
        self.max_speed = get_param("~max_speed", 1.0)
        self.macro_pwm_cycle_time = get_param("~macro_pwm_cycle_time", 0.05)
        self.speed_to_command = 255 / self.max_speed
        self.command_timeout = rospy.Duration.from_sec(get_param("~command_timeout", 0.5))
        self.ping_timeout = get_param("~ping_timeout", 0.5)

        self.left_direction = 1 if get_param("~flip_left", False) else -1
        self.right_direction = 1 if get_param("~flip_right", True) else -1

        self.left_pwm = MacroPwm(self.macro_pwm_cycle_time, 0.0, self.min_speed)
        self.right_pwm = MacroPwm(self.macro_pwm_cycle_time, 0.0, self.min_speed)

        self.destination = ""
        self.blacklist_ips = set()
        self.whitelist_ips = set()

        self.velocities = MotorVelocities(velocities=[0, 0])
        self.last_command_time = rospy.Time.now()

        self.socket = socket.socket(type=socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.port))
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.setblocking(False)

        self.start_time = time.perf_counter()
        self.prev_ping_time = time.perf_counter()

        self.ping_pub = rospy.Publisher("ping", Float64, queue_size=1)
        self.motor_velocities_pub = rospy.Publisher("motor_velocities", MotorVelocities, queue_size=1)
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=1)
        self.ping_timer = rospy.Timer(rospy.Duration.from_sec(0.2), self.ping_timer_callback)
        self.send_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.rate), self.send_timer_callback)

    def send_packet(self, packet: bytes) -> None:
        if self.destination:
            rospy.logdebug(f"Sending packet to {self.destination}:{self.port}. {len(packet)} bytes. {packet}")
            try:
                self.socket.sendto(packet, (self.destination, self.port))
            except BlockingIOError as e:
                rospy.logwarn(f"Failed to send packet. {e}")

    def broadcast_packet(self, packet: bytes) -> None:
        rospy.logdebug(f"Broadcasting packet. {len(packet)} bytes. {packet}")
        try:
            self.socket.sendto(packet, (self.broadcast_address, self.port))
        except BlockingIOError as e:
            rospy.logwarn(f"Failed to broadcast packet. {e}")

    def velocity_to_command(self, velocity: float) -> MotorCommand:
        if abs(velocity) <= self.deadzone:
            return MotorCommand(0, 0)
        direction = 1 if velocity > 0 else -1
        speed = min(255, int(abs(velocity) * self.speed_to_command))
        return MotorCommand(direction, speed)

    def twist_callback(self, msg: Twist) -> None:
        self.last_command_time = rospy.Time.now()
        left_velocity = msg.linear.x - msg.angular.z * self.base_radius
        right_velocity = msg.linear.x + msg.angular.z * self.base_radius
        larger_speed = max(abs(left_velocity), abs(right_velocity))
        if larger_speed > self.max_speed:
            left_velocity *= self.max_speed / larger_speed
            right_velocity *= self.max_speed / larger_speed
        self.set_velocities(left_velocity, right_velocity)

    def set_velocities(self, left_velocity: float, right_velocity: float) -> None:
        self.velocities = MotorVelocities(velocities=[left_velocity, right_velocity])
        self.motor_velocities_pub.publish(self.velocities)

    def send_timer_callback(self, event) -> None:
        if rospy.Time.now() - self.last_command_time > self.command_timeout:
            self.set_velocities(0, 0)
        left_velocity = self.left_pwm.update(self.velocities.velocities[0])
        right_velocity = self.right_pwm.update(self.velocities.velocities[1])
        commands = [
            self.velocity_to_command(self.left_direction * left_velocity),
            self.velocity_to_command(self.right_direction * right_velocity),
        ]
        packet = MotorDescription(self.device_id, commands).to_bytes()
        self.send_packet(packet)

    def get_ping_time(self) -> float:
        return time.perf_counter() - self.start_time

    def ping_timer_callback(self, event) -> None:
        timestamp = self.get_ping_time()
        # Timer will loop at ~2.38 hours
        microseconds = int(timestamp * 1e6) & ((2 << 31) - 1)
        self.broadcast_packet(PingInfo(Header.from_id(0), microseconds).to_bytes())

    def ping_callback(self, ping_info: PingInfo) -> None:
        self.prev_ping_time = time.perf_counter()
        timestamp = self.get_ping_time()
        latency = timestamp - ping_info.timestamp * 1e-6
        self.ping_pub.publish(latency)

    def is_address_ok(self, address: str, port: int) -> bool:
        if address in self.blacklist_ips:
            return False
        if address not in self.whitelist_ips:
            if is_loopback(address):
                self.blacklist_ips.add(address)
                return False
            else:
                self.whitelist_ips.add(address)

        if port != self.port:
            rospy.logwarn(f"Received packet from an unknown address: {address}:{port}")
            return False

        return True

    def parse_header(self, packet: bytes) -> Optional[Header]:
        read_size = len(packet)
        if read_size == 0:
            return None
        if read_size < Header.sizeof():
            rospy.logwarn("Received packet is too small to be valid")
            return None
        header = Header.from_bytes(packet)
        if header.device_id != self.device_id:
            rospy.logdebug(
                f"Received packet for a different device. Received {header.device_id} != expected {self.device_id}"
            )
            return None

        if header.size != read_size:
            rospy.logwarn(f"Received packet size does not match header. Expected: {header.size}. Read: {read_size}")
            return None

        return header

    def receive(self) -> None:
        try:
            (packet, (address, port)) = self.socket.recvfrom(BUFFER_SIZE)
        except BlockingIOError:
            return

        if not self.is_address_ok(address, port):
            return

        rospy.logdebug(f"Received packet from {address}:{port}. {len(packet)} bytes. {packet}")

        header = self.parse_header(packet)
        if not header:
            return
        self.destination = address

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
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.check_ping()
            self.receive()
            rate.sleep()


def main() -> None:
    # log_level = rospy.DEBUG
    log_level = rospy.INFO
    rospy.init_node("mini_bot_bridge", log_level=log_level)
    MiniBotBridge().run()


if __name__ == "__main__":
    main()
