#!/usr/bin/env python
import json
import math
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Callable

import rospkg
import rospy
import serial
from bw_interfaces.msg import TelemetryStatus
from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto
from bw_shared.geometry.rpy import RPY
from bw_tools.get_param import get_param
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from serial.tools.list_ports import comports
from std_msgs.msg import Header

from bw_teleop.lookup_table_config import LookupTableConfig
from bw_teleop.parameters import load_rosparam_robot_config
from bw_teleop.parse_crsf import CrsfAttitude, CrsfBattery, CrsfFlightMode, CrsfLinkStatistics, CrsfParser, FrameType

rospack = rospkg.RosPack()


def find_transmitter() -> serial.Serial:
    for port in comports():
        if port.pid == 22336 and port.vid == 1155:
            return serial.Serial(port.device, 115200)
    raise RuntimeError("Transmitter not found")


class LookupTableKey(EnumAutoLowerStr):
    PASSTHROUGH = auto()
    PREFITTED_LOOKUP = auto()


class LookupInterface(ABC):
    @abstractmethod
    def lookup(self, linear_x: float, angular_z: float) -> tuple[float, float]: ...


class PassthroughLookup(LookupInterface):
    def __init__(self) -> None:
        pass

    def lookup(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        return linear_x, angular_z


class PrefittedLookup(LookupInterface):
    def __init__(self, table_path: Path, wheel_base_width: float, wheel_radius: float) -> None:
        with open(table_path) as file:
            table_data = json.load(file)
        self.config = LookupTableConfig.from_dict(table_data)
        self.wheel_base_width = wheel_base_width
        self.wheel_radius = wheel_radius
        self.wheel_circum = 2 * self.wheel_radius * math.pi

    def lookup(self, linear_x: float, angular_z: float) -> tuple[float, float]:
        left_ground_vel = linear_x - angular_z * self.wheel_base_width * 0.5
        right_ground_vel = linear_x + angular_z * self.wheel_base_width * 0.5

        left_frequency = left_ground_vel / self.wheel_circum
        right_frequency = right_ground_vel / self.wheel_circum

        left_command = self.config.lookup_velocity(left_frequency)
        right_command = self.config.lookup_velocity(right_frequency)

        linear_x_command = (left_command + right_command) / 2
        angular_z_command = (right_command - left_command) / self.wheel_base_width

        return linear_x_command, angular_z_command


class MiniBotBridge:
    def __init__(self) -> None:
        self.mini_bot_config = load_rosparam_robot_config(get_param("~robot_name", "mini_bot"))

        self.poll_rate = get_param("~poll_rate", 60.0)
        self.lookup_table_key = LookupTableKey(get_param("~lookup_table", "prefitted_lookup"))
        self.lookup_table_path = Path(get_param("~lookup_table_path", "config/lookup_table.json"))
        self.command_timeout = rospy.Duration.from_sec(get_param("~command_timeout", 0.5))
        wheel_base_width = get_param("~wheel_base_width", 0.128)
        wheel_radius = get_param("~wheel_radius", 0.05)
        self.neutral_command = 500

        package_path = Path(rospack.get_path("bw_teleop"))

        self.lookup_table: LookupInterface = {
            LookupTableKey.PASSTHROUGH: PassthroughLookup(),
            LookupTableKey.PREFITTED_LOOKUP: PrefittedLookup(
                package_path / self.lookup_table_path, wheel_base_width, wheel_radius
            ),
        }[self.lookup_table_key]

        self.device = find_transmitter()
        self.set_telemetry(True)
        self.parser = CrsfParser()
        self.packet_callbacks: dict[FrameType, Callable[[Any], None]] = {
            FrameType.BATTERY: self.battery_callback,
            FrameType.LINK_STATISTICS: self.link_statistics_callback,
            FrameType.ATTITUDE: self.attitude_callback,
            FrameType.FLIGHT_MODE: self.flight_mode_callback,
        }

        self.header = Header(frame_id=self.mini_bot_config.name)
        self.command: list[bytes] = [b"", b""]
        self.telemetry_status = TelemetryStatus()
        self.did_status_update = False

        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=1)
        self.telemetry_pub = rospy.Publisher("telemetry_status", TelemetryStatus, queue_size=1)

    def set_telemetry(self, telemetry: bool) -> None:
        if telemetry:
            self.device.write(b"telemetry on\r\n")
        else:
            self.device.write(b"telemetry off\r\n")

    def twist_callback(self, msg: Twist) -> None:
        linear_x, angular_z = self.lookup_table.lookup(msg.linear.x, msg.angular.z)
        linear_value = int(self.neutral_command * linear_x)
        angular_value = int(self.neutral_command * angular_z)
        linear_command = f"trainer 3 {linear_value}\r\n"
        rotate_command = f"trainer 0 {angular_value}\r\n"
        self.command = [linear_command.encode(), rotate_command.encode()]

    def get_telemetry(self) -> None:
        response = self.device.read_all()
        if not response:
            return

        self.header.stamp = rospy.Time.now()
        for packet, error_msg in self.parser.parse(response):
            if error_msg:
                rospy.logwarn(f"Failed to parse packet: {error_msg}")
                continue
            self.packet_callbacks[packet.type](packet)
        if self.did_status_update:
            self.telemetry_pub.publish(self.telemetry_status)
            self.did_status_update = False

    def battery_callback(self, packet: CrsfBattery) -> None:
        self.telemetry_status.battery_voltage = packet.voltage
        self.telemetry_status.battery_current = packet.current
        self.telemetry_status.battery_consumption = packet.consumption
        self.did_status_update = True

    def link_statistics_callback(self, packet: CrsfLinkStatistics) -> None:
        self.telemetry_status.link_stats_json = json.dumps(packet.to_dict())
        self.telemetry_status.is_connected = packet.up_link_quality > 0
        if not self.telemetry_status.is_connected:
            self.reset_telemetry_state()
        self.did_status_update = True

    def attitude_callback(self, packet: CrsfAttitude) -> None:
        angles = RPY((packet.roll, packet.pitch, packet.yaw))
        self.imu_pub.publish(Imu(header=self.header, orientation=angles.to_quaternion()))

    def flight_mode_callback(self, packet: CrsfFlightMode) -> None:
        if not self.telemetry_status.is_connected:
            self.reset_telemetry_state()
        else:
            is_armed = packet.flight_mode == "MANU"
            self.telemetry_status.is_armed = is_armed
            self.telemetry_status.is_ready = is_armed or packet.flight_mode == "OK"
            self.telemetry_status.flight_mode = packet.flight_mode
        self.did_status_update = True

    def reset_telemetry_state(self) -> None:
        self.telemetry_status.is_armed = False
        self.telemetry_status.is_ready = False
        self.telemetry_status.flight_mode = ""

    def run(self) -> None:
        rate = rospy.Rate(self.poll_rate)
        while not rospy.is_shutdown():
            if self.command:
                for cmd in self.command:
                    if not cmd:
                        continue
                    rospy.logdebug(f"Sending command: {cmd!r}")
                    self.device.write(cmd)
            self.get_telemetry()
            rate.sleep()

    def shutdown(self) -> None:
        self.set_telemetry(False)
        self.device.close()


def main() -> None:
    rospy.init_node("mini_bot_bridge", log_level=rospy.DEBUG)
    node = MiniBotBridge()
    rospy.on_shutdown(node.shutdown)
    node.run()


if __name__ == "__main__":
    main()
