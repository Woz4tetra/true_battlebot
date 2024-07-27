#!/usr/bin/env python
import rospy
import serial
from bw_interfaces.msg import MotorVelocities, TelemetryStatus
from bw_shared.geometry.rpy import RPY
from bw_tools.get_param import get_param
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from serial.tools.list_ports import comports
from std_msgs.msg import Header

from bw_teleop.parameters import load_rosparam_robot_config


def find_transmitter() -> serial.Serial:
    for port in comports():
        if port.pid == 22336 and port.vid == 1155:
            return serial.Serial(port.device, 115200)
    raise RuntimeError("Transmitter not found")


class MiniBotBridge:
    def __init__(self) -> None:
        self.mini_bot_config = load_rosparam_robot_config(get_param("~robot_name", "mini_bot"))

        self.poll_rate = get_param("~poll_rate", 60.0)
        self.max_linear_scale = 1.0 / get_param("~max_linear", 3.435)  # m/s
        self.max_angular_scale = 1.0 / get_param("~max_angular", 20.0)  # rad/s
        self.command_timeout = rospy.Duration.from_sec(get_param("~command_timeout", 0.5))
        self.min_command = get_param("~min_command", 30)
        self.wheel_base_width = get_param("~wheel_base_width", 0.128)
        self.neutral_command = 500

        self.device = find_transmitter()
        self.set_telemetry(True)

        self.header = Header(frame_id=self.mini_bot_config.name)
        self.command = []

        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=1)
        self.motor_command_sub = rospy.Subscriber(
            "motor_command", MotorVelocities, self.motor_command_callback, queue_size=1
        )
        self.telemetry_pub = rospy.Publisher("telemetry_status", TelemetryStatus, queue_size=1)

    def set_telemetry(self, telemetry: bool) -> None:
        if telemetry:
            self.device.write(b"telemetry on\r\n")
        else:
            self.device.write(b"telemetry off\r\n")

    def motor_command_callback(self, msg: MotorVelocities) -> None:
        left_velocity = msg.velocities[0]
        right_velocity = msg.velocities[1]
        twist = Twist()
        twist.linear.x = (left_velocity + right_velocity) / 2.0
        twist.angular.z = (right_velocity - left_velocity) / self.wheel_base_width
        self.set_twist_command(twist)

    def twist_callback(self, msg: Twist) -> None:
        self.set_twist_command(msg)

    def set_twist_command(self, msg: Twist) -> None:
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        linear_value = int(self.neutral_command * linear_x * self.max_linear_scale)
        angular_value = int(-self.neutral_command * angular_z * self.max_angular_scale)
        if linear_value != 0:
            linear_value += self.min_command if linear_value > 0 else -self.min_command
        if angular_value != 0:
            angular_value += self.min_command if angular_value > 0 else -self.min_command
        linear_command = f"trainer 3 {linear_value}\r\n"
        rotate_command = f"trainer 0 {angular_value}\r\n"
        self.command = [linear_command.encode(), rotate_command.encode()]

    def get_telemetry(self) -> None:
        response = self.device.read_all()
        if not response:
            return

        self.header.stamp = rospy.Time.now()
        for frame in response.split(b"\xea"):  # CRSF start byte
            if not frame:
                continue
            frame_type = frame[1:2]
            if not frame_type == b"\x1e":  # Attitude frame
                continue
            roll = int.from_bytes(frame[2:4], "big", signed=True) / 10000
            pitch = int.from_bytes(frame[4:6], "big", signed=True) / 10000
            yaw = int.from_bytes(frame[6:8], "big", signed=True) / 10000
            angles = RPY((roll, pitch, yaw))
            self.imu_pub.publish(Imu(header=self.header, orientation=angles.to_quaternion()))

    def run(self) -> None:
        rate = rospy.Rate(self.poll_rate)
        while not rospy.is_shutdown():
            if self.command:
                for cmd in self.command:
                    rospy.logdebug(f"Sending command: {cmd}")
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
