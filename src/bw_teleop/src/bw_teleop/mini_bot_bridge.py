#!/usr/bin/env python
import time

import rospy
from bw_interfaces.msg import MotorVelocities
from bw_tools.structs.teleop_bridge.imu_sensor import ImuSensor
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.ping_info import PingInfo
from bw_tools.typing import get_param
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from bw_teleop.bridge_interface import BridgeInterface
from bw_teleop.parameters import load_rosparam_robot_config


class MiniBotBridge:
    def __init__(self) -> None:
        self.mini_bot_config = load_rosparam_robot_config(get_param("~robot_name", "mini_bot"))

        port = get_param("~port", 4176)
        device_id = self.mini_bot_config.bridge_id
        broadcast_address = get_param("~broadcast_address", "192.168.8.255")
        self.robot_frame_prefix = get_param("~robot_frame_prefix", "base_link")

        self.poll_rate = get_param("~poll_rate", 1000.0)
        self.send_rate = get_param("~send_rate", 30.0)
        self.base_radius = self.mini_bot_config.base_width / 2
        self.command_timeout = rospy.Duration.from_sec(get_param("~command_timeout", 0.5))
        self.ping_timeout = get_param("~ping_timeout", 0.5)

        self.last_command_time = rospy.Time.now()
        self.prev_active_time = time.perf_counter()
        self.command = [0.0, 0.0]

        self.bridge = BridgeInterface(broadcast_address, port, device_id)
        self.bridge.register_callback(PingInfo, self.ping_callback)
        self.bridge.register_callback(ImuSensor, self.imu_callback)

        self.ping_pub = rospy.Publisher("ping", Float64, queue_size=1)
        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        self.motor_velocities_pub = rospy.Publisher("motor_velocities", MotorVelocities, queue_size=1)

        self.ping_timer = rospy.Timer(rospy.Duration.from_sec(0.2), self.ping_timer_callback)
        self.command_timer = rospy.Timer(rospy.Duration.from_sec(1 / self.send_rate), self.send_timer_callback)

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=1)

    def twist_callback(self, msg: Twist) -> None:
        self.last_command_time = rospy.Time.now()
        linear_x = msg.linear.x * 5
        angular_z = msg.angular.z * 2
        left_velocity = linear_x - angular_z * self.base_radius
        right_velocity = linear_x + angular_z * self.base_radius
        velocities = self.set_velocities(left_velocity, right_velocity)
        self.motor_velocities_pub.publish(velocities)

    def set_velocities(self, left_velocity: float, right_velocity: float) -> MotorVelocities:
        self.command = [left_velocity, right_velocity]
        return MotorVelocities(velocities=self.command)

    def set_stop(self) -> None:
        self.command = [0.0, 0.0]

    def send_command(self) -> None:
        self.bridge.send_command(
            [
                MotorCommand.from_values(self.command[0]),
                MotorCommand.from_values(self.command[1]),
            ]
        )

    def ping_timer_callback(self, event) -> None:
        self.bridge.send_ping()

    def send_timer_callback(self, event) -> None:
        self.send_command()

    def ping_callback(self, ping_info: PingInfo) -> None:
        latency = self.bridge.compute_latency(ping_info)
        self.prev_active_time = time.perf_counter()
        self.ping_pub.publish(latency)

    def imu_callback(self, imu: ImuSensor) -> None:
        msg = imu.to_msg()
        self.prev_active_time = time.perf_counter()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.robot_frame_prefix + "_" + self.mini_bot_config.name
        self.imu_pub.publish(msg)

    def check_ping(self) -> None:
        delay = time.perf_counter() - self.prev_active_time
        if delay > self.ping_timeout:
            rospy.logwarn_throttle(1.0, f"No data received for {delay:0.4f} seconds")

    def run(self) -> None:
        rate = rospy.Rate(self.poll_rate)
        while not rospy.is_shutdown():
            self.check_ping()
            if rospy.Time.now() - self.last_command_time > self.command_timeout:
                self.set_stop()
            self.bridge.receive()
            rate.sleep()


def main() -> None:
    # log_level = rospy.DEBUG
    log_level = rospy.INFO
    rospy.init_node("mini_bot_bridge", log_level=log_level)
    MiniBotBridge().run()


if __name__ == "__main__":
    main()
