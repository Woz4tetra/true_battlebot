#!/usr/bin/env python
import math
import time

import rospy
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray, MotorVelocities
from bw_tools.structs.teleop_bridge.imu_sensor import ImuSensor
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.ping_info import PingInfo
from bw_tools.typing import get_param
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf import transformations

from bw_teleop.bridge_interface import BridgeInterface
from bw_teleop.parameters import load_rosparam_robot_config


class MiniBotBridge:
    def __init__(self) -> None:
        self.mini_bot_config = load_rosparam_robot_config(get_param("~robot_name", "mini_bot"))

        port = get_param("~port", 4176)
        device_id = self.mini_bot_config.bridge_id
        broadcast_address = get_param("~broadcast_address", "192.168.8.255")
        self.robot_frame_prefix = get_param("~robot_frame_prefix", "base_link")

        self.rate = get_param("~rate", 1000)
        self.base_radius = self.mini_bot_config.base_width / 2
        self.saturation_speed = get_param("~saturation_speed", 10.0)
        self.command_timeout = rospy.Duration.from_sec(get_param("~command_timeout", 0.5))
        self.ping_timeout = get_param("~ping_timeout", 0.5)

        self.rotation_fudge_factor = 1.0
        self.linear_fudge_factor = 1.0
        self.left_right_imbalance_factor = 1.0

        self.last_command_time = rospy.Time.now()

        self.prev_active_time = time.perf_counter()
        self.command = [0.0, 0.0]
        self.mini_bot_state = EstimatedObject()

        self.bridge = BridgeInterface(broadcast_address, port, device_id)
        self.bridge.register_callback(PingInfo, self.ping_callback)
        self.bridge.register_callback(ImuSensor, self.imu_callback)

        self.ping_pub = rospy.Publisher("ping", Float64, queue_size=1)
        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        self.motor_velocities_pub = rospy.Publisher("motor_velocities", MotorVelocities, queue_size=1)

        self.ping_timer = rospy.Timer(rospy.Duration.from_sec(0.2), self.ping_timer_callback)

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=1)
        self.filter_state_array_sub = rospy.Subscriber(
            "filtered_states", EstimatedObjectArray, self.filtered_states_callback, queue_size=50
        )

    def twist_callback(self, msg: Twist) -> None:
        self.last_command_time = rospy.Time.now()
        linear_x = msg.linear.x * self.linear_fudge_factor
        angular_z = msg.angular.z * self.rotation_fudge_factor
        if self.is_right_side_up():
            left_factor = self.left_right_imbalance_factor
            right_factor = 2 - self.left_right_imbalance_factor
        else:
            left_factor = 2 - self.left_right_imbalance_factor
            right_factor = self.left_right_imbalance_factor
        left_velocity = linear_x * left_factor - angular_z * self.base_radius
        right_velocity = linear_x * right_factor + angular_z * self.base_radius
        larger_speed = max(abs(left_velocity), abs(right_velocity))
        if larger_speed > self.saturation_speed:
            left_velocity *= self.saturation_speed / larger_speed
            right_velocity *= self.saturation_speed / larger_speed

        velocities = self.set_velocities(left_velocity, right_velocity)
        self.motor_velocities_pub.publish(velocities)

    def is_right_side_up(self) -> bool:
        return self.is_orientation_right_side_up(self.mini_bot_state.state.pose.pose.orientation)

    def filtered_states_callback(self, msg: EstimatedObjectArray) -> None:
        for robot in msg.robots:
            if robot.label == self.mini_bot_config.name:
                self.mini_bot_state = robot

    def is_orientation_right_side_up(self, quaternion: Quaternion) -> bool:
        angles = transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return abs(angles[0]) < 1.5

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
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.check_ping()
            if rospy.Time.now() - self.last_command_time > self.command_timeout:
                self.set_stop()
            self.bridge.receive()
            self.send_command()
            rate.sleep()


def main() -> None:
    # log_level = rospy.DEBUG
    log_level = rospy.INFO
    rospy.init_node("mini_bot_bridge", log_level=log_level)
    MiniBotBridge().run()


if __name__ == "__main__":
    main()
