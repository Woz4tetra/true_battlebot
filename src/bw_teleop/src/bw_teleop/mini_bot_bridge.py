#!/usr/bin/env python
import math
import time

import rospy
from bw_interfaces.msg import MotorVelocities
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.ping_info import PingInfo
from bw_tools.typing import get_param
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from bw_teleop.bridge_interface import BridgeInterface
from bw_teleop.macro_pwm import MacroPwm
from bw_teleop.parameters import load_rosparam_robot_config
from bw_teleop.src.bw_teleop.motor_characterization.continuous_command_conversion import (
    ConversionConstants,
    get_conversion_function,
)


class MiniBotBridge:
    def __init__(self) -> None:
        self.mini_bot_config = load_rosparam_robot_config(get_param("~robot_name", "mini_bot"))

        port = get_param("~port", 4176)
        device_id = self.mini_bot_config.bridge_id
        broadcast_address = get_param("~broadcast_address", "192.168.8.255")

        self.rate = get_param("~rate", 1000)
        self.base_radius = self.mini_bot_config.base_width / 2
        self.deadzone = get_param("~deadzone", 0.0)
        self.max_speed = get_param("~max_speed", 1.0)
        self.wheel_radius = get_param("~wheel_radius", 0.02)
        self.macro_pwm_cycle_time = get_param("~macro_pwm_cycle_time", 0.1)
        self.command_timeout = rospy.Duration.from_sec(get_param("~command_timeout", 0.5))
        self.ping_timeout = get_param("~ping_timeout", 0.5)

        self.left_direction = 1 if get_param("~flip_left", False) else -1
        self.right_direction = 1 if get_param("~flip_right", True) else -1

        self.constants = ConversionConstants(
            upper_coeffs=(1.3096800864301728, 0.48987082799389764, -1.428279276496851, 67.51232442014316),
            lower_coeffs=(0.4203725313535373, 0.6271734426948566, -2.0787755742640286, 44.16671487514272),
            upper_freq=9.5,
            lower_freq=-8.5,
            upper_vel=100.47172399201347,
            lower_vel=-55.0321287689962,
            ground_vel_to_frequency=1.0 / (math.tau * self.wheel_radius),
        )
        self.velocity_to_command = get_conversion_function(self.constants)
        self.velocity_to_command = get_conversion_function(self.constants)

        self.left_pwm = MacroPwm(self.macro_pwm_cycle_time, 0.0, self.constants.lower_vel, self.constants.upper_vel)
        self.right_pwm = MacroPwm(self.macro_pwm_cycle_time, 0.0, self.constants.lower_vel, self.constants.upper_vel)

        self.last_command_time = rospy.Time.now()

        self.prev_ping_time = time.perf_counter()
        self.command = [0.0, 0.0]

        self.bridge = BridgeInterface(broadcast_address, port, device_id, {PingInfo: self.ping_callback})

        self.ping_pub = rospy.Publisher("ping", Float64, queue_size=1)
        self.motor_velocities_pub = rospy.Publisher("motor_velocities", MotorVelocities, queue_size=1)
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=1)
        self.ping_timer = rospy.Timer(rospy.Duration.from_sec(0.2), self.ping_timer_callback)

    def pwm_command_packet(self, pwm: MacroPwm, command: float) -> MotorCommand:
        if abs(command) <= self.deadzone:
            return MotorCommand.from_values(0)
        command = pwm.update(command)
        direction = 1 if command > 0 else -1
        bounded_command = int(direction * min(255, abs(command)))
        return MotorCommand.from_values(bounded_command)

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
        left_command = self.velocity_to_command(self.left_direction * left_velocity)
        right_command = self.velocity_to_command(self.right_direction * right_velocity)
        self.command = [left_command, right_command]
        velocities = MotorVelocities(velocities=[left_command, right_command])
        self.motor_velocities_pub.publish(velocities)

    def send_command(self) -> None:
        left_command = self.pwm_command_packet(self.left_pwm, self.command[0])
        right_command = self.pwm_command_packet(self.right_pwm, self.command[1])
        self.bridge.send_command([left_command, right_command])

    def ping_timer_callback(self, event) -> None:
        self.bridge.send_ping()

    def ping_callback(self, ping_info: PingInfo) -> None:
        latency = self.bridge.compute_latency(ping_info)
        self.prev_ping_time = time.perf_counter()
        self.ping_pub.publish(latency)

    def check_ping(self) -> None:
        ping_delay = time.perf_counter() - self.prev_ping_time
        if ping_delay > self.ping_timeout:
            rospy.logwarn_throttle(1.0, f"No ping received for {ping_delay:0.4f} seconds")

    def run(self) -> None:
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.check_ping()
            if rospy.Time.now() - self.last_command_time > self.command_timeout:
                self.set_velocities(0, 0)
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
