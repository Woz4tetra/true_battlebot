#!/usr/bin/env python
import random
import time
from typing import Generator

import numpy as np
import rospy
import serial
from bw_interfaces.msg import MotorCharacterizationSample, MotorCharacterizationSampleArray, TelemetryStatus
from bw_tools.get_param import get_param
from geometry_msgs.msg import Twist
from std_msgs.msg import Header


def iterate_samples(num_samples: int) -> Generator[int, None, None]:
    velocities = np.linspace(0, 1, num_samples // 2, dtype=float).tolist()
    velocities.pop(0)
    random.shuffle(velocities)
    velocities = velocities + [0] + [-x for x in velocities]
    for velocity in velocities:
        yield velocity


class HallEffectCharacterizeNode:
    def __init__(self) -> None:
        random.seed(4176)
        self.ping_threshold = rospy.Duration(get_param("~ping_threshold", 1.0))  # type: ignore
        self.ping_timeout = rospy.Duration(get_param("~ping_timeout", 10.0))  # type: ignore
        self.num_samples = get_param("~num_samples", 100)
        self.sample_duration = rospy.Duration(get_param("~sample_duration", 3.0))  # type: ignore
        self.spin_up_time = rospy.Duration(get_param("~spin_up_time", 1.0))  # type: ignore
        self.sensor_path = get_param("~sensor_path", "/dev/ttyACM0")
        self.prev_ping_time = rospy.Time()

        assert self.ping_threshold < self.ping_timeout, "Ping threshold must be less than ping timeout"
        assert self.spin_up_time < self.sample_duration, "Spin up time must be less than sample duration"

        self.device = serial.Serial(self.sensor_path, 115200)
        self.prev_ticks = (0, 0)

        self.sample_pub = rospy.Publisher("motor_sample", MotorCharacterizationSampleArray, queue_size=100)
        self.telemetry_sub = rospy.Subscriber(
            "telemetry_status", TelemetryStatus, self.telemetry_callback, queue_size=10
        )
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def telemetry_callback(self, msg: TelemetryStatus) -> None:
        if msg.is_connected and msg.is_armed:
            self.prev_ping_time = rospy.Time.now()

    def set_linear_velocity(self, velocity: float) -> None:
        command = Twist()
        command.linear.x = velocity
        self.twist_pub.publish(command)

    def is_ping_ok(self) -> bool:
        ping_delay = rospy.Time.now() - self.prev_ping_time
        return ping_delay < self.ping_timeout

    def wait_for_ping(self) -> bool:
        rospy.loginfo("Waiting for ping")
        start_time = rospy.Time.now()
        while True:
            if rospy.is_shutdown():
                return False
            now = rospy.Time.now()
            ping_delay = now - self.prev_ping_time
            if ping_delay < self.ping_threshold:
                rospy.loginfo("Ping received")
                return True
            wait_delay = now - start_time
            if wait_delay > self.ping_timeout:
                rospy.logerr(f"No ping received for {ping_delay.to_sec():0.4f} seconds. Exiting.")
                return False

    def get_num_ticks_and_reset(self) -> tuple[int, int]:
        if not self.device.in_waiting:
            return 0, 0
        while self.device.in_waiting:
            row = self.device.readline()
        channel_0, channel_1 = [int(x) for x in row.split(b"\t")]
        self.device.write(b"0\n")
        self.prev_ticks = channel_0, channel_1
        return self.prev_ticks

    def run(self) -> None:
        while not self.device.in_waiting:
            rospy.loginfo("Waiting for data from sensor")
            rospy.sleep(1)
        for index, velocity in enumerate(iterate_samples(self.num_samples)):
            if not self.wait_for_ping():
                break

            percent_done = (index + 1) / self.num_samples * 100
            rospy.loginfo(f"Recording at velocity {velocity}. {percent_done:.2f}% done")
            are_samples_ok = True
            self.set_linear_velocity(velocity)
            rospy.sleep(self.spin_up_time)
            self.get_num_ticks_and_reset()
            t0 = time.perf_counter()
            if not self.is_ping_ok():
                rospy.logwarn("Ping delay too large. Discarding sample.")
                are_samples_ok = False
            rospy.sleep(self.sample_duration - self.spin_up_time)
            t1 = time.perf_counter()
            duration = t1 - t0
            timestamp = rospy.Time.now()
            channel_0_ticks, channel_1_ticks = self.get_num_ticks_and_reset()

            samples = MotorCharacterizationSampleArray()
            samples.samples.append(
                MotorCharacterizationSample(
                    header=Header(stamp=timestamp),
                    channel=0,
                    velocity=velocity,
                    filename="",
                    feedback=float(channel_0_ticks),
                    valid=are_samples_ok,
                    duration=duration,
                )
            )
            samples.samples.append(
                MotorCharacterizationSample(
                    header=Header(stamp=timestamp),
                    channel=1,
                    velocity=velocity,
                    filename="",
                    feedback=float(channel_1_ticks),
                    valid=are_samples_ok,
                    duration=duration,
                )
            )
            self.sample_pub.publish(samples)


def main() -> None:
    log_level = rospy.DEBUG
    rospy.init_node("motor_characterization", log_level=log_level)
    node = HallEffectCharacterizeNode()
    node.run()


if __name__ == "__main__":
    main()
