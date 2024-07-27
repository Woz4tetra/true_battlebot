#!/usr/bin/env python
import random
import time
from threading import Thread
from typing import Generator

import numpy as np
import rospy
from bw_interfaces.msg import MotorCharacterizationSample, MotorVelocities, TelemetryStatus
from bw_tools.get_param import get_param
from std_msgs.msg import Header

from bw_teleop.motor_characterization.microphone_recorder import MicrophoneRecorder
from bw_teleop.parameters import load_rosparam_robot_config


class MotorCharacterizeNode:
    def __init__(self) -> None:
        self.mini_bot_config = load_rosparam_robot_config(get_param("~robot_name", "mini_bot"))

        self.rate = get_param("~rate", 1000)
        port = get_param("~port", 4176)
        mic_id = get_param("~microphone_id", 0)
        audio_directory = get_param("~audio_directory", "")
        self.ping_timeout = get_param("~ping_timeout", 3.0)
        self.discard_timeout = get_param("~discard_timeout", 0.5)
        self.channel = get_param("~channel", 0)

        random.seed(port)

        self.prev_ping_time = 0.0
        self._should_exit = False
        self.is_sample_valid = False

        self.recording = MicrophoneRecorder(audio_directory, mic_id)

        self.experiment_thread = Thread(target=self.run_experiment, daemon=False)
        self.sample_pub = rospy.Publisher("microphone_sample", MotorCharacterizationSample, queue_size=1)
        self.telemetry_sub = rospy.Subscriber(
            "telemetry_status", TelemetryStatus, self.telemetry_callback, queue_size=1
        )
        self.motor_command_pub = rospy.Publisher("motor_command", MotorVelocities, queue_size=1)

    def signal_exit(self) -> None:
        rospy.loginfo("Signaling exit")
        self._should_exit = True

    def wait_for_ping(self) -> None:
        rospy.loginfo("Waiting for ping")
        while True:
            if self.should_exit():
                break
            ping_delay = time.perf_counter() - self.prev_ping_time
            if ping_delay < self.ping_timeout:
                break
        rospy.loginfo("Ping received")

    def check_ping(self) -> None:
        ping_delay = time.perf_counter() - self.prev_ping_time
        if ping_delay > self.discard_timeout:
            rospy.logwarn(f"Delay is too large ({ping_delay:0.4f} seconds). Discarding sample.")
            self.is_sample_valid = False
        if ping_delay > self.ping_timeout:
            rospy.logerr(f"No ping received for {ping_delay:0.4f} seconds. Exiting.")
            self.signal_exit()

    def iterate_samples(self) -> Generator[int, None, None]:
        while True:
            velocities = np.arange(-255, 256, 5, dtype=int).tolist()
            random.shuffle(velocities)
            for velocity in velocities:
                yield velocity

    def telemetry_callback(self, msg: TelemetryStatus) -> None:
        self.prev_ping_time = time.perf_counter()

    def run_experiment(self) -> None:
        num_channels = 2
        command = MotorVelocities(velocities=[0] * num_channels)
        for velocity in self.iterate_samples():
            self.wait_for_ping()
            if self.should_exit():
                break
            is_sample_valid = False
            rospy.loginfo(f"Recording channel {self.channel} at velocity {velocity}")
            path = self.recording.split()
            command.velocities[self.channel] = velocity
            while not is_sample_valid:
                self.is_sample_valid = True
                self.spin_motor_for(command, 3.0)
                is_sample_valid = self.is_sample_valid
                self.sample_pub.publish(
                    MotorCharacterizationSample(
                        header=Header(stamp=rospy.Time.now()),
                        channel=self.channel,
                        velocity=velocity,
                        filename=path,
                        valid=is_sample_valid,
                    )
                )
        command.velocities[self.channel] = 0
        self.motor_command_pub.publish(command)
        rospy.loginfo("Finished recording samples")

    def should_exit(self) -> bool:
        return self._should_exit or rospy.is_shutdown()

    def spin_motor_for(self, command: MotorVelocities, duration: float) -> None:
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < duration:
            if self.should_exit():
                break
            self.motor_command_pub.publish(command)
            time.sleep(0.02)

    def run(self) -> None:
        self.wait_for_ping()
        self.experiment_thread.start()
        self.recording.start()
        rate = rospy.Rate(self.rate)
        while True:
            self.check_ping()
            rate.sleep()
            if self.should_exit():
                rospy.loginfo("Exiting motor characterization node")
                break
        rospy.loginfo("Main loop exited")
        self.recording.stop()


def main() -> None:
    log_level = rospy.DEBUG
    rospy.init_node("motor_characterization", log_level=log_level)
    node = MotorCharacterizeNode()
    node.run()


if __name__ == "__main__":
    main()
