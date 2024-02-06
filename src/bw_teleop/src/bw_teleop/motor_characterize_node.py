#!/usr/bin/env python
import random
import time
from threading import Thread
from typing import Generator, List, Tuple

import numpy as np
import rospy
from bw_interfaces.msg import MotorCharacterizationSample
from bw_tools.structs.teleop_bridge.motor_command import MotorCommand
from bw_tools.structs.teleop_bridge.ping_info import PingInfo
from bw_tools.typing import get_param
from std_msgs.msg import Header

from bw_teleop.bridge_interface import BridgeInterface
from bw_teleop.motor_characterization.microphone_recorder import MicrophoneRecorder
from bw_teleop.parameters import load_rosparam_robot_config


class MotorCharacterizeNode:
    def __init__(self) -> None:
        self.mini_bot_config = load_rosparam_robot_config(get_param("~robot_name", "mini_bot"))

        self.rate = get_param("~rate", 1000)
        port = get_param("~port", 4176)
        device_id = self.mini_bot_config.bridge_id
        broadcast_address = get_param("~broadcast_address", "192.168.8.255")
        mic_id = get_param("~microphone_id", 0)
        audio_directory = get_param("~audio_directory", "")
        self.ping_timeout = get_param("~ping_timeout", 0.5)

        self.left_direction = 1 if get_param("~flip_left", False) else -1
        self.right_direction = 1 if get_param("~flip_right", True) else -1

        self.prev_ping_time = time.perf_counter()
        self.should_exit = False

        self.bridge = BridgeInterface(broadcast_address, port, device_id, {PingInfo: self.ping_callback})
        self.recording = MicrophoneRecorder(audio_directory, mic_id)

        self.ping_timer = rospy.Timer(rospy.Duration.from_sec(0.2), self.ping_timer_callback)
        self.experiment_thread = Thread(target=self.run_experiment, daemon=False)
        self.sample_pub = rospy.Publisher("microphone_sample", MotorCharacterizationSample, queue_size=1)

    def ping_timer_callback(self, event) -> None:
        self.bridge.send_ping()

    def ping_callback(self, ping_info: PingInfo) -> None:
        latency = self.bridge.compute_latency(ping_info)
        self.prev_ping_time = time.perf_counter()
        if latency > self.ping_timeout:
            rospy.logerr(f"Latency is too large ({latency:0.4f} seconds). Exiting.")
            self.signal_exit()

    def signal_exit(self) -> None:
        self.should_exit = True

    def check_ping(self) -> None:
        ping_delay = time.perf_counter() - self.prev_ping_time
        if ping_delay > self.ping_timeout:
            rospy.logerr(1.0, f"No ping received for {ping_delay:0.4f} seconds. Exiting.")
            self.signal_exit()

    def iterate_samples(self, num_channels: int) -> Generator[Tuple[int, int], None, None]:
        while True:
            velocities = np.arange(-255, 256, 5, dtype=int).tolist()
            random.shuffle(velocities)
            for channel in range(num_channels):
                for velocity in velocities:
                    yield channel, velocity

    def run_experiment(self) -> None:
        num_channels = 2
        stop_motor = MotorCommand.from_values(0)
        commands = [stop_motor] * num_channels
        for channel, velocity in self.iterate_samples(num_channels):
            rospy.loginfo(f"Recording channel {channel} at velocity {velocity}")
            path = self.recording.split()
            command = MotorCommand.from_values(velocity)
            commands[channel] = command
            self.sample_pub.publish(
                MotorCharacterizationSample(
                    header=Header(stamp=rospy.Time.now()),
                    channel=channel,
                    velocity=velocity,
                    filename=path,
                )
            )
            self.spin_motor_for(commands, 3.0)

    def spin_motor_for(self, commands: List[MotorCommand], duration: float) -> None:
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < duration:
            self.bridge.send_command(commands)
            time.sleep(0.02)

    def run(self) -> None:
        self.run_experiment()
        self.recording.start()
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.check_ping()
            self.bridge.receive()
            rate.sleep()
            if self.should_exit:
                rospy.loginfo("Exiting motor characterization node")
                break

    def shutdown(self) -> None:
        self.recording.stop()


def main() -> None:
    log_level = rospy.DEBUG
    rospy.init_node("motor_characterization", log_level=log_level, disable_signals=True)
    node = MotorCharacterizeNode()
    rospy.on_shutdown(node.shutdown)
    node.run()


if __name__ == "__main__":
    main()
