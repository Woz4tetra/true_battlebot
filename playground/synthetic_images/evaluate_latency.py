import time
from dataclasses import dataclass, field

import numpy as np
import rospy
from bw_shared.geometry.twist2d import Twist2D
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from scipy.signal import butter, lfilter


def butter_lowpass(cutoff, fs, order=5):
    return butter(order, cutoff, fs=fs, btype="low", analog=False)


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


@dataclass
class AppData:
    command_speeds: list[tuple[float, float]] = field(default_factory=list)
    measured_speeds: list[tuple[float, float]] = field(default_factory=list)
    ground_truth_speeds: list[tuple[float, float]] = field(default_factory=list)
    measured_xy: list[tuple[float, float, float]] = field(default_factory=list)
    ground_truth_xy: list[tuple[float, float]] = field(default_factory=list)


def cmd_vel_callback(data: AppData, msg: Twist):
    twist = Twist2D.from_msg(msg)
    twist_mag = twist.magnitude()
    data.command_speeds.append((time.time(), twist_mag))
    rospy.loginfo(f"Commanded speed: {twist_mag}")


def ground_truth_callback(data: AppData, msg: Odometry):
    twist = Twist2D.from_msg(msg.twist.twist)
    twist_mag = twist.magnitude()
    data.ground_truth_speeds.append((msg.header.stamp.to_sec(), twist_mag))
    data.ground_truth_xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    rospy.loginfo(f"Ground truth speed: {twist_mag}")


def measured_callback(data: AppData, msg: Odometry):
    twist = Twist2D.from_msg(msg.twist.twist)
    twist_mag = twist.magnitude()
    data.measured_speeds.append((msg.header.stamp.to_sec(), twist_mag))
    data.measured_xy.append((msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y))
    rospy.loginfo(f"Measured speed: {twist_mag}")


def main() -> None:
    rospy.init_node("evaluate_sim_latency")

    data = AppData()
    rospy.Subscriber("/mini_bot/cmd_vel", Twist, lambda msg, data=data: cmd_vel_callback(data, msg))
    rospy.Subscriber("/mini_bot/ground_truth", Odometry, lambda msg, data=data: ground_truth_callback(data, msg))
    rospy.Subscriber("/mini_bot/odom", Odometry, lambda msg, data=data: measured_callback(data, msg))

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    command_speeds = np.array(data.command_speeds) if data.command_speeds else np.array([[time.time(), 0]])
    measured_speeds = np.array(data.measured_speeds) if data.measured_speeds else np.array([[time.time(), 0]])
    ground_truth_speeds = (
        np.array(data.ground_truth_speeds) if data.ground_truth_speeds else np.array([[time.time(), 0]])
    )
    measured_xy = np.array(data.measured_xy) if data.measured_xy else np.array([[0, 0]])
    ground_truth_xy = np.array(data.ground_truth_xy) if data.ground_truth_xy else np.array([[0, 0]])

    xy = np.array(data.measured_xy)[:, 1:]

    # order = 2
    # filter_cutoff = 20  # desired cutoff frequency of the filter, Hz
    # xy = butter_lowpass_filter(xy, filter_cutoff, 50.0, order)

    xy_deltas = np.diff(xy, axis=0)
    distances = np.linalg.norm(xy_deltas, axis=1)

    measured_xy_times = measured_xy[:, 0]
    speeds = distances / np.diff(measured_xy_times)
    speed_timestamps = measured_xy_times[:-1] + np.diff(measured_xy_times) / 2

    plt.figure()
    speed_axis = plt.subplot(2, 1, 1)
    position_axis = plt.subplot(2, 1, 2)

    start_time = min(command_speeds[0, 0], measured_speeds[0, 0], ground_truth_speeds[0, 0])

    speed_axis.plot(command_speeds[:, 0] - start_time, command_speeds[:, 1], label="Commanded Speed")
    speed_axis.plot(measured_speeds[:, 0] - start_time, measured_speeds[:, 1], label="Measured Speed")
    speed_axis.plot(speed_timestamps - start_time, speeds, label="Computed Speed")
    speed_axis.plot(ground_truth_speeds[:, 0] - start_time, ground_truth_speeds[:, 1], label="Ground Truth Speed")
    speed_axis.set_xlabel("Time (s)")
    speed_axis.set_ylabel("Speed (m/s)")
    speed_axis.legend()

    position_axis.plot(measured_xy[:, 1], measured_xy[:, 2], label="Measured Position")
    position_axis.plot(ground_truth_xy[:, 0], ground_truth_xy[:, 1], label="Ground Truth Position")

    plt.show()


if __name__ == "__main__":
    main()
