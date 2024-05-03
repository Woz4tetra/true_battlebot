import argparse
import json
import math
from dataclasses import dataclass, field

import rospy
from bw_interfaces.msg import EstimatedObject
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.twist2d import Twist2D
from bw_tools.structs.xyz import XYZ
from nav_msgs.msg import Odometry


@dataclass
class RobotState:
    timestamp: float
    pose: Pose2D
    twist: Twist2D


@dataclass
class AppData:
    time_interval: float
    states: dict[str, list[RobotState]]

    def __post_init__(self) -> None:
        self.prev_sample_times = {label: 0.0 for label in self.states.keys()}
        self.start_time = 0.0
        self.field_size = XYZ(0.0, 0.0, 0.0)


def ground_truth_callback(label: str, app: AppData, msg: Odometry) -> None:
    if app.field_size == XYZ(0.0, 0.0, 0.0):
        return
    current_time = msg.header.stamp.to_sec()
    if app.start_time == 0.0:
        app.start_time = current_time
        print("Started recording")

    timestamp = current_time - app.start_time
    prev_sample_time = app.prev_sample_times[label]
    if timestamp - prev_sample_time < app.time_interval:
        return
    app.prev_sample_times[label] = timestamp
    pose = Pose2D.from_msg(msg.pose.pose)
    twist = Twist2D.from_msg(msg.twist.twist)
    app.states[label].append(RobotState(timestamp, pose, twist))


def field_callback(app: AppData, msg: EstimatedObject) -> None:
    app.field_size = XYZ(msg.size.x, msg.size.y, msg.size.z)
    print("Field received")


def states_to_dict(states: list[RobotState], field_size: XYZ) -> dict:
    x_scale = field_size.x / 2
    y_scale = field_size.y / 2
    start_x = states[0].pose.x / x_scale
    start_y = states[0].pose.y / y_scale
    start_theta = math.degrees(states[0].pose.theta)
    return {
        "type": "follow",
        "init": {"type": "relative", "x": start_x, "y": start_y, "theta": start_theta},
        "sequence": [
            {
                "timestamp": state.timestamp,
                "x": state.pose.x / x_scale,
                "y": state.pose.y / y_scale,
                "theta": math.degrees(state.pose.theta),
                "vx": state.twist.x / x_scale,
                "vy": state.twist.y / y_scale,
                "vtheta": math.degrees(state.twist.theta),
            }
            for state in states
        ],
    }


def write_to_file(app: AppData, output_prefix: str) -> None:
    for label, states in app.states.items():
        output_file = output_prefix + f"_{label}.json"
        print(f"Writing data to {output_file}.")
        print(f"Label: {label}, States: {len(states)}")
        with open(output_file, "w") as file:
            json.dump(states_to_dict(states, app.field_size), file, indent=4)


def main() -> None:
    parser = argparse.ArgumentParser(description="Record robot pose and twist data.")
    parser.add_argument("prefix", type=str, help="Path to the dir file with prefix.")
    parser.add_argument("-t", "--time", type=float, default=0.1, help="The time interval between data points.")
    args = parser.parse_args()

    topics = {
        "/main_bot/ground_truth": "chase",
        "/opponent_1/ground_truth": "aggressor",
    }

    output_prefix = args.prefix
    time_interval = args.time

    app = AppData(time_interval, states={label: [] for label in topics.values()})

    for topic, name in topics.items():
        rospy.Subscriber(topic, Odometry, lambda msg, n=name: ground_truth_callback(n, app, msg), queue_size=10)
    rospy.Subscriber("/filter/field", EstimatedObject, lambda msg: field_callback(app, msg), queue_size=1)
    try:
        rospy.spin()
    finally:
        write_to_file(app, output_prefix)


if __name__ == "__main__":
    rospy.init_node("record_robot_data", log_level=rospy.DEBUG)
    main()
