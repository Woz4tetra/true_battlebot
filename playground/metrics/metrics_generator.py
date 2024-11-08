import argparse

import numpy as np
from bw_interfaces.msg import EstimatedObjectArray
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.pose2d_stamped import Pose2DStamped
from geometry_msgs.msg import PoseStamped
from matplotlib import pyplot as plt
from rosbag.bag import Bag


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate metrics from a rosbag")
    parser.add_argument("bagfile", type=str, help="Path to the rosbag file")
    args = parser.parse_args()

    bag_path = args.bagfile
    bag = Bag(bag_path)

    positions: dict[str, list[Pose2DStamped]] = {}

    for topic, msg, timestamp in bag.read_messages(topics=["/filtered_states"]):  # type: ignore
        if topic == "/filtered_states":
            msg: EstimatedObjectArray
            for robot in msg.robots:
                positions.setdefault(robot.label, []).append(
                    Pose2DStamped.from_msg(PoseStamped(header=robot.header, pose=robot.pose.pose))
                )

    cage_size = 2.35
    cage_half = cage_size / 2
    bins = 10
    label = "main_bot"
    robot_data = positions[label]
    x = [pose.pose.x for pose in robot_data]
    y = [pose.pose.y for pose in robot_data]
    plt.hist2d(x, y, bins=[np.linspace(-cage_half, cage_half, bins), np.linspace(-cage_half, cage_half, bins)])
    # plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
