import argparse

from bw_interfaces.msg import Trajectory
from geometry_msgs.msg import PoseStamped
from matplotlib import pyplot as plt
from rosbag.bag import Bag


def expand_list(lst: list, size: int) -> list:
    return lst + [None] * (size - len(lst))


def plot_poses(poses: list[PoseStamped], label: str) -> None:
    x = [pose.pose.position.x for pose in poses]
    y = [pose.pose.position.y for pose in poses]
    plt.plot(x, y, label=label)


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate trajectories")
    parser.add_argument("bag", type=str, help="Path to bag")
    args = parser.parse_args()

    expected_trajectories: list[Trajectory] = []
    recorded_trajectories: list[Trajectory] = []

    with Bag(args.bag, "r") as bag:
        for topic, msg, timestamp in bag.read_messages():
            if topic == "expected_trajectory":
                expected_trajectories = expand_list(expected_trajectories, msg.header.seq + 1)
                expected_trajectories[msg.header.seq] = msg
            elif topic == "recorded_trajectory":
                recorded_trajectories = expand_list(recorded_trajectories, msg.header.seq + 1)
                recorded_trajectories[msg.header.seq] = msg

    for index, (expected, recorded) in enumerate(zip(expected_trajectories, recorded_trajectories)):
        if expected is None or recorded is None:
            print(f"Missing trajectory for sequence {index}")
            continue

        expected_poses = expected.poses
        recorded_poses = recorded.poses

        plot_poses(expected_poses, "expected")
        plot_poses(recorded_poses, "recorded")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
