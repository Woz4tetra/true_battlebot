import argparse
import time

import rospy
import tqdm
from bw_interfaces.msg import EstimatedObject
from rosbag.bag import Bag
from sensor_msgs.msg import CameraInfo, Image


def main() -> None:
    rospy.init_node("play_bag_through_motion")

    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="Path to the bag file")
    args = parser.parse_args()

    topics = {
        "/filter/field": EstimatedObject,
        "/camera_1/image_rect": Image,
        "/camera_1/camera_info": CameraInfo,
    }

    publishers = {topic: rospy.Publisher(topic, msg_type, queue_size=10) for topic, msg_type in topics.items()}

    def get_real_time() -> float:
        return time.perf_counter() - start_time

    def get_bag_time(bag_time: float) -> float:
        return bag_time - bag_start_time

    bag_file = args.bag_file
    with Bag(bag_file, "r") as bag:
        bag_duration = round(bag.get_end_time() - bag.get_start_time(), 2)
        with tqdm.tqdm(
            total=bag_duration,
            bar_format="{desc}: {percentage:3.0f}%|{bar}| {n:.2f}/{total_fmt} [{elapsed}<{remaining}]",
        ) as pbar:
            start_time = time.perf_counter()
            bag_start_time = bag.get_start_time()
            for topic, msg, timestamp in bag.read_messages(topics=topics.keys()):  # type: ignore
                bag_time = get_bag_time(timestamp.to_sec())
                real_time = get_real_time()
                pbar.update(round(bag_time - pbar.n, 2))

                if bag_time > real_time:
                    rospy.sleep(bag_time - real_time)

                publishers[topic].publish(msg)

                if rospy.is_shutdown():
                    break


if __name__ == "__main__":
    main()
