import argparse
import time

import rospy
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
        start_time = time.perf_counter()
        bag_start_time = bag.get_start_time()
        for topic, msg, timestamp in bag.read_messages(topics=topics.keys()):  # type: ignore
            bag_time = get_bag_time(timestamp.to_sec())
            real_time = get_real_time()

            if bag_time > real_time:
                rospy.sleep(bag_time - real_time)

            publishers[topic].publish(msg)

            if rospy.is_shutdown():
                break


if __name__ == "__main__":
    main()
