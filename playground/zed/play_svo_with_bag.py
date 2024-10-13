#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK

import argparse
from pathlib import Path

import argcomplete
import pyzed.sl as sl
import rospy
from app.camera.zed.helpers import zed_status_to_str
from bw_interfaces.msg import CageCorner
from bw_shared.script_tools.directories import BAGS_DIR, SVO_DIR
from bw_shared.script_tools.list_files import list_files
from perception_tools.rosbridge.ros_publisher import RosPublisher
from rosbag.bag import Bag
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Empty


def open_svo(svo_path: Path) -> sl.Camera:
    input_type = sl.InputType()
    # Set init parameter to run from the .svo
    input_type.set_from_svo_file(str(svo_path))

    camera = sl.Camera()
    init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
    init_params.coordinate_units = sl.UNIT.METER

    status = camera.open(init_params)
    success = status == sl.ERROR_CODE.SUCCESS
    if not success:
        raise Exception(f"Failed to open camera with error code {status}")

    return camera


def open_bag(bag_path: Path) -> Bag:
    return Bag(str(bag_path), "r")


def main() -> None:
    bag_options = list_files(BAGS_DIR, "bag")
    svo_options = list_files(SVO_DIR, "svo2")
    parser = argparse.ArgumentParser(description="Play SVO with bag")
    parser.add_argument("svo", type=str, help="Path to SVO file", choices=svo_options.keys())
    parser.add_argument("bag", type=str, help="Path to bag file", choices=bag_options.keys())
    parser.add_argument("--start-time", type=float, help="Start time in seconds", default=0.0)

    argcomplete.autocomplete(parser)

    args = parser.parse_args()

    rospy.init_node("play_svo_with_bag")

    svo_path = svo_options[args.svo]
    bag_path = bag_options[args.bag]
    start_time = args.start_time

    if svo_path.stem != bag_path.stem:
        print("WARNING: SVO and bag file names do not match")

    print(f"Playing SVO {svo_path} with bag {bag_path}")

    runtime_parameters = sl.RuntimeParameters()

    camera = open_svo(svo_path)
    bag = open_bag(bag_path)

    color_image = sl.Mat()

    topics = {
        "/set_cage_corner": CageCorner,
        "/manual_plane_request": Empty,
        "/camera_1/camera_info": CameraInfo,
        "/camera_1/image_rect/compressed": CompressedImage,
    }

    publishers = {topic: RosPublisher(topic, msg_type) for topic, msg_type in topics.items()}
    rospy.sleep(2)  # Wait for publishers to connect

    bag_time = bag.get_start_time()
    bag_iters = bag.read_messages(start_time=rospy.Time.from_sec(bag_time + start_time), topics=list(topics.keys()))
    try:
        while not rospy.is_shutdown():
            status = camera.grab(runtime_parameters)
            if status != sl.ERROR_CODE.SUCCESS:
                print(f"Failed to grab frame with error code {zed_status_to_str(status)}")
                break
            camera.retrieve_image(color_image, sl.VIEW.LEFT)
            image_time = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() * 1e-9

            while bag_time < image_time:
                topic, msg, timestamp = next(bag_iters)  # type: ignore
                publishers[topic].publish(msg)
                bag_time = timestamp.to_sec()

            print(f"Timestamp: {image_time}, Bag time: {bag_time}, Diff: {image_time - bag_time}")
    finally:
        camera.close()
        bag.close()


if __name__ == "__main__":
    main()
