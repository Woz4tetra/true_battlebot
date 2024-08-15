import argparse
import time
from dataclasses import dataclass, field
from queue import Empty, Queue
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


@dataclass
class AppData:
    window_name: str
    save_video: bool
    save_directory: str
    video_path: str
    image_queue: Queue[Image] = field(default_factory=Queue)
    bridge: CvBridge = field(default_factory=CvBridge)
    time_samples: Optional[list[float]] = field(default_factory=list)
    num_samples: int = 30
    video_writer: Optional[cv2.VideoWriter] = None


def callback(app: AppData, msg: Image) -> None:
    app.image_queue.put(msg)


def tick(app: AppData):
    try:
        msg = app.image_queue.get(timeout=0.1)
    except Empty:
        return

    try:
        image = app.bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        print(e)
        return
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    time_samples = app.time_samples
    video_path = app.video_path

    if time_samples is not None:
        if len(time_samples) <= app.num_samples:
            time_samples.append(msg.header.stamp.to_sec())
        else:
            time_deltas = np.diff(time_samples)
            average_fps = 1 / np.mean(time_deltas)
            image_size = (image.shape[1], image.shape[0])
            print(f"Average FPS: {average_fps}. Size: {image_size[0]}x{image_size[1]}")
            if app.save_video and average_fps > 0:
                app.video_writer = cv2.VideoWriter(
                    video_path, cv2.VideoWriter_fourcc(*"MJPG"), int(average_fps), image_size
                )
                print(f"Saving video to {video_path}")
            app.time_samples = None

    if app.video_writer is not None:
        app.video_writer.write(image)

    cv2.imshow(app.window_name, image)
    key = chr(cv2.waitKey(1) & 0xFF)
    if key == "q":
        raise KeyboardInterrupt
    elif key == "p":
        filename = f"{app.save_directory}/{time.time()}.jpg"
        cv2.imwrite(filename, image)
        print(f"Saved image to {filename}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("topic", type=str, help="Image topic to subscribe to.")
    parser.add_argument("-s", "--save-directory", default=".", type=str, help="Directory to save images")
    parser.add_argument("-v", "--video", action="store_true", help="Save video to file")
    args = parser.parse_args()

    topic = args.topic
    save_directory = args.save_directory
    save_video = args.video

    app = AppData(topic, save_video, save_directory, f"{save_directory}/{time.time()}.avi")
    cv2.namedWindow(topic)

    rospy.init_node("camera_topic_recorder")
    rospy.Subscriber(topic, Image, lambda msg: callback(app, msg), queue_size=1)

    try:
        while not rospy.is_shutdown():
            tick(app)
    finally:
        if app.video_writer is not None:
            app.video_writer.release()


if __name__ == "__main__":
    main()
