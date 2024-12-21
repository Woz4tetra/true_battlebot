import os
from glob import glob
from typing import Generator

import cv2
import numpy as np
from cv_bridge import CvBridge
from rosbag.bag import Bag
from tqdm import tqdm

BRIDGE = CvBridge()


def load_image_from_bag(path: str, skip_interval: float, filters: tuple[str, ...]) -> Generator[np.ndarray, None, None]:
    bag = Bag(path)
    prev_record_time = 0.0
    with tqdm(total=bag.get_message_count(topic_filters=filters)) as pbar:
        for topic, msg, timestamp in bag.read_messages(topics=filters):  # type: ignore
            pbar.update(1)
            if timestamp.to_sec() - prev_record_time < skip_interval:
                continue
            prev_record_time = timestamp.to_sec()
            type_str = str(type(msg))
            if "sensor_msgs__Image" in type_str:
                yield cv2.cvtColor(BRIDGE.imgmsg_to_cv2(msg), cv2.COLOR_RGB2BGR)
    bag.close()


def load_image_from_video(
    video_path: str, skip_interval: float, filters: tuple[str, ...]
) -> Generator[np.ndarray, None, None]:
    video = cv2.VideoCapture(video_path)
    prev_frame_num = 0
    fps = video.get(cv2.CAP_PROP_FPS)
    print(f"FPS: {fps}")
    skip_frames = int(skip_interval * fps)
    if skip_frames <= 0:
        skip_frames = 1
    with tqdm(total=int(video.get(cv2.CAP_PROP_FRAME_COUNT))) as pbar:
        for frame_num in range(0, int(video.get(cv2.CAP_PROP_FRAME_COUNT)), skip_frames):
            video.set(cv2.CAP_PROP_POS_FRAMES, frame_num)
            ret, frame = video.read()
            if not ret:
                break
            yield frame
            pbar.update(frame_num - prev_frame_num)
            prev_frame_num = frame_num
    video.release()


def load_images_from_dir(image_dir: str, filters: tuple[str, ...]) -> Generator[np.ndarray, None, None]:
    all_paths = glob(os.path.join(image_dir, "*.png")) + glob(os.path.join(image_dir, "*.jpg"))
    with tqdm(total=int(len(all_paths))) as pbar:
        for image_path in sorted(all_paths):
            pbar.update(1)
            yield cv2.imread(image_path)


def load_images(image_source: str, skip_interval: float, filters: tuple[str, ...]) -> Generator[np.ndarray, None, None]:
    if os.path.isfile(image_source):
        if image_source.endswith(".bag"):
            images = load_image_from_bag(image_source, skip_interval, filters)
        else:
            images = load_image_from_video(image_source, skip_interval, filters)
    elif os.path.isdir(image_source):
        images = load_images_from_dir(image_source, filters)
    else:
        raise ValueError(f"Invalid input: {image_source}")

    return images
