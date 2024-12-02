import cv2
import numpy as np


def load_from_video(video_file: str, frame_time: float = 0.0) -> np.ndarray:
    cap = cv2.VideoCapture(video_file)
    if frame_time > 0:
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_number = int(frame_time * fps)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)
    success, frame = cap.read()
    if not success:
        raise RuntimeError(f"Failed to read frame at time {frame_time}")
    return frame
