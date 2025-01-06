from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np
import tqdm
from bw_shared.messages.header import Header
from perception_tools.messages.image import Image


class VideoPlayer:
    def __init__(self, video_path: Path) -> None:
        self.video_path = video_path

    def __enter__(self) -> VideoPlayer:
        self.capture = cv2.VideoCapture(str(self.video_path))
        video_duration = self.capture.get(cv2.CAP_PROP_FRAME_COUNT) / self.capture.get(cv2.CAP_PROP_FPS)
        self.pbar = tqdm.tqdm(total=video_duration, bar_format="{l_bar}{bar}| {n:.2f}/{total:.2f}")
        return self

    def next(self, jump_to: int | None = None) -> Image | None:
        if not self.capture.isOpened():
            return None
        if jump_to is not None:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, jump_to)
        video_time = self.capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        self.pbar.update(video_time - self.pbar.n)
        frame_number = int(self.capture.get(cv2.CAP_PROP_POS_FRAMES))
        ret, frame = self.capture.read()
        if not ret:
            return None
        header = Header(stamp=video_time, frame_id="", seq=frame_number)
        return Image(header=header, data=np.array(frame))

    def get_num_frames(self) -> int:
        return int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.capture.release()
