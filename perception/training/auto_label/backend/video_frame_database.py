from __future__ import annotations

import logging
from pathlib import Path
from typing import OrderedDict

import cv2
import numpy as np
from bw_shared.messages.header import Header
from perception_tools.messages.image import Image

CACHE_SIZE = 100


class VideoFrameCache:
    def __init__(self, video_path: Path) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.video_path = video_path
        self.current_frame = -1
        self.capture = cv2.VideoCapture(str(self.video_path))
        self._cache: OrderedDict[int, Image] = OrderedDict()
        self.num_frames = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))

    def clear(self) -> None:
        self._cache.clear()
        self.logger.info("Cleared video frame cache.")

    def jump_to(self, frame_num: int) -> None:
        if frame_num < 0 or frame_num >= self.get_num_frames():
            raise ValueError(f"Frame number {frame_num} is out of bounds.")
        self.current_frame = frame_num
        self.capture.set(cv2.CAP_PROP_POS_FRAMES, self.current_frame)

    def next(self, jump_count: int | None = None) -> Image | None:
        if jump_count is not None and jump_count != 1:
            self.current_frame += jump_count
        else:
            self.current_frame += 1
        self.current_frame = max(0, min(self.current_frame, self.get_num_frames() - 1))
        if self.current_frame in self._cache:
            return self._cache[self.current_frame]
        if not self.capture.isOpened():
            return None
        if jump_count is not None and jump_count != 1:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, self.current_frame)
        video_time = self.capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        ret, frame = self.capture.read()
        if not ret:
            return None
        header = Header(stamp=video_time, frame_id="", seq=self.current_frame)
        image = Image(header=header, data=np.array(frame))
        self._cache[self.current_frame] = image
        if len(self._cache) > CACHE_SIZE:
            self._cache.popitem(last=False)
        return image

    def get_num_frames(self) -> int:
        return self.num_frames

    def close(self) -> None:
        self.capture.release()
