from __future__ import annotations

import logging
from pathlib import Path

import cv2
import numpy as np
from bw_shared.messages.header import Header
from perception_tools.messages.image import Image


class VideoFrameDatabase:
    def __init__(self, images_path: Path) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.images_path = images_path
        self.current_frame = 0
        self.num_frames = len(list(images_path.glob("*.jpg")))

    def clear(self) -> None:
        pass

    def jump_to(self, frame_num: int) -> Image | None:
        if frame_num < 0 or frame_num >= self.get_num_frames():
            raise ValueError(f"Frame number {frame_num} is out of bounds.")
        return self.next(frame_num - self.current_frame)

    def next(self, jump_count: int | None = None) -> Image | None:
        if jump_count is not None and jump_count != 1:
            self.current_frame += jump_count
        else:
            self.current_frame += 1
        self.current_frame = max(0, min(self.current_frame, self.get_num_frames() - 1))

        frame_path = self.images_path / f"{self.current_frame:06d}.jpg"
        if not frame_path.exists():
            self.logger.error(f"Frame {self.current_frame} does not exist at {frame_path}.")
            return None
        frame = cv2.imread(str(frame_path))
        if frame is None:
            self.logger.error(f"Failed to read frame {self.current_frame} from {frame_path}.")
            return None

        header = Header(stamp=0.0, frame_id="", seq=self.current_frame)
        image = Image(header=header, data=np.array(frame))
        return image

    def get_num_frames(self) -> int:
        return self.num_frames

    def close(self) -> None:
        self.capture.release()
