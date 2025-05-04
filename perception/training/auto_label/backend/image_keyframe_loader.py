import logging
from pathlib import Path

import cv2
import numpy as np
from bw_shared.messages.header import Header
from perception_tools.messages.image import Image


class ImageKeyframeLoader:
    def __init__(self, annotations_path: Path) -> None:
        self.annotations_path = annotations_path  # images are stored in the same directory as annotations
        self.logger = logging.getLogger(self.__class__.__name__)
        self.annotations_path.mkdir(parents=True, exist_ok=True)

    def _save_image(self, image: np.ndarray, image_name: str) -> None:
        image_path = self.annotations_path / image_name
        cv2.imwrite(str(image_path), image)
        self.logger.info(f"Saved image to {image_path}")

    def add_image(self, video_name: str, image: Image) -> None:
        self._save_image(image.data, f"{video_name}-{image.header.seq}.jpg")

    def get_image(self, image_name: str) -> Image | None:
        image_path = self.annotations_path / image_name
        if image_path.exists():
            image = cv2.imread(str(image_path))
            _, seq_str = image_name.split("-")
            seq = int(seq_str.split(".")[0])
            return Image(header=Header(stamp=0.0, frame_id="", seq=seq), data=image)
        self.logger.warning(f"Image {image_name} not found.")
        return None
