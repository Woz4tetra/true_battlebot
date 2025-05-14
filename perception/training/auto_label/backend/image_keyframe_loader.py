import logging
from pathlib import Path

import cv2
from bw_shared.messages.header import Header
from perception_tools.messages.image import Image


class ImageKeyframeLoader:
    def __init__(self, annotations_path: Path) -> None:
        self.annotations_path = annotations_path  # images are stored in the same directory as annotations
        self.logger = logging.getLogger(self.__class__.__name__)
        self.annotations_path.mkdir(parents=True, exist_ok=True)

    def _link_image(self, image_path: Path, image_name: str) -> None:
        linked_image_path = self.annotations_path / image_name
        if linked_image_path.exists():
            self.logger.warning(f"Image {image_name} already exists. Skipping.")
            return
        linked_image_path.symlink_to(image_path)
        self.logger.info(f"Linked image to {linked_image_path}")

    def add_image(self, video_name: str, image_path: Path) -> None:
        self._link_image(image_path, f"{video_name}-{image_path.stem}.jpg")

    def get_image(self, image_name: str) -> Image | None:
        image_path = self.annotations_path / image_name
        if image_path.exists():
            image = cv2.imread(str(image_path))
            _, seq_str = image_name.split("-")
            seq = int(seq_str.split(".")[0])
            return Image(header=Header(stamp=0.0, frame_id="", seq=seq), data=image)
        self.logger.warning(f"Image {image_name} not found.")
        return None
