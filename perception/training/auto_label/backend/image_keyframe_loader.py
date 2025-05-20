import logging
from pathlib import Path

import cv2
from bw_shared.messages.header import Header
from perception_tools.messages.image import Image


class ImageKeyframeLoader:
    def __init__(self, annotations_path: Path, dataset_path: Path) -> None:
        self.annotations_path = annotations_path  # images are stored in the same directory as annotations
        self.dataset_path = dataset_path
        self.logger = logging.getLogger(self.__class__.__name__)

    def _link_image(self, image_path: Path, linked_image_path: Path) -> None:
        if linked_image_path.exists():
            self.logger.warning(f"Image {linked_image_path} already exists. Removing.")
            linked_image_path.unlink()
        try:
            linked_image_path.symlink_to(image_path)
        except FileExistsError as e:
            self.logger.warning(f"Failed to create symlink for {linked_image_path}: {e}")
            return
        self.logger.info(f"Linked image {linked_image_path} to {linked_image_path}")

    def add_image(self, video_name: str, image_path: Path) -> None:
        image_name = f"{video_name}-{image_path.stem}.jpg"
        annotation_image_path = self.annotations_path / image_name
        dataset_image_path = self.dataset_path / image_name
        self._link_image(image_path, annotation_image_path)
        self._link_image(image_path, dataset_image_path)

    def get_image(self, image_name: str) -> Image | None:
        image_path = self.annotations_path / image_name
        if image_path.exists():
            image = cv2.imread(str(image_path))
            _, seq_str = image_name.split("-")
            seq = int(seq_str.split(".")[0])
            return Image(header=Header(stamp=0.0, frame_id="", seq=seq), data=image)
        self.logger.warning(f"Image {image_name} not found.")
        return None

    def delete_image(self, image_name: str) -> None:
        annotation_image_path = self.annotations_path / image_name
        dataset_image_path = self.dataset_path / image_name
        if annotation_image_path.exists():
            annotation_image_path.unlink()
            self.logger.info(f"Deleted image {annotation_image_path}")
        else:
            self.logger.warning(f"Image {annotation_image_path} not found.")
        if dataset_image_path.exists():
            dataset_image_path.unlink()
            self.logger.info(f"Deleted image {dataset_image_path}")
        else:
            self.logger.warning(f"Image {dataset_image_path} not found.")
