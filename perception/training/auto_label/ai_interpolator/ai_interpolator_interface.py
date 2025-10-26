import logging
from abc import ABC, abstractmethod
from pathlib import Path

import cv2
import numpy as np
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointImage


class AiInterpolatorInterface(ABC):
    @abstractmethod
    def interpolate(
        self, images_dir: Path, start_annotation: YoloKeypointImage, images_width: int, images_height: int
    ) -> None:
        """Interpolate keypoints for a sequence of images starting from a given annotation.

        Args:
            images_dir (Path): Directory containing the images to interpolate.
            start_annotation (YoloKeypointImage): The annotation of the starting image.
            images_width (int): Width of the images.
            images_height (int): Height of the images.
        """
        pass

    def load_images_alphabetically(self, images_dir: Path) -> list[np.ndarray]:
        images = []
        for image_path in sorted(images_dir.iterdir()):
            if image_path.suffix.lower() not in {".jpg", ".png"}:
                continue
            image = cv2.imread(str(image_path))
            if image is None:
                self.logger.warning(f"Failed to read image: {image_path}")
                continue
            images.append(image)
        return images

    def save_annotation(self, images_dir: Path, annotation: YoloKeypointImage) -> None:
        annotation_path = images_dir / f"{annotation.image_id}.txt"
        with open(annotation_path, "w") as file:
            file.write(annotation.to_txt())

    def remove_pytorch_loggers(self) -> None:
        for logger_name in ["root", "torch", "torchvision", "cotracker", "LoggingTensor", "hydra", "networkx"]:
            logger = logging.getLogger(logger_name)
            if not logger:
                self.logger.warning(f"Logger {logger_name} not found")
                continue
            logger.setLevel(logging.WARNING)
            for handler in logger.handlers:
                logger.removeHandler(handler)
