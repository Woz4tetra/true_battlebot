import logging
import shutil

import cv2
import numpy as np

from auto_label.backend.manual_label_backend import ManualLabelBackend
from auto_label.config.auto_label_config import TrackerConfig


class TempImageManager:
    def __init__(self, config: TrackerConfig, manual_label_backend: ManualLabelBackend) -> None:
        self.config = config
        self.backend = manual_label_backend
        self.ai_workspace_dir = manual_label_backend.root_path / "ai_workspace"
        self.dataset_dir = manual_label_backend.root_path / "dataset"
        self.ai_workspace_dir.mkdir(parents=True, exist_ok=True)
        self.dataset_dir.mkdir(parents=True, exist_ok=True)
        self.logger = logging.getLogger(self.__class__.__name__)

    def make_batch(self, start_frame: int) -> bool:
        if self.backend.selected_video is None:
            self.logger.warning("No video selected.")
            return False
        annotation = self.backend.get_annotation(start_frame)
        if annotation is None:
            self.logger.warning(f"No annotation found for frame {start_frame}.")
            return False
        if not self.backend.did_annotation_change(annotation):
            self.logger.info(f"Annotation for frame {start_frame} has not changed.")
            return False

        self._clear_images()
        self.logger.debug(f"Creating batch for frame {start_frame}.")
        self.backend.jump_to_frame(start_frame)
        while image := self.backend.next_frame():
            seq_num = image.header.seq
            if seq_num == start_frame:
                self.logger.debug(f"Skipping frame {start_frame}.")
                continue
            if seq_num > start_frame + self.config.interpolation_max_length:
                self.logger.debug(f"Reached end of batch at frame {seq_num}.")
                break
            if self.backend.get_annotation(seq_num) is not None:
                self.logger.debug(f"Annotation found for frame {seq_num}.")
                break
            relative_index = seq_num - start_frame - 1
            self._save_image(image.data, f"{relative_index:06d}.jpg")
        return True

    def _clear_images(self) -> None:
        for path in self.ai_workspace_dir.iterdir():
            if path.is_file():
                path.unlink()
                self.logger.debug(f"Deleted image {path.name}")
        self.logger.debug("Cleared all images from workspace.")

    def _save_image(self, image: np.ndarray, image_name: str) -> None:
        image_path = self.ai_workspace_dir / image_name
        cv2.imwrite(str(image_path), image)
        self.logger.info(f"Saved image to {image_path}")

    def copy_batch_to_dataset(self, start_frame: int) -> None:
        selected_video = self.backend.selected_video
        if selected_video is None:
            self.logger.warning("No video selected.")
            return
        video_name = selected_video.video_path.stem
        for path in self.ai_workspace_dir.iterdir():
            if not path.is_file():
                continue
            index = int(path.stem)
            extension = path.suffix[1:]
            image_id = self.backend.annotations_cache.get_image_id(video_name, start_frame + index)
            path.rename(self.dataset_dir / f"{image_id}.{extension}")

        # copy the original annotation and image to the dataset directory
        original_annotation = self.backend.get_annotation(start_frame)
        if original_annotation is None:
            self.logger.warning(f"No annotation found for frame {start_frame}.")
            return
        annotation_path = self.dataset_dir / f"{original_annotation.image_id}.txt"
        shutil.copyfile(
            self.backend.annotations_cache.annotations_path / f"{original_annotation.image_id}.txt",
            annotation_path,
        )
        image_path = self.dataset_dir / f"{original_annotation.image_id}.jpg"
        shutil.copyfile(
            self.backend.annotations_cache.annotations_path / f"{original_annotation.image_id}.jpg",
            image_path,
        )
