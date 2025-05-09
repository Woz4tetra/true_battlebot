import logging
from threading import Thread

from perception_tools.training.keypoints_config import KeypointsConfig

from auto_label.ai_interpolator.ai_interpolator import AiInterpolator
from auto_label.ai_interpolator.temp_image_manager import TempImageManager
from auto_label.backend.manual_label_backend import ManualLabelBackend
from auto_label.config.auto_label_config import AutoLabelConfig


class InterpolationProcessManager:
    def __init__(
        self, config: AutoLabelConfig, keypoints_config: KeypointsConfig, manual_label_backend: ManualLabelBackend
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.config = config
        self.keypoints_config = keypoints_config
        self.manual_label_backend = manual_label_backend
        self.ai_interpolator = AiInterpolator(self.config.tracker, self.keypoints_config)
        self.temp_image_manager = TempImageManager(self.config.tracker, self.manual_label_backend)
        self.interpolate_thread: Thread | None = None

    def interpolate(self, frame_indices: tuple[int, ...]) -> bool:
        if self.interpolate_thread is not None and self.interpolate_thread.is_alive():
            return False
        self.interpolate_thread = Thread(target=self._interpolate_task, args=(frame_indices,))
        self.interpolate_thread.start()
        return True

    def _interpolate_task(self, frame_indices: tuple[int, ...]) -> None:
        for start_frame in frame_indices:
            self._interpolate_from_index(start_frame)

    def _interpolate_from_index(self, start_frame: int) -> None:
        if not self.temp_image_manager.make_batch(start_frame):
            self.logger.warning(f"Failed to create batch for frame {start_frame}.")
            return
        annotation = self.manual_label_backend.get_annotation(start_frame)
        if annotation is None:
            self.logger.warning(f"No annotation found for frame {start_frame}.")
            return
        image = self.manual_label_backend.next_frame(jump_count=0)
        if image is None:
            self.logger.warning(f"No image found for frame {start_frame}.")
            return
        self.ai_interpolator.interpolate(
            self.temp_image_manager.ai_workspace_dir, annotation, (image.width, image.height)
        )
        self.manual_label_backend.hashes_cache.add_annotation(annotation)
        self.temp_image_manager.copy_batch_to_dataset(start_frame)

        self.logger.info(f"Interpolation complete for frame {start_frame}.")
