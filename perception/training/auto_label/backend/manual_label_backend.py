import logging
from pathlib import Path

from perception_tools.messages.image import Image
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointImage

from auto_label.backend.annotation_cache import AnnotationCache
from auto_label.backend.hashes_cache import HashesCache
from auto_label.backend.image_keyframe_loader import ImageKeyframeLoader
from auto_label.backend.video_frame_database import VideoFrameCache
from auto_label.backend.video_source_collection import VideoSourceCollection


class ManualLabelBackend:
    def __init__(self, root_path: Path) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.root_path = root_path
        self.annotations_path = root_path / "annotations"
        self.hashes_path = root_path / "hashes"
        self._make_dirs()
        self.video_source = VideoSourceCollection(root_path)
        self.annotations_cache = AnnotationCache(self.annotations_path)
        self.hashes_cache = HashesCache(self.hashes_path)
        self.image_keyframe_loader = ImageKeyframeLoader(self.annotations_path)
        self.selected_video: VideoFrameCache | None = None

    def get_video_name(self) -> str:
        return self.selected_video.video_path.name if self.selected_video else ""

    def _make_dirs(self) -> None:
        if not self.root_path.exists():
            self.logger.info(f"Database path {self.root_path} does not exist. Creating it.")
            self.root_path.mkdir(parents=True, exist_ok=True)
        self.annotations_path.mkdir(parents=True, exist_ok=True)
        self.hashes_path.mkdir(parents=True, exist_ok=True)

    def add_video(self, video_path: Path) -> None:
        self.video_source.add_video(video_path)
        self.video_source.select_video(video_path.name)

    def select_video(self, video_name: str) -> None:
        self.selected_video = self.video_source.select_video(video_name)
        if self.selected_video is None:
            self.logger.warning(f"Video {video_name} not found in the database.")
            return
        self.annotations_cache.clear()
        self.hashes_cache.clear()
        self.video_source.clear_selected()

    def next_frame(self, jump_count: int | None = None) -> Image | None:
        if self.selected_video is None:
            self.logger.warning("No video selected.")
            return None
        return self.selected_video.next(jump_count)

    def add_annotation(self, image: Image, annotation: YoloKeypointImage) -> None:
        if self.selected_video is None:
            self.logger.warning("No video selected.")
            return
        self.annotations_cache.add_annotation(annotation)
        self.hashes_cache.add_annotation(annotation)
        self.image_keyframe_loader.add_image(self.selected_video.video_path.stem, image)
