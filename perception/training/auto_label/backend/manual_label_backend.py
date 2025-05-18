import logging
from pathlib import Path

from perception_tools.messages.image import Image
from perception_tools.training.yolo_keypoint_dataset import (
    YoloKeypointAnnotation,
    YoloKeypointData,
    YoloKeypointImage,
    YoloVisibility,
)

from auto_label.backend.annotation_cache import AnnotationCache
from auto_label.backend.hashes_cache import HashesCache
from auto_label.backend.image_keyframe_loader import ImageKeyframeLoader
from auto_label.backend.video_frame_database import VideoFrameDatabase
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
        self.selected_video: VideoFrameDatabase | None = None

    def get_video_name(self) -> str:
        return self.selected_video.images_path.stem if self.selected_video else ""

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

    def get_current_frame_path(self) -> Path | None:
        if self.selected_video is None:
            self.logger.warning("No video selected.")
            return None
        return self.selected_video.images_path / f"{self.selected_video.current_frame:06d}.jpg"

    def jump_to_frame(self, frame_num: int) -> Image | None:
        if self.selected_video is None:
            self.logger.warning("No video selected.")
            return None
        return self.selected_video.jump_to(frame_num)

    def list_annotation_indices_for_selected(self) -> list[int]:
        if self.selected_video is None:
            self.logger.warning("No video selected.")
            return []
        annotation_ids = self.annotations_cache.list_annotations()
        if not annotation_ids:
            self.logger.warning("No annotations found.")
            return []
        indices = []
        for video_name, frame_index in annotation_ids:
            if video_name == self.selected_video.images_path.stem:
                indices.append(frame_index)
        if not indices:
            self.logger.warning("No annotations found for the selected video.")
            return []
        indices.sort()
        return indices

    def get_annotation(self, frame_num: int) -> YoloKeypointImage | None:
        if self.selected_video is None:
            self.logger.warning("No video selected.")
            return None
        return self.annotations_cache.get_annotation(self.selected_video.images_path.stem, frame_num)

    def get_annotation_from_dataset(self, frame_num: int) -> tuple[YoloKeypointImage | None, bool]:
        if manually_labeled_annotation := self.get_annotation(frame_num):
            return manually_labeled_annotation, True
        if self.selected_video is None:
            self.logger.warning("No video selected.")
            return None
        image_id = self.annotations_cache.get_image_id(self.selected_video.images_path.stem, frame_num)
        annotation_path = self.root_path / "dataset" / (image_id + ".txt")
        if annotation_path.exists():
            with open(annotation_path, "r") as file:
                annotation = YoloKeypointImage.from_txt(image_id, file.read())
            return annotation, False
        else:
            return None, False

    def did_annotation_change(self, annotation: YoloKeypointImage) -> bool:
        return hash(annotation) != self.hashes_cache.get_saved_hash(annotation.image_id)

    def add_annotation(
        self,
        image: Image,
        bbox: tuple[float, float, float, float],
        class_index: int,
        keypoints: list[YoloKeypointData] | None = None,
    ) -> YoloKeypointImage | None:
        if self.selected_video is None:
            self.logger.warning("No video selected. Cannot add annotation.")
            return None
        annotation = self.get_annotation(image.header.seq)
        if annotation is None:
            image_id = self.annotations_cache.get_image_id(self.selected_video.images_path.stem, image.header.seq)
            annotation = YoloKeypointImage(image_id=image_id, labels=[])
        x0 = bbox[0]
        y0 = bbox[1]
        x1 = bbox[2]
        y1 = bbox[3]
        if keypoints is None:
            keypoints = [  # placeholder keypoints to be edited later
                ((x0, (y0 + y1) / 2, YoloVisibility.LABELED_VISIBLE)),  # front keypoint
                ((x1, (y0 + y1) / 2, YoloVisibility.LABELED_VISIBLE)),  # back keypoint
            ]
        annotation.labels.append(
            YoloKeypointAnnotation.from_corners(
                x0=x0, y0=y0, x1=x1, y1=y1, class_index=class_index, keypoints=keypoints
            )
        )
        self.hashes_cache.add_annotation(annotation)
        image_path = self.get_current_frame_path()
        self.update_annotation(image_path, annotation)
        return annotation

    def update_annotation(self, image_path: Path, annotation: YoloKeypointImage) -> None:
        if self.selected_video is None:
            self.logger.warning("No video selected. Cannot update annotation.")
            return None

        self.annotations_cache.add_annotation(annotation)
        self.image_keyframe_loader.add_image(self.selected_video.images_path.stem, image_path)
