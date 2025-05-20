import logging
from pathlib import Path
from typing import OrderedDict

from perception_tools.training.yolo_keypoint_dataset import YoloKeypointImage


class AnnotationCache:
    def __init__(self, annotations_path: Path, dataset_path: Path, cache_size: int = 1000) -> None:
        self.annotations_path = annotations_path
        self.dataset_path = dataset_path
        self.cache_size = cache_size
        self._cache: OrderedDict[str, YoloKeypointImage] = OrderedDict()
        self.logger = logging.getLogger(self.__class__.__name__)

    def clear(self) -> None:
        self._cache.clear()
        self.logger.info("Cleared annotation cache.")

    def _save_annotation(self, annotation: YoloKeypointImage) -> None:
        annotation_path = self.annotations_path / (annotation.image_id + ".txt")
        with open(annotation_path, "w") as file:
            file.write(annotation.to_txt())
        dataset_path = self.dataset_path / (annotation.image_id + ".txt")
        with open(dataset_path, "w") as file:
            file.write(annotation.to_txt())
        self.logger.info(f"Saved annotation to {annotation_path} and {dataset_path}.")

    def add_annotation(self, annotation: YoloKeypointImage) -> None:
        self._save_annotation(annotation)
        self._add_to_cache(annotation)

    def _add_to_cache(self, annotation: YoloKeypointImage) -> None:
        if self.cache_size <= 1:
            return
        self._cache[annotation.image_id] = annotation
        while len(self._cache) > self.cache_size:
            self._cache.popitem(last=False)
            self.logger.debug("Annotation cache size exceeded. Removing oldest annotation.")

    def get_image_id(self, video_name: str, frame_num: int) -> str:
        return f"{video_name}-{frame_num:06d}"

    def parse_image_id(self, image_id: str) -> tuple[str, int]:
        image_id_split = image_id.split("-")
        video_name = "-".join(image_id_split[:-1])
        frame_num_str = image_id_split[-1]
        frame_num = int(frame_num_str)
        return video_name, frame_num

    def get_annotation(self, video_name: str, frame_num: int) -> YoloKeypointImage | None:
        image_id = self.get_image_id(video_name, frame_num)
        if image_id in self._cache:
            return self._cache[image_id]
        annotation_path = self.annotations_path / (image_id + ".txt")
        if annotation_path.exists():
            with open(annotation_path, "r") as file:
                annotation = YoloKeypointImage.from_txt(image_id, file.read())
            self._add_to_cache(annotation)
            return annotation
        return None

    def list_annotations(self) -> list[tuple[str, int]]:
        annotations = []
        for annotation in self.annotations_path.iterdir():
            if annotation.is_file() and annotation.suffix == ".txt":
                annotations.append(self.parse_image_id(annotation.stem))
        return annotations

    def _delete_annotation_in_dir(self, directory: Path, video_name: str, frame_num: int) -> None:
        image_id = self.get_image_id(video_name, frame_num)
        annotation_path = directory / (image_id + ".txt")
        if annotation_path.exists():
            annotation_path.unlink()
            self._cache.pop(image_id, None)
            self.logger.info(f"Deleted annotation {image_id}.")
        else:
            self.logger.warning(f"Annotation {image_id} in {directory} not found.")

    def delete_annotation(self, video_name: str, frame_num: int) -> None:
        self._delete_annotation_in_dir(self.annotations_path, video_name, frame_num)
        self._delete_annotation_in_dir(self.dataset_path, video_name, frame_num)
