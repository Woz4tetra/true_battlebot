import logging
from pathlib import Path
from typing import OrderedDict

from perception_tools.training.yolo_keypoint_dataset import YoloKeypointImage


class HashesCache:
    def __init__(self, hashes_path: Path, cache_size: int = 1000) -> None:
        self.hashes_path = hashes_path
        self.cache_size = cache_size
        self._cache: OrderedDict[str, int] = OrderedDict()
        self.logger = logging.getLogger(self.__class__.__name__)
        self.hashes_path.mkdir(parents=True, exist_ok=True)

    def clear(self) -> None:
        self._cache.clear()
        self.logger.info("Cleared annotation cache.")

    def _save_hash(self, annotation: YoloKeypointImage, annotation_hash: int) -> None:
        hash_path = self.hashes_path / (annotation.image_id + ".hash.txt")
        with open(hash_path, "w") as file:
            file.write(str(annotation_hash))

    def add_annotation(self, annotation: YoloKeypointImage) -> None:
        annotation_hash = hash(annotation)
        self._save_hash(annotation, annotation_hash)
        self._add_to_cache(annotation, annotation_hash)

    def _add_to_cache(self, annotation: YoloKeypointImage, annotation_hash: int) -> None:
        if self.cache_size <= 1:
            return
        self._cache[annotation.image_id] = annotation_hash
        while len(self._cache) > self.cache_size:
            self._cache.popitem(last=False)
            self.logger.debug("Annotation cache size exceeded. Removing oldest annotation.")

    def get_saved_hash(self, image_id: str) -> int | None:
        if image_id in self._cache:
            return self._cache[image_id]
        hash_path = self.hashes_path / (image_id + ".hash.txt")
        if hash_path.exists():
            with open(hash_path, "r") as file:
                return int(file.read().strip())
        self.logger.warning(f"Hash for {image_id} not found.")
        return None

    def list_hash_files(self) -> list[str]:
        annotations = []
        for annotation in self.hashes_path.iterdir():
            if annotation.is_file() and annotation.suffix == ".txt":
                annotations.append(annotation.stem)
        return annotations
