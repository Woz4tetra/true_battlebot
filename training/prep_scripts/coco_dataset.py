import datetime
from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class DatasetCategory:
    id: int
    name: str
    supercategory: str


@dataclass
class DatasetImage:
    id: int
    license: int
    file_name: str
    height: int
    width: int
    date_captured: str


@dataclass
class DatasetLicense:
    id: int
    name: str
    url: str


@dataclass
class DatasetAnnotation:
    id: int
    image_id: int
    category_id: int
    segmentation: List[List[float]]
    area: float
    bbox: List[float]
    iscrowd: int


@dataclass
class DatasetInfo:
    year: int
    version: str
    description: str
    contributor: str
    url: str
    date_created: str


@dataclass
class CocoDataset:
    info: DatasetInfo
    licenses: List[DatasetLicense]
    categories: List[DatasetCategory]
    images: List[DatasetImage]
    annotations: List[DatasetAnnotation]
    image_id_to_annotations: Dict[int, List[int]] = field(default_factory=dict)

    def __post_init__(self):
        for annotation in self.annotations:
            if annotation.image_id not in self.image_id_to_annotations:
                self.image_id_to_annotations[annotation.image_id] = []
            self.image_id_to_annotations[annotation.image_id].append(annotation.id)

    def get_annotations(self, image_id: int) -> List[DatasetAnnotation]:
        annotation_ids = self.image_id_to_annotations[image_id]
        return [self.annotations[annotation_id] for annotation_id in annotation_ids]

    def add_annotation(self, image: DatasetImage, annotations: List[DatasetAnnotation]):
        image.id = self._next_image_id()
        self.images.append(image)
        self.image_id_to_annotations[image.id] = []
        image.date_captured = datetime.datetime.now(tz=datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S%z")
        image.date_captured = image.date_captured[:-2] + ":" + image.date_captured[-2:]

        for annotation in annotations:
            annotation.id = self._next_annotation_id()
            annotation.image_id = image.id
            self.annotations.append(annotation)
            self.image_id_to_annotations[annotation.image_id].append(annotation.id)

    def _next_image_id(self) -> int:
        return max([image.id for image in self.images]) + 1

    def _next_annotation_id(self) -> int:
        return max([annotation.id for annotation in self.annotations]) + 1
