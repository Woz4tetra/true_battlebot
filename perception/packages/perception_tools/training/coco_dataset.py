from __future__ import annotations

import copy
import datetime
import warnings
from dataclasses import dataclass, field
from functools import cached_property
from typing import Dict, List

from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class DatasetCategory:
    id: int
    name: str
    supercategory: str

    def __hash__(self):
        return self.id

    def __eq__(self, __value: object) -> bool:
        return isinstance(__value, DatasetCategory) and self.id == __value.id


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

    def __hash__(self) -> int:
        return self.id

    def __eq__(self, __value: object) -> bool:
        return isinstance(__value, DatasetLicense) and self.id == __value.id


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
    year: str
    version: str
    description: str
    contributor: str
    url: str
    date_created: str

    @classmethod
    def make_default(cls) -> DatasetInfo:
        now = datetime.datetime.now(tz=datetime.timezone.utc)
        return DatasetInfo(
            year=now.strftime("%Y"),
            version="1.0",
            description="Default dataset",
            contributor="Unknown",
            url="http://example.com",
            date_created=now.strftime("%Y-%m-%dT%H:%M:%S%z"),
        )


@dataclass
class CocoDataset:
    info: DatasetInfo = field(default_factory=DatasetInfo.make_default)
    licenses: List[DatasetLicense] = field(default_factory=list)
    categories: List[DatasetCategory] = field(default_factory=list)
    images: List[DatasetImage] = field(default_factory=list)
    annotations: List[DatasetAnnotation] = field(default_factory=list)

    def get_annotation_by_id(self, annotation_id: int) -> DatasetAnnotation:
        for annotation in self.annotations:
            if annotation.id == annotation_id:
                return annotation
        raise ValueError(f"Annotation {annotation_id} not found")


@dataclass
class CocoMetaDataset:
    dataset: CocoDataset = field(default_factory=CocoDataset)
    image_id_to_annotations: Dict[int, List[int]] = field(default_factory=dict)

    def __post_init__(self):
        for annotation in self.dataset.annotations:
            if annotation.image_id not in self.image_id_to_annotations:
                self.image_id_to_annotations[annotation.image_id] = []
            self.image_id_to_annotations[annotation.image_id].append(annotation.id)

    @cached_property
    def categories(self) -> Dict[int, DatasetCategory]:
        return {category.id: category for category in self.dataset.categories}

    def get_annotations(self, image_id: int) -> List[DatasetAnnotation]:
        annotation_ids = self.image_id_to_annotations[image_id]
        return [self.dataset.get_annotation_by_id(annotation_id) for annotation_id in annotation_ids]

    def add_annotation(self, image: DatasetImage, annotations: List[DatasetAnnotation]):
        image = copy.deepcopy(image)
        image.id = self._next_image_id()
        self.dataset.images.append(image)
        self.image_id_to_annotations[image.id] = []
        image.date_captured = datetime.datetime.now(tz=datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S%z")
        image.date_captured = image.date_captured[:-2] + ":" + image.date_captured[-2:]

        for annotation in annotations:
            if annotation.category_id not in self.categories.keys():
                raise ValueError(f"Category {annotation.category_id} not found")
            annotation = copy.deepcopy(annotation)
            annotation.id = self._next_annotation_id()
            annotation.image_id = image.id
            self.dataset.annotations.append(annotation)
            self.image_id_to_annotations[annotation.image_id].append(annotation.id)

    def _next_image_id(self) -> int:
        if len(self.dataset.images) == 0:
            return 0
        return max([image.id for image in self.dataset.images]) + 1

    def _next_annotation_id(self) -> int:
        if len(self.dataset.annotations) == 0:
            return 0
        return max([annotation.id for annotation in self.dataset.annotations]) + 1

    def slice(self, lower_index: int, upper_index: int) -> CocoMetaDataset:
        images = self.dataset.images[lower_index:upper_index]
        image_ids = [image.id for image in images]
        return CocoMetaDataset(
            CocoDataset(
                info=self.dataset.info,
                licenses=self.dataset.licenses,
                categories=self.dataset.categories,
                images=images,
                annotations=[annotation for annotation in self.dataset.annotations if annotation.image_id in image_ids],
            )
        )

    def merge(self, other: CocoMetaDataset) -> None:
        for category in other.dataset.categories:
            if category not in self.dataset.categories:
                self.dataset.categories.append(category)
        for index, category in enumerate(self.dataset.categories):
            category.id = index
        for license in other.dataset.licenses:
            if license not in self.dataset.licenses:
                self.dataset.licenses.append(license)
        for image in other.dataset.images:
            try:
                annotations = other.get_annotations(image.id)
            except KeyError:
                warnings.warn(f"Image {image} has no annotations")
            self.add_annotation(image, annotations)

    def remove_images(self, image_ids: List[int]) -> None:
        for image_id in image_ids:
            annotation_ids = self.image_id_to_annotations.pop(image_id)
            image_index = -1
            for index, image in enumerate(self.dataset.images):
                if image.id == image_id:
                    image_index = index
                    break
            if image_index == -1:
                raise ValueError(f"Image {image_id} not found")
            self.dataset.images.pop(image_index)
            for annotation_id in annotation_ids:
                annotation_index = -1
                for index, annotation in enumerate(self.dataset.annotations):
                    if annotation.id == annotation_id:
                        annotation_index = index
                        break
                if annotation_index == -1:
                    raise ValueError(f"Annotation {annotation_id} not found")
                self.dataset.annotations.pop(annotation_index)

    @classmethod
    def from_json(cls, d: Dict) -> CocoMetaDataset:
        return CocoMetaDataset(from_dict(CocoDataset, d))

    def to_json(self) -> Dict:
        return to_dict(self.dataset)
