import json
import os
import shutil
from typing import List

import cv2
import numpy as np

from perception_tools.training.coco_dataset import CocoMetaDataset, DatasetAnnotation


def load_dataset(dataset_path: str) -> CocoMetaDataset:
    with open(dataset_path, "r") as f:
        dataset = json.load(f)
    return CocoMetaDataset.from_json(dataset)


def plot_annotated_image(image: np.ndarray, annotations: List[DatasetAnnotation]) -> None:
    for annotation in annotations:
        box = annotation.bbox
        coords = [
            box[0],
            box[1],
            box[0] + box[2],
            box[1] + box[3],
        ]
        for segmentation in annotation.segmentation:
            segmentation = np.array(segmentation).reshape(-1, 2)
            segmentation = np.append(segmentation, [segmentation[0]], axis=0)
            cv2.polylines(image, [segmentation], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.rectangle(image, (coords[0], coords[1]), (coords[2], coords[3]), (0, 90, 0), 2)  # type: ignore


def copy_dataset(dataset_path: str, destination_path: str) -> None:
    shutil.rmtree(destination_path, ignore_errors=True)
    shutil.copytree(dataset_path, destination_path)


def write_augmented_image(
    image: np.ndarray, new_directory: str, original_image_path: str, augmentation_num: int
) -> str:
    old_image_path, extension = os.path.splitext(original_image_path)
    image_name = os.path.basename(old_image_path)
    new_image_filename = image_name + f"_augment-{augmentation_num:03d}{extension}"
    image_path = os.path.join(new_directory, new_image_filename)
    cv2.imwrite(image_path, image)
    return new_image_filename


def write_dataset(dataset: CocoMetaDataset, path: str) -> None:
    with open(path, "w") as f:
        json.dump(dataset.to_json(), f)


def segmentation_to_mask(
    image_width: int, image_height: int, segmentations: List[List[float]], category_id: int
) -> np.ndarray:
    mask = np.zeros((image_height, image_width), dtype=np.uint8)
    for segmentation in segmentations:
        segmentation_px = np.array(segmentation).reshape(-1, 2).astype(int)
        mask = cv2.fillPoly(mask, [segmentation_px], category_id)  # type: ignore
    return mask


def segmentation_annotations_to_masks(
    image_width: int, image_height: int, annotations: List[DatasetAnnotation]
) -> np.ndarray:
    full_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    for annotation in annotations:
        category_id = annotation.category_id
        assert category_id > 0, f"Category ID must be greater than 0, got {category_id}"
        mask = segmentation_to_mask(image_width, image_height, annotation.segmentation, category_id)
        full_mask = np.maximum(full_mask, mask)
    return full_mask
