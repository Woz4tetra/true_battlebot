import json
import os
import shutil
from typing import List, Tuple

import cv2
import detectron2.data.datasets
import numpy as np
from coco_dataset import CocoMetaDataset, DatasetAnnotation, DatasetImage
from detectron2.data import DatasetCatalog, transforms
from matplotlib import pyplot as plt


def load_dataset(dataset_path: str) -> CocoMetaDataset:
    with open(dataset_path, "r") as f:
        dataset = json.load(f)
    return CocoMetaDataset.from_json(dataset)


def load_coco(dataset_name: str, annotation_path: str, image_dir: str):
    detectron2.data.datasets.register_coco_instances(
        name=dataset_name,
        metadata={},
        json_file=annotation_path,
        image_root=image_dir,
    )
    return DatasetCatalog.get(dataset_name)


def plot_annotated_image(image: np.ndarray, annotations: List[DatasetAnnotation]):
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
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
            line = plt.plot(segmentation[:, 0], segmentation[:, 1])
            plt.plot([coords[0], coords[2]], [coords[1], coords[3]], color=line[0].get_color())


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


def augment_dataset_image(
    image_path: str, dataset: CocoMetaDataset, dataset_image: DatasetImage, augmentations: transforms.AugmentationList
) -> Tuple[np.ndarray, List[DatasetAnnotation]]:
    annotations = dataset.get_annotations(dataset_image.id)

    image = cv2.imread(image_path)

    aug_input = transforms.AugInput(image)
    aug_transform = augmentations(aug_input)
    image_transformed = aug_input.image
    assert image_transformed is not None

    transformed_annotations = []

    for annotation in annotations:
        segmentations = []
        for segmentation in annotation.segmentation:
            segmentations.append(np.array(segmentation).reshape(-1, 2))
        polygons_transformed = np.array(aug_transform.apply_polygons(segmentations))

        all_coords = []
        for segmentation in polygons_transformed:
            coords = [
                np.min(segmentation[:, 0]),
                np.min(segmentation[:, 1]),
                np.max(segmentation[:, 0]),
                np.max(segmentation[:, 1]),
            ]
            all_coords.append(coords)
        fitted_coords = [
            np.min([bbox[0] for bbox in all_coords]),
            np.min([bbox[1] for bbox in all_coords]),
            np.max([bbox[2] for bbox in all_coords]),
            np.max([bbox[3] for bbox in all_coords]),
        ]
        bbox = [
            fitted_coords[0],
            fitted_coords[1],
            fitted_coords[2] - fitted_coords[0],
            fitted_coords[3] - fitted_coords[1],
        ]

        transformed_annotation = DatasetAnnotation(
            id=-1,
            image_id=-1,
            category_id=annotation.category_id,
            segmentation=[polygons_transformed.flatten().tolist()],
            area=annotation.area,
            bbox=bbox,
            iscrowd=annotation.iscrowd,
        )
        transformed_annotations.append(transformed_annotation)

    return image_transformed, transformed_annotations
