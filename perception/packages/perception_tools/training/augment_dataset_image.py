from typing import List, Tuple

import cv2
import numpy as np
from detectron2.data import transforms

from perception_tools.training.coco_dataset import CocoMetaDataset, DatasetAnnotation, DatasetImage


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
        for segmentation_array in polygons_transformed:
            coords = [
                np.min(segmentation_array[:, 0]),
                np.min(segmentation_array[:, 1]),
                np.max(segmentation_array[:, 0]),
                np.max(segmentation_array[:, 1]),
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
        for i in range(len(annotation.segmentation)):
            assert len(transformed_annotation.segmentation[i]) == len(annotation.segmentation[i]), (
                "Transformed segmentation has different length than original segmentation: "
                f"{len(transformed_annotation.segmentation[i])} != {len(annotation.segmentation[i])}"
            )
        transformed_annotations.append(transformed_annotation)

    return image_transformed, transformed_annotations
