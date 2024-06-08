import argparse
import os

import cv2
import numpy as np
from bw_shared.enums.label import Label
from perception_tools.config.model_metadata import LABEL_COLORS
from perception_tools.training.helpers import load_dataset


def main():
    parser = argparse.ArgumentParser(description="Augment a dataset")
    parser.add_argument(
        "images_path",
        type=str,
        help="Path to the directory containing the images and annotations",
    )
    args = parser.parse_args()
    images_path = args.images_path
    annotations_name = "_annotations.coco.json"
    annotations_path = os.path.join(images_path, annotations_name)

    metadataset = load_dataset(annotations_path)
    cv2.namedWindow("image")

    for dataset_image in metadataset.dataset.images:
        print(dataset_image.file_name)
        if "augment" not in dataset_image.file_name:
            continue
        image_path = os.path.join(images_path, dataset_image.file_name)
        print(image_path)
        image = cv2.imread(image_path)

        annotations = metadataset.get_annotations(dataset_image.id)
        for annotation in annotations:
            label = Label(metadataset.categories[annotation.category_id].name)
            color = LABEL_COLORS[label].to_cv_color()
            segmentation = np.array(annotation.segmentation, dtype=int).reshape((-1, 2))
            bbox = annotation.bbox
            cv2.polylines(image, [segmentation], True, color, 1)
            cv2.putText(
                image,
                label.value,
                tuple(segmentation[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )
            bbox_pt1 = (int(bbox[0]), int(bbox[1]))
            bbox_pt2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(image, bbox_pt1, bbox_pt2, color, 1)
        cv2.imshow("image", image)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            break


if __name__ == "__main__":
    main()
