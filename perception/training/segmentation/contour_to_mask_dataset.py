import argparse
import csv
from pathlib import Path

import cv2
import tqdm
from bw_shared.get_image_size import get_image_size
from perception_tools.training.helpers import (
    copy_dataset,
    load_dataset,
    segmentation_annotations_to_masks,
)


def write_classes(classes: dict[int, str], path: str) -> None:
    with open(path, mode="w") as file:
        writer = csv.writer(file)
        writer.writerow(["Pixel Value", "Class"])
        for index, class_name in classes.items():
            writer.writerow([index, class_name])


def main():
    parser = argparse.ArgumentParser(description="Convert to semantic segmentation dataset")
    parser.add_argument(
        "dataset_path",
        type=str,
        help="Path to the instance segmentation dataset",
    )
    parser.add_argument("-o", "--output", nargs="?", type=str, help="Path to the output directory", default="")
    args = parser.parse_args()
    dataset_path = Path(args.dataset_path)
    output_path = Path(args.output) if args.output else Path(str(dataset_path) + "_semantic")
    classes_name = "_classes.csv"
    annotations_name = "_annotations.coco.json"

    copy_dataset(str(dataset_path), str(output_path))

    for subdir in dataset_path.glob("*"):
        annotations_path = subdir / annotations_name
        if not annotations_path.exists():
            raise FileNotFoundError(f"Annotations file {annotations_path} does not exist")

        new_subdir = output_path / subdir.name
        classes_path = new_subdir / classes_name

        classes = {0: "background"}

        metadataset = load_dataset(str(annotations_path))
        try:
            with tqdm.tqdm(total=len(metadataset.dataset.images)) as pbar:
                for dataset_image in metadataset.dataset.images:
                    image_path = subdir / dataset_image.file_name
                    try:
                        annotations = metadataset.get_annotations(dataset_image.id)
                    except KeyError:
                        print(f"Skipping image {image_path} due to missing annotations")
                        continue
                    width, height = get_image_size(image_path)
                    for annotation in annotations:
                        classes[annotation.category_id] = metadataset.categories[annotation.category_id].name
                    mask = segmentation_annotations_to_masks(width, height, annotations)
                    mask_filename = image_path.name.replace(".jpg", "_mask.png")
                    cv2.imwrite(str(new_subdir / mask_filename), mask)
                    pbar.update(1)
        finally:
            write_classes(classes, str(classes_path))
            print(f"Wrote classes to {classes_path}")


if __name__ == "__main__":
    main()
