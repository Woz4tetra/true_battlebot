import argparse
import copy
import os

import numpy as np
import tqdm
from detectron2.data import transforms
from perception_tools.training.augment_dataset_image import augment_dataset_image
from perception_tools.training.augmentations import RandomRadialHomography
from perception_tools.training.instance_helpers import (
    copy_dataset,
    load_dataset,
    write_augmented_image,
    write_dataset,
)


def main():
    parser = argparse.ArgumentParser(description="Augment a dataset")
    parser.add_argument(
        "images_path",
        type=str,
        help="Path to the directory containing the images and annotations",
    )
    parser.add_argument("-o", "--output", nargs="?", type=str, help="Path to the output directory", default="")
    args = parser.parse_args()
    images_path = args.images_path
    if images_path.endswith("/"):
        images_path = images_path[:-1]
    augmented_path = args.output if args.output else images_path + "_augmented"
    annotations_name = "_annotations.coco.json"
    annotations_path = os.path.join(images_path, annotations_name)
    augmented_annotations_path = os.path.join(augmented_path, annotations_name)

    copy_dataset(images_path, augmented_path)

    np.random.seed(4176)
    augmentations = transforms.AugmentationList(
        [
            transforms.RandomBrightness(0.5, 1.5),
            RandomRadialHomography(0.5, 0.55),
        ]
    )
    num_repetitions = 2

    metadataset = load_dataset(annotations_path)
    new_metadataset = copy.deepcopy(metadataset)
    try:
        with tqdm.tqdm(total=len(metadataset.dataset.images) * num_repetitions) as pbar:
            for dataset_image in metadataset.dataset.images:
                for repetition in range(num_repetitions):
                    image_path = os.path.join(images_path, dataset_image.file_name)
                    try:
                        image_transformed, transformed_annotations = augment_dataset_image(
                            image_path, metadataset, dataset_image, augmentations
                        )
                    except KeyError:
                        print(f"Skipping image {image_path} due to missing annotations")
                        continue
                    new_image_filename = write_augmented_image(
                        image_transformed, augmented_path, image_path, repetition
                    )
                    new_dataset_image = copy.deepcopy(dataset_image)
                    new_dataset_image.file_name = new_image_filename
                    new_metadataset.add_annotation(new_dataset_image, transformed_annotations)
                    pbar.update(1)
    finally:
        write_dataset(new_metadataset, augmented_annotations_path)
        print(f"Done augmenting dataset, wrote to {augmented_annotations_path}")


if __name__ == "__main__":
    main()
