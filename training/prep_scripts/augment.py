import copy
import os

import tqdm
from detectron2.data import transforms
from helpers import augment_dataset_image, copy_dataset, load_dataset, write_augmented_dataset, write_augmented_image


def main():
    images_path = "/media/storage/training/labeled/nhrl_dataset_all"
    augmented_path = "/media/storage/training/labeled/nhrl_dataset_augmented"
    annotations_name = "annotations.coco.json"
    annotations_path = os.path.join(images_path, annotations_name)
    augmented_annotations_path = os.path.join(augmented_path, annotations_name)

    copy_dataset(images_path, augmented_path)

    augmentations = transforms.AugmentationList(
        [
            transforms.RandomRotation(angle=[-15, 15], expand=False),
            transforms.RandomBrightness(0.8, 1.2),
        ]
    )
    num_repetitions = 2

    dataset = load_dataset(annotations_path)
    new_dataset = copy.deepcopy(dataset)
    try:
        with tqdm.tqdm(total=len(dataset.images) * num_repetitions) as pbar:
            for dataset_image in dataset.images:
                for repetition in range(num_repetitions):
                    image_path = os.path.join(images_path, dataset_image.file_name)
                    image_transformed, transformed_annotations = augment_dataset_image(
                        image_path, dataset, dataset_image, augmentations
                    )
                    new_image_filename = write_augmented_image(
                        image_transformed, augmented_path, image_path, repetition
                    )
                    new_dataset_image = copy.deepcopy(dataset_image)
                    new_dataset_image.file_name = new_image_filename
                    new_dataset.add_annotation(new_dataset_image, transformed_annotations)
                    pbar.update(1)
    finally:
        write_augmented_dataset(new_dataset, augmented_annotations_path)
        print(f"Done augmenting dataset, wrote to {augmented_annotations_path}")


if __name__ == "__main__":
    main()
