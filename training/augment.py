import copy
import os

import tqdm
from detectron2.data import transforms
from helpers import augment_dataset_image, copy_dataset, load_dataset, write_augmented_image, write_dataset


def main():
    images_path = "/media/storage/training/labeled/2024-02-24/battlebots_all"
    augmented_path = "/media/storage/training/labeled/2024-02-24/battlebots_augmented"
    annotations_name = "_annotations.coco.json"
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

    metadataset = load_dataset(annotations_path)
    new_metadataset = copy.deepcopy(metadataset)
    try:
        with tqdm.tqdm(total=len(metadataset.dataset.images) * num_repetitions) as pbar:
            for dataset_image in metadataset.dataset.images:
                for repetition in range(num_repetitions):
                    image_path = os.path.join(images_path, dataset_image.file_name)
                    image_transformed, transformed_annotations = augment_dataset_image(
                        image_path, metadataset, dataset_image, augmentations
                    )
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
