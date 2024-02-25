import copy
import os
from multiprocessing import Pool, cpu_count

import tqdm
from coco_dataset import DatasetImage
from detectron2.data import transforms
from helpers import augment_dataset_image, copy_dataset, load_dataset, write_augmented_image, write_dataset

augmentations = transforms.AugmentationList(
    [
        transforms.RandomRotation(angle=[-15, 15], expand=False),
        transforms.RandomBrightness(0.8, 1.2),
    ]
)
num_repetitions = 2

images_path = "/media/storage/training/labeled/2024-02-24/battlebots_all"
augmented_path = "/media/storage/training/labeled/2024-02-24/battlebots_augmented"
annotations_name = "_annotations.coco.json"
annotations_path = os.path.join(images_path, annotations_name)
augmented_annotations_path = os.path.join(augmented_path, annotations_name)
metadataset = load_dataset(annotations_path)
new_metadataset = copy.deepcopy(metadataset)


def augment_image(dataset_image: DatasetImage):
    for repetition in range(num_repetitions):
        image_path = os.path.join(images_path, dataset_image.file_name)
        image_transformed, transformed_annotations = augment_dataset_image(
            image_path, metadataset, dataset_image, augmentations
        )
        new_image_filename = write_augmented_image(image_transformed, augmented_path, image_path, repetition)
        new_dataset_image = copy.deepcopy(dataset_image)
        new_dataset_image.file_name = new_image_filename
        new_metadataset.add_annotation(new_dataset_image, transformed_annotations)


def main():
    copy_dataset(images_path, augmented_path)

    try:
        with Pool(cpu_count() - 1) as pool:
            list(
                tqdm.tqdm(
                    pool.imap(
                        augment_image,
                        metadataset.dataset.images,
                    ),
                    total=len(metadataset.dataset.images),
                )
            )
    finally:
        write_dataset(new_metadataset, augmented_annotations_path)
        print(f"Done augmenting dataset, wrote to {augmented_annotations_path}")


if __name__ == "__main__":
    main()
