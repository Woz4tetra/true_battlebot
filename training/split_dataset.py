import os
import random
import shutil

from helpers import load_dataset, write_dataset


def main():
    source_path = "/media/storage/training/labeled/nhrl_dataset_augmented"
    dest_path = "/media/storage/training/labeled/nhrl_dataset"
    annotations_name = "_annotations.coco.json"
    source_annotations_path = os.path.join(source_path, annotations_name)

    train_percent = 0.8
    val_percent = 0.15
    metadataset = load_dataset(source_annotations_path)
    num_images = len(metadataset.dataset.images)
    random.shuffle(metadataset.dataset.images)

    num_train = int(train_percent * num_images)
    num_val = int(val_percent * num_images)

    datasets = {
        "train": metadataset.slice(0, num_train),
        "val": metadataset.slice(num_train, num_train + num_val),
        "test": metadataset.slice(num_train + num_val, num_images),
    }

    for subdir, subdataset in datasets.items():
        subdir_path = os.path.join(dest_path, subdir)
        shutil.rmtree(subdir_path, ignore_errors=True)
        os.makedirs(subdir_path)
        write_dataset(subdataset, os.path.join(subdir_path, annotations_name))
        for dataset_image in subdataset.dataset.images:
            shutil.copyfile(
                os.path.join(source_path, dataset_image.file_name),
                os.path.join(subdir_path, dataset_image.file_name),
            )


if __name__ == "__main__":
    main()
