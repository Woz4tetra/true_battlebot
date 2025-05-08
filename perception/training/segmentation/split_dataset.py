import argparse
import os
import random
import shutil
from pathlib import Path

from perception_tools.training.instance_helpers import load_dataset, write_dataset


def main() -> None:
    parser = argparse.ArgumentParser(description="Split a dataset")
    parser.add_argument(
        "source_path",
        type=str,
        help="Path to the directory containing the images and annotations",
    )
    parser.add_argument("-o", "--output", nargs="?", type=str, help="Path to the output directory", default="")
    parser.add_argument("-t", "--train", type=float, help="Train percentage", default=0.8)
    parser.add_argument("-v", "--val", type=float, help="Validation percentage", default=0.15)
    args = parser.parse_args()
    source_path = Path(args.source_path)
    dest_path = args.output if args.output else Path(str(source_path) + "_split")
    print(f"Splitting dataset from {source_path} to {dest_path}")
    annotations_name = "_annotations.coco.json"
    source_annotations_path = source_path / annotations_name

    train_percent = args.train
    val_percent = args.val
    metadataset = load_dataset(str(source_annotations_path))
    num_images = len(metadataset.dataset.images)
    print(f"Splitting up {num_images} images")
    random.shuffle(metadataset.dataset.images)

    num_train = int(train_percent * num_images)
    num_val = int(val_percent * num_images)

    datasets = {
        "train": metadataset.slice(0, num_train),
        "val": metadataset.slice(num_train, num_train + num_val),
        "test": metadataset.slice(num_train + num_val, num_images),
    }

    for subdir, subdataset in datasets.items():
        subdir_path = dest_path / subdir
        shutil.rmtree(subdir_path, ignore_errors=True)
        subdir_path.mkdir(parents=True)
        write_dataset(subdataset, os.path.join(subdir_path, annotations_name))
        for dataset_image in subdataset.dataset.images:
            shutil.copyfile(
                source_path / dataset_image.file_name,
                subdir_path / dataset_image.file_name,
            )
    print(f"Split dataset into {num_train} train, {num_val} val, {num_images - num_train - num_val} test")


if __name__ == "__main__":
    main()
