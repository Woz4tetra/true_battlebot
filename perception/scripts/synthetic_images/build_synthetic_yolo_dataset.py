import argparse
import random
import shutil
from pathlib import Path

import yaml
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointDataset
from synthetic_dataset_labels import ALL_LABELS


def make_split_structure(dataset_root: Path) -> None:
    shutil.rmtree(dataset_root, ignore_errors=True)
    for subdir in ("train", "val", "test"):
        for subsubdir in ("images", "labels"):
            subdir_path = dataset_root / subdir / subsubdir
            subdir_path.mkdir(parents=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", help="Path to images")
    parser.add_argument("output", help="Output path")
    parser.add_argument("-t", "--train", type=float, help="Train percentage", default=0.8)
    parser.add_argument("-v", "--val", type=float, help="Validation percentage", default=0.2)
    args = parser.parse_args()

    output_path = Path(args.output)
    image_path = Path(args.images)
    train_percent = args.train
    val_percent = args.val

    dataset_metadata = {
        "train": "../train/images",
        "val": "../val/images",
        "test": "../test/images",
    }

    dataset = YoloKeypointDataset(names=ALL_LABELS)
    dataset_metadata.update(dataset.to_dict())

    make_split_structure(output_path)

    with open(output_path / "data.yaml", "w") as file:
        yaml.safe_dump(dataset_metadata, file)

    all_image_paths = list(image_path.glob("*.jpg"))
    num_images = len(all_image_paths)

    print(f"Splitting up {num_images} images")
    random.shuffle(all_image_paths)

    num_train = int(train_percent * num_images)
    num_val = int(val_percent * num_images)

    datasets = {
        "train": all_image_paths[0:num_train],
        "val": all_image_paths[num_train : num_train + num_val],
        "test": all_image_paths[num_train + num_val : num_images],
    }

    for subdir_key, dataset in datasets.items():
        for image_path in dataset:
            annotation_path = image_path.with_suffix(".txt")
            shutil.copyfile(image_path, output_path / subdir_key / "images" / image_path.name)
            shutil.copyfile(annotation_path, output_path / subdir_key / "labels" / annotation_path.name)


if __name__ == "__main__":
    main()
