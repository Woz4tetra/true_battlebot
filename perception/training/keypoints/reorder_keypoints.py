import argparse
import os
import shutil
from pathlib import Path

from perception_tools.training.keypoints_config import load_keypoints_config
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointData, YoloKeypointImage
from tqdm import tqdm


def main() -> None:
    parser = argparse.ArgumentParser(description="Reorder keypoints")
    parser.add_argument("images", help="Path to images")
    parser.add_argument("-o", "--output", type=str, default="", help="Path to the output directory")
    parser.add_argument(
        "exising_order",
        type=str,
        help="Path to the configuration file defining the current keypoint ordering. "
        "ex: ./roboflow_keypoint_names.toml",
    )
    parser.add_argument(
        "new_order",
        type=str,
        help="Path to the configuration file defining the new keypoint ordering. ex: ./keypoint_names_v2.toml",
    )
    args = parser.parse_args()

    image_path = Path(args.images)
    output_path = Path(args.output) if args.output else Path(str(image_path) + "_reorder")
    old_config = load_keypoints_config(args.exising_order)
    new_config = load_keypoints_config(args.new_order)
    images_paths: list[Path] = []
    annotation_paths: dict[str, Path] = {}

    output_path.mkdir(parents=True, exist_ok=True)

    shutil.rmtree(output_path, ignore_errors=True)
    shutil.copytree(image_path, output_path)

    for dirpath, dirnames, filenames in os.walk(output_path):
        for filename in sorted(filenames):
            if filename.endswith(".jpg"):
                image_path = Path(os.path.join(dirpath, filename))
                images_paths.append(image_path)
            elif filename.endswith(".txt"):
                name = os.path.splitext(filename)[0]
                annotation_paths[name] = Path(os.path.join(dirpath, filename))

    with tqdm(total=len(images_paths)) as pbar:
        for index, image_path in enumerate(images_paths):
            pbar.update(1)
            name = image_path.stem
            annotation_path = annotation_paths[name]
            image_id = f"{name}-{index}"

            with open(annotation_path) as file:
                annotations = YoloKeypointImage.from_txt(image_id, file.read())

            for annotation in annotations.labels:
                label = old_config.labels[annotation.class_index]
                new_keypoints: list[YoloKeypointData | None] = [None] * len(annotation.keypoints)
                for old_keypoint_index, keypoint in enumerate(annotation.keypoints):
                    old_keypoint_name = old_config.keypoint_mapping[label][old_keypoint_index]
                    new_keypoint_index = new_config.keypoint_mapping[label].index(old_keypoint_name)
                    new_keypoints[new_keypoint_index] = keypoint
                assert all(new_keypoint is not None for new_keypoint in new_keypoints)
                annotation.keypoints = new_keypoints  # type: ignore

            with open(annotation_path, "w") as file:
                file.write(annotations.to_txt())


if __name__ == "__main__":
    main()
