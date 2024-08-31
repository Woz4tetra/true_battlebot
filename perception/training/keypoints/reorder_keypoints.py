import argparse
import os
import shutil
from pathlib import Path

from perception_tools.training.yolo_keypoint_dataset import YoloKeypointAnnotation, YoloKeypointImage
from tqdm import tqdm


def main() -> None:
    parser = argparse.ArgumentParser(description="Reorder keypoints")
    parser.add_argument("images", help="Path to images")
    parser.add_argument("-o", "--output", type=str, default="", help="Path to the output directory")
    parser.add_argument("class_index", type=int, help="Class index")
    parser.add_argument("new_order", type=int, nargs="+", help="New order of keypoints")
    args = parser.parse_args()

    image_path = Path(args.images)
    output_path = Path(args.output) if args.output else Path(str(image_path) + "_reorder")
    class_index = args.class_index
    new_order = args.new_order
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
                annotation_path = os.path.join(dirpath, filename)
                annotation_paths[name] = Path(annotation_path)

    with tqdm(total=len(images_paths)) as pbar:
        for image_path in images_paths:
            pbar.update(1)
            name = image_path.stem
            annotation_path = annotation_paths[name]

            with open(annotation_path) as file:
                annotations = YoloKeypointImage.from_txt(file.read())

            class_annotations: list[YoloKeypointAnnotation] = []
            remaining_annotations: list[YoloKeypointAnnotation] = []
            for annotation in annotations.labels:
                if annotation.class_index == class_index:
                    class_annotations.append(annotation)
                else:
                    remaining_annotations.append(annotation)

            if len(class_annotations) == 0:
                continue

            for annotation in class_annotations:
                new_keypoints = []
                for index in new_order:
                    if index >= len(annotation.keypoints) or index < 0:
                        raise ValueError(f"Invalid keypoint index: {index}")
                    keypoint = annotation.keypoints[index]
                    new_keypoints.append(keypoint)
                annotation.keypoints = new_keypoints

            with open(annotation_path, "w") as file:
                file.write(annotations.to_txt())


if __name__ == "__main__":
    main()
