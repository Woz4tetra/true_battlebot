import argparse
import os

import cv2
import numpy as np
from perception_tools.config.model_metadata import LABEL_COLORS
from perception_tools.training.semantic_helpers import read_classes


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", help="Path to images")
    parser.add_argument("-i", "--index", type=int, default=0, help="Start index")
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default="",
        help="Path to the configuration file. ex: ./keypoint_names_v1.toml",
    )
    args = parser.parse_args()

    image_path = args.images
    images_paths = []
    annotation_paths = {}
    class_paths = []

    mask_suffix = "_mask.png"

    for dirpath, dirnames, filenames in os.walk(image_path):
        for filename in sorted(filenames):
            if filename.endswith(".jpg"):
                image_path = os.path.join(dirpath, filename)
                images_paths.append(image_path)
            elif filename.endswith(mask_suffix):
                name = filename[: -len(mask_suffix)]
                annotation_path = os.path.join(dirpath, filename)
                annotation_paths[name] = annotation_path
            elif filename == "_classes.csv":
                class_path = os.path.join(dirpath, filename)
                class_paths.append(class_path)

    classes = {}
    for class_path in class_paths:
        next_classes = read_classes(class_path)
        if not classes:
            classes = next_classes
        if classes != next_classes:
            raise ValueError(f"Classes from {class_path} are not consistent with previous classes")
        classes = next_classes

    current_index = args.index
    while True:
        image_path = images_paths[current_index]
        print("Current index:", current_index)
        print(f"Current image: {image_path}")
        name = os.path.basename(os.path.splitext(image_path)[0])
        annotation_path = annotation_paths[name]

        image = cv2.imread(image_path)

        mask = cv2.imread(annotation_path)
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        unique_ids = np.unique(mask).tolist()
        color_seg = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
        for label_id in unique_ids:
            if label_id == 0:
                continue
            label = classes[label_id]
            color = LABEL_COLORS[label].to_cv_color()
            layer_seg = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
            layer_seg[mask == label_id] = color
            color_seg = cv2.addWeighted(color_seg, 1, layer_seg, 1, 0)

        image = cv2.addWeighted(image, 0.5, color_seg, 0.5, 0)

        cv2.imshow("image", image)
        key = chr(cv2.waitKey(-1) & 0xFF)

        if key == "q":
            quit()
        elif key == "w":
            print(f"Deleting {annotation_path} and {image_path}")
            os.remove(annotation_path)
            os.remove(image_path)
            images_paths.pop(current_index)
        elif key == "d":
            current_index = (current_index + 1) % len(images_paths)
        elif key == "a":
            current_index = (current_index - 1) % len(images_paths)


if __name__ == "__main__":
    main()
