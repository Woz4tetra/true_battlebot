import argparse
import os
from pathlib import Path

import cv2
from perception_tools.training.keypoints_config import load_keypoints_config
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointImage


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize YOLO dataset")
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

    draw_width = 1500

    dataset_root = Path(args.images)
    config_path = args.config
    images_paths: list[Path] = []
    annotation_paths = {}

    config = load_keypoints_config(config_path) if config_path else None

    for dirpath, dirnames, filenames in os.walk(dataset_root):
        for filename in sorted(filenames):
            if filename.endswith(".jpg"):
                image_path = Path(os.path.join(dirpath, filename))
                images_paths.append(image_path)
            elif filename.endswith(".txt"):
                name = os.path.splitext(filename)[0]
                annotation_path = Path(os.path.join(dirpath, filename))
                annotation_paths[name] = annotation_path

    image = None
    current_index = int(args.index)
    while True:
        key_value = cv2.waitKey(100)
        key = chr(key_value & 0xFF)

        if key == "q":
            break
        elif key == "w":
            print(f"Deleting {annotation_path} and {image_path}")
            os.remove(annotation_path)
            os.remove(image_path)
            images_paths.pop(current_index)
        elif key == "d":
            current_index = (current_index + 1) % len(images_paths)
        elif key == "a":
            current_index = (current_index - 1) % len(images_paths)
        elif image is not None:
            # check if opencv window is closed
            if cv2.getWindowProperty("image", cv2.WND_PROP_VISIBLE) < 1:
                break

            continue

        image_path = images_paths[current_index]
        print("Current index:", current_index)
        print(f"Current image: {image_path}")
        name = os.path.basename(os.path.splitext(image_path)[0])
        annotation_path = annotation_paths[name]
        image_id = f"{image_path.stem}-{current_index}"
        with open(annotation_path) as file:
            annotations = YoloKeypointImage.from_txt(image_id, file.read())

        image = cv2.imread(str(image_path))
        aspect_ratio = image.shape[1] / image.shape[0]
        image = cv2.resize(image, (int(draw_width), int(draw_width / aspect_ratio)), interpolation=cv2.INTER_LINEAR)
        height, width = image.shape[:2]

        for annotation in annotations.labels:
            x0 = int(annotation.x0 * width)
            y0 = int(annotation.y0 * height)
            x1 = int(annotation.x1 * width)
            y1 = int(annotation.y1 * height)
            cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(
                image,
                str(annotation.class_index) if config is None else config.labels[annotation.class_index],
                (x0, y0 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

            for index, keypoint in enumerate(annotation.keypoints):
                x = int(keypoint[0] * width)
                y = int(keypoint[1] * height)
                cv2.circle(image, (x, y), 3, (0, 0, 255), -1)
                cv2.putText(
                    image,
                    str(index)
                    if config is None
                    else config.keypoint_mapping[config.labels[annotation.class_index]][index],
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                )

        cv2.imshow("image", image)


if __name__ == "__main__":
    main()
