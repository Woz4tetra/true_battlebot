import argparse
import os

import cv2
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointImage


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize YOLO dataset")
    parser.add_argument("images", help="Path to images")
    parser.add_argument("-i", "--index", type=int, default=0, help="Start index")
    args = parser.parse_args()

    image_path = args.images
    images_paths = []
    annotation_paths = {}

    for dirpath, dirnames, filenames in os.walk(image_path):
        for filename in sorted(filenames):
            if filename.endswith(".jpg"):
                image_path = os.path.join(dirpath, filename)
                images_paths.append(image_path)
            elif filename.endswith(".txt"):
                name = os.path.splitext(filename)[0]
                annotation_path = os.path.join(dirpath, filename)
                annotation_paths[name] = annotation_path

    current_index = args.index
    while True:
        image_path = images_paths[current_index]
        print("Current index:", current_index)
        print(f"Current image: {image_path}")
        name = os.path.basename(os.path.splitext(image_path)[0])
        annotation_path = annotation_paths[name]
        with open(annotation_path) as file:
            annotations = YoloKeypointImage.from_txt(file.read())

        image = cv2.imread(image_path)
        height, width = image.shape[:2]

        for annotation in annotations.labels:
            x0 = int(annotation.x0 * width)
            y0 = int(annotation.y0 * height)
            x1 = int(annotation.x1 * width)
            y1 = int(annotation.y1 * height)
            cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(
                image,
                str(annotation.class_index),
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
                    str(index),
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                )

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
