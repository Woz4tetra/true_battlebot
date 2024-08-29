import argparse
import os

import cv2
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointImage


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize YOLO dataset")
    parser.add_argument("images", help="Path to images")
    parser.add_argument("--debug", action="store_true", help="Save debug images")
    args = parser.parse_args()

    image_path = args.images
    debug_dir = os.path.dirname(image_path) + "/debug"

    if not os.path.exists(debug_dir):
        os.makedirs(debug_dir)

    for dirpath, dirnames, filenames in os.walk(image_path):
        for filename in sorted(filenames):
            if not filename.endswith(".jpg"):
                continue
            if args.debug and filename in os.listdir(debug_dir):
                continue

            image_path = os.path.join(dirpath, filename)
            if not os.path.exists(image_path):
                print(f"Image not found: {image_path}")
                continue

            annotation_path = os.path.splitext(image_path)[0] + ".txt"
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

                for keypoint in annotation.keypoints:
                    x = int(keypoint[0] * width)
                    y = int(keypoint[1] * height)
                    cv2.circle(image, (x, y), 3, (0, 0, 255), -1)

            if args.debug:
                debug_image_path = os.path.join(debug_dir, filename)
                cv2.imwrite(debug_image_path, image)
                print(f"Saved debug image: {debug_image_path}")

            else:
                cv2.imshow("image", image)
                key = chr(cv2.waitKey(-1) & 0xFF)

                if key == "q":
                    quit()


if __name__ == "__main__":
    main()
