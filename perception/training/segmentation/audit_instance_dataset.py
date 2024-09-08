import argparse
import os

import cv2
import numpy as np
from perception_tools.training.coco_dataset import DatasetImage
from perception_tools.training.instance_helpers import load_dataset, plot_annotated_image, write_dataset


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images_path", type=str, help="Path to the images directory")
    parser.add_argument("-i", "--index", type=int, default=0, help="Start index")
    args = parser.parse_args()

    images_path = args.images_path
    annotations_name = "_annotations.coco.json"
    annotations_path = os.path.join(images_path, annotations_name)

    metadataset = load_dataset(annotations_path)
    images: list[DatasetImage] = metadataset.dataset.images
    print(f"Plotting {len(images)} images")
    current_index = args.index
    images_to_remove = []
    try:
        while True:
            dataset_image = images[current_index]
            print("Current index:", current_index)
            print(f"Current image: {dataset_image.file_name}")
            annotations = metadataset.get_annotations(dataset_image.id)

            image_path = os.path.join(images_path, dataset_image.file_name)
            if os.path.isfile(image_path):
                image = cv2.imread(image_path)
            else:
                image = np.zeros((dataset_image.height, dataset_image.width, 3), dtype=np.uint8)
            plot_annotated_image(metadataset, image, annotations)

            cv2.imshow("image", image)
            key = chr(cv2.waitKey(-1) & 0xFF)

            if key == "q":
                quit()
            elif key == "w":
                images_to_remove.append(dataset_image)
                current_index = (current_index + 1) % len(images)
            elif key == "d":
                current_index = (current_index + 1) % len(images)
            elif key == "a":
                current_index = (current_index - 1) % len(images)
    finally:
        if images_to_remove:
            metadataset.remove_images([image.id for image in images_to_remove])
            for annotation in images_to_remove:
                print(f"Removing {annotation.file_name}")
                os.remove(os.path.join(images_path, annotation.file_name))
            write_dataset(metadataset, annotations_path)


if __name__ == "__main__":
    main()
