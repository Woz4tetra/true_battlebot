import argparse
import os

import cv2
from perception_tools.training.coco_dataset import DatasetImage
from perception_tools.training.helpers import load_dataset, plot_annotated_image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("images_path", type=str, help="Path to the images directory")
    parser.add_argument("-i", "--index", type=int, default=0, help="Start index")
    args = parser.parse_args()

    images_path = args.images_path
    num_images = args.index
    annotations_name = "_annotations.coco.json"
    annotations_path = os.path.join(images_path, annotations_name)

    metadataset = load_dataset(annotations_path)
    if num_images > 0:
        images: list[DatasetImage] = metadataset.dataset.images[:num_images]
    else:
        images: list[DatasetImage] = metadataset.dataset.images
    print(f"Plotting {len(images)} images")
    current_index = args.index
    while True:
        dataset_image = images[current_index]
        print("Current index:", current_index)
        print(f"Current image: {dataset_image.file_name}")
        annotations = metadataset.get_annotations(dataset_image.id)

        image = cv2.imread(os.path.join(images_path, dataset_image.file_name))
        plot_annotated_image(image, annotations)

        cv2.imshow("image", image)
        key = chr(cv2.waitKey(-1) & 0xFF)

        if key == "q":
            quit()
        elif key == "d":
            current_index = (current_index + 1) % len(images)
        elif key == "a":
            current_index = (current_index - 1) % len(images)


if __name__ == "__main__":
    main()
