import os

import cv2
from helpers import load_dataset, plot_annotated_image
from matplotlib import pyplot as plt


def main():
    images_path = "/media/storage/training/labeled/nhrl_dataset_augmented"
    annotations_name = "_annotations.coco.json"
    annotations_path = os.path.join(images_path, annotations_name)

    metadataset = load_dataset(annotations_path)
    for dataset_image in metadataset.dataset.images[-10:]:
        print(dataset_image)
        annotations = metadataset.get_annotations(dataset_image.id)

        image = cv2.imread(os.path.join(images_path, dataset_image.file_name))
        plot_annotated_image(image, annotations)
        plt.show()


if __name__ == "__main__":
    main()
