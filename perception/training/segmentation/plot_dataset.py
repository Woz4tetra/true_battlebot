import os

import cv2
from matplotlib import pyplot as plt
from perception_tools.training.helpers import load_dataset, plot_annotated_image


def main():
    images_path = "/media/storage/training/labeled/2024-02-24/battlebots_all"
    annotations_name = "_annotations.coco.json"
    annotations_path = os.path.join(images_path, annotations_name)

    metadataset = load_dataset(annotations_path)
    # images = metadataset.dataset.images[-10:]
    images = metadataset.dataset.images[:10]
    for dataset_image in images:
        print(dataset_image)
        annotations = metadataset.get_annotations(dataset_image.id)

        image = cv2.imread(os.path.join(images_path, dataset_image.file_name))
        plot_annotated_image(image, annotations)
        plt.show()


if __name__ == "__main__":
    main()
