import json
import os
import shutil

import tqdm
from coco_dataset import CocoMetaDataset


def load_annotations(annotations_path: str) -> CocoMetaDataset:
    with open(annotations_path, "r") as f:
        dataset = json.load(f)
    return CocoMetaDataset.from_json(dataset)


def save_annotations(annotations: CocoMetaDataset, annotations_path: str) -> None:
    with open(annotations_path, "w") as f:
        json.dump(annotations.to_json(), f)


def main():
    images_path = "/media/storage/training/labeled/true-battlebot-keypoints/2024-06-06"
    merged_path = "/media/storage/training/labeled/true-battlebot-keypoints/2024-06-06-merged"

    all_annotations = None
    image_moves = []
    for root, dirs, files in os.walk(images_path):
        for file in files:
            if file == "_annotations.coco.json":
                annotations = load_annotations(os.path.join(root, file))
                if all_annotations is None:
                    all_annotations = annotations
                else:
                    all_annotations.merge(annotations)
            else:
                image_moves.append((os.path.join(root, file), os.path.join(merged_path, file)))
    if all_annotations is None:
        print("No annotations found")
        return

    os.makedirs(merged_path, exist_ok=True)

    save_annotations(all_annotations, os.path.join(merged_path, "_annotations.coco.json"))

    for source, destination in tqdm.tqdm(image_moves, total=len(image_moves)):
        shutil.copy(source, destination)


if __name__ == "__main__":
    main()
