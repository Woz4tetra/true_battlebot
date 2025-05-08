from typing import cast

import detectron2.data.datasets
from detectron2.data import DatasetCatalog


def load_coco(dataset_name: str, annotation_path: str, image_dir: str) -> list[dict]:
    detectron2.data.datasets.register_coco_instances(
        name=dataset_name,
        metadata={},
        json_file=annotation_path,
        image_root=image_dir,
    )
    return cast(list[dict], DatasetCatalog.get(dataset_name))
