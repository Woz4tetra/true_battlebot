import argparse
import datetime
import json
import os
import shutil

from bw_shared.enums.label import ModelLabel
from perception_tools.config.model_metadata import LABEL_COLORS, LabelColor, ModelMetadata
from perception_tools.data_directory import get_data_directory
from perception_tools.training.keypoints_config import KeypointsConfig, load_keypoints_config
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def copy_to_data(prefix: str, model_path: str, model: YOLO, config: KeypointsConfig) -> None:
    data_dir = get_data_directory()
    destination_dir = data_dir / "models"
    date_str = datetime.datetime.now().strftime("%Y-%m-%d")
    model_base_name = f"{prefix}_{date_str}"
    dest_model_path = destination_dir / (model_base_name + ".pt")
    dest_model_metadata_path = destination_dir / (model_base_name + ".json")

    os.makedirs(destination_dir, exist_ok=True)
    shutil.copy(model_path, str(dest_model_path))

    default_color = LabelColor(r=0.0, g=0.0, b=0.0, a=0.0)

    label_index_pairs = [(index, name) for index, name in model.names.items()]  # type: ignore
    labels = [ModelLabel(name) for _, name in label_index_pairs]
    colors = [LABEL_COLORS.get(label, default_color) for label in labels]

    metadata = ModelMetadata(
        labels=labels, colors=colors, keypoints=[config.keypoint_mapping[label] for label in labels]
    )

    with open(dest_model_metadata_path, "w") as f:
        json.dump(metadata.to_dict(), f)

    print(f"Saving output to {dest_model_path}")


def get_best_model(run_path: str) -> str:
    models = []
    for root, dirs, files in os.walk(run_path):
        for file in files:
            if file == "best.pt":
                models.append(os.path.join(root, file))
    models.sort(key=os.path.getmtime)
    return models[-1]


def main() -> None:
    configs = {
        "yolov8n-pose": {
            "batch": 24,
            "epochs": 200,
            "imgsz": 1280,
        },
        "yolov8s-pose": {
            "batch": 12,
            "epochs": 200,
            "imgsz": 1280,
        },
        "yolov8m-pose": {
            "batch": 8,
            "epochs": 150,
            "imgsz": 1280,
        },
        "yolov8l-pose": {
            "batch": 4,
            "epochs": 150,
            "imgsz": 1280,
        },
        "yolov8x-pose": {
            "batch": 6,
            "epochs": 150,
            "imgsz": 960,
        },
    }

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "config",
        type=str,
        help="Path to the configuration file. ex: ./keypoint_names_v1.toml",
    )
    parser.add_argument(
        "models",
        nargs="+",
        type=str,
        help="Model key to train. ex: yolov8l-pose",
    )
    args = parser.parse_args()

    config_path = args.config
    models = args.models
    config = load_keypoints_config(config_path)

    for model_key in models:
        settings = configs[model_key]

        # Load the model.
        model = YOLO(model_key)

        # Training.
        model.train(
            data="/media/storage/training/labeled/keypoints/2024-09-18/2024-09-18/data.yaml",
            name="battlebots_keypoints",
            device=0,
            **settings,
            hsv_h=0.015,
            hsv_s=0.7,
            hsv_v=0.4,
            degrees=180.0,
            translate=0.1,
            scale=0.1,
            shear=10.0,
            perspective=0.001,
            flipud=0.2,
            fliplr=0.2,
        )
        output_path = get_best_model(os.path.join(BASE_DIR, "runs"))
        copy_to_data(model_key, output_path, model, config)


if __name__ == "__main__":
    main()
