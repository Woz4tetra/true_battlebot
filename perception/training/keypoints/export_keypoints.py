import argparse
import datetime
import json
import os
import shutil

from bw_shared.enums.label import ModelLabel
from perception_tools.config.model_metadata import LABEL_COLORS, LabelColor, ModelMetadata
from perception_tools.data_directory import get_data_directory
from perception_tools.training.keypoints_config import load_keypoints_config
from ultralytics import YOLO


def copy_to_data(model_path: str, model_metadata_path: str) -> None:
    data_dir = get_data_directory()
    destination_dir = data_dir / "models"
    date_str = datetime.datetime.now().strftime("%Y-%m-%d")
    model_base_name = f"yolov8-pose_{date_str}"
    dest_model_path = destination_dir / (model_base_name + ".pt")
    dest_model_metadata_path = destination_dir / (model_base_name + ".json")

    os.makedirs(destination_dir, exist_ok=True)
    shutil.copy(model_path, str(dest_model_path))
    shutil.copy(model_metadata_path, str(dest_model_metadata_path))

    print(f"Saving output to {dest_model_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a YOLO keypoints model to torchscript")
    parser.add_argument(
        "model",
        type=str,
        help="Path to the model (*.pt)",
    )
    parser.add_argument(
        "config",
        type=str,
        help="Path to the configuration file. ex: ./keypoint_names_v1.toml",
    )
    args = parser.parse_args()
    path = args.model
    config_path = args.config
    if not path.endswith(".pt"):
        raise ValueError("Model must be a .pt file")

    model = YOLO(path)

    # TODO: figure out why export isn't working
    # model.export(format="torchscript")
    # exported_path = path.replace(".pt", ".torchscript")
    exported_path = path

    default_color = LabelColor(r=0.0, g=0.0, b=0.0, a=0.0)

    label_index_pairs = [(index, name) for index, name in model.names.items()]  # type: ignore
    labels = [ModelLabel(name) for _, name in label_index_pairs]
    colors = [LABEL_COLORS.get(label, default_color) for label in labels]
    config = load_keypoints_config(config_path)

    metadata = ModelMetadata(labels=labels, colors=colors, keypoints=[config.keypoint_names[label] for label in labels])

    model_metadata_path = os.path.join(os.path.dirname(path), "model_metadata.json")
    with open(model_metadata_path, "w") as f:
        json.dump(metadata.to_dict(), f)

    copy_to_data(exported_path, model_metadata_path)


if __name__ == "__main__":
    main()
