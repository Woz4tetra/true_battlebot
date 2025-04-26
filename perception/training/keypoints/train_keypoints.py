import argparse
import datetime
import json
import os
import shutil

from bw_shared.enums.label import ModelLabel
from perception_tools.config.model_metadata import LABEL_COLORS, LabelColor, ModelMetadata
from perception_tools.directories.data_directory import get_data_directory
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
        "yolo11n-pose": {
            "batch": 16,
            "epochs": 200,
            "imgsz": 960,
        },
        "yolo11x-pose": {
            "batch": 4,
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
        "dataset",
        type=str,
        help="Path to dataset yaml",
    )
    parser.add_argument(
        "models",
        nargs="+",
        type=str,
        help="Model key to train. ex: yolov8l-pose",
    )
    parser.add_argument(
        "-c",
        "--checkpoint",
        default="",
        type=str,
        help="Resume from checkpoint",
    )
    parser.add_argument(
        "-e",
        "--epochs",
        default=0,
        type=int,
        help="Overwrite number of epochs",
    )
    args = parser.parse_args()

    dataset = args.dataset
    config_path = args.config
    models = args.models
    epochs = args.epochs
    checkpoint_path = args.checkpoint
    config = load_keypoints_config(config_path)

    hyper_params = dict(
        lr0=0.01,  # (float) initial learning rate (i.e. SGD=1E-2, Adam=1E-3)
        lrf=0.01,  # (float) final learning rate (lr0 * lrf)
        momentum=0.937,  # (float) SGD momentum/Adam beta1
        weight_decay=0.0005,  # (float) optimizer weight decay 5e-4
        warmup_epochs=3.0,  # (float) warmup epochs (fractions ok)
        warmup_momentum=0.8,  # (float) warmup initial momentum
        warmup_bias_lr=0.1,  # (float) warmup initial bias lr
        box=7.5,  # (float) box loss gain
        cls=0.5,  # (float) cls loss gain (scale with pixels)
        dfl=1.5,  # (float) dfl loss gain
        pose=12.0,  # (float) pose loss gain
        kobj=1.0,  # (float) keypoint obj loss gain
        label_smoothing=0.0,  # (float) label smoothing (fraction)
        nbs=64,  # (int) nominal batch size
        hsv_h=0.015,  # (float) image HSV-Hue augmentation (fraction)
        hsv_s=0.7,  # (float) image HSV-Saturation augmentation (fraction)
        hsv_v=0.4,  # (float) image HSV-Value augmentation (fraction)
        degrees=5.0,  # (float) image rotation (+/- deg)
        translate=0.05,  # (float) image translation (+/- fraction)
        scale=0.1,  # (float) image scale (+/- gain)
        shear=2.0,  # (float) image shear (+/- deg)
        perspective=0.0001,  # (float) image perspective (+/- fraction), range 0-0.001
        flipud=0.2,  # (float) image flip up-down (probability)
        fliplr=0.5,  # (float) image flip left-right (probability)
        bgr=0.0,  # (float) image channel BGR (probability)
        mosaic=0.5,  # (float) image mosaic (probability)
        mixup=0.0,  # (float) image mixup (probability)
        copy_paste=0.0,  # (float) segment copy-paste (probability)
        copy_paste_mode="flip",  # (str) the method to do copy_paste augmentation (flip, mixup)
        # (str) auto augmentation policy for classification (randaugment, autoaugment, augmix)
        auto_augment="randaugment",
        # (float) probability of random erasing during classification training (0-0.9), 0 means no erasing,
        # must be less than 1.0.
        erasing=0.0,
        # (float) image crop fraction for classification (0.1-1), 1.0 means no crop, must be greater than 0.
        crop_fraction=1.0,
    )

    for model_key in models:
        settings = configs[model_key]
        if epochs > 0:
            settings["epochs"] = epochs

        # Load the model.
        model = YOLO(checkpoint_path if checkpoint_path else model_key)

        # Training.
        model.train(
            data=dataset,
            name="battlebots_keypoints",
            device=0,
            cache="ram",
            **hyper_params,
            **settings,
        )
        output_path = get_best_model(os.path.join(BASE_DIR, "runs"))
        copy_to_data(model_key, output_path, model, config)


if __name__ == "__main__":
    main()
