import argparse
import datetime
import json
import os
import re
import shutil
from pathlib import Path

import torch
from perception_tools.config.model_metadata import FIELD_SEMANTIC_MODEL_METADATA
from perception_tools.data_directory import get_data_directory
from perception_tools.inference.common import get_default_device
from perception_tools.training.deeplabv3 import load_model


def copy_to_data(model_path: str, model_metadata_path: str) -> None:
    match = re.search(r"model_(mbv3|r50|r101)", model_path)
    model_type = (match.group(1) + "_") if match else ""
    data_dir = get_data_directory()
    destination_dir = data_dir / "models"
    date_str = datetime.datetime.now().strftime("%Y-%m-%d")
    model_base_name = f"field_deeplabv3_{model_type}{date_str}"
    dest_model_path = destination_dir / (model_base_name + ".torchscript")
    dest_model_metadata_path = destination_dir / (model_base_name + ".json")

    os.makedirs(destination_dir, exist_ok=True)
    shutil.copy(model_path, str(dest_model_path))
    shutil.copy(model_metadata_path, str(dest_model_metadata_path))

    print(f"Saving output to {dest_model_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Test inference of a semantic segmentation model")
    parser.add_argument(
        "model",
        type=str,
        help="Path to the model (*.pth)",
    )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()
    model_path = Path(args.model)
    model_output_path = model_path.parent / (model_path.stem + ".torchscript")
    metadata_output_path = model_path.parent / (model_path.stem + ".json")

    device = get_default_device()
    print(f"Using device: {device}")
    model = load_model(model_path, device)

    scripted_model = torch.jit.script(model)
    torch.jit.save(scripted_model, model_output_path)

    with open(metadata_output_path, "w") as f:
        json.dump(FIELD_SEMANTIC_MODEL_METADATA.to_dict(), f)

    copy_to_data(str(model_output_path), str(metadata_output_path))


if __name__ == "__main__":
    main()
