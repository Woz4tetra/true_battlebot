import argparse
import json
from pathlib import Path

import torch
from perception_tools.config.model_metadata import FIELD_SEMANTIC_MODEL_METADATA
from perception_tools.inference.common import get_default_device
from perception_tools.training.deeplabv3 import load_model


def main() -> None:
    parser = argparse.ArgumentParser(description="Test inference of a semantic segmentation model")
    parser.add_argument(
        "model",
        type=str,
        help="Path to the model (*.pth)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="",
        help="Path to the output model "
        "(no extension, will be saved as *.torchscript. Metadata will be saved as *.json)",
    )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()
    model_path = Path(args.model)
    model_output_path = Path(args.output) if args.output else model_path.parent / (model_path.stem + ".torchscript")
    metadata_output_path = Path(args.output) if args.output else model_path.parent / (model_path.stem + ".json")

    print(f"Saving output to {model_output_path}")

    device = get_default_device()
    print(f"Using device: {device}")
    model = load_model(model_path, device)

    scripted_model = torch.jit.script(model)
    torch.jit.save(scripted_model, model_output_path)

    with open(metadata_output_path, "w") as f:
        json.dump(FIELD_SEMANTIC_MODEL_METADATA.to_dict(), f)


if __name__ == "__main__":
    main()
