import argparse
from pathlib import Path

import cv2
import torch
import tqdm
from perception_tools.config.model_metadata import FIELD_SEMANTIC_MODEL_METADATA
from perception_tools.inference.common import get_default_device
from perception_tools.inference.deeplabv3 import DeepLabV3Inference
from perception_tools.training.deeplabv3 import load_model


def main() -> None:
    parser = argparse.ArgumentParser(description="Test inference of a semantic segmentation model")
    parser.add_argument(
        "model",
        type=str,
        help="Path to the model (*.pth or *.torchscript)",
    )
    parser.add_argument(
        "image_path",
        nargs="+",
        type=str,
        help="Path to the image to test",
    )
    args = parser.parse_args()
    model_path = Path(args.model)
    image_paths = [Path(image_path) for image_path in args.image_path]

    output_path = image_paths[0].parent / Path("output")
    print(f"Saving output to {output_path}")
    output_path.mkdir(exist_ok=True)

    device = get_default_device()

    print(f"Using device: {device}")
    if model_path.suffix == ".torchscript":
        print("Loading torchscript model")
        model = torch.jit.load(model_path, map_location=device)
    else:
        print("Loading PyTorch model")
        model = load_model(model_path, device)

    inference = DeepLabV3Inference(model, device)

    for image_path in tqdm.tqdm(image_paths):
        image = cv2.imread(str(image_path))

        out_mask = inference.compute_inference(image)
        color_seg_resized = inference.draw_debug_image(
            out_mask, image.shape[1], image.shape[0], FIELD_SEMANTIC_MODEL_METADATA
        )

        output_image = cv2.addWeighted(image, 0.5, color_seg_resized, 0.5, 0)
        # output_image = cv2.addWeighted(image_resized, 0.5, color_seg, 0.5, 0)
        cv2.imwrite(str(output_path / image_path.name), output_image)


if __name__ == "__main__":
    main()
