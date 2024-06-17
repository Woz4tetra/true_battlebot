import argparse
from pathlib import Path

import cv2
import numpy as np
import torch
from bw_shared.enums.label import Label
from perception_tools.config.model_metadata import LABEL_COLORS
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor

color_map = {
    0: (0, 0, 0),
    1: LABEL_COLORS[Label.FIELD].to_cv_color(),
}


def prediction_to_vis(prediction):
    vis_shape = prediction.shape + (3,)
    vis = np.zeros(vis_shape)
    for i, c in color_map.items():
        vis[prediction == i] = color_map[i]
    return vis.astype(np.uint8)


def main() -> None:
    parser = argparse.ArgumentParser(description="Test inference of a semantic segmentation model")
    parser.add_argument(
        "model",
        type=str,
        help="Path to directory containing the model (folder contains *.safetensors and config.json)",
    )
    parser.add_argument(
        "image_path",
        nargs="+",
        type=str,
        help="Path to the image to test",
    )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()
    model_path = args.model
    image_paths = [Path(image_path) for image_path in args.image_path]

    processor = SegformerImageProcessor(do_resize=False)
    model = SegformerForSemanticSegmentation.from_pretrained(model_path)
    model.to(device)
    output_path = image_paths[0].parent / Path("output")
    print(f"Saving output to {output_path}")
    output_path.mkdir(exist_ok=True)

    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        pixel_values = processor(image, return_tensors="pt").pixel_values.to(device)

        with torch.no_grad():
            outputs = model(pixel_values, return_dict=True)
        predicted_segmentation_map = processor.post_process_semantic_segmentation(
            outputs, target_sizes=[(image.shape[0], image.shape[1])]
        )
        predicted_segmentation_map = predicted_segmentation_map[0].cpu().numpy()

        color_seg = np.zeros_like(image)
        for label, color in enumerate(color_map.values()):
            color_seg[predicted_segmentation_map == label] = color

        cv2.addWeighted(image, 0.5, color_seg, 0.5, 0, image)
        cv2.imwrite(str(output_path / image_path.name), image)


if __name__ == "__main__":
    main()
