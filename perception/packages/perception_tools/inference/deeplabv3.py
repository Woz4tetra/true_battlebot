import cv2
import numpy as np
import torch
from perception_tools.config.model_metadata import ModelMetadata
from perception_tools.inference.common import get_default_device
from torch import nn
from torchvision import transforms

IMAGE_SIZE = 344
PAD_SIZE = 20


class SquarePad:
    def __call__(self, image, pad_size: int):
        return np.pad(image, ((pad_size, pad_size), (pad_size, pad_size), (0, 0)), mode="constant", constant_values=0)


def common_transforms(mean=(0.4611, 0.4359, 0.3905), std=(0.2193, 0.2150, 0.2109)) -> transforms.Compose:
    return transforms.Compose(
        [
            transforms.ToPILImage(),
            transforms.Pad(PAD_SIZE),
            transforms.ToTensor(),
            transforms.Normalize(mean, std),
        ]
    )


class DeepLabV3Inference:
    def __init__(self, model: nn.Module, device: torch.device | None = None) -> None:
        self.model = model
        self.preprocess = common_transforms()
        self.device = get_default_device() if device is None else device

    def compute_inference(self, image: np.ndarray) -> np.ndarray:
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (IMAGE_SIZE, IMAGE_SIZE), interpolation=cv2.INTER_NEAREST)

        image_model = self.preprocess(image_resized)
        image_model = torch.unsqueeze(image_model, dim=0)  # type: ignore
        image_model = image_model.to(self.device)

        with torch.no_grad():
            out_mask = self.model(image_model)["out"].cpu()

        out_mask = (
            torch.argmax(
                out_mask,
                dim=1,
                keepdims=True,  # type: ignore
            )
            .permute(0, 2, 3, 1)[0]
            .numpy()
            .squeeze()
            .astype(np.int32)
        )
        if PAD_SIZE > 0:
            cropped_mask = out_mask[PAD_SIZE:-PAD_SIZE, PAD_SIZE:-PAD_SIZE]
        else:
            cropped_mask = out_mask
        return cropped_mask

    def draw_debug_image(self, out_mask: np.ndarray, width: int, height: int, metadata: ModelMetadata) -> np.ndarray:
        color_seg = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), dtype=np.uint8)
        for label_index, color in enumerate(metadata.colors):
            color_seg[out_mask == label_index] = color.to_cv_color()
        color_seg_resized = cv2.resize(color_seg, (width, height), interpolation=cv2.INTER_NEAREST)
        return color_seg_resized
