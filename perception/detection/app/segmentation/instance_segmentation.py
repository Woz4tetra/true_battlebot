import logging
import time
from typing import Callable

import cv2
import numpy as np
import torch
import torchvision
from app.config.segmentation_config.instance_segmentation_config import InstanceSegmentationConfig
from app.segmentation.segmentation_interface import SegmentationInterface
from bw_interfaces.msg import SegmentationInstance, SegmentationInstanceArray
from bw_shared.messages.header import Header
from detectron2.layers import paste_masks_in_image
from detectron2.utils.visualizer import GenericMask
from perception_tools.inference.common import contour_to_msg, load_metadata
from perception_tools.messages.image import Image
from torch import Tensor

BoundingBox = tuple[int, int, int, int]


class InstanceSegmentation(SegmentationInterface):
    def __init__(self, config: InstanceSegmentationConfig) -> None:
        self.model_path = config.model_path
        self.metadata_path = config.metadata_path
        self.threshold = config.threshold
        self.nms_threshold = config.nms_threshold
        self.mask_conversion_threshold = config.mask_conversion_threshold
        self.decimate = config.decimate
        self.image_delay_threshold = config.image_delay_threshold
        self.debug = config.debug

        self.metadata = load_metadata(config.metadata_path)

        self.original_dims: tuple[int, int] | None = None
        self.resize_dims: tuple[int, int] | None = None

        self.device = torch.device("cuda")
        self.model = self.load_model(self.model_path)
        self.warmup()

        self.logger = logging.getLogger("perception")

    def load_model(self, model_path: str) -> Callable:
        self.logger.info(f"Loading model from {model_path}")
        t0 = time.perf_counter()
        model = torch.jit.load(model_path).to(self.device)  # type: ignore
        t1 = time.perf_counter()
        self.logger.info(f"Loaded model in {t1 - t0} seconds")
        return model

    def warmup(self) -> None:
        self.logger.info("Warming up model")
        t0 = time.perf_counter()
        for _ in range(3):
            image = Image(Header.auto(), np.random.random_integers(0, 255, size=(1920, 1080, 3)).astype(np.uint8))
            self.predict(image, False)
        t1 = time.perf_counter()
        self.logger.info(f"Model warmed up in {t1 - t0} seconds")

    def update_resize(self, shape: tuple[int, ...]) -> None:
        if self.resize_dims is None:
            self.resize_dims = (int(shape[1] // self.decimate), int(shape[0] // self.decimate))
        if self.original_dims is None:
            self.original_dims = (shape[1], shape[0])

    def preprocess(self, images: list[np.ndarray], device: torch.device) -> list[dict[str, Tensor]]:
        inputs = []
        for image in images:
            if self.resize_dims is not None:
                image = cv2.resize(image, self.resize_dims, interpolation=cv2.INTER_NEAREST)
            image_tensor = torch.from_numpy(image).to(device).permute(2, 0, 1).float()
            inputs.append({"image": image_tensor})
        return inputs

    def predict(self, image: Image, debug: bool) -> tuple[SegmentationInstanceArray, Image | None]:
        t0 = time.perf_counter()
        images = [image.data]
        inputs = self.preprocess(images, self.device)
        t1 = time.perf_counter()
        outputs = self.model(inputs)
        t2 = time.perf_counter()
        results = self.postprocess(outputs[0], image, debug)
        t3 = time.perf_counter()
        self.logger.debug(f"Pre-process took: {t1 - t0}")
        self.logger.debug(f"Inference took: {t2 - t1}")
        self.logger.debug(f"Post-process took: {t3 - t2}. Debug enabled: {debug}")
        return results

    def postprocess(
        self, result: dict[str, Tensor], image: Image, debug: bool
    ) -> tuple[SegmentationInstanceArray, Image | None]:
        image_data = image.data
        height, width = image.data.shape[:2]
        if self.original_dims is not None:
            image_data = cv2.resize(image_data, self.original_dims, interpolation=cv2.INTER_NEAREST)

        msg = SegmentationInstanceArray()
        msg.header = image.header.to_msg()
        msg.height = height
        msg.width = width

        to_keep = torchvision.ops.nms(result["pred_boxes"], result["scores"], self.nms_threshold)
        result["pred_boxes"] = result["pred_boxes"][to_keep].cpu()
        result["pred_classes"] = result["pred_classes"][to_keep].cpu()
        result["pred_masks"] = result["pred_masks"][to_keep].cpu()

        stretched_masks = paste_masks_in_image(
            result["pred_masks"][:, 0, :, :],
            result["pred_boxes"],
            (height, width),
            threshold=self.mask_conversion_threshold,
        )
        masks = [GenericMask(m, height, width) for m in np.asarray(stretched_masks)]
        class_indices = [int(label.item()) for label in result["pred_classes"]]
        object_indices = {}

        debug_image = Image(image.header, np.copy(image_data)) if debug else None

        for mask, bbox_tensor, class_idx, score_tensor in zip(
            masks, result["pred_boxes"], class_indices, result["scores"]
        ):
            contours = []
            for segment in mask.polygons:
                if self.decimate != 1.0:
                    segment *= self.decimate
                contour = np.array(segment.reshape(-1, 2), dtype=np.int32)
                contours.append(contour)
            score = float(score_tensor)
            if score < self.threshold:
                continue
            if class_idx not in object_indices:
                object_indices[class_idx] = 0
            object_idx = object_indices[class_idx]
            object_indices[class_idx] += 1

            label = self.metadata.labels[class_idx]

            segmentation_instance = SegmentationInstance(
                contours=contour_to_msg(contours),
                score=score,
                label=label,
                class_index=class_idx,
                object_index=object_idx,
                has_holes=mask.has_holes,
            )
            msg.instances.append(segmentation_instance)
            if debug_image is not None:
                bbox_array = bbox_tensor * self.decimate if self.decimate != 1.0 else bbox_tensor
                bbox: BoundingBox = tuple(map(int, bbox_array))  # type: ignore
                self.draw_segmentation(debug_image.data, bbox, class_idx, contours, score)
        return msg, debug_image

    def draw_segmentation(
        self, image: np.ndarray, bbox: BoundingBox, class_idx: int, contours: list[np.ndarray], score: float
    ) -> None:
        x1, y1, x2, y2 = bbox
        class_name = self.metadata.labels[class_idx]
        class_color = self.metadata.colors[class_idx]
        cv_color = class_color.to_cv_color()
        cv2.rectangle(image, (x1, y1), (x2, y2), cv_color, 1)
        cv2.putText(
            image,
            f"{class_name} {score * 100:0.1f}",
            (x1, y1),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            cv_color,
        )

        class_color = self.metadata.colors[class_idx]
        cv_color = class_color.to_cv_color()
        for contour in contours:
            cv2.drawContours(image, [contour], -1, cv_color, 1)

    def process_image(self, msg: Image) -> tuple[SegmentationInstanceArray, Image | None]:
        self.logger.debug(f"Image delay is: {time.time() - msg.header.stamp}")
        image = msg.data
        self.update_resize(image.shape)
        segmentations, debug_image = self.predict(msg, self.debug)
        self.logger.debug(f"Callback delay is: {time.time() - msg.header.stamp}")
        return segmentations, debug_image
