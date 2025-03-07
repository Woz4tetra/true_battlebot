import logging
import time

import cv2
import numpy as np
import torch
from app.config.segmentation.semantic_segmentation_config import SemanticSegmentationConfig
from app.segmentation.segmentation_interface import SegmentationInterface
from bw_interfaces.msg import LabelMap, SegmentationInstance, SegmentationInstanceArray
from bw_shared.enums.label import Label
from bw_shared.messages.contours import contour_to_msg
from perception_tools.data_directory import get_data_directory
from perception_tools.inference.common import get_default_device, load_metadata, mask_to_polygons
from perception_tools.inference.deeplabv3 import IMAGE_SIZE, DeepLabV3Inference
from perception_tools.messages.image import Image


class SemanticSegmentation(SegmentationInterface):
    def __init__(self, config: SemanticSegmentationConfig) -> None:
        self.logger = logging.getLogger("perception")
        self.config = config
        self.device = get_default_device()
        data_dir = get_data_directory()

        model_path = data_dir / "models" / self.config.model_path
        if self.config.metadata_path:
            metadata_path = data_dir / "models" / self.config.metadata_path
        else:
            metadata_path = data_dir / "models" / (model_path.stem + ".json")

        self.model = torch.jit.load(model_path, map_location=self.device)
        self.inference = DeepLabV3Inference(self.model, self.device)
        self.metadata = load_metadata(metadata_path)

        self.model_to_system_labels = self.config.model_to_system_labels.labels
        self.class_indices = self.config.model_to_system_labels.get_class_indices(self.metadata.labels)

        self.warmup()
        self.logger.info("SemanticSegmentation initialized")

    def process_image(self, msg: Image) -> tuple[SegmentationInstanceArray | None, Image | None]:
        self.logger.debug(f"Processing image with shape: {msg.data.shape}")
        height, width = msg.data.shape[:2]

        out_mask = self.inference.compute_inference(msg.data)
        self.logger.debug(f"Field detected: {np.sum(out_mask) > 0}")
        if self.config.debug:
            mask_debug_image = self.inference.draw_debug_image(out_mask, self.metadata)
            debug_image = cv2.addWeighted(msg.data, 0.5, mask_debug_image, 0.5, 0)
            debug_msg = Image(msg.header, data=debug_image)
        else:
            debug_msg = None
        polygon_result = mask_to_polygons(out_mask, self.metadata)

        object_indices = {}

        result = SegmentationInstanceArray()
        result.header = msg.header.to_msg()
        result.height = height
        result.width = width

        for model_label, (contours, has_holes) in polygon_result.items():
            label = self.model_to_system_labels[model_label]
            if label == Label.SKIP:
                continue
            class_idx = self.class_indices[label]

            if class_idx not in object_indices:
                object_indices[class_idx] = 0
            object_idx = object_indices[class_idx]
            object_indices[class_idx] += 1

            segmentation_instance = SegmentationInstance(
                contours=contour_to_msg(contours),
                score=1.0,
                label=label,
                class_index=class_idx,
                object_index=object_idx,
                has_holes=has_holes,
            )
            result.instances.append(segmentation_instance)
        self.logger.debug(f"Found {len(result.instances)} instances")
        return result, debug_msg

    def warmup(self) -> None:
        self.logger.info("Warming up model")
        t0 = time.perf_counter()
        warmup_input = torch.randn((1, 3, IMAGE_SIZE, IMAGE_SIZE)).to(self.device)
        _ = self.model(warmup_input)
        t1 = time.perf_counter()
        self.logger.info(f"Model warmed up in {t1 - t0} seconds")

    def get_model_to_system_labels(self) -> LabelMap:
        return self.config.model_to_system_labels.to_msg()
