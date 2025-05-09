from pathlib import Path

import cv2
import numpy as np
import torch
import tqdm
from perception_tools.training.keypoints_config import KeypointsConfig
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointAnnotation, YoloKeypointImage
from sam2.build_sam import build_sam2_video_predictor
from sam2.sam2_video_predictor import SAM2VideoPredictor

from auto_label.config.auto_label_config import TrackerConfig


class AiInterpolator:
    def __init__(self, config: TrackerConfig, keypoints_config: KeypointsConfig) -> None:
        self.config = config
        self.keypoints_config = keypoints_config
        self.predictor: SAM2VideoPredictor | None = None
        self.inference_state: dict = {}

    def _initialize(self, images_dir: str) -> None:
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available")
        device = torch.device("cuda")

        # use bfloat16
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

        self.predictor = build_sam2_video_predictor(self.config.sam2_model_config_path, self.config.checkpoint, device)
        self.inference_state = self.predictor.init_state(video_path=str(images_dir))

    def _add_box(self, object_id: int, box: tuple[float, float, float, float]) -> None:
        if self.predictor is None:
            raise RuntimeError("Predictor is not initialized")
        box_array = np.array(box, dtype=np.float32)
        self.predictor.add_new_points_or_box(
            self.inference_state,
            frame_idx=0,
            obj_id=object_id,
            box=box_array,
        )

    def _convert_to_annotation(
        self,
        image_id: str,
        out_obj_ids: list[int],
        out_mask_logits: torch.Tensor,
        obj_id_to_class_index: list[int],
        frame_dimensions: tuple[int, int],
    ) -> YoloKeypointImage:
        if self.predictor is None:
            raise RuntimeError("Predictor is not initialized")
        annotation = YoloKeypointImage(image_id=image_id, labels=[])
        for index, out_obj_id in enumerate(out_obj_ids):
            mask = (out_mask_logits[index] > 0.0).cpu().numpy().astype(np.uint8)
            height, width = mask.shape[-2:]
            mask_image = mask.reshape(height, width, 1) * 255
            contours, hierarchy = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            absolute_xywh = cv2.boundingRect(contours[0])
            relative_xywh = (
                absolute_xywh[0] / frame_dimensions[0],
                absolute_xywh[1] / frame_dimensions[1],
                absolute_xywh[2] / frame_dimensions[0],
                absolute_xywh[3] / frame_dimensions[1],
            )
            class_index = obj_id_to_class_index[out_obj_id]
            annotation.labels.append(
                YoloKeypointAnnotation.from_xywh(
                    x=relative_xywh[0],
                    y=relative_xywh[1],
                    w=relative_xywh[2],
                    h=relative_xywh[3],
                    class_index=class_index,
                    # TODO add keypoints
                )
            )
        return annotation

    def _save_annotation(self, images_dir: Path, annotation: YoloKeypointImage) -> None:
        annotation_path = images_dir / f"{annotation.image_id}.txt"
        with open(annotation_path, "w") as file:
            file.write(annotation.to_txt())

    def interpolate(
        self, images_dir: Path, start_annotation: YoloKeypointImage, frame_dimensions: tuple[int, int]
    ) -> None:
        self._initialize(str(images_dir))
        assert self.predictor is not None
        obj_id_to_class_index = [label.class_index for label in start_annotation.labels]
        for object_id, label in enumerate(start_annotation.labels):
            corners_relative = label.corners
            corners_absolute = (
                corners_relative[0] * frame_dimensions[0],
                corners_relative[1] * frame_dimensions[1],
                corners_relative[2] * frame_dimensions[0],
                corners_relative[3] * frame_dimensions[1],
            )
            self._add_box(object_id, corners_absolute)

        for out_frame_idx, out_obj_ids, out_mask_logits in self.predictor.propagate_in_video(self.inference_state):
            image_id = f"{out_frame_idx:06d}"
            interpolated_annotation = self._convert_to_annotation(
                image_id, out_obj_ids, out_mask_logits, obj_id_to_class_index, frame_dimensions
            )
            self._save_annotation(images_dir, interpolated_annotation)
        del self.predictor
        self.predictor = None
