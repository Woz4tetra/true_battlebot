import gc
import logging
from pathlib import Path

import cv2
import numpy as np
import torch
from cotracker.predictor import CoTrackerPredictor
from perception_tools.training.keypoints_config import KeypointsConfig
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointAnnotation, YoloKeypointImage, YoloVisibility
from sam2.build_sam import build_sam2_video_predictor
from sam2.sam2_video_predictor import SAM2VideoPredictor

from auto_label.config.auto_label_config import TrackerConfig


class AiInterpolator:
    def __init__(self, config: TrackerConfig, keypoints_config: KeypointsConfig) -> None:
        self.config = config
        self.keypoints_config = keypoints_config
        self.inference_state: dict = {}
        self.logger = logging.getLogger(self.__class__.__name__)

    def _load_images_alphabetically(self, images_dir: Path) -> list[np.ndarray]:
        images = []
        for image_path in sorted(images_dir.iterdir()):
            if image_path.suffix.lower() in {".jpg", ".png"}:
                image = cv2.imread(str(image_path))
                if image is None:
                    self.logger.warning(f"Failed to read image: {image_path}")
                    continue
                images.append(image)
        return images

    def _convert_to_annotation(
        self,
        image_id: str,
        out_obj_ids: list[int],
        out_mask_logits: torch.Tensor,
        class_index: int,
        frame_dimensions: tuple[int, int],
    ) -> YoloKeypointImage:
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

    def _propagate_object_in_video(
        self,
        predictor: SAM2VideoPredictor,
        state: dict,
        class_index: int,
        frame_dimensions: tuple[int, int],
    ) -> list[YoloKeypointImage]:
        annotations: list[YoloKeypointImage] = []
        for frame_idx, out_obj_ids, out_mask_logits in predictor.propagate_in_video(state):
            image_id = f"{frame_idx:06d}"
            interpolated_annotation_single = self._convert_to_annotation(
                image_id, out_obj_ids, out_mask_logits, class_index, frame_dimensions
            )
            annotations.append(interpolated_annotation_single)
        return annotations

    def _merge_annotations(self, separate_annotations: list[list[YoloKeypointImage]]) -> list[YoloKeypointImage]:
        merged_annotations: list[YoloKeypointImage] = [
            YoloKeypointImage(image_id=annotations.image_id, labels=[]) for annotations in separate_annotations[0]
        ]
        for annotations in separate_annotations:
            for frame_index, annotation in enumerate(annotations):
                label = annotation.labels[0]
                merged_annotations[frame_index].labels.append(label)
        return merged_annotations

    def interpolate(
        self, images_dir: Path, start_annotation: YoloKeypointImage, frame_dimensions: tuple[int, int]
    ) -> None:
        bbox_interpolated = self._interpolate_bboxes(images_dir, start_annotation, frame_dimensions)
        interpolated_annotations = self._interpolate_keypoints(
            images_dir, start_annotation, frame_dimensions, bbox_interpolated
        )
        for annotation in interpolated_annotations:
            self._save_annotation(images_dir, annotation)

    def _interpolate_bboxes(
        self, images_dir: Path, start_annotation: YoloKeypointImage, frame_dimensions: tuple[int, int]
    ) -> list[YoloKeypointImage]:
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available")
        device = torch.device("cuda")
        predictor = build_sam2_video_predictor(
            self.config.sam2_model_config_path, self.config.sam2_checkpoint, device=device
        )

        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16):
            state = predictor.init_state(str(images_dir), offload_video_to_cpu=False)
            separate_annotations: list[list[YoloKeypointImage]] = []
            for object_id, label in enumerate(start_annotation.labels):
                self.logger.info(f"Interpolating object {object_id + 1} of {len(start_annotation.labels)}")
                corners_relative = label.corners
                corners_absolute = (
                    corners_relative[0] * frame_dimensions[0],
                    corners_relative[1] * frame_dimensions[1],
                    corners_relative[2] * frame_dimensions[0],
                    corners_relative[3] * frame_dimensions[1],
                )
                predictor.add_new_points_or_box(state, box=corners_absolute, frame_idx=0, obj_id=0)
                single_annotations = self._propagate_object_in_video(
                    predictor, state, label.class_index, frame_dimensions
                )

                separate_annotations.append(single_annotations)
                predictor.reset_state(state)

        # deinitialize cuda
        del predictor
        del state
        gc.collect()
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
        torch.cuda.reset_accumulated_memory_stats()

        interpolated_annotations = self._merge_annotations(separate_annotations)
        return interpolated_annotations

    def _interpolate_keypoints(
        self,
        images_dir: Path,
        start_annotation: YoloKeypointImage,
        frame_dimensions: tuple[int, int],
        bbox_annotations: list[YoloKeypointImage],
    ) -> list[YoloKeypointImage]:
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available")
        device = torch.device("cuda")
        predictor = CoTrackerPredictor(checkpoint=self.config.cotracker_checkpoint)
        predictor = predictor.to(device)

        frames = self._load_images_alphabetically(images_dir)
        video = torch.from_numpy(np.stack(frames)).permute(0, 3, 1, 2)[None].float()
        video = video.to(device)

        query_coordinates = []
        query_to_object_id: dict[int, int] = {}
        for object_index, label in enumerate(start_annotation.labels):
            keypoints = label.keypoints
            for keypoint in keypoints:
                query_to_object_id[len(query_coordinates)] = object_index
                query_coordinates.append(
                    [
                        0,  # frame index is always 0
                        keypoint[0] * frame_dimensions[0],
                        keypoint[1] * frame_dimensions[1],
                    ]
                )
        query = torch.tensor(np.array(query_coordinates, dtype=np.float32)).to(device)
        self.logger.debug("Made co-tracker query")
        model = CoTrackerPredictor(checkpoint=self.config.cotracker_checkpoint)
        model = model.to(device)
        pred_tracks, pred_visibility = model(video, queries=query[None])
        pred_tracks_cpu = pred_tracks.cpu().numpy()[0]
        pred_visibility_cpu = pred_visibility.cpu().numpy()[0]
        for frame_index in range(len(bbox_annotations)):
            for query_index in range(len(pred_tracks_cpu[frame_index])):
                object_keypoint = pred_tracks_cpu[frame_index][query_index]
                object_keypoint_visibility = pred_visibility_cpu[frame_index][query_index]
                keypoint_data = (
                    float(object_keypoint[0] / frame_dimensions[0]),
                    float(object_keypoint[1] / frame_dimensions[1]),
                    YoloVisibility.LABELED_VISIBLE
                    if object_keypoint_visibility
                    else YoloVisibility.LABELED_NOT_VISIBLE,
                )
                object_index = query_to_object_id[query_index]
                bbox_annotations[frame_index].labels[object_index].keypoints.append(keypoint_data)
        self.logger.debug("Co-tracker keypoints interpolated")

        # deinitialize cuda
        del predictor
        del model
        gc.collect()
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
        torch.cuda.reset_accumulated_memory_stats()

        return bbox_annotations
