import gc
import logging
from pathlib import Path

import cv2
import numpy as np
import torch
import tqdm
from cotracker.predictor import CoTrackerPredictor
from perception_tools.training.keypoints_config import KeypointsConfig
from perception_tools.training.yolo_keypoint_dataset import YoloKeypointAnnotation, YoloKeypointImage, YoloVisibility
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

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
            if image_path.suffix.lower() not in {".jpg", ".png"}:
                continue
            image = cv2.imread(str(image_path))
            if image is None:
                self.logger.warning(f"Failed to read image: {image_path}")
                continue
            images.append(image)
        return images

    def _save_annotation(self, images_dir: Path, annotation: YoloKeypointImage) -> None:
        annotation_path = images_dir / f"{annotation.image_id}.txt"
        with open(annotation_path, "w") as file:
            file.write(annotation.to_txt())

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
        self, images_dir: Path, start_annotation: YoloKeypointImage, images_width: int, images_height: int
    ) -> None:
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available")
        frames = self._load_images_alphabetically(images_dir)

        interpolated_annotations = self._interpolate_keypoints(frames, start_annotation, images_width, images_height)
        self._fill_out_bounding_boxes(frames, interpolated_annotations, images_width, images_height)
        for annotation in interpolated_annotations:
            self._save_annotation(images_dir, annotation)

    def _annotation_keypoints_to_input_coordinates(
        self, annotation: YoloKeypointImage, images_width: int, images_height: int
    ) -> list[tuple[float, float]]:
        input_coordinates = []
        for label in annotation.labels:
            keypoints = label.keypoints
            label_coordinates = []
            for keypoint in keypoints:
                label_coordinates.append((keypoint[0] * images_width, keypoint[1] * images_height))
            input_coordinates.append(label_coordinates)
        return input_coordinates

    def _fill_out_bounding_boxes(
        self, frames: list[np.ndarray], annotations: list[YoloKeypointImage], images_width: int, images_height: int
    ) -> None:
        # select the device for computation
        device = torch.device("cuda")

        # use bfloat16 for the entire notebook
        with torch.autocast("cuda", dtype=torch.bfloat16):
            # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True

            sam2_model = build_sam2(self.config.sam2_model_config_path, self.config.sam2_checkpoint, device=device)

            predictor = SAM2ImagePredictor(sam2_model)

            num_frames = len(frames)
            pbar = tqdm.tqdm(total=num_frames, desc="Filling out bounding boxes", unit="frame")
            for index in range(0, num_frames, self.config.sam2_batch_size):
                pbar.update(self.config.sam2_batch_size)
                all_coordinates = []
                all_labels = []
                for batch_index in range(self.config.sam2_batch_size):
                    batch_frame_index = index + batch_index
                    if batch_frame_index >= num_frames:
                        break
                    annotation = annotations[batch_frame_index]
                    per_object_coordinates = self._annotation_keypoints_to_input_coordinates(
                        annotation, images_width, images_height
                    )
                    input_coordinates = []
                    input_labels = []
                    for main_index in range(len(per_object_coordinates)):
                        object_row = []
                        label_row = []
                        for sub_index in range(len(per_object_coordinates)):
                            object_coordinates = per_object_coordinates[sub_index]
                            object_row.extend(object_coordinates)
                            if sub_index == main_index:
                                label_row.extend([1] * len(object_coordinates))
                            else:
                                label_row.extend([0] * len(object_coordinates))
                        input_coordinates.append(object_row)
                        input_labels.append(label_row)
                    all_coordinates.append(input_coordinates)
                    all_labels.append(input_labels)
                predictor.set_image_batch(frames[index : index + self.config.sam2_batch_size])
                batch_masks, batch_scores, batch_logits = predictor.predict_batch(
                    point_coords_batch=all_coordinates,
                    point_labels_batch=all_labels,
                    multimask_output=True,
                )
                all_masks = []
                for batch_index in range(self.config.sam2_batch_size):
                    batch_frame_index = index + batch_index
                    if batch_frame_index >= num_frames:
                        break
                    for mask_index in range(len(batch_masks[batch_index])):
                        frame = frames[batch_frame_index]
                        annotation = annotations[batch_frame_index]
                        masks = batch_masks[batch_index][mask_index]
                        scores = batch_scores[batch_index][mask_index]
                        max_index = np.argmax(scores)
                        mask = masks[max_index]
                        mask_image = mask.astype(np.uint8).reshape(frame.shape[0], frame.shape[1], 1) * 255
                        contours, hierarchy = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

                        max_area_index = 0
                        max_area = 0
                        for contour_index in range(len(contours)):
                            area = cv2.contourArea(contours[contour_index])
                            if area > max_area:
                                max_area = area
                                max_area_index = contour_index
                        absolute_xywh = cv2.boundingRect(contours[max_area_index])

                        relative_xywh = (
                            absolute_xywh[0] / images_width,
                            absolute_xywh[1] / images_height,
                            absolute_xywh[2] / images_width,
                            absolute_xywh[3] / images_height,
                        )
                        center_x = relative_xywh[0] + relative_xywh[2] / 2
                        center_y = relative_xywh[1] + relative_xywh[3] / 2
                        box = (center_x, center_y, relative_xywh[2], relative_xywh[3])
                        all_masks.append(mask)
                        annotation.labels[mask_index].bbox = box

    def _interpolate_keypoints(
        self, frames: list[np.ndarray], start_annotation: YoloKeypointImage, images_width: int, images_height: int
    ) -> list[YoloKeypointImage]:
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available")
        device = torch.device("cuda")
        annotations: list[YoloKeypointImage] = []
        predictor = CoTrackerPredictor(checkpoint=self.config.cotracker_checkpoint)
        predictor = predictor.to(device)
        video_tensor = torch.from_numpy(np.stack(frames)).permute(0, 3, 1, 2)[None].float()
        video_tensor = video_tensor.to(device)

        query_coordinates = []
        query_to_object_id: dict[int, int] = {}
        query_to_class_index: dict[int, int] = {}
        for object_index, label in enumerate(start_annotation.labels):
            keypoints = label.keypoints
            for keypoint in keypoints:
                query_to_object_id[len(query_coordinates)] = object_index
                query_to_class_index[len(query_coordinates)] = label.class_index
                query_coordinates.append(
                    [
                        0,  # frame index is always 0
                        keypoint[0] * images_width,
                        keypoint[1] * images_height,
                    ]
                )
        query = torch.tensor(np.array(query_coordinates, dtype=np.float32)).to(device)
        self.logger.debug("Made co-tracker query")
        model = CoTrackerPredictor(checkpoint=self.config.cotracker_checkpoint)
        model = model.to(device)
        pred_tracks, pred_visibility = model(video_tensor, queries=query[None])
        pred_tracks_cpu = pred_tracks.cpu().numpy()[0]
        pred_visibility_cpu = pred_visibility.cpu().numpy()[0]
        for frame_index in range(len(frames)):
            image_id = f"{frame_index:06d}"
            labels: dict[int, YoloKeypointAnnotation] = {}
            for query_index in range(len(pred_tracks_cpu[frame_index])):
                object_keypoint = pred_tracks_cpu[frame_index][query_index]
                object_keypoint_visibility = pred_visibility_cpu[frame_index][query_index]
                keypoint_data = (
                    float(object_keypoint[0] / images_width),
                    float(object_keypoint[1] / images_height),
                    YoloVisibility.LABELED_VISIBLE
                    if object_keypoint_visibility
                    else YoloVisibility.LABELED_NOT_VISIBLE,
                )
                object_index = query_to_object_id[query_index]
                class_index = query_to_class_index[query_index]
                if object_index not in labels:
                    labels[object_index] = YoloKeypointAnnotation(
                        class_index=class_index,
                        bbox=(0.0, 0.0, 0.0, 0.0),  # filled out later
                        keypoints=[],
                    )
                labels[object_index].keypoints.append(keypoint_data)
            annotation = YoloKeypointImage(
                image_id=image_id,
                labels=[labels[object_index] for object_index in sorted(labels.keys())],
            )
            annotations.append(annotation)
        self.logger.debug("Co-tracker keypoints interpolated")

        # deinitialize cuda
        del predictor
        del model
        del video_tensor
        gc.collect()
        # torch.cuda.empty_cache()
        # torch.cuda.synchronize()
        # torch.cuda.reset_peak_memory_stats()
        # torch.cuda.reset_accumulated_memory_stats()

        return annotations
