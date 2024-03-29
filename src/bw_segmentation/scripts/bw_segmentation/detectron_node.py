#!/usr/bin/env python
import json
import time
from queue import Queue
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
import torch
import torchvision
from bw_interfaces.msg import Contour, SegmentationInstance, SegmentationInstanceArray, UVKeypoint
from bw_tools.configs.model_metadata import ModelMetadata
from bw_tools.typing import get_param
from cv_bridge import CvBridge, CvBridgeError
from detectron2.layers import paste_masks_in_image
from detectron2.utils.visualizer import GenericMask
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from torch import Tensor

BoundingBox = Tuple[int, int, int, int]


class DetectronNode:
    def __init__(self) -> None:
        self.model_path = get_param("~model", "model.torchscript")
        self.metadata_path = get_param("~metadata", "model_metadata.json")
        self.threshold = get_param("~threshold", 0.8)
        self.nms_threshold = get_param("~nms_threshold", 0.4)
        self.mask_conversion_threshold = get_param("~mask_conversion_threshold", 0.5)
        self.decimate = get_param("~decimate", 1.0)
        self.image_delay_threshold = rospy.Duration.from_sec(get_param("~image_delay_threshold", 0.2))

        with open(self.metadata_path, "r") as file:
            self.metadata = ModelMetadata.from_dict(json.load(file))
        assert len(self.metadata.labels) > 0

        self.original_dims: Optional[Tuple[int, int]] = None
        self.resize_dims: Optional[Tuple[int, int]] = None

        self.device = torch.device("cuda")
        self.model = self.load_model(self.model_path)
        self.warmup()

        self.process_queue = Queue(maxsize=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.image_callback, queue_size=1, buff_size=2 << 31)
        self.segmentation_pub = rospy.Publisher("segmentation", SegmentationInstanceArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher("debug_image", Image, queue_size=1)

    def load_model(self, model_path: str) -> Callable:
        rospy.loginfo(f"Loading model from {model_path}")
        t0 = time.perf_counter()
        model = torch.jit.load(model_path).to(self.device)  # type: ignore
        t1 = time.perf_counter()
        rospy.loginfo(f"Loaded model in {t1 - t0} seconds")
        return model

    def warmup(self) -> None:
        rospy.loginfo("Warming up model")
        image = np.random.random_integers(0, 255, size=(1920, 1080, 3)).astype(np.uint8)
        t0 = time.perf_counter()
        for _ in range(3):
            self.predict(Header(), image, False)
        t1 = time.perf_counter()
        rospy.loginfo(f"Model warmed up in {t1 - t0} seconds")

    def update_resize(self, shape: Tuple[int, ...]) -> None:
        if self.resize_dims is None:
            self.resize_dims = (int(shape[1] // self.decimate), int(shape[0] // self.decimate))
        if self.original_dims is None:
            self.original_dims = (shape[1], shape[0])

    def preprocess(self, images: List[np.ndarray], device: torch.device) -> List[Dict[str, Tensor]]:
        inputs = []
        for image in images:
            if self.resize_dims is not None:
                image = cv2.resize(image, self.resize_dims, interpolation=cv2.INTER_NEAREST)
            image_tensor = torch.from_numpy(image).to(device).permute(2, 0, 1).float()
            inputs.append({"image": image_tensor})
        return inputs

    def predict(
        self, header: Header, image: np.ndarray, debug: bool
    ) -> Tuple[List[SegmentationInstanceArray], List[np.ndarray]]:
        t0 = time.perf_counter()
        images = [image]
        inputs = self.preprocess(images, self.device)
        t1 = time.perf_counter()
        outputs = self.model(inputs)
        t2 = time.perf_counter()
        results = self.postprocess(outputs, images, header, debug)
        t3 = time.perf_counter()
        rospy.logdebug(f"Pre-process took: {t1 - t0}")
        rospy.logdebug(f"Inference took: {t2 - t1}")
        rospy.logdebug(f"Post-process took: {t3 - t2}. Debug enabled: {debug}")
        return results

    def postprocess(
        self, outputs: List[Dict[str, Tensor]], images: List[np.ndarray], header: Header, debug: bool
    ) -> Tuple[List[SegmentationInstanceArray], List[np.ndarray]]:
        segmentations = []
        debug_images = []
        for output, image in zip(outputs, images):
            height, width = image.shape[:2]
            if self.original_dims is not None:
                image = cv2.resize(image, self.original_dims, interpolation=cv2.INTER_NEAREST)

            msg = SegmentationInstanceArray()
            msg.header = header
            msg.height = height
            msg.width = width

            to_keep = torchvision.ops.nms(output["pred_boxes"], output["scores"], self.nms_threshold)
            output["pred_boxes"] = output["pred_boxes"][to_keep].cpu()
            output["pred_classes"] = output["pred_classes"][to_keep].cpu()
            output["pred_masks"] = output["pred_masks"][to_keep].cpu()

            stretched_masks = paste_masks_in_image(
                output["pred_masks"][:, 0, :, :],
                output["pred_boxes"],
                (height, width),
                threshold=self.mask_conversion_threshold,
            )
            masks = [GenericMask(m, height, width) for m in np.asarray(stretched_masks)]
            class_indices = [int(label.item()) for label in output["pred_classes"]]
            object_indices = {}

            debug_image = np.copy(image) if debug else None

            for mask, bbox_tensor, class_idx, score_tensor in zip(
                masks, output["pred_boxes"], class_indices, output["scores"]
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
                    contours=self.mask_to_msg(contours),
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
                    self.draw_segmentation(debug_image, bbox, class_idx, contours, score)
                    debug_images.append(debug_image)
            segmentations.append(msg)
        return segmentations, debug_images

    def draw_segmentation(
        self, image: np.ndarray, bbox: BoundingBox, class_idx: int, contours: List[np.ndarray], score: float
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

    def mask_to_msg(self, contours: List[np.ndarray]) -> List[Contour]:
        contour_msgs = []
        for contour in contours:
            points = [UVKeypoint(x, y) for x, y in contour]
            area = cv2.contourArea(contour)
            contour_msg = Contour(points=points, area=area)
            contour_msgs.append(contour_msg)
        return contour_msgs

    def image_callback(self, msg: Image) -> None:
        image_delay = rospy.Time.now() - msg.header.stamp
        rospy.logdebug(f"Callback delay is: {(rospy.Time.now() - msg.header.stamp).to_sec()}")
        if image_delay > self.image_delay_threshold:
            rospy.logdebug("Image is too old. Ignoring.")
            return
        if self.process_queue.full():
            rospy.logdebug("Queue full")
            return
        self.process_queue.put(msg)

    def process_image(self, msg: Image) -> None:
        rospy.logdebug(f"Image delay is: {(rospy.Time.now() - msg.header.stamp).to_sec()}")
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return None
        debug = self.debug_image_pub.get_num_connections() > 0
        self.update_resize(image.shape)
        segmentations, debug_images = self.predict(msg.header, image, debug)
        for segmentation in segmentations:
            self.segmentation_pub.publish(segmentation)
        for debug_image in debug_images:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                rospy.logerr(e)
        rospy.logdebug(f"Callback delay is: {(rospy.Time.now() - msg.header.stamp).to_sec()}")

    def run(self) -> None:
        while not rospy.is_shutdown():
            if self.process_queue.empty():
                rospy.sleep(0.001)
                continue
            self.process_image(self.process_queue.get())


if __name__ == "__main__":
    rospy.init_node("detectron_node", log_level=rospy.DEBUG)
    node = DetectronNode()
    node.run()
