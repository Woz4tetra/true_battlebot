#!/usr/bin/env python
import time
from typing import List, Optional, Tuple

import numpy as np
import rospy
from bw_interfaces.msg import Contour, SegmentationInstance, SegmentationInstanceArray, UVKeypoint
from cv_bridge import CvBridge, CvBridgeError
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog
from detectron2.engine import DefaultPredictor
from detectron2.utils.logger import setup_logger
from detectron2.utils.visualizer import GenericMask, Visualizer
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class DetectronNode:
    def __init__(self) -> None:
        rospy.init_node("detectron_node")
        setup_logger()

        self.cfg = get_cfg()
        # add project-specific config (e.g., TensorMask) here if you're not running a model in detectron2's core library
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set threshold for this model
        # Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        self.predictor = DefaultPredictor(self.cfg)
        self.metadata = MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0])
        self.labels = self.metadata.get("thing_classes", [])

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.image_callback, queue_size=1)
        self.segmentation_pub = rospy.Publisher("segmentation", SegmentationInstanceArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher("debug_image", Image, queue_size=1)

    def predict(
        self, header: Header, image: np.ndarray, debug: bool
    ) -> Tuple[SegmentationInstanceArray, Optional[np.ndarray]]:
        t0 = time.time()
        outputs = self.predictor(image)
        t1 = time.time()
        rospy.logdebug(f"took: {t1 - t0}")

        instances = outputs["instances"].to("cpu")
        height, width = instances.image_size

        masks = np.asarray(instances.pred_masks)
        masks = [GenericMask(x, height, width) for x in masks]

        object_indices = {}

        msg = SegmentationInstanceArray()
        for index in range(len(masks)):
            mask = masks[index]
            score = instances.scores[index]
            class_idx = instances.pred_classes[index]
            label = self.labels[class_idx]
            if class_idx not in object_indices:
                object_indices[class_idx] = 0
            object_idx = object_indices[class_idx]
            object_indices[class_idx] += 1

            segmentation_instance = SegmentationInstance(
                points=self.mask_to_msg(mask),
                score=score,
                label=label,
                class_index=class_idx,
                object_index=object_idx,
                has_holes=mask.has_holes,
            )
            msg.instances.append(segmentation_instance)  # type: ignore
        msg.header = header
        msg.height = height
        msg.width = width

        if debug:
            viz = Visualizer(image[:, :, ::-1], self.metadata, scale=1.2)
            out = viz.draw_instance_predictions(instances)
            viz_image = out.get_image()[:, :, ::-1]
        else:
            viz_image = None

        return msg, viz_image

    def mask_to_msg(self, mask: GenericMask) -> List[Contour]:
        contours = []
        for segment in mask.polygons:
            points = []
            for x, y in segment.reshape(-1, 2):
                keypoint = UVKeypoint(x, y)
                points.append(keypoint)
            contour = Contour(points=points)
            contours.append(contour)
        return contours

    def image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return None
        debug = self.debug_image_pub.get_num_connections() > 0
        segmentation, debug_image = self.predict(msg.header, image, debug)
        self.segmentation_pub.publish(segmentation)
        if debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                rospy.logerr(e)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = DetectronNode()
    node.run()
