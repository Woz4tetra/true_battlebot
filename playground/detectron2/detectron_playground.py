import time

import cv2
import numpy as np
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.engine import DefaultPredictor
from detectron2.utils.logger import setup_logger
from detectron2.utils.visualizer import GenericMask, Visualizer

setup_logger()


im = cv2.imread("./input.jpg")
im = cv2.resize(im, (640 * 2, 480 * 2))

cfg = get_cfg()
# add project-specific config (e.g., TensorMask) here if you're not running a model in detectron2's core library
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set threshold for this model
# Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
predictor = DefaultPredictor(cfg)
metadata = MetadataCatalog.get(cfg.DATASETS.TRAIN[0])
for _ in range(10):
    t0 = time.time()
    outputs = predictor(im)
    t1 = time.time()
    print(f"took: {t1 - t0}")

instances = outputs["instances"].to("cpu")

height, width = instances.image_size

masks = np.asarray(instances.pred_masks)
masks = [GenericMask(x, height, width) for x in masks]

breakpoint()

v = Visualizer(im[:, :, ::-1], metadata, scale=1.2)
out = v.draw_instance_predictions(instances)
cv2.imshow("image", out.get_image()[:, :, ::-1])
cv2.waitKey(-1)
