import os

import cv2
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog
from detectron2.data.datasets import register_coco_instances
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from matplotlib import pyplot as plt

MODELS_DIR = "/data/training/models/nhrl/nhrl_dataset/mask_rcnn_R_50_FPN_3x/2023-11-20-13-20-28"
MODEL_CHECKPOINT = "model_0047999.pth"

IMAGE_PATH = (
    "/data/training/labeled/nhrl_dataset/test/"
    "zed_2023-09-30T13-38-21_repaired_004368_jpg.rf.bf5172740d5c353cbbfb658058e1b4b4.jpg"
)
CONFIG_PATH = "/data/training/labeled/nhrl_dataset/config.json"

register_coco_instances(
    name="nhrl_dataset",
    metadata={},
    json_file=CONFIG_PATH,
    image_root=MODELS_DIR,
)
threshold = 0.8
cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = threshold  # set threshold for this model
cfg.MODEL.WEIGHTS = os.path.join(MODELS_DIR, MODEL_CHECKPOINT)
predictor = DefaultPredictor(cfg)
metadata = MetadataCatalog.get(cfg.DATASETS.TRAIN[0])
labels = metadata.get("thing_classes")

image = cv2.imread(IMAGE_PATH)
outputs = predictor(image)

instances = outputs["instances"].to("cpu")
height, width = instances.image_size

viz = Visualizer(image[:, :, ::-1], metadata, scale=0.8)
out = viz.draw_instance_predictions(instances)
viz_image = out.get_image()[:, :, ::-1]

plt.imshow(cv2.cvtColor(viz_image, cv2.COLOR_BGR2RGB))
plt.show()
