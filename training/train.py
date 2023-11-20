import datetime
import os
from threading import Thread

import cv2
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.data.datasets import register_coco_instances
from detectron2.engine import DefaultPredictor, DefaultTrainer
from detectron2.evaluation import COCOEvaluator, SemSegEvaluator
from detectron2.utils.visualizer import ColorMode, Visualizer
from helpers import load_dataset
from tensorboard import program

RESUME_SESSION = ""

DATA_SET_NAME = "nhrl_dataset"
DATA_SET_DIR = "/media/storage/training/labeled/nhrl_dataset"
OUTPUT_DIR = "/media/storage/training/models/nhrl"
ANNOTATIONS_FILE_NAME = "_annotations.coco.json"

# TRAIN SET
TRAIN_DATA_SET_NAME = f"{DATA_SET_NAME}-train"
TRAIN_DATA_SET_IMAGES_DIR_PATH = os.path.join(DATA_SET_DIR, "train")
TRAIN_DATA_SET_ANN_FILE_PATH = os.path.join(DATA_SET_DIR, "train", ANNOTATIONS_FILE_NAME)

register_coco_instances(
    name=TRAIN_DATA_SET_NAME,
    metadata={},
    json_file=TRAIN_DATA_SET_ANN_FILE_PATH,
    image_root=TRAIN_DATA_SET_IMAGES_DIR_PATH,
)

# TEST SET
TEST_DATA_SET_NAME = f"{DATA_SET_NAME}-test"
TEST_DATA_SET_IMAGES_DIR_PATH = os.path.join(DATA_SET_DIR, "test")
TEST_DATA_SET_ANN_FILE_PATH = os.path.join(DATA_SET_DIR, "test", ANNOTATIONS_FILE_NAME)

register_coco_instances(
    name=TEST_DATA_SET_NAME,
    metadata={},
    json_file=TEST_DATA_SET_ANN_FILE_PATH,
    image_root=TEST_DATA_SET_IMAGES_DIR_PATH,
)

# VALID SET
VALID_DATA_SET_NAME = f"{DATA_SET_NAME}-valid"
VALID_DATA_SET_IMAGES_DIR_PATH = os.path.join(DATA_SET_DIR, "valid")
VALID_DATA_SET_ANN_FILE_PATH = os.path.join(DATA_SET_DIR, "valid", ANNOTATIONS_FILE_NAME)

register_coco_instances(
    name=VALID_DATA_SET_NAME,
    metadata={},
    json_file=VALID_DATA_SET_ANN_FILE_PATH,
    image_root=VALID_DATA_SET_IMAGES_DIR_PATH,
)

assert (
    len([data_set for data_set in MetadataCatalog.list() if data_set.startswith(DATA_SET_NAME)]) == 3
), "Something went wrong while registering the data sets."

train_dataset = load_dataset(TRAIN_DATA_SET_ANN_FILE_PATH)
print("Found categories:", train_dataset.dataset.categories)

# HYPERPARAMETERS
ARCHITECTURE = "mask_rcnn_R_101_FPN_3x"
CONFIG_FILE_PATH = f"COCO-InstanceSegmentation/{ARCHITECTURE}.yaml"
MAX_ITER = 100000
EVAL_PERIOD = 200
BASE_LR = 0.001
NUM_CLASSES = len(train_dataset.dataset.categories)

if RESUME_SESSION:
    TRAINING_SESSION_NAME = RESUME_SESSION
else:
    TRAINING_SESSION_NAME = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")


# OUTPUT DIR
OUTPUT_PATH = os.path.join(OUTPUT_DIR, DATA_SET_NAME, ARCHITECTURE, TRAINING_SESSION_NAME)
os.makedirs(OUTPUT_PATH, exist_ok=True)

cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file(CONFIG_FILE_PATH))
cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(CONFIG_FILE_PATH)
cfg.DATASETS.TRAIN = (TRAIN_DATA_SET_NAME,)
cfg.DATASETS.TEST = ()
cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 64
cfg.TEST.EVAL_PERIOD = EVAL_PERIOD
cfg.DATALOADER.NUM_WORKERS = 2
cfg.SOLVER.IMS_PER_BATCH = 2
cfg.INPUT.MASK_FORMAT = "bitmask"
cfg.SOLVER.BASE_LR = BASE_LR
cfg.SOLVER.MAX_ITER = MAX_ITER
cfg.MODEL.ROI_HEADS.NUM_CLASSES = NUM_CLASSES
cfg.OUTPUT_DIR = OUTPUT_PATH


class Trainer(DefaultTrainer):
    """
    We use the "DefaultTrainer" which contains pre-defined default logic for
    standard training workflow. They may not work for you, especially if you
    are working on a new research project. In that case you can write your
    own training loop. You can use "tools/plain_train_net.py" as an example.
    """

    @classmethod
    def build_evaluator(cls, cfg, dataset_name, output_folder=None):
        if output_folder is None:
            output_folder = os.path.join(cfg.OUTPUT_DIR, "inference")
        evaluator_list = []
        evaluator_type = MetadataCatalog.get(dataset_name).evaluator_type
        if evaluator_type in ["sem_seg", "coco_panoptic_seg"]:
            evaluator_list.append(
                SemSegEvaluator(
                    dataset_name,
                    distributed=True,
                    output_dir=output_folder,
                )
            )
        if evaluator_type in ["coco", "coco_panoptic_seg"]:
            evaluator_list.append(COCOEvaluator(dataset_name, output_dir=output_folder))
        return evaluator_list


def launch_tensorboard():
    tb = program.TensorBoard()
    tb.configure(argv=[None, "--logdir", OUTPUT_PATH])
    url = tb.launch()
    print(f"Tensorflow listening on {url}")


tb_thread = Thread(target=launch_tensorboard)
tb_thread.start()

try:
    trainer = Trainer(cfg)
    trainer.resume_or_load(resume=True)
    trainer.train()
finally:
    cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7
    predictor = DefaultPredictor(cfg)

    dataset_test = DatasetCatalog.get(TEST_DATA_SET_NAME)
    metadata = MetadataCatalog.get(TEST_DATA_SET_NAME)

    TEST_PATH = os.path.join(OUTPUT_PATH, "test_inference")
    os.makedirs(TEST_PATH, exist_ok=True)

    for annotation in dataset_test:
        image = cv2.imread(annotation["file_name"])
        out_path = os.path.join(TEST_PATH, os.path.basename(annotation["file_name"]))
        outputs = predictor(image)

        visualizer = Visualizer(image[:, :, ::-1], metadata=metadata, scale=0.8, instance_mode=ColorMode.IMAGE)
        out = visualizer.draw_instance_predictions(outputs["instances"].to("cpu"))
        viz_image = out.get_image()[:, :, ::-1]
        cv2.imwrite(out_path, viz_image)
    tb_thread.join()
