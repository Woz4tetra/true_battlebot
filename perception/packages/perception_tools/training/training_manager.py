import datetime
import json
import os
import shutil
import time
from threading import Thread
from typing import Dict, List

import cv2
import numpy as np
import torch.jit
import torchvision
from bw_shared.enums.label import ModelLabel
from detectron2 import model_zoo
from detectron2.checkpoint import DetectionCheckpointer
from detectron2.config import CfgNode, get_cfg
from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.data.datasets import register_coco_instances
from detectron2.engine import DefaultPredictor
from detectron2.export import dump_torchscript_IR, scripting_with_instances
from detectron2.layers import paste_masks_in_image
from detectron2.modeling import GeneralizedRCNN, build_model
from detectron2.structures import Boxes
from detectron2.utils.env import TORCH_VERSION
from detectron2.utils.visualizer import ColorMode, GenericMask, Visualizer
from tensorboard import program
from torch import ScriptModule, Tensor, nn
from torch.jit._serialization import save as save_model

from perception_tools.config.model_metadata import LABEL_COLORS, LabelColor, ModelMetadata
from perception_tools.training.helpers import load_dataset
from perception_tools.training.nhrl_trainer import NhrlTrainer


class TrainingManager:
    def __init__(self, dataset_name: str, dataset_dir: str, output_dir: str, zoo_config_path: str) -> None:
        self.dataset_name = dataset_name
        self.dataset_dir = dataset_dir
        self.output_dir = output_dir
        self.annotations_file_name = "_annotations.coco.json"
        self.expected_number_of_categories = 3

        self.default_color = LabelColor(r=0.0, g=0.0, b=0.0, a=0.0)

        self.architecture = os.path.splitext(os.path.basename(zoo_config_path))[0]
        self.config_file_path = zoo_config_path

        # TRAIN SET
        self.train_data_set_name = f"{self.dataset_name}-train"
        self.train_data_set_images_dir_path = os.path.join(self.dataset_dir, "train")
        self.train_data_set_ann_file_path = os.path.join(self.dataset_dir, "train", self.annotations_file_name)

        register_coco_instances(
            name=self.train_data_set_name,
            metadata={},
            json_file=self.train_data_set_ann_file_path,
            image_root=self.train_data_set_images_dir_path,
        )

        # TEST SET
        self.test_dataset_name = f"{self.dataset_name}-test"
        self.test_data_set_images_dir_path = os.path.join(self.dataset_dir, "test")
        self.test_data_set_ann_file_path = os.path.join(self.dataset_dir, "test", self.annotations_file_name)

        register_coco_instances(
            name=self.test_dataset_name,
            metadata={},
            json_file=self.test_data_set_ann_file_path,
            image_root=self.test_data_set_images_dir_path,
        )

        # VALID SET
        self.valid_data_set_name = f"{self.dataset_name}-valid"
        self.valid_data_set_images_dir_path = os.path.join(self.dataset_dir, "val")
        self.valid_data_set_ann_file_path = os.path.join(self.dataset_dir, "val", self.annotations_file_name)

        register_coco_instances(
            name=self.valid_data_set_name,
            metadata={},
            json_file=self.valid_data_set_ann_file_path,
            image_root=self.valid_data_set_images_dir_path,
        )

        assert (
            len([data_set for data_set in MetadataCatalog.list() if data_set.startswith(self.dataset_name)])
            == self.expected_number_of_categories
        ), "Something went wrong while registering the data sets."

        self.train_dataset = load_dataset(self.train_data_set_ann_file_path)
        print("Found categories:", self.train_dataset.dataset.categories)

        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file(self.config_file_path))
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(self.config_file_path)
        self.cfg.DATASETS.TRAIN = (self.train_data_set_name,)
        self.cfg.DATASETS.TEST = (self.valid_data_set_name,)
        self.cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 64
        self.cfg.TEST.EVAL_PERIOD = 1000
        self.cfg.DATALOADER.NUM_WORKERS = 2
        self.cfg.SOLVER.IMS_PER_BATCH = 2
        self.cfg.INPUT.MASK_FORMAT = "bitmask"
        self.cfg.SOLVER.CHECKPOINT_PERIOD = 2000
        self.cfg.SOLVER.BASE_LR = 0.001
        self.cfg.SOLVER.MAX_ITER = 60000
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(self.train_dataset.dataset.categories)

    def train(self, output_path: str):
        self.cfg.OUTPUT_DIR = output_path
        os.makedirs(output_path, exist_ok=True)
        tb_thread = Thread(target=self.launch_tensorboard, args=(output_path,))
        tb_thread.start()

        trainer = NhrlTrainer(self.cfg)
        trainer.resume_or_load(resume=True)
        trainer.train()

        tb_thread.join()

    def export(self, model_path: str, checkpoint_filename: str = "model_final.pth", ir_dump: bool = False) -> None:
        output_dir = os.path.dirname(model_path)
        model_name = os.path.basename(os.path.splitext(model_path)[0])
        model_metadata_path = os.path.join(output_dir, model_name + "_metadata.json")
        checkpoint_path = os.path.join(output_dir, checkpoint_filename)

        self.cfg.freeze()
        torch_model = build_model(self.cfg)
        DetectionCheckpointer(torch_model).load(checkpoint_path)
        torch_model.eval()
        export_scripting(model_path, torch_model=torch_model, cfg=self.cfg, ir_dump=ir_dump)

        label_index_pairs = [(info.id, info.name) for info in self.train_dataset.dataset.categories]
        label_index_pairs.sort(key=lambda x: x[0])
        labels = [ModelLabel(name) for _, name in label_index_pairs]
        colors = [LABEL_COLORS.get(label, self.default_color) for label in labels]

        metadata = ModelMetadata(labels=labels, colors=colors)
        with open(model_metadata_path, "w") as f:
            json.dump(metadata.to_dict(), f)
        print("Exported model to", model_path)
        print("Exported metadata to", model_metadata_path)

    def test_export(self, model_path: str, metadata_path: str, threshold: float = 0.8) -> None:
        with open(metadata_path, "r") as f:
            metadata = ModelMetadata.from_dict(json.load(f))
        dataset_test = DatasetCatalog.get(self.test_dataset_name)
        device = torch.device("cuda")
        model = torch.jit.load(model_path).to(device)

        model_dir = os.path.dirname(model_path)
        export_dir = os.path.join(model_dir, "test_export")
        os.makedirs(export_dir, exist_ok=True)

        for annotation in dataset_test:
            in_image_path = annotation["file_name"]
            print("Loading", in_image_path)
            image = cv2.imread(in_image_path)
            drawn_image = self.script_inference(model, metadata, image, device, threshold)
            out_image_path = os.path.join(export_dir, os.path.basename(in_image_path))
            cv2.imwrite(out_image_path, drawn_image)
            print("Wrote", out_image_path)

    def script_inference(
        self,
        model: ScriptModule,
        metadata: ModelMetadata,
        image: np.ndarray,
        device,
        threshold: float,
        nms_threshold: float = 0.8,
        mask_conversion_threshold: float = 0.5,
    ) -> np.ndarray:
        h, w = image.shape[:2]
        debug_image = np.copy(image)
        with torch.jit.optimized_execution(True):  # type: ignore
            with torch.no_grad():
                # Convert to channels first, convert to float datatype. Convert (H, W, C) to (C, H, W)
                image_tensor = torch.from_numpy(image).to(device).permute(2, 0, 1).float()
                # outputs = model([{"image": image_tensor}, {"image": other}])  # batch images here if necessary
                print("Running model...")
                t0 = time.perf_counter()
                outputs = model([{"image": image_tensor}])  # type: ignore
                t1 = time.perf_counter()
                print(f"Got {len(outputs)} outputs. Took {t1 - t0:0.2f}s")
                output = outputs[0]
                # Some optional postprocessing, you can change the threshold iou overlap as needed
                to_keep = torchvision.ops.nms(output["pred_boxes"], output["scores"], nms_threshold)
                output["pred_boxes"] = output["pred_boxes"][to_keep].cpu()
                output["pred_classes"] = output["pred_classes"][to_keep].cpu()
                output["pred_masks"] = output["pred_masks"][to_keep].cpu()

                stretched_masks = paste_masks_in_image(
                    output["pred_masks"][:, 0, :, :],
                    output["pred_boxes"],
                    (h, w),
                    threshold=mask_conversion_threshold,
                )
                masks = [GenericMask(x, h, w) for x in np.asarray(stretched_masks)]
                class_indices = [label.item() for label in output["pred_classes"]]

                # Draw you box predictions:
                for mask, bbox, class_idx, score in zip(masks, output["pred_boxes"], class_indices, output["scores"]):
                    if score < threshold:
                        continue
                    bbox = list(map(int, bbox))
                    x1, y1, x2, y2 = bbox
                    class_name = metadata.labels[class_idx]
                    class_color = metadata.colors[class_idx]
                    cv_color = class_color.to_cv_color()
                    cv2.rectangle(debug_image, (x1, y1), (x2, y2), cv_color, 1)
                    cv2.putText(
                        debug_image,
                        f"{class_name} {score * 100:0.1f}",
                        (x1, y1),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        cv_color,
                    )

                    class_color = metadata.colors[class_idx]
                    cv_color = class_color.to_cv_color()
                    for segment in mask.polygons:
                        contour = np.array(segment.reshape(-1, 2), dtype=np.int32)
                        cv2.drawContours(debug_image, [contour], -1, cv_color, 1)
        # debug_image = cv2.addWeighted(all_masks, 0.5, debug_image, 0.5, 0)
        return debug_image

    def inference(self, training_dir: str) -> None:
        self.cfg.OUTPUT_DIR = training_dir
        self.cfg.MODEL.WEIGHTS = os.path.join(self.cfg.OUTPUT_DIR, "model_final.pth")
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7
        predictor = DefaultPredictor(self.cfg)

        dataset_test = DatasetCatalog.get(self.test_dataset_name)
        metadata = MetadataCatalog.get(self.test_dataset_name)

        test_path = os.path.join(training_dir, "test_inference")
        os.makedirs(test_path, exist_ok=True)

        for annotation in dataset_test:
            image = cv2.imread(annotation["file_name"])
            out_path = os.path.join(test_path, os.path.basename(annotation["file_name"]))
            outputs = predictor(image)

            visualizer = Visualizer(image[:, :, ::-1], metadata=metadata, scale=0.8, instance_mode=ColorMode.IMAGE)
            out = visualizer.draw_instance_predictions(outputs["instances"].to("cpu"))
            viz_image = out.get_image()[:, :, ::-1]
            cv2.imwrite(out_path, viz_image)
            print("Wrote", out_path)

    def launch_tensorboard(self, output_path: str):
        tb = program.TensorBoard()
        tb.configure(argv=[None, "--logdir", output_path])
        url = tb.launch()
        print(f"Tensorflow listening on {url}")

    def get_training_dir(self, resume_session: str = "") -> str:
        if resume_session:
            training_session_name = resume_session
        else:
            training_session_name = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

        output_path = os.path.join(self.output_dir, self.dataset_name, self.architecture, training_session_name)
        return output_path

    def copy_to_data(self, model_path: str, model_metadata_path: str) -> None:
        organization = os.environ["ORGANIZATION"]
        project_name = os.environ["PROJECT_NAME"]
        data_dir = f"/opt/{organization}/{project_name}/perception/data"
        destination_dir = os.path.join(data_dir, "models")
        date_str = datetime.datetime.now().strftime("%Y-%m-%d")
        model_base_name = f"{self.dataset_name}_{self.architecture}_{date_str}"
        dest_model_path = os.path.join(destination_dir, model_base_name + ".torchscript")
        dest_model_metadata_path = os.path.join(destination_dir, model_base_name + "_metadata.json")

        os.makedirs(destination_dir, exist_ok=True)
        shutil.copy(model_path, dest_model_path)
        shutil.copy(model_metadata_path, dest_model_metadata_path)


def export_scripting(model_path: str, torch_model: GeneralizedRCNN, cfg: CfgNode, ir_dump: bool = False) -> None:
    """
    Function for exporting a Detectron2 model to a .ts file
    for deployment
    """
    # exporting only works for torch >= 1.8
    assert TORCH_VERSION >= (1, 8)
    fields = {
        "proposal_boxes": Boxes,
        "objectness_logits": Tensor,
        "pred_boxes": Boxes,
        "scores": Tensor,
        "pred_classes": Tensor,
        "pred_masks": Tensor,
        "pred_keypoints": Tensor,
        "pred_keypoint_heatmaps": Tensor,
    }

    class ScriptableAdapter(nn.Module):
        # Use this adapter to workaround https://github.com/pytorch/pytorch/issues/46944
        # by not returning instances but dicts. Otherwise the exported model is not deployable
        def __init__(self) -> None:
            super().__init__()
            self.model = torch_model
            self.eval()

        def forward(self, inputs: List[Dict[str, Tensor]]) -> List[Dict[str, Tensor]]:
            instances = self.model.inference(inputs, do_postprocess=False)
            return [i.get_fields() for i in instances]

    # convert model to scripted/jitted version
    ts_model = scripting_with_instances(ScriptableAdapter(), fields)

    # save model to output directory
    # Get model name from config output directory. Add timestamp
    # TODO make seperate field for save name after LazyConfig switch
    with open(model_path, "wb") as f:
        save_model(ts_model, f)
    # TODO remove this or have it save files using model_name
    if ir_dump:
        dump_torchscript_IR(ts_model, str(os.path.dirname(model_path)))
