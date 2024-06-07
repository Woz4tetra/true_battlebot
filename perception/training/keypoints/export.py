import datetime
import json
import os
import shutil

from bw_shared.enums.label import Label
from perception_tools.config.model_metadata import LABEL_COLORS, LabelColor, ModelMetadata
from ultralytics import YOLO


def copy_to_data(model_path: str, model_metadata_path: str) -> None:
    organization = os.environ["ORGANIZATION"]
    project_name = os.environ["PROJECT_NAME"]
    data_dir = f"/opt/{organization}/{project_name}/perception/data"
    destination_dir = os.path.join(data_dir, "models")
    date_str = datetime.datetime.now().strftime("%Y-%m-%d")
    model_base_name = f"yolov8-pose_{date_str}"
    dest_model_path = os.path.join(destination_dir, model_base_name + ".torchscript")
    dest_model_metadata_path = os.path.join(destination_dir, model_base_name + "_metadata.json")

    os.makedirs(destination_dir, exist_ok=True)
    shutil.copy(model_path, dest_model_path)
    shutil.copy(model_metadata_path, dest_model_metadata_path)


path = "/media/storage/training/models/battlebots/yolov8-pose/runs/pose/battlebots_keypoints8/weights/best.pt"
model = YOLO(path)
model.export(format="torchscript")

exported_path = path.replace(".pt", ".torchscript")

default_color = LabelColor(r=0.0, g=0.0, b=0.0, a=0.0)

label_index_pairs = [(index, name) for index, name in model.names.items()]
labels = [Label(name) for _, name in label_index_pairs]
colors = [LABEL_COLORS.get(label, default_color) for label in labels]

metadata = ModelMetadata(labels=labels, colors=colors)

model_metadata_path = os.path.join(os.path.dirname(path), "model_metadata.json")
with open(model_metadata_path, "w") as f:
    json.dump(metadata.to_dict(), f)

copy_to_data(exported_path, model_metadata_path)
