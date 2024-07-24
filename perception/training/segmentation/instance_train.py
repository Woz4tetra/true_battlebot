import datetime
import os

from perception_tools.training.training_manager import TrainingManager

training_dir = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
# training_dir = "2023-11-20-13-20-28"

dataset_name = "battlebots"
dataset_dir = "/media/storage/training/labeled/true-battlebot-segmentation/2024-02-24/battlebots"
output_dir = "/media/storage/training/models"
config_path = "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"

manager = TrainingManager(dataset_name, dataset_dir, output_dir, config_path)
train_output = manager.get_training_dir(training_dir)
manager.train(train_output)
manager.inference(train_output)

model_path = os.path.join(train_output, "model.torchscript")
model_metadata_path = os.path.join(train_output, "model_metadata.json")
manager.export(model_path)
manager.test_export(model_path, model_metadata_path)
manager.copy_to_data(model_path, model_metadata_path)
