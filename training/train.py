import os

from training_manager import TrainingManager

manager = TrainingManager()
train_output = manager.get_training_dir("2023-11-20-13-20-28")
manager.train(train_output)
manager.inference(train_output)

model_path = os.path.join(train_output, "model.torchscript")
model_metadata_path = os.path.join(train_output, "model_metadata.json")
manager.export(model_path)
manager.test_export(model_path, model_metadata_path)
manager.copy_to_data(model_path, model_metadata_path)