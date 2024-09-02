from ultralytics import YOLO

model_path = "yolov8l-pose.pt"

# Load the model.
model = YOLO(model_path)

# Training.
results = model.train(
    data="/media/storage/training/labeled/keypoints/2024-09-01/2024-09-01/data.yaml",
    imgsz=1280,
    epochs=150,
    batch=4,
    name="battlebots_keypoints",
    device=0,
)
