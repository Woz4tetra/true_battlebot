from ultralytics import YOLO

# Load the model.
model = YOLO("yolov8l-pose.pt")

# Training.
results = model.train(
    data="/media/storage/training/labeled/keypoints/2024-08-24/data.yaml",
    imgsz=1280,
    epochs=500,
    batch=4,
    name="battlebots_synthetic_keypoints",
    save_dir="/media/storage/training/models/battlebots/yolov8-pose/2024-08-24/",
    device=0,
)
