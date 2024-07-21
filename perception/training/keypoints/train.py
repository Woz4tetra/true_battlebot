from ultralytics import YOLO

# Load the model.
model = YOLO("yolov8l-pose.pt")

# Training.
results = model.train(
    data="/media/storage/training/labeled/keypoints/2024-07-20/data.yaml",
    imgsz=1280,
    epochs=100,
    batch=4,
    name="battlebots_keypoints",
    save_dir="/media/storage/true-battlebot-media/training/models/battlebots/yolov8-pose/2024-07-20",
)
