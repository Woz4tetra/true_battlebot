from ultralytics import YOLO

# Load the model.
model = YOLO("yolov8m-pose.pt")

# Training.
results = model.train(
    data="/media/storage/training/labeled/true-battlebot-keypoints/2024-06-06/data.yaml",
    imgsz=1280,
    epochs=50,
    batch=4,
    name="battlebots_keypoints",
    save_dir="/media/storage/true-battlebot-media/training/models/battlebots/yolov8-pose",
)
