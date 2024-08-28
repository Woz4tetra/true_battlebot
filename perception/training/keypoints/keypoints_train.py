from ultralytics import YOLO

# model_path = "yolov8l-pose.pt"
model_path = "./runs/pose/battlebots_synthetic_keypoints12/weights/best.pt"

# Load the model.
model = YOLO(model_path)

# Training.
results = model.train(
    data="/media/storage/training/labeled/keypoints/2024-08-24_2000/data.yaml",
    imgsz=1280,
    epochs=500,
    batch=4,
    name="battlebots_synthetic_keypoints",
    save_dir="/media/storage/training/models/battlebots/yolov8-pose/2024-08-24_2000/",
    device=0,
)
