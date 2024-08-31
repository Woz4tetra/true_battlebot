from ultralytics import YOLO

model_path = "yolov8x-pose.pt"
# model_path = "./runs/pose/battlebots_synthetic_keypoints24/weights/best.pt"

# Load the model.
model = YOLO(model_path)

# Training.
results = model.train(
    data="/media/storage/training/labeled/keypoints/2024-08-31/2024-08-31/data.yaml",
    imgsz=960,
    epochs=150,
    batch=6,
    name="battlebots_keypoints",
    device=0,
)
