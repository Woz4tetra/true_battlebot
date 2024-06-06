import os

import cv2
from ultralytics import YOLO

model = YOLO("/media/storage/training/models/battlebots/yolov8-pose/runs/pose/battlebots_keypoints8/weights/best.pt")

test_image_dir = "/media/storage/training/labeled/true-battlebot-keypoints/2024-06-06/test/images"

for filename in os.listdir(test_image_dir):
    if not filename.endswith(".jpg"):
        continue
    image_path = os.path.join(test_image_dir, filename)
    image = cv2.imread(image_path)
    results = model(image)

    for result in results:
        keypoints = result.keypoints.cpu().xy.int().numpy()  # get the keypoints
        img_array = result.plot(kpt_line=True, kpt_radius=6)  # plot a BGR array of predictions
        cv2.imshow("image", img_array)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            break
