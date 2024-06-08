import logging
import os
import time

logger = logging.getLoggerClass()

import cv2

logging.setLoggerClass(logging.Logger)
from bw_shared.enums.label import Label
from ultralytics import YOLO

loggers = [logging.getLogger(name) for name in logging.root.manager.loggerDict]
print(loggers)
model = YOLO("/media/storage/training/models/battlebots/yolov8-pose/runs/pose/battlebots_keypoints8/weights/best.pt")

test_image_dir = "/media/storage/training/labeled/true-battlebot-keypoints/2024-06-06/test/images"

diffs = []
for filename in os.listdir(test_image_dir):
    if not filename.endswith(".jpg"):
        continue
    image_path = os.path.join(test_image_dir, filename)
    image = cv2.imread(image_path)
    t0 = time.perf_counter()
    results = model(image)
    t1 = time.perf_counter()
    delta = t1 - t0
    diffs.append(delta)

    should_exit = False
    for result in results:
        ids = result.boxes.cpu().cls.int().numpy()  # get the class ids
        keypoints = result.keypoints.cpu().xy.int().numpy()  # get the keypoints
        labels = [Label(result.names[index]) for index in ids]
        for keypoint, label in zip(keypoints, labels):
            print(label, keypoint.tolist())
        img_array = result.plot(kpt_line=True, kpt_radius=6)  # plot a BGR array of predictions
        cv2.imshow("image", img_array)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            should_exit = True
            break
    if should_exit:
        break
print("Warmup time:", diffs.pop(0))
print("Average inference time:", sum(diffs) / len(diffs))
