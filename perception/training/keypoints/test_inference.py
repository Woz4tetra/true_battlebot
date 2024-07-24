import logging
import os
import time

logger = logging.getLoggerClass()

import cv2

logging.setLoggerClass(logging.Logger)
from ultralytics import YOLO

loggers = [logging.getLogger(name) for name in logging.root.manager.loggerDict]
print(loggers)
model = YOLO("./runs/pose/battlebots_keypoints2/weights/best.pt")

test_image_dir = "/media/storage/training/labeled/keypoints/2024-07-20/test/images"

keypoint_names = {
    "mr_stabs_mk1": ["front", "back"],
    "mr_stabs_mk2": ["back", "front"],
    "mrs_buff_mk1": ["front", "back"],
    "mrs_buff_mk2": ["back", "front"],
    "robot": ["front", "back"],
    "referee": ["front", "back"],
}

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
        labels = [result.names[index] for index in ids]
        for keypoint, label in zip(keypoints, labels):
            print(label, keypoint.tolist())
        img_array = result.plot(kpt_line=True, kpt_radius=6)  # plot a BGR array of predictions
        # render keypoint names in image
        for i, (keypoint, label) in enumerate(zip(keypoints, labels)):
            keypoint = keypoint.tolist()
            front_label = keypoint_names[label][0]
            back_label = keypoint_names[label][1]
            cv2.putText(
                img_array,
                front_label,
                (keypoint[0][0], keypoint[0][1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                img_array,
                back_label,
                (keypoint[1][0], keypoint[1][1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        cv2.imshow("image", img_array)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            should_exit = True
            break
    if should_exit:
        break
print("Warmup time:", diffs.pop(0))
print("Average inference time:", sum(diffs) / len(diffs))
