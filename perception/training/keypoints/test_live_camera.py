import argparse
import os
import time
from typing import Any

import cv2
import numpy as np
import pyzed.sl as sl
from perception_tools.config.model_metadata import ModelMetadata
from perception_tools.inference.common import load_metadata
from ultralytics import YOLO


def draw_keypoints(result: Any, metadata: ModelMetadata) -> np.ndarray:
    ids = result.boxes.cpu().cls.int().numpy()  # get the class ids
    keypoints = result.keypoints.cpu().xy.int().numpy()  # get the keypoints
    labels = [result.names[index] for index in ids]
    img_array = result.plot(kpt_line=True, kpt_radius=6)  # plot a BGR array of predictions

    # render keypoint names in image
    for keypoint, label in zip(keypoints, labels):
        keypoint = keypoint.tolist()
        front_label = metadata.keypoint_map[label][0]
        back_label = metadata.keypoint_map[label][1]
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
    return np.array(img_array)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "model",
        type=str,
        help="Path to the model (*.pth or *.torchscript)",
    )
    parser.add_argument("-s", "--save-directory", default=".", type=str, help="Directory to save images")
    parser.add_argument(
        "-m",
        "--metadata",
        type=str,
        default="",
        help="Path to the metadata file (*.json)",
    )
    args = parser.parse_args()

    model_path = args.model
    metadata_path = args.metadata
    save_directory = args.save_directory

    print("Loading YOLO model")
    model = YOLO(model_path)

    if not metadata_path:
        metadata_path = os.path.splitext(model_path)[0] + ".json"
    print(f"Loading metadata from {metadata_path}")
    metadata = load_metadata(metadata_path)

    init_params = sl.InitParameters()

    # Create a ZED camera object
    zed = sl.Camera()
    zed.open(init_params)

    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 15
    init_params.coordinate_units = sl.UNIT.METER

    color_image = sl.Mat()

    try:
        while True:
            # Each new frame is added to the SVO file
            zed.grab()

            zed.retrieve_image(color_image, sl.VIEW.LEFT)
            color_image_data = color_image.get_data()[..., 0:3]
            results = model(color_image_data, verbose=False, conf=0.85)[0]
            debug_image = draw_keypoints(results, metadata)

            cv2.imshow("video", debug_image)
            key = chr(cv2.waitKey(1) & 0xFF)
            if key == "q":
                break
            elif key == "p":
                timestamp = time.time()
                path = f"{save_directory}/image_{timestamp}.jpg"
                print(f"Saving photo to {path}")
                cv2.imwrite(path, color_image_data)

    finally:
        zed.close()


if __name__ == "__main__":
    main()
