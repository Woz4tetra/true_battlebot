import logging
import time

logger = logging.getLoggerClass()

import argparse
from pathlib import Path

import cv2
import tqdm
from keypoints_config import load_keypoints_config

logging.setLoggerClass(logging.Logger)  # fix rospy breaking logs
from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description="Test inference of a YOLO keypoints model")
    parser.add_argument(
        "model",
        type=str,
        help="Path to the model (*.pt or *.torchscript)",
    )
    parser.add_argument(
        "config",
        type=str,
        help="Path to the configuration file. ex: ./keypoint_names_v1.toml",
    )
    parser.add_argument(
        "image_path",
        nargs="+",
        type=str,
        help="Path to the image to test",
    )

    args = parser.parse_args()
    model_path = Path(args.model)
    config_path = args.config
    image_paths = [Path(image_path) for image_path in args.image_path]

    output_path = image_paths[0].parent / Path("output")
    print(f"Saving output to {output_path}")
    output_path.mkdir(exist_ok=True)

    print("Loading YOLO model")
    model = YOLO(model_path)

    keypoint_names = load_keypoints_config(config_path)

    diffs = []
    for image_path in tqdm.tqdm(image_paths):
        if not image_path.is_file():
            raise FileNotFoundError(f"Image not found: {image_path}")

        image = cv2.imread(str(image_path))
        t0 = time.perf_counter()
        results = model(image, verbose=False)
        t1 = time.perf_counter()
        delta = t1 - t0
        diffs.append(delta)

        result = results[0]
        ids = result.boxes.cpu().cls.int().numpy()  # get the class ids
        keypoints = result.keypoints.cpu().xy.int().numpy()  # get the keypoints
        labels = [result.names[index] for index in ids]
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

            cv2.imwrite(str(output_path / image_path.name), img_array)
    print("Warmup time:", diffs.pop(0))
    print("Average inference time:", sum(diffs) / len(diffs))


if __name__ == "__main__":
    main()
