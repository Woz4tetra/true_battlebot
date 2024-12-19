import argparse
import os

import aprilgrid
import cv2
import numpy as np
import tqdm


def detect_tags(detector: aprilgrid.Detector, image: np.ndarray) -> np.ndarray:
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    try:
        results = detector.detect(grey)
    except BaseException as e:
        print(e)
        return image
    debug_image = np.copy(image)
    for result in results:
        corners: np.ndarray = result.corners
        cv2.drawContours(debug_image, corners.astype(int), -1, (0, 255, 0), 10)
    return debug_image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+")
    parser.add_argument("-o", "--output", help="output directory. Defaults to created subdirectory of first image.")
    args = parser.parse_args()

    images = args.images

    if not args.output:
        out_dir = os.path.join(os.path.dirname(images[0]), "out")

    os.makedirs(out_dir, exist_ok=True)

    detector = aprilgrid.Detector("t36h11b1")

    for path in tqdm.tqdm(images):
        path_lower = path.lower()
        if not (path_lower.endswith(".jpg") or path_lower.endswith(".png")):
            continue
        out_path = os.path.join(out_dir, os.path.basename(path))
        image = cv2.imread(path)
        out_image = detect_tags(detector, image)
        cv2.imwrite(out_path, out_image)


if __name__ == "__main__":
    main()
