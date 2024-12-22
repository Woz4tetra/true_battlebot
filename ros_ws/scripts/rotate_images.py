#!/usr/bin/env python
import argparse
import os

import cv2
import numpy as np
import tqdm


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("images", type=str, nargs="+", help="path to input image file(s)")
    parser.add_argument("-r", "--rotate", type=int, default=0, help="number of 90 degree rotations")
    args = parser.parse_args()

    num_rotations = args.rotate
    if num_rotations == 0:
        print("Number of rotation is set to zero. Nothing to do.")
        return

    for image_path in tqdm.tqdm(args.images):
        if not os.path.isfile(image_path):
            print(f"Image file not found: {image_path}")
            continue
        image = cv2.imread(image_path)
        image = np.rot90(image, num_rotations)
        cv2.imwrite(image_path, image)
        print(f"Written to {image_path}")


if __name__ == "__main__":
    main()
