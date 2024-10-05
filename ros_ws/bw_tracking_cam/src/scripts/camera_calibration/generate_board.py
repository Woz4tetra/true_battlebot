import argparse
import os

import cv2
from bw_shared.camera_calibration.board_config import BoardConfig

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("board_config", type=str, help="path to config")
    args = parser.parse_args()

    board_config_path = args.board_config
    image_path = os.path.splitext(board_config_path)[0] + ".png"

    config = BoardConfig.from_file(board_config_path)

    cv2.imwrite(image_path, config.generate_image())
    print(f"Generated board image at {image_path}")
