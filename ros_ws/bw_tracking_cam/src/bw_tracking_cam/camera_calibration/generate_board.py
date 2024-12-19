#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
import argparse
import os

import argcomplete
import cv2
from bw_shared.camera_calibration import aruco_board
from bw_shared.camera_calibration.board_config import BoardConfig, BoardType
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
os.chdir(SCRIPT_DIR)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("board_config", type=str, help="path to config")
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    board_config_path = args.board_config
    image_path = os.path.abspath(os.path.splitext(board_config_path)[0] + ".png")
    pdf_path = os.path.abspath(os.path.splitext(board_config_path)[0] + ".pdf")

    config = BoardConfig.from_file(board_config_path)

    if config.board_type in (BoardType.ARUCO_GRID, BoardType.CHARUCO):
        board_image = aruco_board.generate_image(config, aruco_board.make_aruco_board(config))
    else:
        raise NotImplementedError(f"Board type '{config.board_type}' not supported for image generation.")
    pil_image = Image.fromarray(board_image)

    cv2.imwrite(image_path, board_image)
    print(f"Generated board image ({board_image.shape[1]}x{board_image.shape[0]}) at {image_path}")

    dpi = config.px_per_meter * 0.0254

    pil_image.save(pdf_path, "PDF", resolution=dpi, save_all=True)
    print(f"Generated board pdf at {pdf_path}. (DPI: {dpi})")
