import cv2
from board_config import BoardConfig

if __name__ == "__main__":
    config = BoardConfig()

    cv2.imwrite("board.png", config.generate_image())
