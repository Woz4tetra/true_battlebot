import argparse
from pathlib import Path

import cv2
import numpy as np
import rospy
import toml
from app.camera.camera_interface import CameraMode
from app.camera.zed.video_settings import Zed2iVideoSettings
from app.camera.zed_camera import ZedCamera, ZedCameraConfig
from app.config.config_loader import load_config

SCRIPT_DIR = Path(__file__).parent


def main() -> None:
    rospy.init_node("apply_camera_settings", anonymous=True, disable_signals=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", type=str)
    parser.add_argument("config_dir", nargs="?", type=str, default=str(SCRIPT_DIR.parent / "configs"))
    parser.add_argument("-m", "--mode", type=str, default=CameraMode.ROBOT_FINDER.value)
    args = parser.parse_args()

    config_dir = args.config_dir
    robot = args.robot
    mode = CameraMode(args.mode)
    config = load_config(str(config_dir), robot)
    if not isinstance(config.camera, ZedCameraConfig):
        raise Exception(f"Expected ZedCameraConfig. Got {type(config.camera)}")

    camera = ZedCamera(config.camera, config.camera_topic, None, None)
    if not camera.open(mode):
        raise Exception("Failed to open ZED camera")

    print("Applied camera settings:")
    settings = Zed2iVideoSettings.from_camera(camera.camera)
    print(toml.dumps(settings.to_dict()))

    while True:
        camera_data = camera.poll()
        if camera_data is None:
            image = np.zeros((300, 300, 3), dtype=np.uint8)
        else:
            image = camera_data.color_image.data

        cv2.imshow("Image", image)
        key = chr(cv2.waitKey(1) & 0xFF)
        if key == "q":
            break


if __name__ == "__main__":
    main()
