import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import toml
from app.camera.camera_interface import CameraMode
from app.camera.zed.video_settings import Zed2iVideoSettings
from app.camera.zed_camera import ZedCamera, ZedCameraConfig
from app.config.config_loader import load_config
from bw_shared.radio.transmitter.frsky import FrSkyTransmitter

PERCEPTION_DIR = Path(__file__).parent.parent.parent / "perception"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", type=str)
    parser.add_argument("config_dir", nargs="?", type=str, default=str(PERCEPTION_DIR / "configs"))
    parser.add_argument("-m", "--mode", type=str, default=CameraMode.ROBOT_FINDER.value)
    args = parser.parse_args()

    transmitter = FrSkyTransmitter()
    transmitter.open()
    transmitter.set_telemetry(True)

    config_dir = Path(args.config_dir)
    robot = args.robot
    mode = CameraMode(args.mode)
    config = load_config(config_dir, robot)
    if not isinstance(config.camera, ZedCameraConfig):
        raise Exception(f"Expected ZedCameraConfig. Got {type(config.camera)}")

    camera = ZedCamera(config.camera, config.camera_topic, None, None, None, None, None)
    if not camera.open():
        raise Exception("Failed to open ZED camera")
    if not camera.switch_mode(mode):
        raise Exception("Failed to open ZED camera")

    print("Applied camera settings:")
    settings = Zed2iVideoSettings.from_camera(camera.camera)
    print(toml.dumps(settings.to_dict()))

    commands = [
        (0.75, (0.0, 0.0)),
        (0.75, (0.25, 0.0)),
    ] * 20
    commands.append((0.75, (0.0, 0.0)))

    fps = config.camera.fps
    width, height = config.camera.resolution.to_dims()

    file_stem = f"zed_latency_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    video_path = Path("/data/videos")
    video_path.mkdir(exist_ok=True)
    video_file_path = video_path / (file_stem + ".avi")
    meta_file_path = video_path / (file_stem + ".csv")

    video = cv2.VideoWriter(str(video_file_path), cv2.VideoWriter_fourcc(*"XVID"), fps, (width, height))  # type: ignore
    metadata = open(meta_file_path, "w")

    next_command_time = time.perf_counter()
    command = (0.0, 0.0)
    frame_num = 0

    def write_row(time: float, key: str, data: list[float]) -> None:
        metadata.write(f"{time},{key},{','.join(map(str, data))}\n")

    try:
        while True:
            now = time.perf_counter()
            if now >= next_command_time:
                if len(commands) == 0:
                    break
                duration, command = commands.pop(0)
                next_command_time = now + duration
                transmitter.set_command(*command)
                write_row(now, "command", list(command))
            transmitter.write()

            camera_data = camera.poll()
            if camera_data is None:
                image = np.zeros((300, 300, 3), dtype=np.uint8)
            else:
                image = camera_data.color_image.data
                if image.shape[:2] != (height, width):
                    raise Exception(f"Invalid image shape: {image.shape}")
                write_row(now, "frame", [frame_num])
                frame_num += 1
                video.write(image)

            # cv2.imshow("Image", image)
            # key = chr(cv2.waitKey(1) & 0xFF)
            # if key == "q":
            #     break
    finally:
        camera.close()
        transmitter.close()
        video.release()
        metadata.close()


if __name__ == "__main__":
    main()
