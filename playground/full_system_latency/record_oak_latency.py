import time
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai
from bw_shared.radio.transmitter.frsky import FrSkyTransmitter

PERCEPTION_DIR = Path(__file__).parent.parent.parent / "perception"


def main() -> None:
    transmitter = FrSkyTransmitter()
    transmitter.open()
    transmitter.set_telemetry(True)

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define source and output
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    xout_video = pipeline.create(dai.node.XLinkOut)

    xout_video.setStreamName("video")

    fps = 60

    # Properties
    cam_key = dai.CameraBoardSocket.CAM_A
    cam_rgb.setBoardSocket(cam_key)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(fps)

    xout_video.input.setBlocking(False)
    xout_video.input.setQueueSize(1)

    # Linking
    cam_rgb.video.link(xout_video.input)

    commands = [
        (0.75, (0.0, 0.0)),
        (0.75, (0.25, 0.0)),
    ] * 20
    commands.append((0.75, (0.0, 0.0)))

    file_stem = f"oak_latency_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    video_path = Path("/data/videos")
    video_path.mkdir(exist_ok=True)
    video_file_path = video_path / (file_stem + ".avi")
    meta_file_path = video_path / (file_stem + ".csv")

    metadata = open(meta_file_path, "w")

    def write_row(time: float, key: str, data: list[float]) -> None:
        metadata.write(f"{time},{key},{','.join(map(str, data))}\n")

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        camera = device.getOutputQueue(name="video", maxSize=1, blocking=False)

        next_command_time = time.perf_counter()
        command = (0.0, 0.0)
        frame_num = 0
        video = None

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

                image = camera.get().getCvFrame()
                write_row(now, "frame", [frame_num])
                frame_num += 1
                if video is None:
                    height, width = image.shape[:2]
                    video = cv2.VideoWriter(str(video_file_path), cv2.VideoWriter_fourcc(*"XVID"), fps, (width, height))  # type: ignore
                    print(f"video is {width}x{height}")

                video.write(image)

                # cv2.imshow("Image", image)
                # key = chr(cv2.waitKey(1) & 0xFF)
                # if key == "q":
                #     break
        finally:
            transmitter.close()
            if video is not None:
                video.release()
            metadata.close()


if __name__ == "__main__":
    main()
