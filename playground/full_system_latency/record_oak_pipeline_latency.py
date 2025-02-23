import time
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai
from bw_shared.radio.transmitter.frsky import FrSkyTransmitter

PERCEPTION_DIR = Path(__file__).parent.parent.parent / "perception"
import numpy as np


def main() -> None:
    transmitter = FrSkyTransmitter()
    transmitter.open()
    transmitter.set_telemetry(True)

    # Create pipeline
    pipeline = dai.Pipeline()
    pipeline.setOpenVINOVersion(dai.OpenVINO.VERSION_2021_4)

    # Define source and output
    cam_rgb = pipeline.create(dai.node.ColorCamera)

    fps = 60

    # Properties
    cam_key = dai.CameraBoardSocket.CAM_A
    cam_rgb.setBoardSocket(cam_key)
    cam_rgb.setVideoSize(720, 720)
    cam_rgb.setPreviewSize(720, 720)
    cam_rgb.setInterleaved(False)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(fps)

    nn = pipeline.create(dai.node.NeuralNetwork)
    nn.setBlobPath("models/diff_openvino_2021.4_6shave.blob")

    script = pipeline.create(dai.node.Script)
    pipeline.preview.link(script.inputs["in"])
    script.setScript("""
old = node.io['in'].get()
while True:
    frame = node.io['in'].get()
    node.io['img1'].send(old)
    node.io['img2'].send(frame)
    old = frame
    """)

    script.outputs["img1"].link(nn.inputs["img1"])
    script.outputs["img2"].link(nn.inputs["img2"])

    # Send frame diff to the host
    nn_xout = pipeline.create(dai.node.XLinkOut)
    nn_xout.setStreamName("nn")
    nn.out.link(nn_xout.input)

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

    def get_frame(data: dai.NNData, shape: tuple[int, ...]):
        diff = np.array(data.getFirstLayerFp16()).reshape(shape)
        colorize = cv2.normalize(diff, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        return cv2.applyColorMap(colorize, cv2.COLORMAP_JET)

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        queue_nn = device.getOutputQueue(name="nn", maxSize=1, blocking=False)

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

                image = get_frame(queue_nn.get().getCvFrame(), (720, 720))
                write_row(now, "frame", [frame_num])
                frame_num += 1
                if video is None:
                    height, width = image.shape[:2]
                    video = cv2.VideoWriter(str(video_file_path), cv2.VideoWriter_fourcc(*"XVID"), fps, (width, height))  # type: ignore
                    print(f"video is {width}x{height}")

                video.write(image)

                cv2.imshow("Image", image)
                key = chr(cv2.waitKey(1) & 0xFF)
                if key == "q":
                    break
        finally:
            transmitter.close()
            if video is not None:
                video.release()
            metadata.close()


if __name__ == "__main__":
    main()
