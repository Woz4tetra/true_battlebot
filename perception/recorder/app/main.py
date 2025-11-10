import json
import select
import time
from datetime import datetime
from pathlib import Path

from bw_shared.radio.transmitter.frsky import FrSkyTransmitter
from perception_tools.directories.data_directory import get_data_directory
from recorder_cpp import DEPTH_MODE, RESOLUTION, InitParameters, ZEDCamera


class App:
    def __init__(self, transmitter: FrSkyTransmitter, zed: ZEDCamera, recorder_base_path: Path) -> None:
        self.transmitter = transmitter
        self.zed = zed
        self.recorder_path_svo = recorder_base_path.with_suffix(".svo2")
        self.recorder_path_transmitter = recorder_base_path.with_suffix(".jsonl")
        self.buffer_length = 50
        self.num_packets_written = 0
        self.buffer = []

    def write_buffer_data(self, path: Path, data: list[dict]) -> None:
        with open(path, "a") as f:
            for entry in data:
                f.write(json.dumps(entry) + "\n")
            self.num_packets_written += len(data)
            print(f"Wrote {self.num_packets_written} transmitter packets to {path}")

    def start(self) -> None:
        self.recorder_path_svo.parent.mkdir(parents=True, exist_ok=True)
        self.zed.start_recording(str(self.recorder_path_svo))
        print(f"Starting recording to {self.recorder_path_svo}")

    def update(self) -> None:
        packets = self.transmitter.read()
        now = time.time()
        for packet, error in packets:
            entry = {
                "timestamp": now,
                "packet": packet.to_dict() if packet else None,
                "error": error,
            }
            self.buffer.append(entry)
            if len(self.buffer) >= self.buffer_length:
                self.write_buffer_data(self.recorder_path_transmitter, self.buffer)
                self.buffer = []

    def stop(self) -> None:
        if self.buffer:
            self.write_buffer_data(self.recorder_path_transmitter, self.buffer)
        self.zed.stop_recording()
        print("Recording stopped.")


def main() -> None:
    transmitter = FrSkyTransmitter()
    transmitter.open()
    transmitter.set_telemetry(True)

    init_parameters = InitParameters()
    init_parameters.camera_resolution = RESOLUTION.HD1080
    init_parameters.depth_mode = DEPTH_MODE.NONE
    init_parameters.camera_fps = 30
    zed = ZEDCamera(init_parameters)
    if not zed.open():
        print("Failed to open camera:", zed.get_last_error())
        exit(1)

    recorder_base_path = get_data_directory() / "recordings" / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    app = App(transmitter, zed, recorder_base_path)
    app.start()
    transmitter_fileno = transmitter.fileno()

    try:
        while True:
            rlist, _, _ = select.select([transmitter_fileno], [], [], 0.1)
            if transmitter_fileno in rlist:
                app.update()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
        print("Camera closed.")
