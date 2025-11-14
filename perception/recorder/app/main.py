import json
import select
import time
from datetime import datetime
from pathlib import Path

from bw_shared.radio.transmitter.frsky import FrSkyTransmitter
from perception_tools.directories.media_directory import get_media_directory
from recorder_cpp import DEPTH_MODE, RESOLUTION, InitParameters, ZEDCamera


class App:
    def __init__(self, transmitter: FrSkyTransmitter, zed: ZEDCamera, svo_path: Path, telemetry_path: Path) -> None:
        self.transmitter = transmitter
        self.zed = zed
        self.svo_path = svo_path
        self.telemetry_path = telemetry_path
        self.buffer_length = 50
        self.num_packets_written = 0
        self.record_start_time: float | None = None
        self.record_cooldown = 2.0  # seconds
        self.is_recording = False
        self.buffer = []

    def write_buffer_data(self, path: Path, data: list[dict]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "a") as f:
            for entry in data:
                f.write(json.dumps(entry) + "\n")
            self.num_packets_written += len(data)
            print(f"Wrote {self.num_packets_written} transmitter packets to {path}")

    def start(self) -> None:
        self.svo_path.parent.mkdir(parents=True, exist_ok=True)

    def _start_recording(self) -> None:
        if self.record_start_time is not None:
            elapsed = time.monotonic() - self.record_start_time
            if elapsed < self.record_cooldown:
                return
        if self.is_recording:
            return
        self.record_start_time = time.monotonic()
        self.zed.start_recording(str(self.svo_path))
        self.is_recording = True
        self.write_buffer_data(
            self.telemetry_path,
            {
                "timestamp": time.time(),
                "event": "started_recording",
                "svo_path": str(self.svo_path),
            },
        )
        print(f"Starting recording to {self.svo_path}")

    def _stop_recording(self) -> None:
        if not self.is_recording:
            return
        self.zed.stop_recording()
        self.write_buffer_data(
            self.telemetry_path,
            {
                "timestamp": time.time(),
                "event": "stopped_recording",
                "svo_path": str(self.svo_path),
            },
        )
        self.is_recording = False
        print("Stopped recording.")

    def update(self) -> None:
        transmitter_data = self.transmitter.read()
        now = time.time()
        for packet, error in transmitter_data.crsf_packets:
            entry = {
                "timestamp": now,
                "event": "received_packet",
                "packet": packet.to_dict() if packet else None,
                "error": error,
            }
            self.buffer.append(entry)
            if len(self.buffer) >= self.buffer_length:
                self.write_buffer_data(self.telemetry_path, self.buffer)
                self.buffer = []
        for channel_update in transmitter_data.channel_updates:
            if channel_update[8] >= 0:
                self._start_recording()
            else:
                self._stop_recording()
            entry = {
                "timestamp": now,
                "event": "channel_update",
                "channels": channel_update,
            }
            self.buffer.append(entry)
            if len(self.buffer) >= self.buffer_length:
                self.write_buffer_data(self.telemetry_path, self.buffer)
                self.buffer = []

    def stop(self) -> None:
        if self.buffer:
            self.write_buffer_data(self.telemetry_path, self.buffer)
        self._stop_recording()
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

    base_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    svo_base_path = get_media_directory() / "svo" / (base_name + ".svo2")
    telemetry_base_path = get_media_directory() / "telemetry" / (base_name + "_telemetry.jsonl")

    app = App(transmitter, zed, svo_base_path, telemetry_base_path)
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
