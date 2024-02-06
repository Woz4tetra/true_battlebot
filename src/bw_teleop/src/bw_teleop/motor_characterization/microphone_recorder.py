import queue
from datetime import datetime
from threading import Thread
from typing import Union

import numpy  # Make sure NumPy is loaded before it is used in the callback
import rospy
from sounddevice import InputStream, query_devices
from soundfile import SoundFile

assert numpy  # avoid "imported but unused" message (W0611)


class MicrophoneRecorder:
    def __init__(self, device_id: Union[int, str]) -> None:
        self.device_id = device_id
        device_info: dict = query_devices(self.device_id, "input")  # type: ignore
        self.sample_rate = int(device_info["default_samplerate"])
        self.channels = 1
        self.filename = ""
        self.split_queue = queue.Queue()
        self.audio_queue = queue.Queue()
        self.record_thread = Thread(target=self.record_task, daemon=False)
        self.should_record = False

    def start(self) -> str:
        self.split()
        self.record_thread.start()
        self.should_record = True
        return self.filename

    def stop(self) -> None:
        self.should_record = False
        self.record_thread.join()

    def is_recording(self) -> bool:
        return self.should_record

    def split(self) -> str:
        self.filename = datetime.now().strftime("audio_%Y-%m-%d_%H-%M-%S.wav")
        self.split_queue.put(self.filename)
        return self.filename

    def get_filename(self) -> str:
        return self.filename

    def callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            rospy.logerr(f"Error while recording audio: {status}")
        self.audio_queue.put(indata.copy())

    def record_task(self) -> None:
        while self._keep_recording():
            filename = self.split_queue.get()
            self._record_split(filename)

    def _record_split(self, filename: str) -> None:
        with SoundFile(filename, mode="x", samplerate=self.sample_rate, channels=self.channels, subtype="") as file:
            with InputStream(
                samplerate=self.sample_rate, device=self.device_id, channels=self.channels, callback=self.callback
            ):
                while self.split_queue.empty() and self._keep_recording():
                    file.write(self.audio_queue.get())

    def _keep_recording(self) -> bool:
        return self.should_record and not rospy.is_shutdown()
