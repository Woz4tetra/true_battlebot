import os
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
    def __init__(self, directory: str, device_id: Union[int, str]) -> None:
        self.directory = directory
        self.device_id = device_id
        device_info: dict = query_devices(self.device_id, "input")  # type: ignore
        self.sample_rate = int(device_info["default_samplerate"])
        self.channels = 1
        self.split_queue = queue.Queue()
        self.audio_queue = queue.Queue()
        self.record_thread = Thread(target=self.record_task, daemon=False)
        self.should_record = False

    def start(self) -> None:
        self.record_thread.start()
        self.should_record = True

    def stop(self) -> None:
        self.should_record = False
        self.record_thread.join()

    def is_recording(self) -> bool:
        return self.should_record

    def split(self) -> str:
        filename = datetime.now().strftime("audio_%Y-%m-%d_%H-%M-%S.wav")
        path = os.path.join(self.directory, filename)
        self.split_queue.put(path)
        return path

    def callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            rospy.logerr(f"Error while recording audio: {status}")
        self.audio_queue.put(indata.copy())

    def record_task(self) -> None:
        while self._keep_recording():
            path = self.split_queue.get()
            self._record_split(path)

    def _record_split(self, path: str) -> None:
        with SoundFile(path, mode="x", samplerate=self.sample_rate, channels=self.channels, subtype="") as file:
            with InputStream(
                samplerate=self.sample_rate, device=self.device_id, channels=self.channels, callback=self.callback
            ):
                while self.split_queue.empty() and self._keep_recording():
                    file.write(self.audio_queue.get())

    def _keep_recording(self) -> bool:
        return self.should_record and not rospy.is_shutdown()
