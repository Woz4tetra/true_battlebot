from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass

import numpy as np
from bw_interfaces.msg import MotorCharacterizationSample as MotorCharacterizationSampleMsg
from bw_shared.messages.header import Header
from matplotlib import pyplot as plt
from rosbag import Bag
from scipy.io import wavfile
from scipy.signal import find_peaks

AMPLITUDE_CAP = 2000
MIN_FREQUENCY = 0.1
MAX_FREQUENCY = 100.0
SHOW_FFT = True


@dataclass
class MotorCharacterizationSample:
    header: Header
    channel: int
    velocity: float
    filename: str

    @classmethod
    def from_msg(cls, msg: MotorCharacterizationSampleMsg) -> MotorCharacterizationSample:
        return cls(
            header=Header.from_msg(msg.header),
            channel=msg.channel,
            velocity=msg.velocity,
            filename=msg.filename,
        )


def compute_frequency(path: str) -> float:
    sample_rate, data = wavfile.read(path)

    duration = len(data) / sample_rate
    times = np.linspace(0, duration, len(data))

    time_cutoff = 1.0
    time_clip = times > time_cutoff
    times = times[time_clip]
    data = data[time_clip]

    max_amplitude = np.max(np.abs(data))
    print("max amplitude:", max_amplitude)
    if max_amplitude < AMPLITUDE_CAP:
        return 0.0
    normalized = data / max_amplitude

    fft_data = np.fft.fft(normalized)
    frequencies = np.fft.fftfreq(len(fft_data)) * sample_rate

    fft_clip = (frequencies >= MIN_FREQUENCY) & (frequencies <= MAX_FREQUENCY)

    fft_data = np.abs(fft_data)

    frequencies = frequencies[fft_clip]
    fft_data = fft_data[fft_clip]

    indices, _ = find_peaks(fft_data, height=0.2, distance=5, prominence=0.2)
    index = indices[np.argmax(fft_data[indices])]
    freq = frequencies[index]
    freq_in_hertz = abs(freq)

    if SHOW_FFT:
        plt.figure(2)
        waveform = plt.subplot(2, 1, 1)
        fft_graph = plt.subplot(2, 1, 2)
        waveform.plot(times, normalized)
        fft_graph.plot(frequencies, fft_data)
        fft_graph.plot(frequencies[index], fft_data[index], "x")
        print(max_amplitude, freq_in_hertz)
        plt.show()

    return freq_in_hertz


def main() -> None:
    parser = argparse.ArgumentParser(description="analyze_experiments")
    parser.add_argument("bags", nargs="+", type=str, help="Bag files")
    args = parser.parse_args()

    samples: list[MotorCharacterizationSample] = []
    bags = args.bags
    for bag in bags:
        if not bag.endswith(".bag"):
            continue
        with Bag(bag, "r") as bag:
            for topic, msg, t in bag.read_messages():
                if topic == "microphone_sample":
                    if not msg.valid:
                        continue
                    samples.append(MotorCharacterizationSample.from_msg(msg))
    print(f"Found {len(samples)} samples")
    data: dict[int, dict[str, list[float]]] = {}
    for sample in samples:
        if sample.channel not in data:
            data[sample.channel] = {
                "velocities": [],
                "frequencies": [],
            }
        frequency = compute_frequency(sample.filename)
        data[sample.channel]["velocities"].append(sample.velocity)
        if sample.velocity < 0:
            frequency = -frequency
        data[sample.channel]["frequencies"].append(frequency)

    for channel, channel_data in data.items():
        with open(f"channel_{channel}.csv", "w") as file:
            writer = csv.DictWriter(file, fieldnames=["velocities", "frequencies"])
            writer.writeheader()
            for velocity, frequency in zip(channel_data["velocities"], channel_data["frequencies"]):
                writer.writerow({"velocities": velocity, "frequencies": frequency})

    plots = {channel: plt.subplot(1, max(data.keys()) + 1, channel + 1) for channel in data.keys()}

    for channel, channel_data in data.items():
        subplot = plots[channel]
        subplot.plot(channel_data["velocities"], channel_data["frequencies"], ".")
        subplot.set_xlabel("Velocity")
        subplot.set_ylabel("Frequency")
        subplot.set_title(f"Channel {channel}")
    plt.show()


if __name__ == "__main__":
    main()
