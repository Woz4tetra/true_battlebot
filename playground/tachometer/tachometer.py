import argparse

import numpy as np
from matplotlib import pyplot as plt
from scipy.io import wavfile
from scipy.signal import find_peaks


def main() -> None:
    parser = argparse.ArgumentParser(description="Tachometer")
    parser.add_argument("wavfile", type=str, help="Wav file")
    args = parser.parse_args()

    path = args.wavfile
    sample_rate, data = wavfile.read(path)

    max_amplitude = np.max(np.abs(data))
    normalized = data / max_amplitude

    duration = len(normalized) / sample_rate
    times = np.linspace(0, duration, len(normalized))

    indices, _ = find_peaks(normalized, height=0.5, distance=2000, width=10, prominence=0.5)

    frequency = 1.0 / np.diff(times[indices])
    print(np.mean(frequency), np.std(frequency))

    plt.plot(times, normalized, label="waveform")
    plt.plot(times[indices], normalized[indices], "x", label="peaks")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
