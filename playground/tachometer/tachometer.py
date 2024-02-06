import argparse

import numpy as np
from matplotlib import pyplot as plt
from scipy.io import wavfile


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

    plt.plot(times, normalized, label="waveform")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
