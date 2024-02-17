import argparse

import numpy as np
from matplotlib import pyplot as plt
from scipy.io import wavfile
from scipy.signal import butter, lfilter


def butter_lowpass(cutoff, fs, order=5):
    return butter(order, cutoff, fs=fs, btype="low", analog=False)


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


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

    time_cutoff = 1.5
    time_clip = times > time_cutoff
    times = times[time_clip]
    normalized = normalized[time_clip]

    order = 2
    filter_cutoff = 2  # desired cutoff frequency of the filter, Hz

    normalized = butter_lowpass_filter(normalized, filter_cutoff, sample_rate, order)
    fft_data = np.fft.fft(normalized)
    frequencies = np.fft.fftfreq(len(fft_data)) * sample_rate

    waveform = plt.subplot(2, 1, 1)
    fft_graph = plt.subplot(2, 1, 2)

    fft_cutoff = 20
    fft_clip = (frequencies >= 0) & (frequencies <= fft_cutoff)

    fft_data = np.abs(fft_data)
    fft_data[np.bitwise_not(fft_clip)] = 0.0
    idx = np.argmax(fft_data)
    freq = frequencies[idx]
    freq_in_hertz = abs(freq)
    print(freq_in_hertz)

    frequencies = frequencies[fft_clip]
    fft_data = fft_data[fft_clip]

    waveform.plot(times, normalized)
    fft_graph.plot(frequencies, fft_data)
    plt.show()


if __name__ == "__main__":
    main()
