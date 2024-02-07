import argparse
import csv
from typing import Any, Callable

import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit


def fit_function(x, a, b, c, d):
    return a * np.exp(b * x + c) + d


def moving_average(x, w):
    return np.convolve(x, np.ones(w), "valid") / w


def fit_data_segment(data: np.ndarray) -> tuple[Callable[[Any], float], np.ndarray, np.ndarray]:
    # data = data[data[:, 1].argsort()]
    velocities = np.abs(data[:, 0])
    frequencies = data[:, 1]

    bad_frequencies = (frequencies > 15.0) | (frequencies < 0.01)
    frequencies = np.delete(frequencies, bad_frequencies)
    velocities = np.delete(velocities, bad_frequencies)

    frequencies = moving_average(frequencies, 3)
    velocities = velocities[len(velocities) - len(frequencies) :]

    plt.plot(velocities, frequencies, ".")
    plt.show()
    popt, pcov = curve_fit(fit_function, frequencies, velocities, p0=[0.5, 0.5, -1.0, 50.0])

    samples = np.linspace(0.0, np.max(frequencies), 100)
    return fit_function, samples, popt  # type: ignore


def main() -> None:
    parser = argparse.ArgumentParser(description="compute_conversion_function")
    parser.add_argument("path", type=str, help="Path to the table")
    args = parser.parse_args()

    path = args.path

    with open(path) as file:
        reader = csv.reader(file)
        next(reader)
        data = []
        for row in reader:
            data.append([float(x) for x in row])
    data = np.array(data, dtype=float)
    data = data[data[:, 0].argsort()]
    velocities = data[:, 0]
    frequencies = data[:, 1]

    mid_index = np.argmin(np.abs(velocities))
    lower_cutoff = 0
    upper_cutoff = len(data) - 1
    for i in range(mid_index, -1, -1):
        if frequencies[i] > 0.01:
            lower_cutoff = i
            break
    for i in range(mid_index, len(data)):
        if frequencies[i] > 0.01:
            upper_cutoff = i
            break

    upper_fit, upper_frequencies, upper_coeffs = fit_data_segment(data[upper_cutoff:])
    lower_fit, lower_frequencies, lower_coeffs = fit_data_segment(data[:lower_cutoff])

    print("upper:", upper_coeffs.tolist())
    print("lower:", lower_coeffs.tolist())

    vel_freq_plot = plt.subplot(2, 1, 1)
    freq_vel_plot = plt.subplot(2, 1, 2)
    vel_freq_plot.plot(velocities, frequencies, ".")
    vel_freq_plot.axvspan(velocities[lower_cutoff], velocities[upper_cutoff], color="red", alpha=0.5)
    vel_freq_plot.plot(upper_fit(upper_frequencies, *upper_coeffs), upper_frequencies, "-", label="upper fit")
    vel_freq_plot.plot(-lower_fit(lower_frequencies, *lower_coeffs), lower_frequencies, "-", label="lower fit")
    plt.legend()
    freq_vel_plot.plot(frequencies, velocities, ".")
    freq_vel_plot.plot(upper_frequencies, upper_fit(upper_frequencies, *upper_coeffs), "-", label="upper fit")
    freq_vel_plot.plot(lower_frequencies, -lower_fit(lower_frequencies, *lower_coeffs), "-", label="lower fit")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
