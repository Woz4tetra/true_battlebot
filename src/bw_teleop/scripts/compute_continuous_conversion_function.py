import argparse
import csv
from typing import Any, Callable

import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit


def fit_function(x, a, b, c, d):
    return a * np.exp(b * x + c) + d


def linear_function(x, a, b):
    return (a / b) * x


def frequency_to_velocity(
    frequency: float,
    lower_nonlinear_fit: Callable[[float], float],
    upper_nonlinear_fit: Callable[[float], float],
    lower_linear_fit: Callable[[float], float],
    upper_linear_fit: Callable[[float], float],
    lower_freq: float,
    upper_freq: float,
) -> float:
    if frequency < lower_freq:
        return -lower_nonlinear_fit(-frequency)
    elif frequency > upper_freq:
        return upper_nonlinear_fit(frequency)
    elif frequency < 0:
        return lower_linear_fit(frequency)
    else:
        return upper_linear_fit(frequency)


def moving_average(x, w):
    return np.convolve(x, np.ones(w), "valid") / w


def fit_data_segment(data: np.ndarray) -> tuple[Callable[[Any], float], np.ndarray, np.ndarray]:
    velocities = np.abs(data[:, 0])
    frequencies = np.abs(data[:, 1])

    delete_indices = (frequencies > 15.0) | (frequencies < 0.01)
    frequencies = np.delete(frequencies, delete_indices)
    velocities = np.delete(velocities, delete_indices)

    # frequencies = moving_average(frequencies, 3)
    # velocities = velocities[len(velocities) - len(frequencies) :]

    popt, pcov = curve_fit(fit_function, frequencies, velocities, p0=[0.5, 0.5, -1.0, 50.0])
    samples = np.linspace(np.min(frequencies), np.max(frequencies), 100)

    # plt.plot(velocities, frequencies, ".")
    # plt.plot(fit_function(samples, *popt), samples, ".", label="fit")
    # plt.show()

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

    abs_frequencies = np.abs(frequencies)
    mid_index = np.argmin(np.abs(velocities))
    lower_cutoff = 0
    upper_cutoff = len(data) - 1
    for i in range(mid_index, -1, -1):
        if abs_frequencies[i] > 0.01:
            lower_cutoff = i
            break
    for i in range(mid_index, len(data)):
        if abs_frequencies[i] > 0.01:
            upper_cutoff = i
            break

    upper_fit_fn, upper_frequencies, upper_coeffs = fit_data_segment(data[upper_cutoff:])
    lower_fit_fn, lower_frequencies, lower_coeffs = fit_data_segment(data[:lower_cutoff])
    lower_frequencies = -lower_frequencies

    upper_freq = np.min(upper_frequencies)
    upper_vel = upper_fit_fn(upper_freq, *upper_coeffs)
    lower_freq = np.max(lower_frequencies)
    lower_vel = lower_fit_fn(-lower_freq, *lower_coeffs)
    lower_vel = -lower_vel

    upper_nonlinear_fit = lambda freq: upper_fit_fn(freq, *upper_coeffs)  # noqa: E731
    lower_nonlinear_fit = lambda freq: lower_fit_fn(freq, *lower_coeffs)  # noqa: E731
    upper_linear_fit = lambda freq: linear_function(freq, upper_vel, upper_freq)  # noqa: E731
    lower_linear_fit = lambda freq: linear_function(freq, lower_vel, lower_freq)  # noqa: E731

    output_velocities = []
    input_frequencies = np.linspace(np.min(lower_frequencies), np.max(upper_frequencies), 50)
    for frequency in input_frequencies:
        output_velocities.append(
            frequency_to_velocity(
                frequency,
                lower_nonlinear_fit,
                upper_nonlinear_fit,
                lower_linear_fit,
                upper_linear_fit,
                lower_freq,
                upper_freq,
            )
        )

    print(f"upper_coeffs={tuple(upper_coeffs.tolist())},")
    print(f"lower_coeffs={tuple(lower_coeffs.tolist())},")
    print(f"upper_freq={upper_freq},")
    print(f"lower_freq={lower_freq},")
    print(f"upper_vel={upper_vel},")
    print(f"lower_vel={lower_vel},")

    vel_freq_plot = plt.subplot(2, 1, 1)
    freq_vel_plot = plt.subplot(2, 1, 2)
    vel_freq_plot.plot(velocities, frequencies, ".")
    vel_freq_plot.axvspan(velocities[lower_cutoff], velocities[upper_cutoff], color="red", alpha=0.5)
    vel_freq_plot.plot(upper_fit_fn(upper_frequencies, *upper_coeffs), upper_frequencies, "-", label="upper fit")
    vel_freq_plot.plot(-lower_fit_fn(-lower_frequencies, *lower_coeffs), lower_frequencies, "-", label="lower fit")
    plt.legend()
    freq_vel_plot.plot(frequencies, velocities, ".")
    freq_vel_plot.axhspan(frequencies[lower_cutoff], frequencies[upper_cutoff], color="red", alpha=0.5)
    freq_vel_plot.plot(upper_frequencies, upper_fit_fn(upper_frequencies, *upper_coeffs), "-", label="upper fit")
    freq_vel_plot.plot(lower_frequencies, -lower_fit_fn(-lower_frequencies, *lower_coeffs), "-", label="lower fit")
    freq_vel_plot.plot(input_frequencies, output_velocities, "-", label="output")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
