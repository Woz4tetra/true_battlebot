import argparse
import csv
from typing import Any, Callable

import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit


def fit_function(x, a, b, c, d):
    return a * np.log(-b * x + d) + c


def fit_data_segment(data: np.ndarray) -> tuple[Callable[[Any], float], np.ndarray, np.ndarray]:
    velocities = data[:, 0]
    frequencies = data[:, 1]

    bad_frequencies = (frequencies > 15.0) | (frequencies < 0.01)
    frequencies = np.delete(frequencies, bad_frequencies)
    velocities = np.delete(velocities, bad_frequencies)

    if np.min(velocities) < 0:
        start_val = 1.0
    else:
        start_val = -1.0
    popt, pcov = curve_fit(fit_function, velocities, frequencies, p0=[1.0, start_val, 1.0, 1.0])
    return fit_function, velocities, popt  # type: ignore


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

    upper_fit, upper_velocities, upper_coeffs = fit_data_segment(data[upper_cutoff:])
    lower_fit, lower_velocities, lower_coeffs = fit_data_segment(data[:lower_cutoff])

    print("upper:", upper_coeffs.tolist())
    print("lower:", lower_coeffs.tolist())

    plt.plot(velocities, frequencies, ".")
    plt.axvspan(velocities[lower_cutoff], velocities[upper_cutoff], color="red", alpha=0.5)
    plt.plot(upper_velocities, upper_fit(upper_velocities, *upper_coeffs), "-", label="upper fit")
    plt.plot(lower_velocities, lower_fit(lower_velocities, *lower_coeffs), "-", label="lower fit")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
