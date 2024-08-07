import argparse
import csv
import json
import math
from typing import Any, Callable

import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit


def fit_function(x, a, b, c, d):
    return a / (1 + np.exp(-c * (x - b))) + d


def linear_function(x, a, b):
    return (a / b) * x


def symmetric_frequency_to_velocity(
    velocity: float,
    nonlinear_fit: Callable[[float], float],
    linear_fit: Callable[[float], float],
    lower_vel: float,
    upper_vel: float,
) -> float:
    abs_velocity = np.abs(velocity)
    if abs_velocity > max(upper_vel, lower_vel):
        return math.copysign(nonlinear_fit(abs_velocity), velocity)
    else:
        return linear_fit(velocity)


def asymmetric_frequency_to_velocity(
    velocity: float,
    nonlinear_fit: Callable[[float], float],
    lower_linear_fit: Callable[[float], float],
    upper_linear_fit: Callable[[float], float],
    lower_vel: float,
    upper_vel: float,
) -> float:
    if velocity < lower_vel or velocity > upper_vel:
        return nonlinear_fit(velocity)
    elif velocity < 0:
        return lower_linear_fit(velocity)
    else:
        return upper_linear_fit(velocity)


def fit_data_segment(data: np.ndarray) -> tuple[Callable[[Any], float], np.ndarray]:
    velocities = data[:, 0]
    frequencies = data[:, 1]
    abs_frequencies = np.abs(frequencies)

    delete_indices = (abs_frequencies > 15.0) | (abs_frequencies < 0.01)
    frequencies = np.delete(frequencies, delete_indices)
    velocities = np.delete(velocities, delete_indices)
    frequencies = np.append(frequencies, 0.0)
    velocities = np.append(velocities, 0.0)

    p0 = [max(frequencies), np.median(velocities), 1, min(frequencies)]  # this is an mandatory initial guess
    try:
        popt, pcov = curve_fit(fit_function, velocities, frequencies, p0=p0, method="dogbox")
    except RuntimeError:
        popt = p0

    return fit_function, popt  # type: ignore


MIN_COMMAND = -1.0
MAX_COMMAND = 1.0
NUM_SAMPLES = 100


def main() -> None:
    parser = argparse.ArgumentParser(description="compute_lookup_table")
    parser.add_argument("path", type=str, help="Path to the table")
    parser.add_argument("--asymmetric", action="store_true", help="Use asymmetric fit")
    args = parser.parse_args()

    path = args.path
    is_asymmetric = args.asymmetric

    data = []
    with open(path) as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            data.append([float(x) for x in row])
    table = np.array(data, dtype=float)
    table = table[table[:, 0].argsort()]
    velocities = table[:, 0]
    frequencies = table[:, 1]

    abs_frequencies = np.abs(frequencies)
    mid_index = np.argmin(np.abs(velocities))
    lower_cutoff = 0
    upper_cutoff = len(table) - 1
    for i in range(mid_index, -1, -1):
        if abs_frequencies[i] > 0.01:
            lower_cutoff = i
            break
    for i in range(mid_index, len(table)):
        if abs_frequencies[i] > 0.01:
            upper_cutoff = i
            break

    fit_fn, coeffs = fit_data_segment(table)

    upper_vel = velocities[upper_cutoff]
    lower_vel = velocities[lower_cutoff]
    upper_freq = fit_fn(velocities[upper_cutoff], *coeffs)
    lower_freq = fit_fn(velocities[lower_cutoff], *coeffs)

    nonlinear_fit = lambda freq: fit_fn(freq, *coeffs)  # noqa: E731
    upper_linear_fit = lambda freq: linear_function(freq, upper_freq, upper_vel)  # noqa: E731
    lower_linear_fit = lambda freq: linear_function(freq, lower_freq, lower_vel)  # noqa: E731

    input_velocities = np.linspace(MIN_COMMAND, MAX_COMMAND, NUM_SAMPLES)
    output_frequencies = []
    for velocity in input_velocities:
        if is_asymmetric:
            output_frequencies.append(
                asymmetric_frequency_to_velocity(
                    velocity,
                    nonlinear_fit,
                    lower_linear_fit,
                    upper_linear_fit,
                    lower_vel,
                    upper_vel,
                )
            )
        else:
            output_frequencies.append(
                symmetric_frequency_to_velocity(
                    velocity,
                    nonlinear_fit,
                    upper_linear_fit,
                    lower_vel,
                    upper_vel,
                )
            )
    with open("lookup_table.json", "w") as file:
        output = {
            "frequencies": output_frequencies,
            "min_command": float(np.min(input_velocities)),
            "max_command": float(np.max(input_velocities)),
            "lower_cutoff": lower_vel,
            "upper_cutoff": upper_vel,
        }
        file.write(json.dumps(output, indent=4))

    vel_freq_plot = plt.subplot(1, 1, 1)
    vel_freq_plot.plot(velocities, frequencies, ".")
    vel_freq_plot.axvspan(velocities[lower_cutoff], velocities[upper_cutoff], color="red", alpha=0.1)
    vel_freq_plot.plot(input_velocities, output_frequencies, label="fit")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
