import argparse
import csv
import json

import numpy as np
from bw_teleop.lookup_table_config import LookupTableConfig
from matplotlib import pyplot as plt


def linear_function(x: float, m: float, b: float) -> float:
    return m * x + b


def fit_data_segment(data: np.ndarray) -> np.ndarray:
    velocities = data[:, 0]
    frequencies = data[:, 1]

    coeffs = np.polyfit(velocities, frequencies, 1)
    return coeffs


MIN_COMMAND = -1.0
MAX_COMMAND = 1.0
NUM_SAMPLES = 100


def main() -> None:
    parser = argparse.ArgumentParser(description="compute_lookup_table")
    parser.add_argument("path", type=str, help="Path to the table")
    args = parser.parse_args()

    path = args.path

    data = []
    with open(path) as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            data.append([float(x) for x in row])
    table = np.array(data, dtype=float)[..., 1:3]
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

    upper_table = table[upper_cutoff:]
    lower_table = table[:lower_cutoff]
    upper_coeffs = fit_data_segment(upper_table)
    lower_coeffs = fit_data_segment(lower_table)
    coeffs = np.mean([upper_coeffs, lower_coeffs], axis=0)
    y_intercept = max(lower_coeffs[1], upper_coeffs[1])
    lower_coeffs = np.array([coeffs[0], y_intercept])
    upper_coeffs = np.array([coeffs[0], -y_intercept])

    upper_vel = velocities[upper_cutoff]
    lower_vel = velocities[lower_cutoff]
    velocity_cutoff = max(abs(lower_vel), abs(upper_vel))

    input_velocities = []
    output_frequencies = []
    for velocity in np.linspace(MIN_COMMAND, MAX_COMMAND, NUM_SAMPLES):
        if velocity < -velocity_cutoff:
            frequency = linear_function(velocity, *lower_coeffs)
        elif velocity > velocity_cutoff:
            frequency = linear_function(velocity, *upper_coeffs)
        else:
            continue
        output_frequencies.append(frequency)
        input_velocities.append(velocity)
    with open("lookup_table.json", "w") as file:
        config = LookupTableConfig(
            output_frequencies,
            input_velocities,
            linear_function(lower_vel, *lower_coeffs),
            linear_function(upper_vel, *upper_coeffs),
        )
        file.write(json.dumps(config.to_dict(), indent=4))

    vel_freq_plot = plt.subplot(1, 1, 1)
    vel_freq_plot.plot(velocities, frequencies, ".")
    vel_freq_plot.axvspan(velocities[lower_cutoff], velocities[upper_cutoff], color="red", alpha=0.1)
    vel_freq_plot.plot(input_velocities, output_frequencies, label="fit")
    vel_freq_plot.set_xlabel("Velocity")
    vel_freq_plot.set_ylabel("Frequency")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
