import argparse
import json

import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import splev, splprep


def compute_spline(
    times: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    theta: np.ndarray,
    spline_degree: int = 3,
    smoothing_condition: float = 1,
) -> tuple[np.ndarray, np.ndarray]:
    inputs = np.array([x, y, theta])
    inputs += np.random.random(inputs.shape) * 1e-6
    tck, _ = splprep(inputs, k=spline_degree, s=smoothing_condition)
    u = np.linspace(0.0, 1.0, num=len(times), endpoint=True)
    path = splev(u, tck)
    # compute the 1st derivative (despite what der=2 may look like)
    curvature = splev(u, tck, der=2)
    path = np.array(path, dtype=np.float64)
    velocities = curvature / times

    return path, velocities


def compute_delta(
    times: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    theta: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    path = np.array([x, y, theta], dtype=np.float64)
    velocities = np.zeros_like(path)
    velocities[:, 0] = np.zeros(3)
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]
        dtheta = theta[i] - theta[i - 1]
        velocities[:, i] = np.array([dx / dt, dy / dt, dtheta / dt], dtype=np.float64)

    return path, velocities


def main() -> None:
    parser = argparse.ArgumentParser(description="Compute the velocity of a path")
    parser.add_argument("input", type=str, help="Path to the input JSON file")
    parser.add_argument("output", type=str, default=None, nargs="?", help="Path to the output JSON file")
    args = parser.parse_args()
    output_path = args.output if args.output else args.input.replace(".json", "_velocity.json")

    with open(args.input, "r") as f:
        data = json.load(f)
    sequence = data["sequence"]
    times = []
    xs = []
    ys = []
    thetas = []
    for element in sequence:
        times.append(element["timestamp"])
        xs.append(element["x"])
        ys.append(element["y"])
        thetas.append(element["theta"])
    times = np.array(times, dtype=np.float64) + 0.001
    xs = np.array(xs, dtype=np.float64)
    ys = np.array(ys, dtype=np.float64)
    thetas = np.array(thetas, dtype=np.float64)

    duplicate_indices = np.where(np.diff(times) == 0)[0]
    if len(duplicate_indices) > 0:
        print(f"Warning: found {len(duplicate_indices)} duplicate timestamps")
        for i in duplicate_indices:
            print(f"Removing duplicate timestamp {times[i]}")
        times = np.delete(times, duplicate_indices)
        xs = np.delete(xs, duplicate_indices)
        ys = np.delete(ys, duplicate_indices)
        thetas = np.delete(thetas, duplicate_indices)

    # path, velocities = compute_spline(times, xs, ys, thetas)
    path, velocities = compute_delta(times, xs, ys, thetas)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(xs, ys, label="Original path")
    # ax.plot(path[0], path[1], label="Path")
    ax.quiver(path[0], path[1], np.cos(path[2]), np.sin(path[2]), label="Orientation")
    ax.set_aspect("equal")
    ax.legend()
    plt.show()

    data["sequence"] = []
    for i in range(len(times)):
        data["sequence"].append(
            {
                "timestamp": times[i],
                "x": path[0][i],
                "y": path[1][i],
                "theta": path[2][i],
                "vx": velocities[0][i],
                "vy": velocities[1][i],
                "vtheta": velocities[2][i],
            }
        )

    print(f"Saving the output to {output_path}")
    with open(output_path, "w") as f:
        json.dump(data, f, indent=4)


if __name__ == "__main__":
    main()