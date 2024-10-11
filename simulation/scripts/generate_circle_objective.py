import argparse
import json

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate objective that runs the robot in a circle")
    parser.add_argument("-o", "--output", type=str, default="run_in_a_circle.json", help="Output file")
    parser.add_argument("-r", "--radius", type=float, default=1.0, help="Radius of the circle")
    parser.add_argument(
        "-t", "--time", type=float, default=10.0, help="How long the objective should run for in seconds"
    )
    args = parser.parse_args()

    radius = args.radius
    total_time = args.time
    output_path = args.output

    num_samples = 90
    angles = np.linspace(0, 2 * np.pi, num_samples)
    xs = radius * np.cos(angles)
    ys = radius * np.sin(angles)
    tangent_angles = np.rad2deg(angles + np.pi / 2)
    times = np.linspace(0, total_time, num_samples)

    sequence = [
        {"x": x, "y": y, "yaw": tangent_angle, "timestamp": time}
        for x, y, tangent_angle, time in zip(xs, ys, tangent_angles, times)
    ]

    objective = {
        "type": "teleport",
        "smooth_teleports": True,
        "init": {"type": "relative", "x": xs[0], "y": ys[0], "yaw": tangent_angles[0]},
        "sequence": sequence,
    }
    print(f"Writing objective to {output_path}")
    with open(output_path, "w") as f:
        json.dump(objective, f, indent=4)


if __name__ == "__main__":
    main()
