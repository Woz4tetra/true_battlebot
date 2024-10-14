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
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Reverse the direction of the circle (i.e. clockwise instead of counter-clockwise)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Loop the objective (i.e. the robot will return to the starting position at the end)",
    )
    args = parser.parse_args()

    radius = args.radius
    total_time = args.time
    output_path = args.output
    reverse = args.reverse
    loop = args.loop

    num_samples = 90
    angles = np.linspace(0, 2 * np.pi, num_samples)
    if reverse:
        angles = angles[::-1]
    xs = radius * np.cos(angles)
    ys = radius * np.sin(angles)
    if reverse:
        tangent_angles = np.rad2deg(angles - np.pi / 2)
    else:
        tangent_angles = np.rad2deg(angles + np.pi / 2)
    times = np.linspace(0, total_time, num_samples)

    sequence = [
        {"x": x, "y": y, "yaw": tangent_angle, "timestamp": time}
        for x, y, tangent_angle, time in zip(xs, ys, tangent_angles, times)
    ]
    if loop:
        sequence[-1]["reset"] = True

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
