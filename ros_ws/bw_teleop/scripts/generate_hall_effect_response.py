from __future__ import annotations

import argparse

import pandas as pd  # type: ignore
from matplotlib import pyplot as plt
from rosbag import Bag


def main() -> None:
    parser = argparse.ArgumentParser(description="analyze_experiments")
    parser.add_argument("bags", nargs="+", type=str, help="Bag files")
    args = parser.parse_args()

    samples: list[dict[str, float]] = []
    bags = args.bags
    for bag in bags:
        if not bag.endswith(".bag"):
            continue
        with Bag(bag, "r") as bag:
            for topic, msg, timestamp in bag.read_messages():  # type: ignore
                if topic != "motor_sample" or not msg.valid:
                    continue
                for sample in msg.samples:
                    samples.append(
                        {
                            "timestamp": sample.header.stamp.to_sec(),
                            "channel": sample.channel,
                            "velocity": sample.velocity,
                            "hall_effect_counts": sample.feedback,
                        }
                    )
    print(f"Found {len(samples)} samples")
    sample_df = pd.DataFrame(samples)
    for channel in sample_df["channel"].unique():
        channel_df: pd.DataFrame = sample_df[sample_df["channel"] == channel]
        result_df = pd.DataFrame(columns=["velocities", "frequencies"])
        for velocity in channel_df["velocity"].unique():
            velocity_df: pd.DataFrame = channel_df[channel_df["velocity"] == velocity]
            velocity_df = velocity_df.sort_values("timestamp")
            # drop first sample. It contains spin up data.
            velocity_df = velocity_df.iloc[1:]

            # calculate deltas
            time_deltas = velocity_df["timestamp"].diff()
            count_deltas = velocity_df["hall_effect_counts"].diff()

            # delete NAN rows
            not_nan = time_deltas.notna()
            time_deltas = time_deltas[not_nan]
            count_deltas = count_deltas[not_nan]

            frequency = count_deltas / time_deltas

            average_frequency = frequency.mean()
            average_frequency /= 2.0  # two ticks per rotation
            result_df = result_df.append(  # type: ignore
                pd.Series([velocity, average_frequency], index=result_df.columns), ignore_index=True
            )
        filename = f"channel_{channel}.csv"
        print(f"Writing to {filename}")
        result_df.to_csv(filename, index=False)

    plt.show()


if __name__ == "__main__":
    main()
