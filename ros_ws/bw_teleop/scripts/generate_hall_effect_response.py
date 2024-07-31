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
                if "motor_sample" not in topic:
                    continue
                for sample in msg.samples:
                    if not sample.valid:
                        continue
                    num_rotations = sample.feedback / 2  # 2 counts per rotation
                    frequency = num_rotations / sample.duration
                    if sample.velocity < 0:
                        frequency *= -1
                    samples.append(
                        {
                            "channel": sample.channel,
                            "velocities": sample.velocity,
                            "frequencies": frequency,
                        }
                    )
    print(f"Found {len(samples)} samples")
    sample_df = pd.DataFrame(samples)
    sample_df = sample_df.sort_values(by=["channel", "velocities"])
    sample_df.to_csv("channels.csv", index=False)

    channels = sample_df["channel"].unique()
    plots = {channel: plt.subplot(1, len(channels), index + 1) for index, channel in enumerate(channels)}

    for channel in channels:
        channel_df: pd.DataFrame = sample_df[sample_df["channel"] == channel]
        subplot = plots[channel]
        subplot.plot(channel_df["velocities"].values, channel_df["frequencies"].values, ".")
        subplot.set_xlabel("Velocity")
        subplot.set_ylabel("Frequency")
        subplot.set_title(f"Channel {channel}")

    plt.show()


if __name__ == "__main__":
    main()
