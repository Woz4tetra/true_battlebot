import argparse
import os

from rosbag.bag import Bag


def main() -> None:
    parser = argparse.ArgumentParser("merge_bags")
    parser.add_argument("bags", nargs="+", help="Paths of bags to merge")
    args = parser.parse_args()

    merged_name = "-".join([os.path.splitext(os.path.basename(bag))[0] for bag in args.bags]) + ".bag"
    with Bag(merged_name, "w") as merged_bag:
        for bag in args.bags:
            with Bag(bag, "r", allow_unindexed=True) as current_bag:
                for topic, msg, timestamp in current_bag.read_messages():
                    merged_bag.write(topic, msg, timestamp)
    print(f"Wrote to {merged_name}")


if __name__ == "__main__":
    main()
