import argparse
import json

from matplotlib import pyplot as plt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=str)
    args = parser.parse_args()

    file_path = args.file
    times = []
    lefts = []
    rights = []

    with open(file_path) as file:
        for line in file.readlines():
            data = json.loads(line)
            timestamp = data["time"]
            left_command = data["left_command"]
            right_command = data["right_command"]
            times.append(timestamp)
            lefts.append(left_command)
            rights.append(right_command)
    plt.plot(times, lefts)
    plt.plot(times, rights)
    plt.show()


if __name__ == "__main__":
    main()
