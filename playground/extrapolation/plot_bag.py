from data_extractor import load_data
from label_colors import COLORS
from matplotlib import pyplot as plt


def main() -> None:
    path = "/media/storage/bags/simulation_2024-05-09T21-15-25.bag"
    data = load_data(path)

    fig, ax = plt.subplots()
    for label, poses in data.ground_truth_data.items():
        times = [pose.header.stamp.to_sec() for pose in poses]
        x = [pose.pose.position.y for pose in poses]
        ax.plot(times, x, label=label, color=COLORS[label])
    for label, poses in data.measurements_in_map.items():
        times = [pose.header.stamp.to_sec() for pose in poses]
        x = [pose.pose.position.y for pose in poses]
        ax.plot(times, x, label=label, color=COLORS[label], marker=".", linestyle="")

    fig.legend()
    plt.show()


if __name__ == "__main__":
    main()
