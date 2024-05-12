import numpy as np
from data_extractor import load_data
from label_colors import COLORS
from matplotlib import pyplot as plt


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def main() -> None:
    path = "/media/storage/bags/simulation_2024-05-11T22-24-22.bag"
    data = load_data(path)

    fig = plt.figure(1)
    fig.tight_layout()
    fig.set_figwidth(15)
    fig.set_figheight(10)
    axes = [
        fig.add_subplot(1, 2, 1),
        fig.add_subplot(1, 2, 2),
        # fig.add_subplot(2, 2, 3, projection="3d"),
    ]
    # axes[2].set_aspect("equal", "box")
    t0 = None

    for label, poses in data.ground_truth_data.items():
        color = COLORS[label]
        if t0 is None:
            t0 = poses[0].header.stamp.to_sec()
        times = [pose.header.stamp.to_sec() - t0 for pose in poses]
        x = [pose.pose.position.x for pose in poses]
        y = [pose.pose.position.y for pose in poses]
        label += " (GT)"
        axes[0].plot(times, x, label=label, color=color)
        axes[1].plot(times, y, label=label, color=color)
    for label, poses in data.filtered_data.items():
        color = COLORS[label]
        color = tuple([c * 0.5 for c in color])
        if t0 is None:
            t0 = poses[0].header.stamp.to_sec()
        times = [pose.header.stamp.to_sec() - t0 for pose in poses]
        x = [pose.pose.position.x for pose in poses]
        y = [pose.pose.position.y for pose in poses]
        axes[0].plot(times, x, label=label, color=color)
        axes[1].plot(times, y, label=label, color=color)
    # for label, poses in data.measurements_in_map.items():
    #     if t0 is None:
    #         t0 = poses[0].header.stamp.to_sec()
    #     times = [pose.header.stamp.to_sec() - t0 for pose in poses]
    #     x = [pose.pose.position.x for pose in poses]
    #     y = [pose.pose.position.y for pose in poses]
    #     axes[0].plot(times, x, label=label, color=COLORS[label], marker=".", linestyle="")
    #     axes[1].plot(times, y, label=label, color=COLORS[label], marker=".", linestyle="")
    #     # axes[2].scatter3D(times, x, y, color=COLORS[label], marker=".", linestyle="")

    axes[1].legend()
    plt.show()


if __name__ == "__main__":
    main()
