import numpy as np
import shapely
from bw_shared.geometry.in_plane import does_circle_collide_with_path, nearest_point_on_line
from bw_shared.geometry.xy import XY
from matplotlib import pyplot as plt


def plot_nearest_point_on_line() -> None:
    point, line, expected_point = (
        np.array([0.5, -1.0]),
        np.array([(-2.0, 1.0), (2.0, 2.0)]),
        np.array([-0.11764706, 1.47058824]),
    )
    nearest = nearest_point_on_line(point, line)
    print(nearest)
    plt.plot([point[0], nearest[0]], [point[1], nearest[1]], "ro-", label="Nearest point")
    plt.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], "bo-", label="Line")
    plt.plot(expected_point[0], expected_point[1], "go", label="Expected point")
    plt.legend()
    plt.show()


def plot_does_circle_collide_with_path() -> None:
    circle_center, circle_diameter, path_start, path_end, path_width, expected_to_collide = (
        XY(0.0, 0.0),
        1.0,
        XY(0.0, 1.0),
        XY(0.0, 2.0),
        1.0,
        False,
    )
    does_collide = does_circle_collide_with_path(circle_center, circle_diameter, path_start, path_end, path_width)
    print(f"calculated {does_collide} == expected {expected_to_collide}: {does_collide == expected_to_collide}")

    line = shapely.geometry.LineString([(path_start.x, path_start.y), (path_end.x, path_end.y)]).buffer(path_width / 2)
    circle = shapely.geometry.Point(circle_center.x, circle_center.y).buffer(circle_diameter / 2)

    plt.gca().set_aspect("equal")
    plt.plot(*line.exterior.xy, "b-", label="Path")
    plt.plot(*circle.exterior.xy, "r-", label="Circle")

    plt.legend()
    plt.show()


def main() -> None:
    # plot_nearest_point_on_line()
    plot_does_circle_collide_with_path()


if __name__ == "__main__":
    main()
