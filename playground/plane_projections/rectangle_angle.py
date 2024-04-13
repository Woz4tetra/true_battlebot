import numpy as np
from bw_tools.projection_math.find_minimum_rectangle import get_rectangle_angle
from bw_tools.projection_math.points_transform import points_transform
from bw_tools.structs.pose2d import Pose2D


def make_rectangle(x, y, width, height, angle):
    transform = Pose2D(x, y, angle)
    dx = width / 2
    dy = height / 2
    return points_transform(
        np.array(
            [
                [dx, dy],
                [-dx, dy],
                [-dx, -dy],
                [dx, -dy],
            ]
        ),
        transform.to_matrix(),
    )


input_angle = np.pi / 5
output_angle = get_rectangle_angle(make_rectangle(2.0, 3.0, 1.5, 1.2, input_angle))
print(input_angle, output_angle)


input_angle = -np.pi / 5
output_angle = get_rectangle_angle(make_rectangle(2.0, 3.0, 1.5, 1.2, input_angle))
print(input_angle, output_angle)


input_angle = -np.pi / 5
output_angle = get_rectangle_angle(make_rectangle(2.0, 3.0, 1.5, 1.2, input_angle))
print(input_angle, output_angle)
