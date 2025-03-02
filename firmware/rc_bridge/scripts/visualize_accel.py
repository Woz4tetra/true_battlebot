import argparse

import numpy as np
import serial
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default="/dev/ttyACM0")
    args = parser.parse_args()
    port = args.port
    device = serial.Serial(port, 115200)
    plt.ion()

    # make 3d interactive plot
    fig = plt.figure()
    ax: Axes3D = fig.add_subplot(111, projection="3d")
    limit = 20
    ax.set_title("Accelerometer Data")

    column_mapping = {"x": 0, "y": 1, "z": 2}

    while True:
        try:
            while device.in_waiting > 0:
                data = device.readline().decode()
            if not data:
                continue
            row = data.split("\t")
            acceleration = [0.0, 0.0, 0.0]
            for column in row:
                elements = column.split(":")
                if len(elements) != 2:
                    continue
                prefix = elements[0].lower()
                value = elements[1]
                if prefix not in column_mapping:
                    continue
                index = column_mapping[prefix]
                try:
                    acceleration[index] = float(value)
                except ValueError:
                    continue
            print("X: {:.2f}, Y: {:.2f}, Z: {:.2f}".format(*acceleration))

            # draw arrow
            ax.clear()
            ax.quiver(0, 0, 0, acceleration[0], acceleration[1], acceleration[2])
            ax.set_xlim3d(-limit, limit)
            ax.set_ylim3d(-limit, limit)
            ax.set_zlim3d(-limit, limit)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            plt.draw()
            plt.pause(0.01)

            # if exit, break
            if not plt.fignum_exists(fig.number):
                break

        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    main()
