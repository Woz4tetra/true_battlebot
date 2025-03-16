import argparse
import socket

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def make_udp_server(ip: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((ip, port))
    return server


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.4.1")
    args = parser.parse_args()
    udp = make_udp_server(args.ip, 4176)
    plt.ion()

    # make 3d interactive plot
    fig = plt.figure()
    ax: Axes3D = fig.add_subplot(111, projection="3d")
    limit = 20
    ax.set_title("Accelerometer Data")

    column_mapping = {"x": 0, "y": 1, "z": 2}

    while True:
        try:
            data, _ = udp.recvfrom(1024)
            if not data:
                continue
            data = data.decode("utf-8")
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
