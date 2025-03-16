import argparse
import json
import socket

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def make_udp_server(ip: str, port: int) -> socket.socket:
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((ip, port))
    server.setblocking(False)
    return server


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=4176)
    args = parser.parse_args()
    udp = make_udp_server("0.0.0.0", args.port)
    plt.ion()

    # make 3d interactive plot
    fig = plt.figure()
    ax: Axes3D = fig.add_subplot(111, projection="3d")
    limit = 20
    ax.set_title("Accelerometer Data")

    while True:
        try:
            counter = 0
            packet = b""
            while True:
                try:
                    packet, _ = udp.recvfrom(1024)
                    counter += 1
                except BlockingIOError:
                    break
            if not packet:
                continue
            if counter > 1:
                print(f"Skipped {counter - 1} packets")
            data = json.loads(packet.decode("utf-8"))
            acceleration = [data["accel"]["x"], data["accel"]["y"], data["accel"]["z"]]
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
            plt.pause(0.001)

            # if exit, break
            if not plt.fignum_exists(fig.number):
                break

        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    main()
