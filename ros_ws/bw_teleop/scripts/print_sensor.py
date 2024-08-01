import argparse
import copy
import math
import time

import serial


def main() -> None:
    parser = argparse.ArgumentParser(description="print_sensor")
    parser.add_argument("device_path", type=str, help="Path to the serial device")
    parser.add_argument("-r", "--radius", type=float, help="Wheel radius", default=0.025)
    args = parser.parse_args()
    device_path = args.device_path
    wheel_radius = args.radius
    device = serial.Serial(device_path, 115200)
    while not device.in_waiting:
        print("Waiting for data from sensor")
        time.sleep(1)
    prev_ticks = [0, 0]
    prev_time = time.perf_counter()
    while True:
        time.sleep(0.5)
        if not device.in_waiting:
            continue
        while device.in_waiting:
            row = device.readline()
        now = time.perf_counter()
        try:
            ticks = [int(x) for x in row.split(b"\t")]
        except ValueError:
            continue
        if ticks == prev_ticks:
            continue
        delta_time = now - prev_time
        ground_speeds = []
        for channel in range(len(prev_ticks)):
            num_rotations = (ticks[channel] - prev_ticks[channel]) / 2  # 2 counts per rotation
            frequency = num_rotations / delta_time
            ground_speed = 2 * wheel_radius * frequency * math.pi
            ground_speeds.append(ground_speed)
        print(f"Left ground speed: {ground_speeds[0]:.3f} m/s, Right ground speed: {ground_speeds[1]:.3f} m/s")
        prev_ticks = copy.copy(ticks)
        prev_time = now


if __name__ == "__main__":
    main()
