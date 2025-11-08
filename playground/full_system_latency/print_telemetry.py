import time

import numpy as np
from bw_shared.radio.crsf.crsf_packet import CrsfBattery
from bw_shared.radio.transmitter.frsky import FrSkyTransmitter


def main() -> None:
    transmitter = FrSkyTransmitter()
    transmitter.open()
    transmitter.set_telemetry(True)

    attitude_receive_times = []
    try:
        while True:
            now = time.perf_counter()
            packets = transmitter.read()

            if not packets:
                time.sleep(0.001)
                continue
            for packet, error in packets:
                if error:
                    print(f"Failed to parse packet: {error}")
                if isinstance(packet, CrsfBattery):
                    attitude_receive_times.append(now)
                    if len(attitude_receive_times) > 10:
                        attitude_receive_times.pop(0)
                    if len(attitude_receive_times) > 1:
                        print(np.mean(np.diff(attitude_receive_times)))
            transmitter.write()
    finally:
        transmitter.close()


if __name__ == "__main__":
    main()
