import argparse
from pathlib import Path

from depthai_sdk import OakCamera, RecordType


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", type=str, default="/data/oak", help="Path to save the recording")
    args = parser.parse_args()

    path = Path(args.output)
    path.mkdir(parents=True, exist_ok=True)

    with OakCamera() as oak:
        color = oak.create_camera("CAM_A", resolution="1080p", fps=30, encode="H265")
        left = oak.create_camera("CAM_B", resolution="800p", fps=30, encode="H265")
        right = oak.create_camera("CAM_C", resolution="800p", fps=30, encode="H265")

        # Synchronize & save all (encoded) streams
        oak.record([color.out.encoded, left.out.encoded, right.out.encoded], str(path), RecordType.VIDEO)
        # Show color stream
        # oak.visualize([color.out.camera], scale=2 / 3, fps=True)

        oak.start(blocking=True)


if __name__ == "__main__":
    main()
