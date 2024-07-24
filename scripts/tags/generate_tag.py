#!/usr/bin/env python
import argparse

from tag_tools import RASTER_FORMATS, VECTOR_FORMATS, generate_tag, image_formats


def main() -> None:
    parser = argparse.ArgumentParser(description="tag")

    parser.add_argument("tag", choices=list(image_formats.keys()), help="tag family")
    parser.add_argument("codes", nargs="+", type=int, default=[], help="codes to generate")
    parser.add_argument("-p", "--pixels_per_mm", type=float, default=20.0, help="pixels per mm")
    parser.add_argument(
        "-l",
        "--length_mm",
        type=float,
        default=200.0,
        help="length (mm) from the top left black corner to the top right black corner",
    )
    parser.add_argument(
        "-t",
        "--type",
        type=lambda s: s.lower(),
        default="pdf",
        choices=RASTER_FORMATS + VECTOR_FORMATS,
        help="export file type",
    )

    args = parser.parse_args()

    tag_family = args.tag

    if len(args.codes) == 0:
        print("No codes provided! Nothing to do.")

    for code in args.codes:
        px_per_mm = args.pixels_per_mm
        length_mm = args.length_mm
        extension = args.type

        generate_tag(tag_family, code, px_per_mm, length_mm, extension)


if __name__ == "__main__":
    main()
