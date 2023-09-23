import pickle
import io
import argparse
import os
from dataclasses import dataclass
from typing import Dict, Optional

import requests
from PIL.Image import open as open_image
from PIL.Image import Image, Resampling


image_formats = {
    "tag16h5": "tag16_05_%05d.png",
    "tag25h9": "tag25_09_%05d.png",
    "tag36h11": "tag36_11_%05d.png",
    "tagCircle21h7": "tag21_07_%05d.png",
    "tagCircle49h12": "tag49_12_%05d.png",
    "tagCustom48h12": "tag48_12_%05d.png",
    "tagStandard41h12": "tag41_12_%05d.png",
    "tagStandard52h13": "tag52_13_%05d.png",
}


@dataclass(frozen=True)
class Tag:
    family: str
    code: int

    @property
    def filename(self) -> str:
        return image_formats[self.family] % self.code


def download_image(tag: Tag):
    url = f"https://github.com/AprilRobotics/apriltag-imgs/raw/master/{tag.family}/{tag.filename}"
    print(f"Downloading from {url}")
    img_data = requests.get(url).content
    return open_image(io.BytesIO(img_data))


CACHE_PATH = "cache.pkl"


def load_from_cache(tag: Tag) -> Optional[Image]:
    if not os.path.exists(CACHE_PATH):
        return None
    with open(CACHE_PATH, "rb") as file:
        cache: Dict[Tag, Image] = pickle.load(file)
    return cache.get(tag, None)


def write_to_cache(tag: Tag, image: Image) -> None:
    if os.path.exists(CACHE_PATH):
        with open(CACHE_PATH, "rb") as file:
            cache: Dict[Tag, Image] = pickle.load(file)
    else:
        cache = {}
    cache[tag] = image
    with open(CACHE_PATH, "wb") as file:
        pickle.dump(cache, file)


def load_tag_image(tag: Tag) -> Image:
    image = load_from_cache(tag)
    if image is None:
        image = download_image(tag)
        write_to_cache(tag, image)
    return image


def generate_tag(
    tag_family: str,
    code: int,
    px_per_mm: float,
    length_mm: float,
    extension: str = "pdf",
) -> None:
    tag = Tag(tag_family, code)
    image = load_tag_image(tag)
    assert image.size[0] == image.size[1]
    image_size_px = image.size[0]
    tag_size_px = image_size_px - 2
    assert tag_size_px > 0
    tag_to_image_ratio = image_size_px / tag_size_px
    new_tag_size_px = int(length_mm * px_per_mm)
    new_image_size = int(new_tag_size_px * tag_to_image_ratio)
    print(f"Resizing to {new_image_size} px")

    image = image.convert("RGB")
    image = image.resize((new_image_size, new_image_size), Resampling.NEAREST)
    dpi = px_per_mm * 25.4
    pdf_path = os.path.splitext(tag.filename)[0] + "." + extension
    image.save(pdf_path, resolution=dpi)
    print(f"PDF path is {pdf_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="tag")

    parser.add_argument("tag", choices=list(image_formats.keys()), help="tag family")
    parser.add_argument(
        "codes", nargs="+", type=int, default=[], help="codes to generate"
    )
    parser.add_argument(
        "-p", "--pixels_per_mm", type=float, default=20.0, help="pixels per mm"
    )
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
        type=str,
        default="pdf",
        choices=["pdf", "png", "jpg"],
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
