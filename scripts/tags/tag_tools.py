import io
import os
import pickle
from dataclasses import dataclass, field
from typing import Dict, Generator, Optional, Tuple

import requests
import svgwrite
import toml
from bw_shared.messages.dataclass_utils import from_dict
from PIL import Image, ImageDraw

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


@dataclass
class GeneralConfig:
    pixels_per_mm: int = 20
    tag_family: str = "tag36h11"
    default_size: float = 200.0
    spacing: float = 10.0
    grid: list[list[str]] = field(default_factory=lambda: [[""]])
    draw_cut_lines: bool = False


@dataclass
class TagConfig:
    name: str = ""
    code: int = -1
    size: Optional[float] = None


@dataclass
class GridConfig:
    general: GeneralConfig = field(default_factory=GeneralConfig)
    tags: list[TagConfig] = field(default_factory=lambda: [])


def load_config(path: str) -> GridConfig:
    with open(path, "r") as file:
        return from_dict(GridConfig, toml.load(file))


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
    response = requests.get(url)
    if response.status_code != 200:
        raise Exception(
            f"Failed to download image from {url}. Status code: {response.status_code}. Reason: {response.reason}"
        )
    img_data = response.content
    return Image.open(io.BytesIO(img_data))


CACHE_PATH = "cache.pkl"


def load_from_cache(tag: Tag) -> Optional[Image.Image]:
    if not os.path.exists(CACHE_PATH):
        return None
    with open(CACHE_PATH, "rb") as file:
        cache: Dict[Tag, Image.Image] = pickle.load(file)
    return cache.get(tag, None)


def write_to_cache(tag: Tag, image: Image.Image) -> None:
    if os.path.exists(CACHE_PATH):
        with open(CACHE_PATH, "rb") as file:
            cache: Dict[Tag, Image.Image] = pickle.load(file)
    else:
        cache = {}
    cache[tag] = image
    with open(CACHE_PATH, "wb") as file:
        pickle.dump(cache, file)


def load_tag_image(tag: Tag) -> Image.Image:
    image = load_from_cache(tag)
    if image is None:
        image = download_image(tag)
        write_to_cache(tag, image)
    return image


RASTER_FORMATS = ("png", "jpg", "pdf")
VECTOR_FORMATS = ("svg",)


def generate_raster_tag(tag: Tag, image: Image.Image, length_mm: float, px_per_mm: float, extension: str) -> str:
    assert image.size[0] == image.size[1]
    image_size_px = image.size[0]
    tag_size_px = image_size_px - 2
    assert tag_size_px > 0
    tag_to_image_ratio = image_size_px / tag_size_px
    new_tag_size_px = int(length_mm * px_per_mm)
    new_image_size = int(new_tag_size_px * tag_to_image_ratio)
    print(f"Resizing to {new_image_size} px")

    image = image.convert("RGB")
    image = image.resize((new_image_size, new_image_size), Image.Resampling.NEAREST)
    dpi = px_per_mm * 25.4
    out_path = os.path.splitext(tag.filename)[0] + "." + extension
    image.save(out_path, resolution=dpi)
    print(f"Path is {out_path}")
    return out_path


def iter_image(image: Image.Image) -> Generator[Tuple[int, int, Tuple[int, int, int]], None, None]:
    for y in range(image.size[1]):
        for x in range(image.size[0]):
            yield x, y, image.getpixel((x, y))


def generate_svg_tag(tag: Tag, image: Image.Image, length_mm: float, px_per_mm: float, extension: str) -> str:
    out_path = os.path.splitext(tag.filename)[0] + "." + extension
    dwg = svgwrite.Drawing(out_path, profile="tiny")
    width, height = image.size
    assert width == height
    square_size = length_mm / width
    white = svgwrite.rgb(255, 255, 255)
    black = svgwrite.rgb(0, 0, 0)
    dwg.add(dwg.rect((0, 0), (length_mm, length_mm), fill=white))
    for x, y, pixel in iter_image(image):
        if pixel[0:3] == (0, 0, 0):
            dwg.add(dwg.rect((x * square_size, y * square_size), (square_size, square_size), fill=black))
    dwg.save()
    return out_path


def generate_tag(
    tag_family: str,
    code: int,
    px_per_mm: float,
    length_mm: float,
    extension: str = "pdf",
) -> None:
    tag = Tag(tag_family, code)
    image = load_tag_image(tag)
    generators = {}
    for format in RASTER_FORMATS:
        generators[format] = generate_raster_tag
    generators["svg"] = generate_svg_tag
    generators[extension](tag, image, length_mm, px_per_mm, extension)


def generate_from_config(config_path: str):
    config = load_config(config_path)
    out_path = os.path.splitext(config_path)[0] + ".pdf"

    num_rows = len(config.general.grid)
    num_cols = len(config.general.grid[0])
    print("Grid is %d x %d" % (num_cols, num_rows))
    assert all(len(row) == num_cols for row in config.general.grid), "Grid must be rectangular"

    pixels_per_mm = config.general.pixels_per_mm

    image_lookup: dict[str, Image.Image] = {}
    for tag_config in config.tags:
        size = config.general.default_size if tag_config.size is None else tag_config.size
        tag = Tag(config.general.tag_family, tag_config.code)
        image_path = generate_raster_tag(tag, load_tag_image(tag), size, pixels_per_mm, "png")
        image = Image.open(image_path)
        image_lookup[tag_config.name] = image

    grid: dict[tuple[int, int], Image.Image] = {}
    for y, row in enumerate(config.general.grid):
        for x, tag_name in enumerate(row):
            if tag_name:
                grid[x, y] = image_lookup[tag_name]

    largest_grid_width = int(max(img.width for img in grid.values()))
    largest_grid_height = int(max(img.height for img in grid.values()))
    spacing_px = config.general.spacing * pixels_per_mm
    half_spacing_px = spacing_px // 2
    width_spaced = int(largest_grid_width + spacing_px)
    height_spaced = int(largest_grid_height + spacing_px)

    grid_image = Image.new(  # type: ignore
        "RGB",
        (width_spaced * num_cols, height_spaced * num_rows),
        (255, 255, 255),
    )

    for (x, y), image in grid.items():
        grid_image.paste(image, (x * width_spaced + half_spacing_px, y * height_spaced + half_spacing_px))

    if config.general.draw_cut_lines:
        for x, y in grid.keys():
            draw = ImageDraw.Draw(grid_image)
            draw.rectangle(
                [
                    (x * width_spaced + half_spacing_px, y * height_spaced + half_spacing_px),
                    (
                        x * width_spaced + half_spacing_px + largest_grid_width,
                        y * height_spaced + half_spacing_px + largest_grid_height,
                    ),
                ],
                outline=(255, 0, 0),
            )
    print("Image size is %0.2f mm x %0.2f mm" % (grid_image.width / pixels_per_mm, grid_image.height / pixels_per_mm))

    dpi = pixels_per_mm * 25.4
    grid_image.save(out_path, resolution=dpi)
    print(f"Generated grid path is {out_path}")
