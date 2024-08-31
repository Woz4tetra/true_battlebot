import argparse
import os
from pathlib import Path

from tqdm import tqdm


def main() -> None:
    parser = argparse.ArgumentParser(description="Flatten a yolo dataset")
    parser.add_argument("images", help="Path to images")
    parser.add_argument("-o", "--output", type=str, default="", help="Path to the output directory")
    args = parser.parse_args()

    image_path = Path(args.images)
    output_path = Path(args.output) if args.output else Path(str(image_path) + "_flat")
    images_paths: list[Path] = []
    annotation_paths: dict[str, Path] = {}

    output_path.mkdir(parents=True, exist_ok=True)

    for dirpath, dirnames, filenames in os.walk(image_path):
        for filename in sorted(filenames):
            if filename.endswith(".jpg"):
                image_path = Path(os.path.join(dirpath, filename))
                images_paths.append(image_path)
            elif filename.endswith(".txt"):
                name = os.path.splitext(filename)[0]
                annotation_path = os.path.join(dirpath, filename)
                annotation_paths[name] = Path(annotation_path)

    with tqdm(total=len(images_paths)) as pbar:
        for image_path in images_paths:
            name = image_path.stem
            annotation_path = annotation_paths[name]

            image_path.rename(output_path / image_path.name)
            annotation_path.rename(output_path / annotation_path.name)

            pbar.update(1)


if __name__ == "__main__":
    main()
