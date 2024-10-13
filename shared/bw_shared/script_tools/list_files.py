import os
from glob import glob
from pathlib import Path


def list_files(root_dir: Path, extension: str) -> dict[str, Path]:
    search_dir = root_dir.absolute()
    paths = [Path(file) for file in glob(os.path.join(search_dir, "**", f"*.{extension}"), recursive=True)]
    files = {str(file.stem + file.suffix): file for file in paths}
    files[""] = Path("")
    return files
