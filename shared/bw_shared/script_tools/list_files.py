import os
from glob import glob
from pathlib import Path


def list_files(root_dir: Path, extension: str, extension_in_key: bool = True) -> dict[str, Path]:
    search_dir = root_dir.absolute()
    paths = [Path(file) for file in glob(os.path.join(search_dir, "**", f"*.{extension}"), recursive=True)]
    if extension_in_key:
        files = {str(file.stem + file.suffix): file for file in paths}
    else:
        files = {str(file.stem): file for file in paths}
    files[""] = Path("")
    return files
