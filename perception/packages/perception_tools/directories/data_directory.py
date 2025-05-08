import os
from pathlib import Path


def get_data_directory() -> Path:
    organization = os.environ["ORGANIZATION"]
    project_name = os.environ["PROJECT_NAME"]
    return Path(f"/opt/{organization}/{project_name}/perception/data")


def list_data_directory(subdirectory: Path | str | None = None) -> list[Path]:
    data_directory = get_data_directory()
    directory = (data_directory / subdirectory) if subdirectory else data_directory
    if not directory.exists():
        return []
    return [path for path in directory.iterdir() if path.is_file()]
