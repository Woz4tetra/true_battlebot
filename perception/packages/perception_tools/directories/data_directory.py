import os
from pathlib import Path


def get_data_directory() -> Path:
    organization = os.environ["ORGANIZATION"]
    project_name = os.environ["PROJECT_NAME"]
    return Path(f"/opt/{organization}/{project_name}/perception/data")
