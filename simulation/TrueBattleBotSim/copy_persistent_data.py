import os
import platform
import shutil
from pathlib import Path

SCRIPT_DIR = Path(os.path.dirname(os.path.realpath(__file__)))
PROJECT_NAME = "TrueBattleBotSim"
COMPANY_NAME = "DefaultCompany"


def main() -> None:
    # Get the current platform
    current_platform = platform.system()

    source_path = SCRIPT_DIR / "Assets" / "PersistentData"
    if current_platform == "Windows":
        destination_path = Path(os.environ["USERPROFILE"]) / "Appdata" / "LocalLow" / COMPANY_NAME / PROJECT_NAME
    elif current_platform == "Linux":
        destination_path = Path.home() / ".config" / "unity3d" / COMPANY_NAME / PROJECT_NAME
    elif current_platform == "Darwin":
        destination_path = Path.home() / "Library" / "Application Support" / f"unity.{COMPANY_NAME}.{PROJECT_NAME}"
    else:
        raise NotImplementedError(f"Platform {current_platform} is not supported!")

    # Copy the files
    for file in source_path.iterdir():
        if file.is_file():
            shutil.copy(file, destination_path / file.name)
        elif file.is_dir():
            full_path = destination_path / file.name
            shutil.rmtree(full_path, ignore_errors=True)
            shutil.copytree(file, full_path, dirs_exist_ok=True)

    print(f"File copied to {destination_path} successfully!")


if __name__ == "__main__":
    main()
