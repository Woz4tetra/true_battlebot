from pathlib import Path


def list_cache_image_paths(cache_dir: Path) -> list[Path]:
    return sorted(cache_dir.glob("*.jpg"))


def list_image_timestamps(cache_dir: Path) -> list[float]:
    return [float(p.stem) / 1000 for p in list_cache_image_paths(cache_dir)]
