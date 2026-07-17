from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
BUILD_DIR = PROJECT_ROOT / "build"
EXPORTS_DIR = PROJECT_ROOT / "exports"
PREVIEWS_DIR = PROJECT_ROOT / "previews"


def blend_path(stage: str) -> Path:
    return BUILD_DIR / f"{stage}.blend"


def ensure_dirs() -> None:
    for d in (BUILD_DIR, EXPORTS_DIR, PREVIEWS_DIR):
        d.mkdir(parents=True, exist_ok=True)
