import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]

# 프로필별 출력 네임스페이스 (task-10 Fix 2). env를 직접 읽는다 — lib.profiles를 import하면
# 순환 임포트가 생기므로 profile 모듈이 아니라 env var를 직접 읽는다.
_PROFILE = os.environ.get("ANIME_PROFILE", "dummy")
BUILD_DIR = PROJECT_ROOT / "build" / _PROFILE
EXPORTS_DIR = PROJECT_ROOT / "exports" / _PROFILE
PREVIEWS_DIR = PROJECT_ROOT / "previews" / _PROFILE


def blend_path(stage: str) -> Path:
    return BUILD_DIR / f"{stage}.blend"


def ensure_dirs() -> None:
    for d in (BUILD_DIR, EXPORTS_DIR, PREVIEWS_DIR):
        d.mkdir(parents=True, exist_ok=True)
