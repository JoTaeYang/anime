import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lib import paths
from lib.profiles import get_profile

PROFILE = get_profile()

fbx = paths.EXPORTS_DIR / PROFILE.FBX_NAME
assert fbx.exists(), f"{PROFILE.FBX_NAME} missing"
size = fbx.stat().st_size
assert size > 100_000, f"suspiciously small fbx: {size} bytes"
print("CHECK_04 OK")
