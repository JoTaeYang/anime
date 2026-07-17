import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lib import paths

fbx = paths.EXPORTS_DIR / "dummy.fbx"
assert fbx.exists(), "dummy.fbx missing"
size = fbx.stat().st_size
assert size > 100_000, f"suspiciously small fbx: {size} bytes"
print("CHECK_04 OK")
