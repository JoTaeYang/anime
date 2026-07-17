import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib.blender_utils import ensure_rigify
from lib import paths

print("BLENDER", bpy.app.version_string)
assert bpy.app.version_string.startswith("5.1"), f"unexpected: {bpy.app.version_string}"
ensure_rigify()
paths.ensure_dirs()
print("SMOKE OK")
