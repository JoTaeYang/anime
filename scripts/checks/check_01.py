import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile

PROFILE = get_profile()

open_blend(paths.blend_path("01_rigged"))

rig = bpy.data.objects.get("rig")
assert rig is not None and rig.type == 'ARMATURE', "generated rig missing"

def_bones = {b.name for b in rig.data.bones if b.name.startswith("DEF-")}
expected = PROFILE.expected_def_bones()
assert def_bones == expected, (
    f"DEF mismatch\n missing: {sorted(expected - def_bones)}\n extra: {sorted(def_bones - expected)}"
)

dummy = bpy.data.objects.get(PROFILE.MESH_OBJECT)
assert dummy is not None
mods = [m for m in dummy.modifiers if m.type == 'ARMATURE']
assert len(mods) == 1 and mods[0].object == rig, "armature modifier not bound to rig"

vg_names = {vg.name for vg in dummy.vertex_groups}
missing_vg = expected - vg_names
assert len(missing_vg) < len(expected) * 0.15, f"too many DEF bones without vertex group: {sorted(missing_vg)[:10]}"

weighted = 0
for v in dummy.data.vertices:
    if any(g.weight > 0.01 for g in v.groups):
        weighted += 1
ratio = weighted / len(dummy.data.vertices)
assert ratio > 0.98, f"only {ratio:.1%} vertices weighted"
print("CHECK_01 OK")
