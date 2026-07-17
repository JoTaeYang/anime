import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend

open_blend(paths.blend_path("00_mesh"))

ob = bpy.data.objects.get("Dummy")
assert ob is not None and ob.type == 'MESH', "Dummy mesh missing"
assert 1.55 < ob.dimensions.z < 1.80, f"height {ob.dimensions.z}"
assert 1000 < len(ob.data.vertices) < 50000, f"verts {len(ob.data.vertices)}"
# 트랜스폼 적용 확인: 원점 월드 0, 스케일 1, 회전 0
assert ob.location.length < 1e-6, f"origin not at world zero: {tuple(ob.location)}"
assert all(abs(s - 1.0) < 1e-6 for s in ob.scale), f"scale not applied: {tuple(ob.scale)}"
assert all(abs(a) < 1e-6 for a in ob.rotation_euler), "rotation not applied"
mat = ob.data.materials[0] if ob.data.materials else None
assert mat is not None and mat.name == "Checker", "checker material missing"
assert any(n.bl_idname == "ShaderNodeTexChecker" for n in mat.node_tree.nodes)
print("CHECK_00 OK")
