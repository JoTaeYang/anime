import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import ensure_rigify, open_blend, save_as
from lib.proportions import SCALE, P

open_blend(paths.blend_path("00_mesh"))
ensure_rigify()

# 1) 메타리그 추가 + 1.7m 스케일
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.armature_human_metarig_add()
meta = bpy.context.active_object
meta.scale = (SCALE,) * 3
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

# 2) 얼굴 서브트리 + breast 제거 (박스 더미에 얼굴 본 100개는 노이즈)
bpy.ops.object.mode_set(mode='EDIT')
eb = meta.data.edit_bones
doomed = set()


def mark(bone):
    doomed.add(bone.name)
    for c in bone.children:
        mark(c)


if "face" in eb:
    mark(eb["face"])
for n in ("breast.L", "breast.R"):
    if n in eb:
        doomed.add(n)
for n in doomed:
    eb.remove(eb[n])
bpy.ops.object.mode_set(mode='OBJECT')

# 3) 스케일 검증 (proportions 테이블과 일치해야 함)
hips_z = meta.data.bones["spine"].head_local.z
assert abs(hips_z - P["spine"]["head"][2]) < 0.005, f"metarig scale drift: hips at {hips_z}"

# 4) 컨트롤 리그 생성 → "rig" 오브젝트
bpy.ops.pose.rigify_generate()
rig = bpy.context.active_object
assert rig.name == "rig", f"unexpected rig name {rig.name}"

# 5) 자동 웨이트 바인딩 (DEF 본만 use_deform=True이므로 DEF에만 붙는다)
dummy = bpy.data.objects["Dummy"]
bpy.ops.object.select_all(action='DESELECT')
dummy.select_set(True)
rig.select_set(True)
bpy.context.view_layer.objects.active = rig
bpy.ops.object.parent_set(type='ARMATURE_AUTO')

save_as(paths.blend_path("01_rigged"))
print("STAGE 01_rig OK")
