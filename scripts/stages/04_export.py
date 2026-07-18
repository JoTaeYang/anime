import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend, op_kwargs
from lib.profiles import get_profile

PROFILE = get_profile()

open_blend(paths.blend_path("03_baked"))
paths.ensure_dirs()

# --- Armature root-rotation 교정 (Task 9) ---
# bake_space_transform=True는 메시는 잘 굽지만 아마추어 노드에는 축변환 회전(-90°X)을
# 잔여로 남긴다(Blender FBX의 알려진 아마추어 한계; "Apply Transform"은 실험적).
# 대안: 데이터를 미리 Y-up으로 구운 뒤 축변환이 노드 회전을 상쇄하게 한다.
#   1) 아마추어+메시를 -90°X 회전 후 적용 → 정점/본 rest가 Y-up으로 baked
#   2) +90°X 회전을 적용하지 않은 오브젝트 회전으로 남김
#   3) bake_space_transform=False로 내보내면 축변환(-90°X)이 (2)의 +90°X를 상쇄해
#      Unity에서 루트/아마추어 모두 identity가 된다.
# RESOLVED (2026-07-18): 이 pre-rotate 트릭은 아마추어 루트 −90°X 회전만 교정한다.
# bone_map.py의 (구)L/R 스왑과는 무관함이 config 매트릭스로 입증됐다: pre-rotate를 꺼도
# (config B, bake_space_transform=True) 좌우 이름·지오메트리는 그대로였고, 켜고 끔이 바꾼
# 것은 오직 아마추어 루트 회전(끄면 DummyRig에 90° 잔류 → root_children_identity 실패,
# 켜면 identity)뿐이었다. 애초에 미러는 없었다. 상세: sdd/lr-swap-investigation.md
import math
_objs = [bpy.data.objects["DummyRig"], bpy.data.objects[PROFILE.MESH_OBJECT]]
bpy.ops.object.mode_set(mode='OBJECT')
bpy.ops.object.select_all(action='DESELECT')
for _o in _objs:
    _o.select_set(True)
    _o.rotation_euler = (math.radians(-90), 0.0, 0.0)
bpy.context.view_layer.objects.active = _objs[0]
bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
for _o in _objs:
    _o.rotation_euler = (math.radians(90), 0.0, 0.0)

kwargs = dict(
    filepath=str(paths.EXPORTS_DIR / PROFILE.FBX_NAME),
    use_selection=False,
    object_types={'ARMATURE', 'MESH'},
    apply_unit_scale=True,
    global_scale=1.0,
    apply_scale_options='FBX_SCALE_ALL',   # 본 스케일 100배 함정 방지의 핵심
    axis_forward='-Z',
    axis_up='Y',
    use_space_transform=True,
    bake_space_transform=False,            # 위 pre-rotate 트릭이 아마추어 노드 회전을 대신 상쇄
    add_leaf_bones=False,                  # `_end` 본 생성 방지
    primary_bone_axis='Y',
    secondary_bone_axis='X',
    armature_nodetype='NULL',
    use_armature_deform_only=False,        # 이미 03에서 deform만 남김
    bake_anim=True,
    bake_anim_use_all_bones=True,
    bake_anim_use_nla_strips=False,
    bake_anim_use_all_actions=False,       # 활성 액션(Idle)만
    bake_anim_force_startend_keying=True,
    mesh_smooth_type='OFF',
    path_mode='COPY',                      # 팩된 텍스처를 익스포트에 포함시키는 전제
    embed_textures=True,                   # 텍스처를 FBX 안에 임베드 (캐릭터 텍스처는 .blend 팩 상태 — 이게 없으면 흰 재질로 나감)
)
bpy.ops.export_scene.fbx(**op_kwargs(bpy.ops.export_scene.fbx, **kwargs))

import json
for _stale in paths.EXPORTS_DIR.glob("*_meta.json"):
    _stale.unlink()
meta = {
    "fbx": PROFILE.FBX_NAME,
    "expectedHipsY": PROFILE.HIPS_HEIGHT,
    "hipsTol": PROFILE.HIPS_TOL,
    "checkLateralityMarker": PROFILE.USE_LATERALITY_MARKER,
    "appendageBones": sorted(PROFILE.appendage_bone_rename().values()),
}
with open(paths.EXPORTS_DIR / f"{PROFILE.NAME}_meta.json", "w") as f:
    json.dump(meta, f, indent=1)
print("META OK:", meta)
print("STAGE 04_export OK")
