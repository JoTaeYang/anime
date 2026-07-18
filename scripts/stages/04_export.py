import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend, op_kwargs

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
# PHASE 1 BLOCKER: 이 사전 회전 트릭은 bone_map.py의 L/R 스왑과 결합되어 있을 수 있다 — bone_map.py 상단 주석 참조
# 실험 토글 (investigate/lr-swap): PRE_ROTATE=0 이면 사전 회전을 끄고 bake_space_transform=True로
# 표준 익스포트(원래 pre-correction 설정). 기본(미설정/1)은 기존 pre-rotate + bake_space_transform=False.
import math
import os
PRE_ROTATE = os.environ.get("PRE_ROTATE", "1") != "0"
if PRE_ROTATE:
    _objs = [bpy.data.objects["DummyRig"], bpy.data.objects["Dummy"]]
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
    filepath=str(paths.EXPORTS_DIR / "dummy.fbx"),
    use_selection=False,
    object_types={'ARMATURE', 'MESH'},
    apply_unit_scale=True,
    global_scale=1.0,
    apply_scale_options='FBX_SCALE_ALL',   # 본 스케일 100배 함정 방지의 핵심
    axis_forward='-Z',
    axis_up='Y',
    use_space_transform=True,
    bake_space_transform=(not PRE_ROTATE), # pre-rotate ON→False(트릭이 노드회전 상쇄), OFF→True(표준)
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
)
bpy.ops.export_scene.fbx(**op_kwargs(bpy.ops.export_scene.fbx, **kwargs))
print("STAGE 04_export OK")
