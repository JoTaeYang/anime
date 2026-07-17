import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend, op_kwargs

open_blend(paths.blend_path("03_baked"))
paths.ensure_dirs()

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
    bake_space_transform=True,             # 아마추어 루트 -89.98° 회전 방지
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
