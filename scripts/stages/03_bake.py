import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend, save_as, op_kwargs
from lib.bone_map import BONE_RENAME

open_blend(paths.blend_path("02_animated"))

rig = bpy.data.objects["rig"]
bpy.context.view_layer.objects.active = rig
bpy.ops.object.mode_set(mode='POSE')

# 1) DEF 본만 선택해 비주얼 키잉 베이크 (컨트롤/IK가 만든 최종 자세를 DEF에 굽는다)
bpy.ops.pose.select_all(action='DESELECT')
for pb in rig.pose.bones:
    if pb.name.startswith("DEF-"):
        pb.select = True   # Blender 5.1: 선택은 PoseBone.select (Bone.select 제거됨)

rig.animation_data.action.name = "Idle_src"
selected_pose_bones = [pb for pb in rig.pose.bones if pb.name.startswith("DEF-")]
bake_kwargs = op_kwargs(
    bpy.ops.nla.bake,
    frame_start=1, frame_end=48, step=1,
    only_selected=True, visual_keying=True,
    clear_constraints=True, clear_parents=False,
    use_current_action=False, bake_types={'POSE'},
)
# 알려진 실패 모드: headless에서는 pb.select만으로는 오퍼레이터가 선택 상태를
# 못 봐서 "Nothing to bake"로 조용히 실패한다 -> temp_override로 컨텍스트 명시.
with bpy.context.temp_override(active_object=rig, selected_pose_bones=selected_pose_bones):
    bpy.ops.nla.bake(**bake_kwargs)
rig.animation_data.action.name = "Idle"
bpy.ops.object.mode_set(mode='OBJECT')

# 2) 각 DEF 본의 부모가 될 DEF 본을 삭제 전에 계산 — 2단계:
#    (a) Blender 부모 체인에서 DEF 조상 탐색 (연결 체인: 척추/팔다리/손가락 내부)
#    (b) 없으면 ORG 계층 우회: Rigify는 분기점(어깨→팔, 골반→다리, 손→손가락)에서
#        DEF 본을 ORG 본의 "형제"로 두므로, DEF-X ↔ ORG-X 대응으로 ORG 조상 중
#        DEF 대응이 존재하는 첫 본을 부모로 삼는다 (Blender 5.1.2 실측 토폴로지)
bones = rig.data.bones


def def_parent(bone):
    p = bone.parent
    while p is not None:
        if p.name.startswith("DEF-"):
            return p.name
        p = p.parent
    org = bones.get("ORG-" + bone.name[len("DEF-"):])
    p = org.parent if org else None
    while p is not None:
        cand = None
        if p.name.startswith("ORG-"):
            cand = "DEF-" + p.name[len("ORG-"):]
        elif p.name.startswith("DEF-"):
            cand = p.name
        if cand and cand in bones:
            return cand
        p = p.parent
    return None


parent_map = {}
for bone in bones:
    if bone.name.startswith("DEF-"):
        parent_map[bone.name] = def_parent(bone)

# 3) DEF가 아닌 본 전부 삭제 + DEF 계층 재부모화
bpy.ops.object.mode_set(mode='EDIT')
eb = rig.data.edit_bones
for bone in list(eb):
    if not bone.name.startswith("DEF-"):
        eb.remove(bone)
for name, pname in parent_map.items():
    eb[name].use_connect = False
    eb[name].parent = eb[pname] if pname else None
bpy.ops.object.mode_set(mode='OBJECT')

# 4) Unity 이름으로 리네임 (본 리네임은 fcurve 경로를 자동 갱신한다)
for old, new in BONE_RENAME.items():
    rig.data.bones[old].name = new

# 5) 버텍스 그룹도 명시적으로 리네임 (자동 동기화에 의존하지 않는다)
dummy = bpy.data.objects["Dummy"]
for old, new in BONE_RENAME.items():
    vg = dummy.vertex_groups.get(old)
    if vg is not None:
        vg.name = new

# 6) 잔여물 정리: 위젯, 메타리그, 원본 액션, 드라이버
for ob in list(bpy.data.objects):
    if ob.name.startswith("WGT-") or ob.name == "metarig":
        bpy.data.objects.remove(ob, do_unlink=True)
for act in list(bpy.data.actions):
    if act.name != "Idle":
        bpy.data.actions.remove(act)
if rig.animation_data:
    for drv in list(rig.animation_data.drivers):
        rig.animation_data.drivers.remove(drv)

rig.name = "DummyRig"
rig.data.name = "DummyRig"

save_as(paths.blend_path("03_baked"))
print("STAGE 03_bake OK")
