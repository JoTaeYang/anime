import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile

PROFILE = get_profile()

open_blend(paths.blend_path("03_baked"))

rig = bpy.data.objects.get("DummyRig")
assert rig is not None and rig.type == 'ARMATURE'

names = {b.name for b in rig.data.bones}
expected_names = set(PROFILE.bone_rename().values())
assert names == expected_names, (
    f"bone set mismatch\n missing: {sorted(expected_names - names)}\n"
    f" extra: {sorted(names - expected_names)}"
)

b = rig.data.bones
assert b["Hips"].parent is None


def is_ancestor(anc, child):
    p = b[child].parent
    while p is not None:
        if p.name == anc:
            return True
        p = p.parent
    return False


# 모든 본이 Hips를 루트로 하는 단일 트리에 속한다
for bone in b:
    if bone.name != "Hips":
        assert is_ancestor("Hips", bone.name), f"{bone.name} not rooted at Hips"

# Unity Humanoid가 요구하는 조상 순서 (중간 여분 본 — 트위스트 등 — 은 허용)
CHAINS = [
    ("Hips", "Spine"), ("Spine", "Chest"), ("Chest", "UpperChest"),
    ("UpperChest", "Neck"), ("Neck", "Head"),
]
for side in ("Left", "Right"):
    CHAINS += [
        ("UpperChest", f"{side}Shoulder"), (f"{side}Shoulder", f"{side}UpperArm"),
        (f"{side}UpperArm", f"{side}LowerArm"), (f"{side}LowerArm", f"{side}Hand"),
        ("Hips", f"{side}UpperLeg"), (f"{side}UpperLeg", f"{side}LowerLeg"),
        (f"{side}LowerLeg", f"{side}Foot"), (f"{side}Foot", f"{side}Toes"),
    ]
for anc, child in CHAINS:
    assert is_ancestor(anc, child), f"{child} not descended from {anc}"

acts = list(bpy.data.actions)
assert len(acts) == 1 and acts[0].name == "Idle", [a.name for a in acts]


def action_fcurves(action):
    # Blender 5.1: Action.fcurves 제거, layered action API
    # (layers -> strips -> channelbags -> fcurves)로 대체됨.
    if hasattr(action, "fcurves"):
        return list(action.fcurves)
    out = []
    for layer in action.layers:
        for strip in layer.strips:
            for cb in strip.channelbags:
                out.extend(cb.fcurves)
    return out


paths_in_action = {fc.data_path for fc in action_fcurves(acts[0])}
assert any('pose.bones["Hips"]' in p for p in paths_in_action), "Hips not keyed in baked action"

dummy = bpy.data.objects[PROFILE.MESH_OBJECT]
vg = {g.name for g in dummy.vertex_groups}
assert not any(n.startswith("DEF-") for n in vg), "vertex groups not renamed"
assert dummy.modifiers[0].object == rig

leftovers = [o.name for o in bpy.data.objects if o.name.startswith("WGT-") or o.name == "metarig"]
assert not leftovers, f"leftover objects: {leftovers}"

# 베이크가 실제 움직임을 담았는지 (IK 경유 다리 포함)
scene = bpy.context.scene
def hips_z(f):
    scene.frame_set(f)
    return rig.pose.bones["Hips"].matrix.translation.z
assert abs(hips_z(1) - hips_z(24)) > 0.005, "baked Idle does not move Hips"
print("CHECK_03 OK")
