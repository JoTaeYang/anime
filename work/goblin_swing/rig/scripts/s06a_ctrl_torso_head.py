"""s06a_ctrl_torso_head.py - T24: torso / head controls, PROPS bone, bone collections (G6.1, G6.2, G6.4 head, G6.5).

Input : rig/gob_r03_skinned.blend (GOB_rig 24 DEF bones + twist Transformation constraints, GOB_mesh, GOB_club)
Output: rig/work/r04a.blend (save copy, save_version 0), rig/data/ctrl_manifest.json (controls / spaces entries of s06a)

Run:  bl.ps1 -Script s06a_ctrl_torso_head.py -Blend gob_r03_skinned.blend

New bones in GOB_rig (all use_deform False):
  CTRL_root > CTRL_torso > {CTRL_pelvis, CTRL_spine_01 > CTRL_chest}     (rest = DEF root/pelvis/spine_01/spine_02,
  MCH_CTRL_head_space (no parent) > CTRL_head                            CTRL_torso = COG at pelvis head, head = DEF head)
  MCH_world (no parent, locked, never animated: the fixed world-space target of head_space = world)
  CTRL_root > PROPS (custom properties of the shared contract, plan "공간 전환·IK/FK 규약")
DEF root/spine_01/spine_02/head: Copy Transforms (WORLD/WORLD) from their CTRL. DEF spine_01 copies CTRL_spine_01
  (child of CTRL_torso), so it does not follow DEF/CTRL pelvis rotation. Other DEF bones and twist constraints untouched.
T24c waist pivot (spec 5): CTRL_pelvis head = DEF pelvis tail (waist, z 0.5777), pointing down to the pelvis head;
  its child MCH_pelvis (rest = DEF pelvis) drives DEF pelvis (Copy Transforms WORLD/WORLD), so pelvis rotation swings
  the hips and legs about the waist and the waist (DEF pelvis tail = DEF spine_01 head) stays put.
head_space (T24b: world = rotation-only world, location always follows the chest): MCH_CTRL_head_space has
  space_chest_follow (Armature -> CTRL_chest, influence 1, no driver) then space_world_rot (Copy Rotation WORLD/WORLD
  from MCH_world, influence = driver PROPS["head_space"] == 1). Snap/space-switch operator is T28.
HR2 (rig/data/hr2_contract.md): PROPS float mouth_open [0, 1], default 0; driver on GOB_mesh
  shape_keys.key_blocks["mouth_open"].value = AVERAGE of one SINGLE_PROP variable PROPS["mouth_open"] (no expression).
Bone collections: CTRL (incl. PROPS), MCH, DEF (24 DEF bones moved here); only CTRL visible (T28 re-applies).
Self-check G6.2: all CTRL identity + PROPS defaults -> DEF pose matrix vs canonical rest_matrix (max abs element diff).
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402

OUT_BLEND = goblib.WORK / "r04a.blend"
MANIFEST = "ctrl_manifest.json"
OWNER = "s06a"
ARM = "GOB_rig"
MESH = "GOB_mesh"
MOUTH_PROP = "mouth_open"   # HR2: PROPS property = shape key name on GOB_mesh

# control -> DEF bone whose rest (head, tail, roll) it copies and which it drives (Copy Transforms WORLD/WORLD)
DRIVE = [("CTRL_root", "root"), ("MCH_pelvis", "pelvis"), ("CTRL_spine_01", "spine_01"),
         ("CTRL_chest", "spine_02"), ("CTRL_head", "head")]
COG_LEN = 0.2
PROPS_HEAD = (0.0, 0.0, 1.45)
PROPS_LEN = 0.10
MCH_WORLD_LEN = 0.10

# name -> (parent, rest source): rest source = DEF bone name, or "cog" / "props" / "world" / "waist"
NEW_BONES = [
    ("CTRL_root", None, "root"),
    ("CTRL_torso", "CTRL_root", "cog"),
    ("CTRL_pelvis", "CTRL_torso", "waist"),  # T24c: pivot at the waist (DEF pelvis tail), pointing down
    ("MCH_pelvis", "CTRL_pelvis", "pelvis"),  # T24c: rest = DEF pelvis, drives DEF pelvis
    ("CTRL_spine_01", "CTRL_torso", "spine_01"),
    ("CTRL_chest", "CTRL_spine_01", "spine_02"),
    ("MCH_world", None, "world"),
    ("MCH_CTRL_head_space", None, "head"),
    ("CTRL_head", "MCH_CTRL_head_space", "head"),
    ("PROPS", "CTRL_root", "props"),
]
CTRLS = ["CTRL_root", "CTRL_torso", "CTRL_pelvis", "CTRL_spine_01", "CTRL_chest", "CTRL_head"]
MCHS = ["MCH_world", "MCH_CTRL_head_space", "MCH_pelvis"]
NO_LOC = {"CTRL_spine_01", "CTRL_chest", "CTRL_head"}  # FK rotation-only controls (DEF chain stays connected)

HEAD_SPACE = {"prop": "head_space", "control": "CTRL_head", "mch": "MCH_CTRL_head_space",
              "values": ["chest", "world"],
              "constraints": ["space_chest_follow", "space_world_rot"],  # evaluation order on the MCH bone
              "constraint_type": {"space_chest_follow": "ARMATURE", "space_world_rot": "COPY_ROTATION"},
              "targets": {"space_chest_follow": "CTRL_chest", "space_world_rot": "MCH_world"},
              # constraint -> PROPS value k that turns it on (influence driver `hs == k`); None = constant influence 1
              "driven_by_value": {"space_chest_follow": None, "space_world_rot": 1}}

# PROPS contract: name, default, min, max, description, enum values (int) or None (float)
_ROLL = "deg; clamp range is ctrl_manifest.clamps (T26); starting range, T26 narrows it"
PROPS_SPEC = [
    ("arm_ik_fk_l", 0.0, 0.0, 1.0, "Left arm FK/IK blend: 0 = FK, 1 = IK", None),
    ("arm_ik_fk_r", 0.0, 0.0, 1.0, "Right arm FK/IK blend: 0 = FK, 1 = IK", None),
    ("leg_ik_fk_l", 1.0, 0.0, 1.0, "Left leg FK/IK blend: 0 = FK, 1 = IK", None),
    ("leg_ik_fk_r", 1.0, 0.0, 1.0, "Right leg FK/IK blend: 0 = FK, 1 = IK", None),
    ("head_space", 0, 0, 1, "Head space: 0 = chest, 1 = world (switch with the snap operator)",
     ["chest", "world"]),
    ("hand_ik_space_l", 0, 0, 3, "Left hand IK space: 0 = world, 1 = root, 2 = torso, 3 = weapon (switch with the "
     "snap operator)", ["world", "root", "torso", "weapon"]),
    ("hand_ik_space_r", 0, 0, 2, "Right hand IK space: 0 = world, 1 = root, 2 = torso (no weapon space: the right "
     "hand drives the weapon)", ["world", "root", "torso"]),
    ("weapon_space", 0, 0, 3, "Weapon space: 0 = hand_r, 1 = hand_l, 2 = torso, 3 = world (switch with the snap "
     "operator)", ["hand_r", "hand_l", "torso", "world"]),
    ("knee_pole_space_l", 0, 0, 1, "Left knee pole space: 0 = foot, 1 = world", ["foot", "world"]),
    ("knee_pole_space_r", 0, 0, 1, "Right knee pole space: 0 = foot, 1 = world", ["foot", "world"]),
    ("foot_roll_l", 0.0, -45.0, 60.0, "Left foot roll (- = heel pivot, + = toe pivot), " + _ROLL, None),
    ("foot_roll_r", 0.0, -45.0, 60.0, "Right foot roll (- = heel pivot, + = toe pivot), " + _ROLL, None),
    ("foot_bank_l", 0.0, -30.0, 30.0, "Left foot bank (shoe inner/outer edge pivot), " + _ROLL, None),
    ("foot_bank_r", 0.0, -30.0, 30.0, "Right foot bank (shoe inner/outer edge pivot), " + _ROLL, None),
    ("heel_twist_l", 0.0, -45.0, 45.0, "Left heel twist, " + _ROLL, None),
    ("heel_twist_r", 0.0, -45.0, 45.0, "Right heel twist, " + _ROLL, None),
    ("toe_twist_l", 0.0, -45.0, 45.0, "Left toe twist, " + _ROLL, None),
    ("toe_twist_r", 0.0, -45.0, 45.0, "Right toe twist, " + _ROLL, None),
    (MOUTH_PROP, 0.0, 0.0, 1.0, "Mouth open (GOB_mesh shape key mouth_open): 0 = closed, 1 = roar", None),
]


def rot(axis, lo, hi):
    return {"channel": "rot", "axis": axis, "min": lo, "max": hi}


def loc(axis, lo, hi):
    return {"channel": "loc", "axis": axis, "min": lo, "max": hi}


SWEEP = {
    "CTRL_root": [loc("X", -1.0, 1.0), loc("Y", -1.0, 1.0), loc("Z", -1.0, 1.0), rot("Y", -180, 180)],
    # T28b: loc limited to the character's leg (hip -> ankle 150 mm, hip z 0.274, ankle z 0.124); see DOC sweep_ranges
    "CTRL_torso": [loc("X", -0.08, 0.08), loc("Y", -0.08, 0.01), loc("Z", -0.08, 0.08)]
    + [rot(a, -45, 45) for a in "XYZ"],
    "CTRL_pelvis": [loc(a, -0.03, 0.03) for a in "XYZ"] + [rot(a, -30, 30) for a in "XYZ"],
    "CTRL_spine_01": [rot(a, -30, 30) for a in "XYZ"],
    "CTRL_chest": [rot(a, -30, 30) for a in "XYZ"],
    "CTRL_head": [rot(a, -45, 45) for a in "XYZ"],
}

DOC = {
    "s06a_owner": "Entries with \"owner\": \"s06a\" are written by s06a_ctrl_torso_head.py (T24) and replaced on its "
                  "rerun; T25-T27 add their own entries/sections (controls, spaces, ikfk, clamps) with their own owner "
                  "tag and leave other owners' entries as they are.",
    "sweep": "controls.<CTRL>.sweep = usable animation range per channel: channel rot = Euler degrees in the "
             "control's rotation_mode, loc = metres; axis = bone-local channel axis.",
    "s06a_sweep_ranges": "T28b: CTRL_torso loc up/down (local Y) -0.08..+0.01 m (hips 0.274 -> 0.194 m keep 70 mm "
                         "above the ankles 0.124 m so the 150 mm legs bend but never fold through; +0.01 = legs "
                         "already straight at rest), side (X) and forward/back (Z) +-0.08 m, rot +-45; CTRL_pelvis loc "
                         "+-0.03 m (relative hip shift inside the COG, rot +-30 unchanged); CTRL_root unchanged.",
    "torso_axes": "CTRL_root/torso/spine_01/chest/head point +Z with roll 0: local X = world +X (rot X+ = bend "
                  "forward), local Y = world +Z (rot Y = twist/yaw, loc Y = up), local Z = world -Y (rot Z+ = lean "
                  "to the character right, loc Z+ = forward). CTRL_pelvis differs (T24c), see pelvis_pivot.",
    "pelvis_pivot": "T24c (spec 5, user decision): CTRL_pelvis head = DEF pelvis tail = waist (0, 0, 0.5777, top of "
                    "the belt = DEF spine_01 head), tail = DEF pelvis head (points down, world -Z), local X = world +X, "
                    "local Z = world +Y. rot X+ = same world rotation sense as a spine forward bend (hips swing "
                    "back); rot Y+ = turn about world -Z (opposite sense to the spine Y twist); rot Z+ = turn about "
                    "world +Y (opposite sense to the spine Z lean); loc X = world +X, loc Y = down, loc Z = back. "
                    "Child MCH_pelvis (rest = DEF pelvis) drives DEF pelvis (Copy Transforms WORLD/WORLD); the "
                    "thigh FK / IK controls stay children of CTRL_pelvis (the same rigid frame), so the waist stays "
                    "put and the hips and legs swing.",
    "cog": "CTRL_torso (COG) head = DEF pelvis head (0, 0, 0.304), tail +0.2 m Z, roll 0: just above the hip joints "
           "(z 0.274) so torso lean/turn pivots at the hips and keeps the short legs in place; same axes as the spine "
           "controls.",
    "head_space": "MCH_CTRL_head_space (no parent, parent of CTRL_head) has 2 constraints in this order: "
                  "space_chest_follow = ARMATURE -> CTRL_chest, constant influence 1 (full chest follow, no driver); "
                  "space_world_rot = COPY_ROTATION WORLD/WORLD REPLACE from MCH_world (parentless, locked, "
                  "never-animated bone at the armature origin, same rest orientation as head, so it does not follow "
                  "CTRL_root), influence = driver PROPS[\"head_space\"] == 1. chest (0): full follow of CTRL_chest; "
                  "world (1): location follows CTRL_chest, rotation fixed to world. Switching = T28 snap operator "
                  "(world matrix kept); influence is driven only by PROPS[\"head_space\"].",
    "props": "PROPS pose bone (parent CTRL_root) holds every contract property; ik_fk 0 = FK, 1 = IK; foot_* in "
             "degrees; mouth_open 0..1 (HR2).",
    "mouth_open": "HR2 (rig/data/hr2_contract.md): PROPS float mouth_open [0, 1], default 0 (0 = closed, 1 = roar). "
                  "Driver on GOB_mesh.data.shape_keys.key_blocks[\"mouth_open\"].value: type AVERAGE, one SINGLE_PROP "
                  "variable mouth_open = GOB_rig pose.bones[\"PROPS\"][\"mouth_open\"], no F-curve modifiers. "
                  "goblin.reset_rig / goblin.ready_pose set it to 0. The export stage (s07b) keys the evaluated value "
                  "on goblin_mesh's Key instead of the driver.",
}


# ---------------------------------------------------------------- build
def edit_bones(arm):
    vl = bpy.context.view_layer
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    for name, _, _ in NEW_BONES:
        if ebs.get(name) is not None:
            raise RuntimeError(f"bone {name} already exists in {ARM} (expected input gob_r03_skinned.blend)")
    for name, parent, src in NEW_BONES:
        eb = ebs.new(name)
        if src == "cog":
            h = Vector(ebs["pelvis"].head)
            eb.head, eb.tail, eb.roll = h, h + Vector((0, 0, COG_LEN)), 0.0
        elif src == "props":
            h = Vector(PROPS_HEAD)
            eb.head, eb.tail, eb.roll = h, h + Vector((0, 0, PROPS_LEN)), 0.0
        elif src == "world":
            eb.head, eb.tail, eb.roll = Vector((0, 0, 0)), Vector((0, 0, MCH_WORLD_LEN)), 0.0
        elif src == "waist":  # T24c: head = DEF pelvis tail, tail = DEF pelvis head; local X = world +X
            d = ebs["pelvis"]
            eb.head, eb.tail = d.tail.copy(), d.head.copy()
            eb.align_roll(Vector((0.0, 1.0, 0.0)))  # local Z = world +Y, so local X = Y x Z = world +X
        else:
            d = ebs[src]
            eb.head, eb.tail, eb.roll = d.head.copy(), d.tail.copy(), d.roll
        eb.use_deform = False
        eb.use_connect = False
        eb.parent = ebs[parent] if parent else None
    bpy.ops.object.mode_set(mode="OBJECT")


def pose_setup(arm):
    pbs = arm.pose.bones
    for n in CTRLS:
        pb = pbs[n]
        pb.rotation_mode = "XYZ"
        pb.lock_scale = (True, True, True)
        if n in NO_LOC:
            pb.lock_location = (True, True, True)
    for n in MCHS[:1] + ["PROPS"]:  # MCH_world and PROPS: fully locked
        pb = pbs[n]
        pb.rotation_mode = "XYZ"
        pb.lock_location = pb.lock_rotation = pb.lock_scale = (True, True, True)
    for ctrl, d in DRIVE:
        c = pbs[d].constraints.new("COPY_TRANSFORMS")
        c.name = "CTRL_copy"
        c.target = arm
        c.subtarget = ctrl
        c.target_space = "WORLD"
        c.owner_space = "WORLD"
        c.mix_mode = "REPLACE"


def add_props(arm):
    pb = arm.pose.bones["PROPS"]
    enum_ui = {}
    for name, dflt, lo, hi, desc, values in PROPS_SPEC:
        is_int = values is not None
        pb[name] = int(dflt) if is_int else float(dflt)
        ui = pb.id_properties_ui(name)
        if is_int:
            ui.update(min=lo, max=hi, soft_min=lo, soft_max=hi, default=int(dflt), step=1, description=desc)
            try:
                ui.update(items=[(v.upper(), v, f"{k} = {v}", k) for k, v in enumerate(values)])
                enum_ui[name] = "items" in ui.as_dict()
            except (TypeError, ValueError) as e:
                enum_ui[name] = f"no items ({type(e).__name__}: {e})"
        else:
            ui.update(min=lo, max=hi, soft_min=lo, soft_max=hi, default=float(dflt), description=desc)
        pb.property_overridable_library_set(f'["{name}"]', True)
    return enum_ui


def mouth_driver(arm):
    """HR2: driver GOB_mesh key_blocks[MOUTH_PROP].value <- PROPS[MOUTH_PROP] (AVERAGE, SINGLE_PROP)."""
    gm = bpy.data.objects[MESH]
    key = gm.data.shape_keys
    if key is None or MOUTH_PROP not in key.key_blocks:
        raise RuntimeError(f"{MESH} has no shape key {MOUTH_PROP} (expected from s02e_mouth.py)")
    if not key.use_relative:
        raise RuntimeError(f"{MESH} shape keys are not relative")
    kb = key.key_blocks[MOUTH_PROP]
    kb.value = 0.0
    fc = kb.driver_add("value")
    for m in list(fc.modifiers):
        fc.modifiers.remove(m)
    drv = fc.driver
    drv.type = "AVERAGE"
    v = drv.variables.new()
    v.name = MOUTH_PROP
    v.type = "SINGLE_PROP"
    v.targets[0].id_type = "OBJECT"
    v.targets[0].id = arm
    v.targets[0].data_path = f'pose.bones["PROPS"]["{MOUTH_PROP}"]'
    return key, fc


def head_space(arm):
    pb = arm.pose.bones[HEAD_SPACE["mch"]]
    dflt = next(s[1] for s in PROPS_SPEC if s[0] == HEAD_SPACE["prop"])
    info = []
    for cname in HEAD_SPACE["constraints"]:
        ctype, tgt = HEAD_SPACE["constraint_type"][cname], HEAD_SPACE["targets"][cname]
        k = HEAD_SPACE["driven_by_value"][cname]
        c = pb.constraints.new(ctype)
        c.name = cname
        if ctype == "ARMATURE":
            c.use_deform_preserve_volume = False
            c.use_bone_envelopes = False
            c.use_current_location = False
            t = c.targets.new()
            t.target = arm
            t.subtarget = tgt
            t.weight = 1.0
        else:  # COPY_ROTATION: full world rotation of the fixed MCH_world, owner location/scale kept
            c.target = arm
            c.subtarget = tgt
            c.target_space = "WORLD"
            c.owner_space = "WORLD"
            c.mix_mode = "REPLACE"
            c.use_x = c.use_y = c.use_z = True
            c.invert_x = c.invert_y = c.invert_z = False
        if k is None:
            c.influence = 1.0
            info.append((cname, tgt, None))
            continue
        c.influence = 1.0 if k == dflt else 0.0
        fc = c.driver_add("influence")
        for m in list(fc.modifiers):
            fc.modifiers.remove(m)
        drv = fc.driver
        drv.type = "SCRIPTED"
        v = drv.variables.new()
        v.name = "hs"
        v.type = "SINGLE_PROP"
        v.targets[0].id_type = "OBJECT"
        v.targets[0].id = arm
        v.targets[0].data_path = f'pose.bones["PROPS"]["{HEAD_SPACE["prop"]}"]'
        drv.expression = f"1.0 if hs == {k} else 0.0"
        info.append((cname, tgt, drv))
    return info


def collections(arm, def_names):
    ad = arm.data
    before = [c.name for c in ad.collections_all]
    colls = {n: ad.collections.new(n) for n in ("CTRL", "MCH", "DEF")}
    for b in ad.bones:
        for c in list(b.collections):
            c.unassign(b)
    for n in def_names:
        colls["DEF"].assign(ad.bones[n])
    for n in CTRLS + ["PROPS"]:
        colls["CTRL"].assign(ad.bones[n])
    for n in MCHS:
        colls["MCH"].assign(ad.bones[n])
    removed = []
    for c in list(ad.collections_all):
        if c.name not in colls and len(c.bones) == 0 and len(c.children) == 0:
            removed.append(c.name)
            ad.collections.remove(c)
    for n, c in colls.items():
        c.is_visible = n == "CTRL"
    return before, removed


# ---------------------------------------------------------------- manifest
def write_manifest():
    p = goblib.DATA / MANIFEST
    old = {}
    if p.exists():
        with open(p, "r", encoding="utf-8") as f:
            old = json.load(f)
    man = {"_doc": dict(old.get("_doc", {}))}
    man["_doc"].update(DOC)
    for sec in ("controls", "spaces"):
        man[sec] = {k: v for k, v in old.get(sec, {}).items() if not (isinstance(v, dict) and v.get("owner") == OWNER)}
    for k, v in old.items():
        if k not in man:
            man[k] = v
    for n in CTRLS:
        man["controls"][n] = {"owner": OWNER, "side": "C", "part": "head" if n == "CTRL_head" else "torso",
                              "rotation_mode": "XYZ", "sweep": SWEEP[n]}
    man["spaces"][HEAD_SPACE["prop"]] = {
        "owner": OWNER, "control": HEAD_SPACE["control"], "prop": HEAD_SPACE["prop"],
        "values": HEAD_SPACE["values"], "mch": HEAD_SPACE["mch"],
        "constraints": HEAD_SPACE["constraints"], "constraint_type": HEAD_SPACE["constraint_type"],
        "targets": HEAD_SPACE["targets"],
        "influence": {c: ("constant 1" if k is None else f"driver PROPS[prop] == {k}")
                      for c, k in HEAD_SPACE["driven_by_value"].items()},
        "value_effect": {"chest": "full follow of CTRL_chest (location + rotation)",
                         "world": "location follows CTRL_chest, rotation fixed to world (MCH_world)"}}
    return goblib.save_json(MANIFEST, man)


# ---------------------------------------------------------------- report / self-check
def fmt_v(v):
    return "(" + ", ".join(f"{x:+.4f}" for x in v) + ")"


def report(arm):
    ad = arm.data
    for name, _, _ in NEW_BONES:
        b = ad.bones[name]
        pb = arm.pose.bones[name]
        cons = "; ".join(f"{c.type}:{c.name}->" + (",".join(t.subtarget for t in c.targets) if c.type == "ARMATURE"
                                                   else c.subtarget) for c in pb.constraints) or "-"
        print(f"[s06a] BONE {name}: parent={b.parent.name if b.parent else None}, head={fmt_v(b.head_local)}, "
              f"tail={fmt_v(b.tail_local)}, deform={b.use_deform}, coll={[c.name for c in b.collections]}, "
              f"rot_mode={pb.rotation_mode}, lock_loc={tuple(pb.lock_location)}, lock_scale={tuple(pb.lock_scale)}, "
              f"constraints={cons}")
    for _, d in DRIVE:
        c = arm.pose.bones[d].constraints["CTRL_copy"]
        print(f"[s06a] DEF {d}: {c.type} <- {c.subtarget} ({c.target_space}/{c.owner_space}, {c.mix_mode})")


def g62_selfcheck(arm, canon):
    pbs = arm.pose.bones
    for name, _, _ in NEW_BONES:
        pb = pbs[name]
        pb.location = (0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.scale = (1, 1, 1)
    props = pbs["PROPS"]
    for name, dflt, *_ in PROPS_SPEC:
        props[name] = type(props[name])(dflt)
    arm.data.pose_position = "POSE"
    bpy.context.view_layer.update()
    worst, worst_name = 0.0, None
    for b in canon["bones"]:
        m = pbs[b["name"]].matrix
        r = b["rest_matrix"]
        e = max(abs(m[i][j] - r[i][j]) for i in range(4) for j in range(4))
        if e > worst:
            worst, worst_name = e, b["name"]
    print(f"[s06a] G6.2 SELFCHECK DEF pose matrix vs canonical rest_matrix (24 bones, all CTRL identity, PROPS "
          f"defaults): max abs diff = {worst:.3e} (bone {worst_name}), threshold 1e-4")
    return worst


def pelvis_pivot_test(arm, deg=20.0):
    """T24c: CTRL_pelvis rot X / Y +deg separately: waist (DEF pelvis tail, DEF spine_01 head) vs hip joint (DEF thigh
    heads) displacement; state restored after."""
    import math
    pbs = arm.pose.bones
    pel = pbs["CTRL_pelvis"]
    mw = arm.matrix_world

    def pts():
        return {"pelvis_tail": mw @ pbs["pelvis"].tail, "spine_01_head": mw @ pbs["spine_01"].head,
                "thigh_l_head": mw @ pbs["thigh_l"].head, "thigh_r_head": mw @ pbs["thigh_r"].head}

    bpy.context.view_layer.update()
    p0 = pts()
    for ax in (0, 1):
        pel.rotation_euler = (0.0, 0.0, 0.0)
        pel.rotation_euler[ax] = math.radians(deg)
        bpy.context.view_layer.update()
        p1 = pts()
        print(f"[s06a] PELVIS_PIVOT CTRL_pelvis rot {'XY'[ax]} +{deg:.0f} deg: displacement DEF pelvis tail (waist) "
              f"{(p1['pelvis_tail'] - p0['pelvis_tail']).length * 1000:.3f} mm, DEF spine_01 head "
              f"{(p1['spine_01_head'] - p0['spine_01_head']).length * 1000:.3f} mm (expected <= 0.5), DEF thigh head "
              f"l/r {(p1['thigh_l_head'] - p0['thigh_l_head']).length * 1000:.2f}/"
              f"{(p1['thigh_r_head'] - p0['thigh_r_head']).length * 1000:.2f} mm (expected > 0)")
    pel.rotation_euler = (0.0, 0.0, 0.0)
    bpy.context.view_layer.update()


def head_world_follow_test(arm, dx=0.1):
    """head_space = 1, CTRL_torso loc X +dx: CTRL_head world location/rotation change; state restored after."""
    pbs = arm.pose.bones
    props, torso, head = pbs["PROPS"], pbs["CTRL_torso"], pbs["CTRL_head"]
    hs0, loc0 = props[HEAD_SPACE["prop"]], torso.location.copy()
    props[HEAD_SPACE["prop"]] = 1
    bpy.context.view_layer.update()
    m0 = arm.matrix_world @ head.matrix
    torso.location = (loc0.x + dx, loc0.y, loc0.z)
    bpy.context.view_layer.update()
    m1 = arm.matrix_world @ head.matrix
    t0 = (arm.matrix_world @ torso.matrix).translation
    d = m1.translation - m0.translation
    infl = pbs[HEAD_SPACE["mch"]].constraints["space_world_rot"].influence
    print(f"[s06a] HEAD_WORLD_FOLLOW head_space=1, CTRL_torso loc X +{dx} m (torso world head {fmt_v(t0)}): "
          f"CTRL_head world loc change = {d.length:.6f} m {fmt_v(d)} (expected ~{dx}), world rot change = "
          f"{goblib.quat_angle_deg(m0.to_quaternion(), m1.to_quaternion()):.6f} deg (expected 0), "
          f"space_world_rot influence={infl:.3f}")
    props[HEAD_SPACE["prop"]] = hs0
    torso.location = loc0
    bpy.context.view_layer.update()
    print(f"[s06a] HEAD_WORLD_FOLLOW restored head_space={props[HEAD_SPACE['prop']]}, CTRL_torso loc="
          f"{fmt_v(torso.location)}")


def main():
    print(f"[s06a] Blender {bpy.app.version_string}")
    arm = bpy.data.objects[ARM]
    canon = goblib.load_json("canonical_skeleton.json")
    def_names = [b["name"] for b in canon["bones"]]
    existing = sorted(b.name for b in arm.data.bones)
    if sorted(def_names) != existing:
        raise RuntimeError(f"{ARM} bones differ from canonical 24: {existing}")
    edit_bones(arm)
    pose_setup(arm)
    enum_ui = add_props(arm)
    mkey, mfc = mouth_driver(arm)
    hs = head_space(arm)
    before, removed = collections(arm, def_names)
    print(f"[s06a] COLLECTIONS before={before}, removed(empty)={removed}, now="
          + ", ".join(f"{c.name}(visible={c.is_visible}, bones={len(c.bones)})" for c in arm.data.collections_all))
    report(arm)
    pr = arm.pose.bones["PROPS"]
    for name, *_ in PROPS_SPEC:
        d = pr.id_properties_ui(name).as_dict()
        print(f"[s06a] PROP {name}: value={pr[name]!r}, default={d.get('default')!r}, min={d.get('min')!r}, "
              f"max={d.get('max')!r}, enum_ui={enum_ui.get(name, '-')}, desc={d.get('description')!r}")
    worst = g62_selfcheck(arm, canon)
    for cname, tgt, drv in hs:
        c = arm.pose.bones[HEAD_SPACE["mch"]].constraints[cname]
        dtxt = ("no driver (constant)" if drv is None else f"expr '{drv.expression}', valid={drv.is_valid}, "
                f"simple_expression={drv.is_simple_expression}")
        sp = f", space {c.target_space}/{c.owner_space}, mix {c.mix_mode}" if c.type == "COPY_ROTATION" else ""
        print(f"[s06a] HEAD_SPACE {cname} ({c.type}) -> {tgt}{sp}: {dtxt}, "
              f"influence(head_space=default)={c.influence:.3f}")
    head_world_follow_test(arm)
    pelvis_pivot_test(arm)
    pr[MOUTH_PROP] = 1.0
    arm.update_tag()
    bpy.context.view_layer.update()
    v1 = mkey.evaluated_get(bpy.context.evaluated_depsgraph_get()).key_blocks[MOUTH_PROP].value
    pr[MOUTH_PROP] = 0.0
    arm.update_tag()
    bpy.context.view_layer.update()
    v0 = mkey.evaluated_get(bpy.context.evaluated_depsgraph_get()).key_blocks[MOUTH_PROP].value
    d = mfc.driver
    print(f"[s06a] MOUTH driver {mkey.name}.{mfc.data_path}: type {d.type}, variables "
          f"{[(x.name, x.type, x.targets[0].id.name, x.targets[0].data_path) for x in d.variables]}, valid {d.is_valid}, "
          f"fcurve modifiers {len(mfc.modifiers)}; shape key value with PROPS {MOUTH_PROP} = 1 -> {v1:.4f}, = 0 -> "
          f"{v0:.4f}; {MESH} shape keys {[k.name for k in mkey.key_blocks]}, materials "
          f"{[m.name if m else None for m in bpy.data.objects[MESH].data.materials]}")
    nondef = [n for n, _, _ in NEW_BONES if arm.data.bones[n].use_deform]
    print(f"[s06a] new bones with use_deform True: {nondef}; total bones={len(arm.data.bones)}")
    out = write_manifest()
    print(f"[s06a] MANIFEST {out}")
    OUT_BLEND.parent.mkdir(parents=True, exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s06a] OUTPUT {OUT_BLEND}")
    print(f"[s06a] G6.2 max_err={worst:.3e}")


if __name__ == "__main__":
    goblib.run_main(main)
