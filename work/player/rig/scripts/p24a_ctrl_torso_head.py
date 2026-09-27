"""p24a_ctrl_torso_head.py - T240 (P2.5): torso / head controls, PROPS bone, bone collections (goblin G6 analogue).

Input : work/player/rig/pl_r03_skinned.blend (PL_rig 41 DEF bones, PL_mesh; opened as the main file, not saved over)
        work/player/rig/data/canonical_skeleton.json (G6.2 self-check)
Output: work/player/rig/work/pl_r04a.blend (save copy), work/player/rig/data/ctrl_manifest.json (entries owner "p24a")

CLI:  blender --background --factory-startup work/player/rig/pl_r03_skinned.blend --python p24a_ctrl_torso_head.py

Copied / adapted from work/goblin_swing/rig/scripts/s06a_ctrl_torso_head.py (T24 / T24b / T24c), goblin files unchanged.
New bones in PL_rig (all use_deform False):
  CTRL_root > CTRL_torso (COG: head = DEF Pelvis head = hips, +Z) > {CTRL_pelvis (waist pivot: head = DEF Pelvis tail,
  pointing down to the Pelvis head, local X = world +X) > MCH_pelvis (rest = DEF Pelvis), CTRL_spine_01 > CTRL_chest}
  MCH_CTRL_head_space (no parent) > CTRL_head;  MCH_world (no parent, locked);  CTRL_root > PROPS.
DEF Root / Pelvis / Spine_01 / Spine_02 / Head: Copy Transforms WORLD/WORLD from CTRL_root / MCH_pelvis /
  CTRL_spine_01 / CTRL_chest / CTRL_head.  DEF Neck, HeadEquipmentSocket, BackSocket, BackWeaponSocket and the 16
  Skirt_* bones get no constraint: they follow their DEF parents (Skirt_* follow Pelvis only; runtime springs).
  T253: p24c adds the skirt-follows-thigh MCH layer on Skirt_*_01; PROPS skirt_follow (float 0..1, default 0.5) is
  added here with the other PROPS.
head_space: MCH_CTRL_head_space: space_chest_follow (Armature -> CTRL_chest, influence 1) then space_world_rot
  (Copy Rotation WORLD/WORLD from MCH_world, influence = driver PROPS["head_space"] == 1).
PROPS contract (player): arm / leg IK-FK blends, head_space, hand_ik_space_l (world/root/torso/weapon), hand_ik_space_r
  (world/root/torso), weapon_space_r / weapon_space_l (hand/torso/world), knee_pole_space_l/r (foot/world), foot roll /
  bank / heel twist / toe twist per side (start ranges; p24c narrows them to the planted clamps). No mouth_open.
Bone collections CTRL (incl. PROPS), MCH, DEF (41 DEF bones); only CTRL visible.
Self-check G6.2: all CTRL identity + PROPS defaults -> DEF pose matrix vs canonical rest_matrix.
"""
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
IN_BLEND = RIG / "pl_r03_skinned.blend"
OUT_BLEND = RIG / "work" / "pl_r04a.blend"
MANIFEST = RIG / "data" / "ctrl_manifest.json"
CANON = RIG / "data" / "canonical_skeleton.json"
OWNER = "p24a"
ARM = "PL_rig"

DRIVE = [("CTRL_root", "Root"), ("MCH_pelvis", "Pelvis"), ("CTRL_spine_01", "Spine_01"),
         ("CTRL_chest", "Spine_02"), ("CTRL_head", "Head")]
COG_LEN = 0.2
PROPS_HEAD = (0.0, 0.0, 1.25)
PROPS_LEN = 0.10
MCH_WORLD_LEN = 0.10
NEW_BONES = [
    ("CTRL_root", None, "Root"),
    ("CTRL_torso", "CTRL_root", "cog"),
    ("CTRL_pelvis", "CTRL_torso", "waist"),
    ("MCH_pelvis", "CTRL_pelvis", "Pelvis"),
    ("CTRL_spine_01", "CTRL_torso", "Spine_01"),
    ("CTRL_chest", "CTRL_spine_01", "Spine_02"),
    ("MCH_world", None, "world"),
    ("MCH_CTRL_head_space", None, "Head"),
    ("CTRL_head", "MCH_CTRL_head_space", "Head"),
    ("PROPS", "CTRL_root", "props"),
]
CTRLS = ["CTRL_root", "CTRL_torso", "CTRL_pelvis", "CTRL_spine_01", "CTRL_chest", "CTRL_head"]
MCHS = ["MCH_world", "MCH_CTRL_head_space", "MCH_pelvis"]
NO_LOC = {"CTRL_spine_01", "CTRL_chest", "CTRL_head"}
HEAD_SPACE = {"prop": "head_space", "control": "CTRL_head", "mch": "MCH_CTRL_head_space",
              "values": ["chest", "world"], "constraints": ["space_chest_follow", "space_world_rot"],
              "constraint_type": {"space_chest_follow": "ARMATURE", "space_world_rot": "COPY_ROTATION"},
              "targets": {"space_chest_follow": "CTRL_chest", "space_world_rot": "MCH_world"},
              "driven_by_value": {"space_chest_follow": None, "space_world_rot": 1}}
_ROLL = "deg; start range, p24c narrows it to the planted clamp (ctrl_manifest.clamps)"
PROPS_SPEC = [
    ("arm_ik_fk_l", 0.0, 0.0, 1.0, "Left arm FK/IK blend: 0 = FK, 1 = IK", None),
    ("arm_ik_fk_r", 0.0, 0.0, 1.0, "Right arm FK/IK blend: 0 = FK, 1 = IK", None),
    ("leg_ik_fk_l", 1.0, 0.0, 1.0, "Left leg FK/IK blend: 0 = FK, 1 = IK", None),
    ("leg_ik_fk_r", 1.0, 0.0, 1.0, "Right leg FK/IK blend: 0 = FK, 1 = IK", None),
    ("head_space", 0, 0, 1, "Head space: 0 = chest, 1 = world (switch with the snap operator)", ["chest", "world"]),
    ("hand_ik_space_l", 0, 0, 3, "Left hand IK space: 0 = world, 1 = root, 2 = torso, 3 = weapon (CTRL_weapon_r)",
     ["world", "root", "torso", "weapon"]),
    ("hand_ik_space_r", 0, 0, 2, "Right hand IK space: 0 = world, 1 = root, 2 = torso", ["world", "root", "torso"]),
    ("weapon_space_r", 0, 0, 2, "Right weapon space: 0 = hand (DEF Hand_R), 1 = torso, 2 = world",
     ["hand", "torso", "world"]),
    ("weapon_space_l", 0, 0, 2, "Left weapon space: 0 = hand (DEF Hand_L), 1 = torso, 2 = world",
     ["hand", "torso", "world"]),
    ("knee_pole_space_l", 0, 0, 1, "Left knee pole space: 0 = foot, 1 = world", ["foot", "world"]),
    ("knee_pole_space_r", 0, 0, 1, "Right knee pole space: 0 = foot, 1 = world", ["foot", "world"]),
    ("foot_roll_l", 0.0, -45.0, 60.0, "Left foot roll (- = heel pivot, + = toe pivot), " + _ROLL, None),
    ("foot_roll_r", 0.0, -45.0, 60.0, "Right foot roll (- = heel pivot, + = toe pivot), " + _ROLL, None),
    ("foot_bank_l", 0.0, -30.0, 30.0, "Left foot bank (+ = outer edge, - = inner edge), " + _ROLL, None),
    ("foot_bank_r", 0.0, -30.0, 30.0, "Right foot bank (+ = outer edge, - = inner edge), " + _ROLL, None),
    ("heel_twist_l", 0.0, -45.0, 45.0, "Left heel twist, " + _ROLL, None),
    ("heel_twist_r", 0.0, -45.0, 45.0, "Right heel twist, " + _ROLL, None),
    ("toe_twist_l", 0.0, -45.0, 45.0, "Left toe twist, " + _ROLL, None),
    ("toe_twist_r", 0.0, -45.0, 45.0, "Right toe twist, " + _ROLL, None),
    ("skirt_follow", 0.5, 0.0, 1.0, "Skirt follows thigh: fraction of the push-only thigh tilt given to the skirt "
     "panels (T253, p24c MCH layer, data/skirt_follow.json)", None),
]


def rot(axis, lo, hi):
    return {"channel": "rot", "axis": axis, "min": lo, "max": hi}


def loc(axis, lo, hi):
    return {"channel": "loc", "axis": axis, "min": lo, "max": hi}


SWEEP = {
    "CTRL_root": [loc("X", -1.0, 1.0), loc("Y", -1.0, 1.0), loc("Z", -1.0, 1.0), rot("Y", -180, 180)],
    "CTRL_torso": [loc("X", -0.10, 0.10), loc("Y", -0.12, 0.005), loc("Z", -0.10, 0.10)]
    + [rot(a, -45, 45) for a in "XYZ"],
    "CTRL_pelvis": [loc(a, -0.03, 0.03) for a in "XYZ"] + [rot(a, -30, 30) for a in "XYZ"],
    "CTRL_spine_01": [rot(a, -30, 30) for a in "XYZ"],
    "CTRL_chest": [rot(a, -30, 30) for a in "XYZ"],
    "CTRL_head": [rot(a, -45, 45) for a in "XYZ"],
}

DOC = {
    "owners": "Entries with \"owner\" are written by that script (p24a torso/head/PROPS, p24b arms, p24c legs + "
              "clamps, p24d weapons) and replaced on its rerun; other owners' entries are kept.",
    "sweep": "controls.<CTRL>.sweep = usable animation range per channel: channel rot = Euler degrees in the "
             "control's rotation_mode (XYZ), loc = metres; axis = bone-local channel axis.",
    "torso_axes": "CTRL_root / torso / spine_01 / chest / head point +Z with the DEF roll (hint -Y): local X = world "
                  "+X (rot X+ = bend forward), local Y = world +Z (rot Y = twist, loc Y = up), local Z = world -Y "
                  "(rot Z+ = lean to the character right, loc Z+ = forward).",
    "cog": "CTRL_torso (COG) head = DEF Pelvis head (0, 0.0223, 0.4246) = hip joint height, tail +0.2 m Z, roll 0.",
    "pelvis_pivot": "goblin T24c: CTRL_pelvis head = DEF Pelvis tail = waist (0, 0.0209, 0.5170), tail = DEF Pelvis "
                    "head (points down, world -Z), local X = world +X, local Z = world +Y. rot X+ = same world sense as "
                    "a spine forward bend (hips swing back); child MCH_pelvis (rest = DEF Pelvis) drives DEF Pelvis; "
                    "thigh FK / IK chains are children of CTRL_pelvis; the skirt bones are DEF children of Pelvis.",
    "head_space": "MCH_CTRL_head_space (no parent, parent of CTRL_head): space_chest_follow = ARMATURE -> CTRL_chest "
                  "(constant 1), space_world_rot = COPY_ROTATION WORLD/WORLD from MCH_world (parentless, locked, "
                  "never animated), influence = driver PROPS[\"head_space\"] == 1. chest: full follow; world: "
                  "location follows the chest, rotation fixed to world. DEF Neck has no control (DEF child of "
                  "Spine_02); DEF Head copies CTRL_head.",
    "props": "PROPS pose bone (parent CTRL_root) holds every contract property; ik_fk 0 = FK, 1 = IK; foot_* in "
             "degrees. Python writes need arm.update_tag() + view_layer.update() for the drivers.",
    "p24a_sweep_ranges": "CTRL_torso loc Y (= up) -0.12..+0.005 m: hips 0.425 -> 0.305 m keep the ankles (0.107 m) "
                         "0.198 m below the hips (legs 2 x 0.160 m bend, never fold through); X / Z +-0.10 m, rot +-45; "
                         "CTRL_pelvis loc +-0.03, rot +-30; spine / chest rot +-30; head rot +-45; root as goblin.",
}


def log(msg):
    print(f"[p24a] {msg}")
    sys.stdout.flush()


def fmt_v(v):
    return "(" + ", ".join(f"{x:+.4f}" for x in v) + ")"


def edit_bones(arm):
    vl = bpy.context.view_layer
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    for name, _, _ in NEW_BONES:
        if ebs.get(name) is not None:
            raise RuntimeError(f"bone {name} already exists (expected input pl_r03_skinned.blend)")
    for name, parent, src in NEW_BONES:
        eb = ebs.new(name)
        if src == "cog":
            h = Vector(ebs["Pelvis"].head)
            eb.head, eb.tail, eb.roll = h, h + Vector((0, 0, COG_LEN)), 0.0
        elif src == "props":
            h = Vector(PROPS_HEAD)
            eb.head, eb.tail, eb.roll = h, h + Vector((0, 0, PROPS_LEN)), 0.0
        elif src == "world":
            eb.head, eb.tail, eb.roll = Vector((0, 0, 0)), Vector((0, 0, MCH_WORLD_LEN)), 0.0
        elif src == "waist":
            d = ebs["Pelvis"]
            eb.head, eb.tail = d.tail.copy(), d.head.copy()
            eb.align_roll(Vector((0.0, 1.0, 0.0)))
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
    for n in ("MCH_world", "PROPS"):
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
    for name, dflt, lo, hi, desc, values in PROPS_SPEC:
        is_int = values is not None
        pb[name] = int(dflt) if is_int else float(dflt)
        ui = pb.id_properties_ui(name)
        if is_int:
            ui.update(min=lo, max=hi, soft_min=lo, soft_max=hi, default=int(dflt), step=1, description=desc)
        else:
            ui.update(min=lo, max=hi, soft_min=lo, soft_max=hi, default=float(dflt), description=desc)
        pb.property_overridable_library_set(f'["{name}"]', True)


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
        else:
            c.target = arm
            c.subtarget = tgt
            c.target_space = "WORLD"
            c.owner_space = "WORLD"
            c.mix_mode = "REPLACE"
            c.use_x = c.use_y = c.use_z = True
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
    return removed


def write_manifest():
    old = json.loads(MANIFEST.read_text(encoding="utf-8")) if MANIFEST.exists() else {}
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
        "owner": OWNER, "control": HEAD_SPACE["control"], "prop": HEAD_SPACE["prop"], "values": HEAD_SPACE["values"],
        "mch": HEAD_SPACE["mch"], "constraints": HEAD_SPACE["constraints"],
        "constraint_type": HEAD_SPACE["constraint_type"], "targets": HEAD_SPACE["targets"],
        "influence": {c: ("constant 1" if k is None else f"driver PROPS[prop] == {k}")
                      for c, k in HEAD_SPACE["driven_by_value"].items()},
        "value_effect": {"chest": "full follow of CTRL_chest (location + rotation)",
                         "world": "location follows CTRL_chest, rotation fixed to world (MCH_world)"}}
    man["props"] = {"owner": OWNER, "bone": "PROPS",
                    "spec": [{"name": n, "type": "int" if v is not None else "float", "default": d, "min": lo,
                              "max": hi, "values": v, "description": desc} for n, d, lo, hi, desc, v in PROPS_SPEC]}
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(man, indent=1) + "\n", encoding="utf-8")
    return MANIFEST


def def_vs_canon(arm, canon):
    bpy.context.view_layer.update()
    worst, wn = 0.0, None
    for b in canon["bones"]:
        m = arm.pose.bones[b["name"]].matrix
        r = b["rest_matrix"]
        e = max(abs(m[i][j] - r[i][j]) for i in range(4) for j in range(4))
        if e > worst:
            worst, wn = e, b["name"]
    return worst, wn


def pelvis_pivot_test(arm, deg=20.0):
    pbs = arm.pose.bones
    pel = pbs["CTRL_pelvis"]

    def pts():
        return {"waist": pbs["Pelvis"].tail.copy(), "spine": pbs["Spine_01"].head.copy(),
                "thl": pbs["Thigh_L"].head.copy(), "skirt": pbs["Skirt_F_02"].tail.copy()}

    bpy.context.view_layer.update()
    p0 = pts()
    for ax in (0, 1):
        pel.rotation_euler = (0.0, 0.0, 0.0)
        pel.rotation_euler[ax] = math.radians(deg)
        bpy.context.view_layer.update()
        p1 = pts()
        log(f"PELVIS_PIVOT CTRL_pelvis rot {'XY'[ax]} +{deg:.0f}: DEF Pelvis tail (waist) moved "
            f"{(p1['waist'] - p0['waist']).length * 1000:.3f} mm, Spine_01 head {(p1['spine'] - p0['spine']).length * 1000:.3f}"
            f" mm, Thigh_L head {(p1['thl'] - p0['thl']).length * 1000:.2f} mm, Skirt_F_02 tail "
            f"{(p1['skirt'] - p0['skirt']).length * 1000:.2f} mm (skirt follows Pelvis)")
    pel.rotation_euler = (0.0, 0.0, 0.0)
    bpy.context.view_layer.update()


def main():
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    log(f"Blender {bpy.app.version_string}")
    arm = bpy.data.objects[ARM]
    canon = json.loads(CANON.read_text(encoding="utf-8"))
    def_names = [b["name"] for b in canon["bones"]]
    if sorted(def_names) != sorted(b.name for b in arm.data.bones):
        raise RuntimeError(f"{ARM} bones differ from canonical")
    edit_bones(arm)
    pose_setup(arm)
    add_props(arm)
    hs = head_space(arm)
    removed = collections(arm, def_names)
    log(f"COLLECTIONS removed(empty)={removed}, now " + ", ".join(
        f"{c.name}(visible={c.is_visible}, bones={len(c.bones)})" for c in arm.data.collections_all))
    for name, _, _ in NEW_BONES:
        b = arm.data.bones[name]
        pb = arm.pose.bones[name]
        cons = "; ".join(f"{c.type}:{c.name}->" + (",".join(t.subtarget for t in c.targets) if c.type == "ARMATURE"
                                                   else c.subtarget) for c in pb.constraints) or "-"
        log(f"BONE {name}: parent={b.parent.name if b.parent else None}, head={fmt_v(b.head_local)}, "
            f"tail={fmt_v(b.tail_local)}, x_axis(world)={fmt_v(b.matrix_local.col[0][:3])}, deform={b.use_deform}, "
            f"lock_loc={tuple(pb.lock_location)}, constraints={cons}")
    for _, d in DRIVE:
        c = arm.pose.bones[d].constraints["CTRL_copy"]
        log(f"DEF {d}: {c.type} <- {c.subtarget} ({c.target_space}/{c.owner_space})")
    for cname, tgt, drv in hs:
        log(f"HEAD_SPACE {cname} -> {tgt}: " + ("constant" if drv is None else
                                                  f"expr '{drv.expression}', valid {drv.is_valid}"))
    arm.data.pose_position = "POSE"
    worst, wn = def_vs_canon(arm, canon)
    log(f"G6.2 SELFCHECK (all CTRL identity, PROPS defaults): DEF vs canonical rest max abs diff {worst:.3e} "
        f"(bone {wn}), threshold 1e-4")
    pelvis_pivot_test(arm)
    out = write_manifest()
    log(f"MANIFEST {out}")
    OUT_BLEND.parent.mkdir(parents=True, exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    log(f"OUTPUT {OUT_BLEND}; bones {len(arm.data.bones)}")
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
