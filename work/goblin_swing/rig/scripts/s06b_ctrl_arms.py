"""s06b_ctrl_arms.py - T25: left / right arm FK + IK controls (G6.1-G6.6 arms, G6.8).

Input : rig/work/r04a.blend (s06a: torso/head controls, PROPS, MCH_world, bone collections CTRL/MCH/DEF)
Output: rig/work/r04b.blend (save copy, save_version 0), rig/data/ctrl_manifest.json (adds owner "s06b" entries only)

Run:  bl.ps1 -Script s06b_ctrl_arms.py -Blend work/r04a.blend

Per side x (l, r), each built from its own DEF rest (no mirroring; all new bones use_deform False):
  CTRL_chest > CTRL_shoulder_x (FK/IK shared, rest = DEF shoulder_x) -> DEF shoulder_x Copy Transforms WORLD/WORLD
  FK : CTRL_shoulder_x > CTRL_upperarm_fk_x > CTRL_lowerarm_fk_x > CTRL_hand_fk_x   (rest/roll/connect = DEF)
  IK : CTRL_shoulder_x > MCH_upperarm_ik_x > MCH_lowerarm_ik_x > MCH_hand_ik_x      (rest/roll/connect = DEF)
       MCH_lowerarm_ik_x: IK -> CTRL_hand_ik_x (chain 2, use_tail, no stretch, pole CTRL_elbow_pole_x, pole_angle from
       the rest chain); ik_stretch 0 on both chain bones.  MCH_hand_ik_x: Copy Rotation WORLD/WORLD <- CTRL_hand_ik_x.
       MCH_CTRL_hand_ik_x_space (no parent, rest = DEF hand) > CTRL_hand_ik_x (rest = DEF hand).
       Space constraints on the MCH (Armature, influence = driver PROPS["hand_ik_space_x"] == k):
         space_world -> MCH_world (0), space_root -> CTRL_root (1), space_torso -> CTRL_torso (2);
         left value 3 (weapon) is an EMPTY slot: no constraint yet, T27 adds space_weapon -> CTRL_weapon.
       CTRL_torso > CTRL_elbow_pole_x at DEF lowerarm head + POLE_DIST * (+Y) (preferred elbow bend side).
  DEF upperarm/lowerarm/hand_x: fk_copy (Copy Transforms WORLD/WORLD, influence 1) <- FK control, then ik_copy
       (Copy Transforms WORLD/WORLD, influence = driver PROPS["arm_ik_fk_x"]) <- MCH IK bone.  0 = FK, 1 = IK.
DEF twist bones and their Transformation constraints are untouched (they read DEF upperarm / hand in LOCAL space).
Self-checks: G6.2 FK (defaults) and IK (arm_ik_fk = 1), DEF pose matrix vs canonical rest; twist through CTRL FK.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402

OUT_BLEND = goblib.WORK / "r04b.blend"
MANIFEST = "ctrl_manifest.json"
OWNER = "s06b"
ARM = "GOB_rig"
SIDES = ("l", "r")
POLE_DIST = 0.30
POLE_DIR = Vector((0.0, 1.0, 0.0))  # preferred elbow bend (+Y, G4.9)
POLE_LEN = 0.05
POLE_PARENT = "CTRL_torso"
IK_SPACES = {  # value k -> (space name, constraint name, target); None target = empty slot (T27)
    "l": [("world", "space_world", "MCH_world"), ("root", "space_root", "CTRL_root"),
          ("torso", "space_torso", "CTRL_torso"), ("weapon", "space_weapon", None)],
    "r": [("world", "space_world", "MCH_world"), ("root", "space_root", "CTRL_root"),
          ("torso", "space_torso", "CTRL_torso")],
}
CHAIN = ("upperarm", "lowerarm", "hand")


def names(x):
    return {
        "shoulder": f"CTRL_shoulder_{x}",
        "fk": [f"CTRL_{b}_fk_{x}" for b in CHAIN],
        "mch": [f"MCH_{b}_ik_{x}" for b in CHAIN],
        "def": [f"{b}_{x}" for b in CHAIN],
        "ik": f"CTRL_hand_ik_{x}",
        "space": f"MCH_CTRL_hand_ik_{x}_space",
        "pole": f"CTRL_elbow_pole_{x}",
        "prop_ikfk": f"arm_ik_fk_{x}",
        "prop_space": f"hand_ik_space_{x}",
    }


def new_bone_list(x):
    """(name, parent, rest source DEF bone or 'pole', use_connect from DEF)"""
    n = names(x)
    out = [(n["shoulder"], "CTRL_chest", f"shoulder_{x}", False)]
    parent_fk, parent_mch = n["shoulder"], n["shoulder"]
    for b, fk, mch in zip(CHAIN, n["fk"], n["mch"]):
        out.append((fk, parent_fk, f"{b}_{x}", None))
        out.append((mch, parent_mch, f"{b}_{x}", None))
        parent_fk, parent_mch = fk, mch
    out += [(n["space"], None, f"hand_{x}", False), (n["ik"], n["space"], f"hand_{x}", False),
            (n["pole"], POLE_PARENT, "pole", False)]
    return out


# ---------------------------------------------------------------- sweep (rom_poses _doc sign convention)
def rot(axis, lo, hi):
    return {"channel": "rot", "axis": axis, "min": lo, "max": hi}


def loc(axis, lo, hi):
    return {"channel": "loc", "axis": axis, "min": lo, "max": hi}


def sweeps(x):
    n = names(x)
    raise_z = [-90, 170] if x == "l" else [-170, 90]  # raise = upperarm_l Z+, upperarm_r Z-
    return {
        n["shoulder"]: [rot("X", -30, 30), rot("Z", -30, 30)],
        n["fk"][0]: [rot("X", -90, 90), rot("Y", -90, 90), rot("Z", *raise_z)],
        n["fk"][1]: [rot("X", 0, 140)],
        n["fk"][2]: [rot("X", -60, 60), rot("Y", -90, 90), rot("Z", -30, 30)],
        n["ik"]: [loc("X", -0.3, 0.3), loc("Y", -0.25, 0.0), loc("Z", -0.3, 0.3)]  # T28b, DOC s06b_sweep_ranges
        + [rot(a, -60, 60) for a in "XYZ"],
        n["pole"]: [loc(a, -0.2, 0.2) for a in "XYZ"],
    }


DOC = {
    "s06b_arms": "Arms (s06b, T25), per side from its own DEF rest (no mirroring). CTRL_shoulder_x (parent CTRL_chest) "
                 "is shared by FK and IK and always drives DEF shoulder_x. FK CTRL_upperarm/lowerarm/hand_fk_x and MCH "
                 "IK chain MCH_upperarm/lowerarm/hand_ik_x copy DEF rest, roll and use_connect, so the G4 roll "
                 "convention and the rom_poses sign convention hold for the FK controls (upperarm X+ = forward, raise "
                 "= upperarm_l Z+ / upperarm_r Z-, lowerarm X+ = elbow flexion, hand X+ = hand toward -Y, Y = twist). "
                 "DEF upperarm/lowerarm/hand: fk_copy (influence 1) then ik_copy (influence = arm_ik_fk_x; 0 = FK, "
                 "1 = IK). IK: no stretch (use_stretch False, ik_stretch 0).",
    "s06b_pole": "CTRL_elbow_pole_x: DEF lowerarm_x head + 0.30 m along +Y (preferred elbow bend side, G4.9), bone "
                 "points +Z, roll 0; parent CTRL_torso so the pole follows COG moves/turns but not spine/chest "
                 "bends (the elbow plane stays stable while the chest animates). pole_angle computed from the rest "
                 "chain (IK at identity = DEF rest, self-checked).",
    "s06b_sweep_ranges": "T28b: CTRL_hand_ik_x local Y runs along the arm (rest arm already at full reach, shoulder -> "
                         "wrist about 362 mm): loc Y -0.25..0 m (elbow bends, target stays >= 110 mm from the "
                         "shoulder, no fold-through; + would only pull a straight arm); local X / Z (up-down, "
                         "forward-back) +-0.3 m swing the arm up to about 40 deg; rot +-60; poles +-0.2 m unchanged.",
    "s06b_ik_modes": "controls.<CTRL>.mode = PROPS value the control is swept in (FK controls: arm_ik_fk_x = 0, IK "
                     "hand / pole: arm_ik_fk_x = 1). CTRL_shoulder_x has no mode (active in both).",
}


# ---------------------------------------------------------------- build
def signed_angle(u, v, normal):
    a = u.angle(v)
    if u.cross(v).angle(normal) < 1:
        a = -a
    return a


def pole_angle(base, tip, pole_loc):
    """Blender IK pole angle keeping the rest chain (base = chain root edit bone, tip = IK bone)."""
    pole_normal = (tip.tail - base.head).cross(pole_loc - base.head)
    projected = pole_normal.cross(base.tail - base.head)
    return signed_angle(base.x_axis, projected, base.tail - base.head)


def edit_bones(arm):
    vl = bpy.context.view_layer
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    angles = {}
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            if ebs.get(name) is not None:
                raise RuntimeError(f"bone {name} already exists (expected input work/r04a.blend)")
        for p in ("CTRL_chest", POLE_PARENT, "MCH_world", "CTRL_root"):
            if ebs.get(p) is None:
                raise RuntimeError(f"s06a bone {p} missing")
        for name, parent, src, connect in new_bone_list(x):
            eb = ebs.new(name)
            if src == "pole":
                h = ebs[f"lowerarm_{x}"].head + POLE_DIR * POLE_DIST
                eb.head, eb.tail, eb.roll = h, h + Vector((0, 0, POLE_LEN)), 0.0
            else:
                d = ebs[src]
                eb.head, eb.tail, eb.roll = d.head.copy(), d.tail.copy(), d.roll
                if connect is None:
                    connect = d.use_connect
            eb.use_deform = False
            eb.parent = ebs[parent] if parent else None
            eb.use_connect = bool(connect) and parent is not None
        n = names(x)
        angles[x] = pole_angle(ebs[n["mch"][0]], ebs[n["mch"][1]], ebs[n["pole"]].head)
    bpy.ops.object.mode_set(mode="OBJECT")
    return angles


def add_driver(arm, con, prop, expr):
    fc = con.driver_add("influence")
    for m in list(fc.modifiers):
        fc.modifiers.remove(m)
    drv = fc.driver
    drv.type = "SCRIPTED"
    v = drv.variables.new()
    v.name = "v"
    v.type = "SINGLE_PROP"
    v.targets[0].id_type = "OBJECT"
    v.targets[0].id = arm
    v.targets[0].data_path = f'pose.bones["PROPS"]["{prop}"]'
    drv.expression = expr
    return drv


def copy_tf(arm, pb, name, sub, influence=1.0):
    c = pb.constraints.new("COPY_TRANSFORMS")
    c.name = name
    c.target = arm
    c.subtarget = sub
    c.target_space = "WORLD"
    c.owner_space = "WORLD"
    c.mix_mode = "REPLACE"
    c.influence = influence
    return c


def pose_setup(arm, x, angle):
    pbs = arm.pose.bones
    props = pbs["PROPS"]
    n = names(x)
    drivers = []
    for name in [n["shoulder"]] + n["fk"] + [n["ik"], n["pole"]]:
        pb = pbs[name]
        pb.rotation_mode = "XYZ"
        pb.lock_scale = (True, True, True)
        if name in [n["shoulder"]] + n["fk"]:
            pb.lock_location = (True, True, True)
        if name == n["pole"]:
            pb.lock_rotation = (True, True, True)
    # DEF shoulder: shared control
    copy_tf(arm, pbs[f"shoulder_{x}"], "CTRL_copy", n["shoulder"])
    # IK chain
    for m in n["mch"][:2]:
        pbs[m].ik_stretch = 0.0
    ik = pbs[n["mch"][1]].constraints.new("IK")
    ik.name = "IK"
    ik.target = arm
    ik.subtarget = n["ik"]
    ik.pole_target = arm
    ik.pole_subtarget = n["pole"]
    ik.pole_angle = angle
    ik.chain_count = 2
    ik.use_tail = True
    ik.use_stretch = False
    ik.use_rotation = False
    cr = pbs[n["mch"][2]].constraints.new("COPY_ROTATION")
    cr.name = "IK_rot"
    cr.target = arm
    cr.subtarget = n["ik"]
    cr.target_space = "WORLD"
    cr.owner_space = "WORLD"
    cr.mix_mode = "REPLACE"
    # hand IK space
    sp = pbs[n["space"]]
    cur = props[n["prop_space"]]
    for k, (_, cname, tgt) in enumerate(IK_SPACES[x]):
        if tgt is None:
            continue  # empty weapon slot (T27)
        c = sp.constraints.new("ARMATURE")
        c.name = cname
        c.use_deform_preserve_volume = False
        c.use_bone_envelopes = False
        c.use_current_location = False
        t = c.targets.new()
        t.target = arm
        t.subtarget = tgt
        t.weight = 1.0
        c.influence = 1.0 if cur == k else 0.0
        drivers.append((f"{n['space']}.{cname}", add_driver(arm, c, n["prop_space"], f"1.0 if v == {k} else 0.0")))
    # DEF blend
    ikfk = float(props[n["prop_ikfk"]])
    for fk, mch, d in zip(n["fk"], n["mch"], n["def"]):
        copy_tf(arm, pbs[d], "fk_copy", fk)
        c = copy_tf(arm, pbs[d], "ik_copy", mch, influence=ikfk)
        drivers.append((f"{d}.ik_copy", add_driver(arm, c, n["prop_ikfk"], "v")))
    return drivers


def assign_collections(arm):
    ad = arm.data
    colls = {c.name: c for c in ad.collections_all}
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            b = ad.bones[name]
            for c in list(b.collections):
                c.unassign(b)
            colls["MCH" if name.startswith("MCH_") else "CTRL"].assign(b)


# ---------------------------------------------------------------- manifest
def manifest_entries(angles):
    controls, spaces, ikfk = {}, {}, {}
    for x in SIDES:
        n = names(x)
        side = x.upper()
        sw = sweeps(x)
        for name in [n["shoulder"]] + n["fk"] + [n["ik"], n["pole"]]:
            e = {"owner": OWNER, "side": side, "part": "arm", "rotation_mode": "XYZ", "sweep": sw[name]}
            if name in n["fk"]:
                e["mode"] = {n["prop_ikfk"]: 0}
            elif name in (n["ik"], n["pole"]):
                e["mode"] = {n["prop_ikfk"]: 1}
            controls[name] = e
        vals = [s[0] for s in IK_SPACES[x]]
        cons = [s[1] for s in IK_SPACES[x] if s[2] is not None]
        spaces[n["prop_space"]] = {
            "owner": OWNER, "control": n["ik"], "prop": n["prop_space"], "values": vals, "mch": n["space"],
            "constraints": cons,
            "constraint_type": {c: "ARMATURE" for c in cons},
            "targets": {s[1]: s[2] for s in IK_SPACES[x] if s[2] is not None},
            "influence": {s[1]: f"driver PROPS[prop] == {k}" for k, s in enumerate(IK_SPACES[x]) if s[2] is not None},
            "value_constraint": {s[0]: (s[1] if s[2] is not None else None) for s in IK_SPACES[x]},
            "value_effect": {s[0]: (f"full follow of {s[2]}" if s[2] is not None else "EMPTY SLOT (T27 fills)")
                             for s in IK_SPACES[x]},
        }
        if x == "l":
            spaces[n["prop_space"]]["empty_slots"] = {
                "weapon": {"value": 3, "filled": False, "constraint": "space_weapon", "type": "ARMATURE",
                           "target": "CTRL_weapon", "influence": "driver PROPS[prop] == 3",
                           "note": "not created yet: T27 adds the constraint + driver and moves it into constraints"}}
        ikfk[f"arm_{x}"] = {
            "owner": OWNER, "prop": n["prop_ikfk"], "fk_value": 0, "ik_value": 1,
            "fk": n["fk"], "ik": n["ik"], "pole": n["pole"], "def": n["def"], "mch_ik": n["mch"],
            "shared": {"control": n["shoulder"], "def": f"shoulder_{x}"},
            "fk_to_def": dict(zip(n["fk"], n["def"])),
            "fk_to_mch_ik": dict(zip(n["fk"], n["mch"])),
            "def_constraints": {"fk": "fk_copy", "ik": "ik_copy"},
            "ik_constraint": {"bone": n["mch"][1], "name": "IK", "chain_count": 2, "use_tail": True,
                              "use_stretch": False, "pole_angle_rad": angles[x],
                              "pole_angle_deg": math.degrees(angles[x])},
            "ik_hand_rotation": {"bone": n["mch"][2], "constraint": "IK_rot", "type": "COPY_ROTATION"},
            "ik_space": {"prop": n["prop_space"], "mch": n["space"]},
            "pole_parent": POLE_PARENT, "pole_distance": POLE_DIST,
            "snap": {"fk_from_ik": "for fk, mch in fk_to_mch_ik in chain order: fk world matrix = mch world matrix "
                                   "(update depsgraph between bones), then prop = fk_value",
                     "ik_from_fk": "ik world matrix = DEF/CTRL_hand_fk world matrix; pole world location = "
                                   "lowerarm head + pole_distance * unit(elbow offset from the upperarm-head -> "
                                   "hand-head line, FK result); then prop = ik_value"},
        }
    return controls, spaces, ikfk


def write_manifest(angles):
    p = goblib.DATA / MANIFEST
    with open(p, "r", encoding="utf-8") as f:
        old = json.load(f)
    man = dict(old)
    man["_doc"] = dict(old.get("_doc", {}))
    man["_doc"].update(DOC)
    controls, spaces, ikfk = manifest_entries(angles)
    for sec, new in (("controls", controls), ("spaces", spaces), ("ikfk", ikfk)):
        keep = {k: v for k, v in old.get(sec, {}).items() if not (isinstance(v, dict) and v.get("owner") == OWNER)}
        keep.update(new)
        man[sec] = keep
    return goblib.save_json(MANIFEST, man)


# ---------------------------------------------------------------- self-checks
def fmt_v(v):
    return "(" + ", ".join(f"{a:+.4f}" for a in v) + ")"


def reset_new(arm):
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            pb = arm.pose.bones[name]
            pb.location = (0, 0, 0)
            pb.rotation_euler = (0, 0, 0)
            pb.rotation_quaternion = (1, 0, 0, 0)
            pb.scale = (1, 1, 1)


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


def rel_twist_deg(arm, bone):
    pb = arm.pose.bones[bone]
    par = pb.parent
    rel = par.matrix.inverted() @ pb.matrix
    rest = par.bone.matrix_local.inverted() @ pb.bone.matrix_local
    _, tw = (rest.inverted() @ rel).to_quaternion().to_swing_twist("Y")
    return math.degrees(tw)


def selfchecks(arm, canon):
    props = arm.pose.bones["PROPS"]
    reset_new(arm)
    arm.data.pose_position = "POSE"
    fk = def_vs_canon(arm, canon)
    print(f"[s06b] G6.2 SELFCHECK FK (arm_ik_fk_l={props['arm_ik_fk_l']}, arm_ik_fk_r={props['arm_ik_fk_r']}, all "
          f"CTRL identity): DEF vs canonical rest max abs diff = {fk[0]:.3e} (bone {fk[1]}), threshold 1e-4")
    saved = {x: props[f"arm_ik_fk_{x}"] for x in SIDES}
    for x in SIDES:
        props[f"arm_ik_fk_{x}"] = 1.0
    arm.update_tag()  # ID-property writes from Python send no depsgraph tag; drivers need it to re-evaluate
    ik = def_vs_canon(arm, canon)
    infl = {x: arm.pose.bones[f"upperarm_{x}"].constraints["ik_copy"].influence for x in SIDES}
    print(f"[s06b] G6.2 SELFCHECK IK (arm_ik_fk_l/r = 1, ik_copy influence l={infl['l']:.3f} r={infl['r']:.3f}, all "
          f"CTRL identity): DEF vs canonical rest max abs diff = {ik[0]:.3e} (bone {ik[1]}), threshold 1e-4")
    for x in SIDES:
        props[f"arm_ik_fk_{x}"] = saved[x]
    arm.update_tag()
    bpy.context.view_layer.update()
    for x in SIDES:
        n = names(x)
        pbs = arm.pose.bones
        pbs[n["fk"][0]].rotation_euler = (0, math.radians(90), 0)
        bpy.context.view_layer.update()
        ua_def = rel_twist_deg(arm, f"upperarm_{x}")
        ua_tw = rel_twist_deg(arm, f"upperarm_twist_{x}")
        pbs[n["fk"][0]].rotation_euler = (0, 0, 0)
        pbs[n["fk"][2]].rotation_euler = (0, math.radians(90), 0)
        bpy.context.view_layer.update()
        h_def = rel_twist_deg(arm, f"hand_{x}")
        la_tw = rel_twist_deg(arm, f"lowerarm_twist_{x}")
        pbs[n["fk"][2]].rotation_euler = (0, 0, 0)
        bpy.context.view_layer.update()
        print(f"[s06b] TWIST SELFCHECK {x} (FK): {n['fk'][0]} local Y +90 -> DEF upperarm_{x} twist rel. parent = "
              f"{ua_def:+.3f} deg, upperarm_twist_{x} twist rel. parent = {ua_tw:+.3f} deg (expected ~-45); "
              f"{n['fk'][2]} local Y +90 -> DEF hand_{x} twist rel. parent = {h_def:+.3f} deg, lowerarm_twist_{x} "
              f"twist rel. parent = {la_tw:+.3f} deg (expected ~+45)")
    after = def_vs_canon(arm, canon)
    print(f"[s06b] restored state: arm_ik_fk_l={props['arm_ik_fk_l']}, arm_ik_fk_r={props['arm_ik_fk_r']}, DEF vs "
          f"canonical max abs diff = {after[0]:.3e}")
    return fk[0], ik[0]


def report(arm, drivers, angles):
    ad = arm.data
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            b = ad.bones[name]
            pb = arm.pose.bones[name]
            cons = []
            for c in pb.constraints:
                if c.type == "ARMATURE":
                    cons.append(f"ARMATURE:{c.name}->{','.join(t.subtarget for t in c.targets)}")
                elif c.type == "IK":
                    cons.append(f"IK:{c.name}->{c.subtarget} pole {c.pole_subtarget} angle "
                                f"{math.degrees(c.pole_angle):+.4f} deg chain {c.chain_count} "
                                f"stretch {c.use_stretch}")
                else:
                    cons.append(f"{c.type}:{c.name}->{c.subtarget}")
            print(f"[s06b] BONE {name}: parent={b.parent.name if b.parent else None}, connect={b.use_connect}, "
                  f"head={fmt_v(b.head_local)}, tail={fmt_v(b.tail_local)}, deform={b.use_deform}, "
                  f"coll={[c.name for c in b.collections]}, ik_stretch={pb.ik_stretch:.1f}, "
                  f"constraints={'; '.join(cons) or '-'}")
        for d in [f"shoulder_{x}"] + names(x)["def"]:
            pb = arm.pose.bones[d]
            print(f"[s06b] DEF {d}: " + "; ".join(f"{c.type}:{c.name}<-{c.subtarget} infl {c.influence:.3f}"
                                                  for c in pb.constraints))
        print(f"[s06b] POLE {x}: pole_angle={angles[x]:+.6f} rad ({math.degrees(angles[x]):+.4f} deg), pole head="
              f"{fmt_v(ad.bones[names(x)['pole']].head_local)}")
    for tag, drv in drivers:
        print(f"[s06b] DRIVER {tag}: expr '{drv.expression}', valid={drv.is_valid}, "
              f"simple={drv.is_simple_expression}")


def main():
    print(f"[s06b] Blender {bpy.app.version_string}")
    arm = bpy.data.objects[ARM]
    canon = goblib.load_json("canonical_skeleton.json")
    angles = edit_bones(arm)
    drivers = []
    for x in SIDES:
        drivers += pose_setup(arm, x, angles[x])
    assign_collections(arm)
    report(arm, drivers, angles)
    fk_err, ik_err = selfchecks(arm, canon)
    nondef = [nm for x in SIDES for nm, _, _, _ in new_bone_list(x) if arm.data.bones[nm].use_deform]
    print(f"[s06b] new bones with use_deform True: {nondef}; total bones={len(arm.data.bones)}; collections: "
          + ", ".join(f"{c.name}(visible={c.is_visible}, bones={len(c.bones)})" for c in arm.data.collections_all))
    out = write_manifest(angles)
    print(f"[s06b] MANIFEST {out}")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s06b] OUTPUT {OUT_BLEND}")
    print(f"[s06b] G6.2 FK max_err={fk_err:.3e}, IK max_err={ik_err:.3e}")


if __name__ == "__main__":
    goblib.run_main(main)
