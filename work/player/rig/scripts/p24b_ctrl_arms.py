"""p24b_ctrl_arms.py - T240 (P2.5): left / right arm FK + IK controls and clavicle controls (goblin G6 analogue).

Input : work/player/rig/work/pl_r04a.blend (p24a)
Output: work/player/rig/work/pl_r04b.blend (save copy), work/player/rig/data/ctrl_manifest.json (owner "p24b" entries)

CLI:  blender --background --factory-startup work/player/rig/work/pl_r04a.blend --python p24b_ctrl_arms.py

Copied / adapted from work/goblin_swing/rig/scripts/s06b_ctrl_arms.py (T25), goblin files unchanged. Per side x (l, r;
DEF side suffix _L / _R), all new bones use_deform False, rest / roll copied from the DEF bones:
  CTRL_chest > CTRL_shoulder_x (rest = DEF Clavicle_X) -> DEF Clavicle_X Copy Transforms WORLD/WORLD
  FK : CTRL_shoulder_x > CTRL_upperarm_fk_x > CTRL_lowerarm_fk_x > CTRL_hand_fk_x   (DEF Arm / Forearm / Hand rest)
  IK : CTRL_shoulder_x > MCH_upperarm_ik_x > MCH_lowerarm_ik_x > MCH_hand_ik_x      (DEF rest; MCH lower / hand
       connected - the DEF heads already sit on the parent tails)
       MCH_lowerarm_ik_x: IK -> CTRL_hand_ik_x (chain 2, use_tail, no stretch, pole CTRL_elbow_pole_x, pole_angle from
       the rest chain); ik_stretch 0.  MCH_hand_ik_x: Copy Rotation WORLD/WORLD <- CTRL_hand_ik_x.
       Preferred bend (T240): the DEF chain is almost straight (0.62 deg), so the bend side is set by (a) the pole on
       the goblin side (DEF Forearm head + POLE_DIST * (+Y), elbow goes back / hand forward, G4.9) and (b) an IK hinge
       limit on MCH_lowerarm_ik_x: IK rotation locked on local Y / Z, local X limited to [ELBOW_IK_MIN, ELBOW_IK_MAX]
       (Forearm +X = flexion, hand toward -Y; same sign as goblin G4.4). No edit-bone offset: MCH = DEF rest keeps
       the IK rest exact (G6.2) and the FK <-> IK snap exact.
       MCH_CTRL_hand_ik_x_space (no parent, rest = DEF Hand) > CTRL_hand_ik_x; Armature space constraints (influence =
       driver PROPS["hand_ik_space_x"] == k): world -> MCH_world, root -> CTRL_root, torso -> CTRL_torso; left value 3
       (weapon) is an empty slot filled by p24d (-> CTRL_weapon_r).
       CTRL_torso > CTRL_elbow_pole_x.
  DEF Arm / Forearm / Hand: fk_copy (Copy Transforms WORLD/WORLD, 1) <- FK control, then ik_copy (influence = driver
       PROPS["arm_ik_fk_x"]) <- MCH IK bone.  0 = FK, 1 = IK.  No twist bones (P1 v0 tree).
Self-checks: G6.2 FK (defaults) and IK (arm_ik_fk = 1), DEF vs canonical rest.
"""
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
IN_BLEND = RIG / "work" / "pl_r04a.blend"
OUT_BLEND = RIG / "work" / "pl_r04b.blend"
MANIFEST = RIG / "data" / "ctrl_manifest.json"
CANON = RIG / "data" / "canonical_skeleton.json"
OWNER = "p24b"
ARM = "PL_rig"
SIDES = ("l", "r")
POLE_DIST = 0.30
POLE_DIR = Vector((0.0, 1.0, 0.0))
POLE_LEN = 0.05
POLE_PARENT = "CTRL_torso"
ELBOW_IK_MIN = 0.0      # deg, MCH_lowerarm_ik local X (flexion +)
ELBOW_IK_MAX = 150.0
IK_SPACES = {
    "l": [("world", "space_world", "MCH_world"), ("root", "space_root", "CTRL_root"),
          ("torso", "space_torso", "CTRL_torso"), ("weapon", "space_weapon", None)],
    "r": [("world", "space_world", "MCH_world"), ("root", "space_root", "CTRL_root"),
          ("torso", "space_torso", "CTRL_torso")],
}
CHAIN = (("upperarm", "Arm"), ("lowerarm", "Forearm"), ("hand", "Hand"))


def log(msg):
    print(f"[p24b] {msg}")
    sys.stdout.flush()


def names(x):
    S = x.upper()
    return {
        "shoulder": f"CTRL_shoulder_{x}", "clav": f"Clavicle_{S}",
        "fk": [f"CTRL_{b}_fk_{x}" for b, _ in CHAIN],
        "mch": [f"MCH_{b}_ik_{x}" for b, _ in CHAIN],
        "def": [f"{d}_{S}" for _, d in CHAIN],
        "ik": f"CTRL_hand_ik_{x}", "space": f"MCH_CTRL_hand_ik_{x}_space", "pole": f"CTRL_elbow_pole_{x}",
        "prop_ikfk": f"arm_ik_fk_{x}", "prop_space": f"hand_ik_space_{x}",
    }


def new_bone_list(x):
    """(name, parent, rest source DEF bone or 'pole', use_connect)"""
    n = names(x)
    out = [(n["shoulder"], "CTRL_chest", n["clav"], False)]
    pf, pm = n["shoulder"], n["shoulder"]
    for i, (fk, mch, d) in enumerate(zip(n["fk"], n["mch"], n["def"])):
        out.append((fk, pf, d, False))
        out.append((mch, pm, d, i > 0))
        pf, pm = fk, mch
    out += [(n["space"], None, n["def"][2], False), (n["ik"], n["space"], n["def"][2], False),
            (n["pole"], POLE_PARENT, "pole", False)]
    return out


def rot(axis, lo, hi):
    return {"channel": "rot", "axis": axis, "min": lo, "max": hi}


def loc(axis, lo, hi):
    return {"channel": "loc", "axis": axis, "min": lo, "max": hi}


def sweeps(x):
    n = names(x)
    raise_z = [-35, 135] if x == "l" else [-135, 35]
    return {
        n["shoulder"]: [rot("X", -20, 20), rot("Z", -20, 20)],
        n["fk"][0]: [rot("X", -60, 90), rot("Y", -90, 90), rot("Z", *raise_z)],
        n["fk"][1]: [rot("X", 0, 140)],
        n["fk"][2]: [rot("X", -60, 60), rot("Y", -90, 90), rot("Z", -30, 30)],
        n["ik"]: [loc("X", -0.25, 0.25), loc("Y", -0.20, 0.0), loc("Z", -0.25, 0.25)]
        + [rot(a, -60, 60) for a in "XYZ"],
        n["pole"]: [loc(a, -0.2, 0.2) for a in "XYZ"],
    }


DOC = {
    "p24b_arms": "Arms (p24b): CTRL_shoulder_x (parent CTRL_chest) drives DEF Clavicle_X in FK and IK. FK "
                 "CTRL_upperarm/lowerarm/hand_fk_x and MCH IK chain MCH_upperarm/lowerarm/hand_ik_x copy DEF Arm / "
                 "Forearm / Hand rest and roll (P1 roll convention, chain local X parallel): upperarm X+ = swing "
                 "forward, raise = upperarm_l Z+ / upperarm_r Z-, Y = twist; lowerarm X+ = elbow flexion (hand toward "
                 "-Y, same as goblin); hand X+ = hand toward -Y. DEF Arm/Forearm/Hand: fk_copy then ik_copy "
                 "(influence = arm_ik_fk_x; 0 = FK, 1 = IK). IK: no stretch.",
    "p24b_pole": "CTRL_elbow_pole_x: DEF Forearm head + 0.30 m along +Y (goblin side: elbow back, hand forward), bone "
                 "+Z, roll 0, parent CTRL_torso. pole_angle from the rest chain (IK at identity = DEF rest).",
    "p24b_preferred_bend": "The DEF arm chain is 0.62 deg from straight (elbow ring centre 0.71 mm toward +Y), so the "
                           "IK bend side is fixed by the pole side plus an IK hinge limit on MCH_lowerarm_ik_x: "
                           "lock_ik_y / lock_ik_z, use_ik_limit_x with ik_min_x 0, ik_max_x 150 deg (flexion only). "
                           "No edit-bone offset (the DEF skeleton and MCH rest stay equal: G6.2 and FK <-> IK snap "
                           "exact).",
    "p24b_sweep_ranges": "CTRL_hand_ik_x local Y runs along the arm (rest arm at full reach, shoulder -> wrist 0.332 "
                         "m): loc Y -0.20..0 m (elbow bends, no fold-through); X / Z +-0.25 m; rot +-60; poles +-0.2 m. "
                         "FK: upperarm raise -35..+135 (A-pose rest 45 deg down: -35 keeps the arm off the tunic), "
                         "forward -60..90, twist +-90; lowerarm 0..140; hand X +-60, Y +-90, Z +-30; shoulder +-20.",
    "p24b_ik_modes": "controls.<CTRL>.mode = PROPS value the control is swept in (FK: arm_ik_fk_x = 0, IK hand / pole: "
                     "1). CTRL_shoulder_x is active in both.",
}


def signed_angle(u, v, normal):
    a = u.angle(v)
    if u.cross(v).angle(normal) < 1:
        a = -a
    return a


def pole_angle(base, tip, pole_loc):
    pole_normal = (tip.tail - base.head).cross(pole_loc - base.head)
    projected = pole_normal.cross(base.tail - base.head)
    return signed_angle(base.x_axis, projected, base.tail - base.head)


def edit_bones(arm):
    vl = bpy.context.view_layer
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    angles = {}
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            if ebs.get(name) is not None:
                raise RuntimeError(f"bone {name} already exists (expected input pl_r04a.blend)")
        for name, parent, src, connect in new_bone_list(x):
            eb = ebs.new(name)
            if src == "pole":
                h = ebs[names(x)["def"][1]].head + POLE_DIR * POLE_DIST
                eb.head, eb.tail, eb.roll = h, h + Vector((0, 0, POLE_LEN)), 0.0
            else:
                d = ebs[src]
                eb.head, eb.tail, eb.roll = d.head.copy(), d.tail.copy(), d.roll
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
    copy_tf(arm, pbs[n["clav"]], "CTRL_copy", n["shoulder"])
    for m in n["mch"][:2]:
        pbs[m].ik_stretch = 0.0
    lo = pbs[n["mch"][1]]
    lo.lock_ik_y = lo.lock_ik_z = True
    lo.use_ik_limit_x = True
    lo.ik_min_x = math.radians(ELBOW_IK_MIN)
    lo.ik_max_x = math.radians(ELBOW_IK_MAX)
    ik = lo.constraints.new("IK")
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
    sp = pbs[n["space"]]
    cur = props[n["prop_space"]]
    for k, (_, cname, tgt) in enumerate(IK_SPACES[x]):
        if tgt is None:
            continue
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


def manifest_entries(angles):
    controls, spaces, ikfk = {}, {}, {}
    for x in SIDES:
        n = names(x)
        sw = sweeps(x)
        for name in [n["shoulder"]] + n["fk"] + [n["ik"], n["pole"]]:
            e = {"owner": OWNER, "side": x.upper(), "part": "arm", "rotation_mode": "XYZ", "sweep": sw[name]}
            if name in n["fk"]:
                e["mode"] = {n["prop_ikfk"]: 0}
            elif name in (n["ik"], n["pole"]):
                e["mode"] = {n["prop_ikfk"]: 1}
            if name == n["shoulder"]:
                e["drives"] = n["clav"]
            controls[name] = e
        vals = [s[0] for s in IK_SPACES[x]]
        cons = [s[1] for s in IK_SPACES[x] if s[2] is not None]
        spaces[n["prop_space"]] = {
            "owner": OWNER, "control": n["ik"], "prop": n["prop_space"], "values": vals, "mch": n["space"],
            "constraints": cons, "constraint_type": {c: "ARMATURE" for c in cons},
            "targets": {s[1]: s[2] for s in IK_SPACES[x] if s[2] is not None},
            "influence": {s[1]: f"driver PROPS[prop] == {k}" for k, s in enumerate(IK_SPACES[x]) if s[2] is not None},
            "value_constraint": {s[0]: (s[1] if s[2] is not None else None) for s in IK_SPACES[x]},
            "value_effect": {s[0]: (f"full follow of {s[2]}" if s[2] is not None else "EMPTY SLOT (p24d fills)")
                             for s in IK_SPACES[x]}}
        if x == "l":
            spaces[n["prop_space"]]["empty_slots"] = {
                "weapon": {"value": 3, "filled": False, "constraint": "space_weapon", "type": "ARMATURE",
                           "target": "CTRL_weapon_r", "influence": "driver PROPS[prop] == 3"}}
        ikfk[f"arm_{x}"] = {
            "owner": OWNER, "prop": n["prop_ikfk"], "fk_value": 0, "ik_value": 1,
            "fk": n["fk"], "ik": n["ik"], "pole": n["pole"], "def": n["def"], "mch_ik": n["mch"],
            "shared": {"control": n["shoulder"], "def": n["clav"]},
            "fk_to_def": dict(zip(n["fk"], n["def"])), "fk_to_mch_ik": dict(zip(n["fk"], n["mch"])),
            "def_constraints": {"fk": "fk_copy", "ik": "ik_copy"},
            "ik_constraint": {"bone": n["mch"][1], "name": "IK", "chain_count": 2, "use_tail": True,
                              "use_stretch": False, "pole_angle_rad": angles[x],
                              "pole_angle_deg": math.degrees(angles[x]),
                              "hinge_limit": {"lock_ik_y": True, "lock_ik_z": True, "ik_x_deg": [ELBOW_IK_MIN,
                                                                                               ELBOW_IK_MAX]}},
            "ik_hand_rotation": {"bone": n["mch"][2], "constraint": "IK_rot", "type": "COPY_ROTATION"},
            "ik_space": {"prop": n["prop_space"], "mch": n["space"]},
            "pole_parent": POLE_PARENT, "pole_distance": POLE_DIST,
            "flexion_sign": {"bone": n["fk"][1], "axis": "X", "sign": 1},
            "snap": {"fk_from_ik": "for fk, mch in fk_to_mch_ik in chain order: fk world matrix = mch world matrix, "
                                   "then prop = fk_value",
                     "ik_from_fk": "ik world matrix = CTRL_hand_fk world matrix; pole placed from the FK upper bone X "
                                   "axis and corrected until the MCH elbow matches the FK elbow; prop = ik_value"}}
    return controls, spaces, ikfk


def write_manifest(angles):
    old = json.loads(MANIFEST.read_text(encoding="utf-8"))
    man = dict(old)
    man["_doc"] = dict(old.get("_doc", {}))
    man["_doc"].update(DOC)
    for sec, new in zip(("controls", "spaces", "ikfk"), manifest_entries(angles)):
        keep = {k: v for k, v in old.get(sec, {}).items() if not (isinstance(v, dict) and v.get("owner") == OWNER)}
        keep.update(new)
        man[sec] = keep
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


def bend_test(arm):
    """IK on, hand IK moved 0.10 m toward the shoulder (local -Y): elbow offset direction from the shoulder -> hand line."""
    pbs = arm.pose.bones
    props = pbs["PROPS"]
    for x in SIDES:
        n = names(x)
        props[n["prop_ikfk"]] = 1.0
        arm.update_tag()
        pbs[n["ik"]].location = (0.0, -0.10, 0.0)
        bpy.context.view_layer.update()
        a, b = pbs[n["def"][0]], pbs[n["def"][1]]
        r, m, t = a.head, b.head, b.tail
        d = (t - r).normalized()
        off = (m - r) - d * (m - r).dot(d)
        ang = math.degrees((a.tail - a.head).angle(b.tail - b.head))
        log(f"BENDTEST {x}: hand IK loc Y -0.10 -> elbow bend {ang:.2f} deg, elbow offset from the shoulder-wrist line "
            f"{off.length * 1000:.1f} mm dir ({off.x / off.length:+.3f}, {off.y / off.length:+.3f}, "
            f"{off.z / off.length:+.3f}) (expected +Y = back)")
        pbs[n["ik"]].location = (0.0, 0.0, 0.0)
        props[n["prop_ikfk"]] = 0.0
        arm.update_tag()
        bpy.context.view_layer.update()


def main():
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    arm = bpy.data.objects[ARM]
    canon = json.loads(CANON.read_text(encoding="utf-8"))
    angles = edit_bones(arm)
    drivers = []
    for x in SIDES:
        drivers += pose_setup(arm, x, angles[x])
    assign_collections(arm)
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            b = arm.data.bones[name]
            pb = arm.pose.bones[name]
            cons = []
            for c in pb.constraints:
                if c.type == "ARMATURE":
                    cons.append(f"ARMATURE:{c.name}->{','.join(t.subtarget for t in c.targets)}")
                elif c.type == "IK":
                    cons.append(f"IK->{c.subtarget} pole {c.pole_subtarget} angle {math.degrees(c.pole_angle):+.3f}")
                else:
                    cons.append(f"{c.type}:{c.name}->{c.subtarget}")
            log(f"BONE {name}: parent={b.parent.name if b.parent else None}, connect={b.use_connect}, "
                f"head={tuple(round(v, 4) for v in b.head_local)}, constraints={'; '.join(cons) or '-'}")
        log(f"POLE {x}: pole_angle {math.degrees(angles[x]):+.4f} deg, pole head "
            f"{tuple(round(v, 4) for v in arm.data.bones[names(x)['pole']].head_local)}; hinge limit "
            f"MCH_lowerarm_ik_{x} X [{ELBOW_IK_MIN}, {ELBOW_IK_MAX}] deg, Y/Z locked")
    for tag, drv in drivers:
        log(f"DRIVER {tag}: expr '{drv.expression}', valid={drv.is_valid}")
    arm.data.pose_position = "POSE"
    props = arm.pose.bones["PROPS"]
    fk = def_vs_canon(arm, canon)
    for x in SIDES:
        props[f"arm_ik_fk_{x}"] = 1.0
    arm.update_tag()
    ik = def_vs_canon(arm, canon)
    for x in SIDES:
        props[f"arm_ik_fk_{x}"] = 0.0
    arm.update_tag()
    bpy.context.view_layer.update()
    log(f"G6.2 SELFCHECK FK {fk[0]:.3e} ({fk[1]}), IK {ik[0]:.3e} ({ik[1]}), threshold 1e-4")
    bend_test(arm)
    out = write_manifest(angles)
    log(f"MANIFEST {out}")
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
