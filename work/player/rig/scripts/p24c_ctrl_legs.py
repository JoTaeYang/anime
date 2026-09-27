"""p24c_ctrl_legs.py - T240 (P2.5): left / right leg FK + IK controls, foot roll pivots and planted clamps.

Input : work/player/rig/work/pl_r04b.blend (p24a torso / head / PROPS, p24b arms), work/player/rig/data/parts.json
Output: work/player/rig/work/pl_r04c.blend (save copy), work/player/rig/data/ctrl_manifest.json (owner "p24c" entries,
        clamps, clamps_reference)

CLI:  blender --background --factory-startup work/player/rig/work/pl_r04b.blend --python p24c_ctrl_legs.py

Copied / adapted from work/goblin_swing/rig/scripts/s06c_ctrl_legs.py (T26 / T26b-e: pivots, pivot search, planted
sweep with the cumulative-slip rule, clamps into drivers and PROPS UI), goblin files unchanged. Per side x (l, r):
  FK : CTRL_pelvis > CTRL_thigh_fk_x > CTRL_calf_fk_x > CTRL_foot_fk_x          (DEF Thigh / Calf / Foot rest)
  IK : CTRL_pelvis > MCH_thigh_ik_x > MCH_calf_ik_x > MCH_foot_ik_x               (DEF rest; MCH calf / foot connected)
       MCH_calf_ik_x: IK -> MCH_roll_foot_x (chain 2, use_tail, no stretch, pole CTRL_knee_pole_x, pole_angle from the
       rest chain); ik_stretch 0.  MCH_foot_ik_x: Copy Rotation WORLD/WORLD <- MCH_roll_foot_x.
       Preferred bend + calf sign (T240): the DEF leg chain is 0.42 deg from straight and its knee ring centre sits
       0.28 mm BEHIND the hip-ankle line, and the player Calf roll is opposite to goblin G4.4 (Calf +X = foot forward,
       so knee flexion = Calf -X). The bend side is set by the pole on the goblin side (DEF Calf head + POLE_DIST *
       (-Y), knee forward) and an IK hinge limit on MCH_calf_ik_x: IK rotation locked on Y / Z, X limited to
       [KNEE_IK_MIN, KNEE_IK_MAX] = [-150, 0] deg (flexion only, sign adapted), and a goblin-style small offset on
       the MCH chain only: MCH knee placed in the hinge plane KNEE_BEND (0.5 mm) forward of the hip-ankle line (a
       straight chain compressed along its axis does not start to bend); DEF-equivalent helpers MCH_thigh_ik_def_x /
       MCH_calf_ik_def_x (children of the MCH bones, rest = DEF Thigh / Calf) are the ik_copy targets, so DEF rest and
       the FK <- IK snap stay exact (the IK knee joint may open by up to the offset length, < 1 mm).
  CTRL_root > CTRL_foot_ik_x (head = ankle = DEF Foot head, points world -Y, local Z = world +Z)
       > MCH_roll_heel_x > MCH_roll_toe_x > MCH_roll_bank_in_x > MCH_roll_bank_out_x > MCH_roll_foot_x (rest = DEF Foot).
       Pivot start points from the shoe_x part of PL_mesh (no pivots.json on the player): heel = (ankle x, rearmost
       shoe y, 0), toe = (ankle x, frontmost shoe y, 0), bank edges = innermost / outermost sole vertex (z <= min +
       SOLE_BAND); then the goblin T26d grid search (height 0..20 mm, 1 mm steps across the rotation axis) keeps
       the pivot with the longest planted range. Driver formulas as goblin (heel X = max(-roll, 0), toe X =
       -max(roll, 0), heel / toe Z = twist, bank_in / bank_out Y = s * min / max(bank, 0)).
  MCH_CTRL_knee_pole_x_space (no parent) > CTRL_knee_pole_x; knee_pole_space foot -> CTRL_foot_ik_x, world -> CTRL_root.
  DEF Thigh / Calf / Foot: fk_copy (1) <- FK control, ik_copy (influence = driver PROPS["leg_ik_fk_x"]) <- helper /
  MCH_foot_ik_x.
Skirt follows thigh (T253, spec d-02 §8 b): MCH_knee_probe_x (Copy Location <- DEF Calf_X head, parent MCH_pelvis),
  MCH_skirt_hinge_T (parent MCH_pelvis, head = DEF Skirt_T_01 head, rotation X driven: skirt_follow * weight * push-only
  thigh tilt); DEF Skirt_T_01 follow_copy <- hinge (Copy Transforms LOCAL_OWNER_ORIENT -> LOCAL). Design +
  expressions: data/skirt_follow.json.
Clamps: stances CTRL_torso loc Y 0 / READY_TORSO_Y / -15 mm, leg IK, other controls identity, foot IK at rest; each
  roll property alone in 1 deg steps; planted = IK reach error <= 1 mm, -1 mm <= lowest shoe vertex z <= +3 mm, and for
  roll / bank cumulative contact slip <= 5 mm (goblin T26e rule). The READY_TORSO_Y stance (the ready pose) range is
  the final clamp (drivers + PROPS UI); the others are reference.
"""
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
IN_BLEND = RIG / "work" / "pl_r04b.blend"
OUT_BLEND = RIG / "work" / "pl_r04c.blend"
MANIFEST = RIG / "data" / "ctrl_manifest.json"
CANON = RIG / "data" / "canonical_skeleton.json"
PARTS_JSON = RIG / "data" / "parts.json"
OWNER = "p24c"
ARM = "PL_rig"
MESH = "PL_mesh"
SIDES = ("l", "r")
CHAIN = (("thigh", "Thigh"), ("calf", "Calf"), ("foot", "Foot"))
POLE_DIST = 0.30
POLE_DIR = Vector((0.0, -1.0, 0.0))
POLE_LEN = 0.05
FOOT_IK_LEN = 0.15
PIVOT_LEN = 0.05
KNEE_IK_MIN = -150.0      # deg, MCH_calf_ik local X (flexion = -X on the player, sign opposite to goblin)
KNEE_IK_MAX = 0.0
KNEE_BEND = 0.0005      # m, in-plane forward offset of the MCH IK knee from the hip-ankle line (preferred bend)
SOLE_BAND = 0.01
REACH_TOL = 0.001
Z_MIN = -0.001
Z_MAX = 0.003
SLIP_MAX = 0.005
CONTACT_BAND = 0.001
SLIP_PROPS = ("foot_roll", "foot_bank")
READY_TORSO_Y = -0.004    # m, CTRL_torso local Y (= world +Z) of the ready pose (addon READY_TORSO_Y)
STANCES_M = (0.0, READY_TORSO_Y, -0.015)
CLAMP_STANCE_M = READY_TORSO_Y
DEG = 0.017453292519943295
START_CLAMPS = {"foot_roll": (-45, 60), "foot_bank": (-30, 30), "heel_twist": (-45, 45), "toe_twist": (-45, 45)}
POLE_SPACES = [("foot", "space_foot", "foot_ik"), ("world", "space_world", "CTRL_root")]
PIVOT_SEARCH = {
    "heel": ("lat", "fwd", (-0.02, 0.30), range(-1, START_CLAMPS["foot_roll"][0] - 1, -1)),
    "toe": ("lat", "fwd", (-0.30, 0.02), range(1, START_CLAMPS["foot_roll"][1] + 1)),
    "bank_in": ("fwd", "outward", (-0.03, 0.25), range(-1, START_CLAMPS["foot_bank"][0] - 1, -1)),
    "bank_out": ("fwd", "outward", (-0.25, 0.03), range(1, START_CLAMPS["foot_bank"][1] + 1)),
}
SEARCH_STEP = 0.001
SEARCH_H = (0.0, 0.02)
SEARCH_REACH_TOL = 0.0005
SEARCH_Z_MARGIN = 0.00002
SEARCH_SLIP_MARGIN = 0.00005


def log(msg):
    print(f"[p24c] {msg}")
    sys.stdout.flush()


def fmt_v(v):
    return "(" + ", ".join(f"{a:+.4f}" for a in v) + ")"


def names(x):
    S = x.upper()
    return {
        "fk": [f"CTRL_{b}_fk_{x}" for b, _ in CHAIN],
        "mch": [f"MCH_{b}_ik_{x}" for b, _ in CHAIN],
        "hlp": [f"MCH_{b}_ik_def_{x}" for b, _ in CHAIN[:2]],
        "def": [f"{d}_{S}" for _, d in CHAIN],
        "ik": f"CTRL_foot_ik_{x}", "heel": f"MCH_roll_heel_{x}", "toe": f"MCH_roll_toe_{x}",
        "bank_in": f"MCH_roll_bank_in_{x}", "bank_out": f"MCH_roll_bank_out_{x}", "roll_foot": f"MCH_roll_foot_{x}",
        "space": f"MCH_CTRL_knee_pole_{x}_space", "pole": f"CTRL_knee_pole_{x}",
        "prop_ikfk": f"leg_ik_fk_{x}", "prop_space": f"knee_pole_space_{x}",
        "roll_props": {k: f"{k}_{x}" for k in START_CLAMPS},
    }


def new_bone_list(x):
    n = names(x)
    out = []
    pf, pm = "CTRL_pelvis", "CTRL_pelvis"
    for i, (fk, mch, d) in enumerate(zip(n["fk"], n["mch"], n["def"])):
        out.append((fk, pf, d, False))
        out.append((mch, pm, d, i > 0))
        pf, pm = fk, mch
    out += [(h, m, d, False) for h, m, d in zip(n["hlp"], n["mch"][:2], n["def"][:2])]
    out += [(n["ik"], "CTRL_root", "foot_ik", False), (n["heel"], n["ik"], "heel", False),
            (n["toe"], n["heel"], "toe", False), (n["bank_in"], n["toe"], "bank_in", False),
            (n["bank_out"], n["bank_in"], "bank_out", False), (n["roll_foot"], n["bank_out"], n["def"][2], False),
            (n["space"], None, "pole", False), (n["pole"], n["space"], "pole", False)]
    return out


def rot(axis, lo, hi):
    return {"channel": "rot", "axis": axis, "min": lo, "max": hi}


def loc(axis, lo, hi):
    return {"channel": "loc", "axis": axis, "min": lo, "max": hi}


def sweeps(x):
    n = names(x)
    abd = [-15, 45] if x == "l" else [-45, 15]
    return {
        n["fk"][0]: [rot("X", -45, 90), rot("Y", -45, 45), rot("Z", *abd)],
        n["fk"][1]: [rot("X", -130, 0)],
        n["fk"][2]: [rot("X", -30, 30), rot("Z", -30, 30)],
        n["ik"]: [loc("X", -0.10, 0.10), loc("Y", -0.18, 0.18), loc("Z", 0.0, 0.15)] + [rot(a, -45, 45) for a in "XYZ"],
        n["pole"]: [loc(a, -0.2, 0.2) for a in "XYZ"],
    }


DOC = {
    "p24c_legs": "Legs (p24c): FK CTRL_thigh/calf/foot_fk_x (thigh parent CTRL_pelvis) and MCH IK chain copy DEF Thigh "
                 "/ Calf / Foot rest and roll (P1 roll convention). Player signs (opposite to goblin G4.4): thigh X+ = "
                 "hip flexion (goblin X-), abduction thigh_l Z+ / thigh_r Z- (goblin Z- / Z+), calf X- = knee flexion "
                 "(goblin X+); foot X / Z +-30. DEF: fk_copy then ik_copy (influence = leg_ik_fk_x; 0 = FK, 1 = IK, "
                 "default 1). IK: no stretch.",
    "p24c_preferred_bend": "DEF leg chain 0.42 deg from straight with the knee ring centre 0.28 mm behind the hip-ankle "
                           "line (wrong side for IK). Bend side = pole on the goblin side (DEF Calf head + 0.30 m "
                           "along -Y, knee forward) + IK hinge limit on MCH_calf_ik_x: lock_ik_y / lock_ik_z, "
                           "use_ik_limit_x, ik_min_x -150, ik_max_x 0 deg (flexion only; the sign is adapted to the "
                           "player Calf roll) + MCH-only offset: MCH knee 0.5 mm forward of the hip-ankle line in the "
                           "hinge plane (ikfk.leg_x.preferred_bend); DEF follows the helpers MCH_thigh/calf_ik_def_x "
                           "(rest = DEF), canonical_skeleton.json unchanged.",
    "p24c_foot_ik": "CTRL_foot_ik_x (parent CTRL_root): head = ankle (DEF Foot head), points world -Y, local Z = world "
                    "+Z: loc X = world -X, loc Y = forward, loc Z = up, rot X+ = toes up about the ankle.",
    "p24c_roll": "MCH_roll_heel_x > MCH_roll_toe_x > MCH_roll_bank_in_x > MCH_roll_bank_out_x > MCH_roll_foot_x (rest "
                 "= DEF Foot) = IK target + foot rotation. Pivot axes: Y = heel -> toe horizontal, Z = up. foot_roll - "
                 "= toes up on the heel, + = heel up on the toe; foot_bank + = onto the outer edge, - = inner edge; "
                 "twists + = counter-clockwise seen from above. Degrees; driver expressions clamp to clamps[prop].",
    "p24c_pole": "CTRL_knee_pole_x: DEF Calf head + 0.30 m along -Y (knee forward), child of MCH_CTRL_knee_pole_x_space; "
                 "knee_pole_space foot = CTRL_foot_ik_x, world = CTRL_root (as goblin).",
    "p24c_clamps": "clamps.<prop> = [min, max] deg: planted range at the ready stance (CTRL_torso loc Y = "
                   "READY_TORSO_Y), leg IK, other controls identity, foot IK at rest, each property alone in 1 deg "
                   "steps; planted = IK reach error <= 1 mm AND lowest shoe vertex z in [-1, +3] mm AND (roll / bank) "
                   "cumulative contact slip <= 5 mm (goblin T26e: contact set of step k = shoe verts with z <= lowest z "
                   "+ 1 mm, increment = mean XY displacement of that set to step k+1). Twist: slip reported only. "
                   "Stances 0 and -15 mm: clamps_reference only.",
    "p24c_skirt_follow": "T253 (spec d-02 §8 b): DEF Skirt_T_01 (8 panels) copy the local rotation of "
                         "MCH_skirt_hinge_T (child of MCH_pelvis) in their own orientation; hinge rotation X = driver skirt_follow * weight * "
                         "max(thigh tilt toward the panel - rest tilt, 0) from MCH_knee_probe_x (Copy Location <- DEF "
                         "Calf_X head). PROPS skirt_follow (p24a, default 0.5). Full design: data/skirt_follow.json.",
    "p24c_sweep_ranges": "CTRL_foot_ik_x loc Z 0..+0.15 m (ankle 0.107 -> 0.257 m stays 0.168 m below the hip "
                         "0.425 m), Y +-0.18, X +-0.10, rot +-45; poles +-0.2. FK thigh X -45..90, Y +-45, abduction "
                         "-15..45 (sign per side), calf -130..0, foot +-30.",
}


# ---------------------------------------------------------------- geometry
def shoe_vertex_ids(mesh_obj, part_id):
    me = mesh_obj.data
    pid = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid)
    ids = set()
    for p in me.polygons:
        if pid[p.index] == part_id:
            ids.update(p.vertices)
    return np.array(sorted(ids), dtype=np.int64)


def foot_frames(arm, mesh_obj, parts):
    me = mesh_obj.data
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    out = {}
    for x in SIDES:
        foot = arm.data.bones[f"Foot_{x.upper()}"]
        a = foot.head_local
        d = foot.tail_local - foot.head_local
        fwd = Vector((d.x, d.y, 0.0)).normalized()
        outward = Vector((-fwd.y, fwd.x, 0.0))
        if (outward.x > 0) != (x == "l"):
            outward = -outward
        ids = shoe_vertex_ids(mesh_obj, parts[f"shoe_{x}"])
        sc = co[ids]
        zmin = float(sc[:, 2].min())
        sole = sc[sc[:, 2] <= zmin + SOLE_BAND]
        pf = sc[:, :2] @ np.array([fwd.x, fwd.y])
        a2 = np.array([a.x, a.y])
        heel = a2 + np.array([fwd.x, fwd.y]) * (float(pf.min()) - float(a2 @ np.array([fwd.x, fwd.y])))
        tip = a2 + np.array([fwd.x, fwd.y]) * (float(pf.max()) - float(a2 @ np.array([fwd.x, fwd.y])))
        s = sole[:, :2] @ np.array([outward.x, outward.y])
        v_out, v_in = sole[int(np.argmax(s))], sole[int(np.argmin(s))]
        out[x] = {"fwd": fwd, "outward": outward, "shoe_ids": ids, "shoe_zmin_rest": zmin, "sole_verts": int(len(sole)),
                  "heel": Vector((heel[0], heel[1], 0.0)), "toe": Vector((tip[0], tip[1], 0.0)),
                  "bank_in": Vector((float(v_in[0]), float(v_in[1]), 0.0)),
                  "bank_out": Vector((float(v_out[0]), float(v_out[1]), 0.0)), "shoe_co": sc}
    return out


def pivot_angle_deg(kind, v, bank_s):
    if kind == "heel":
        return max(-v, 0.0)
    if kind == "toe":
        return -max(v, 0.0)
    if kind == "bank_in":
        return bank_s * min(v, 0.0)
    return bank_s * max(v, 0.0)


def search_pivots(arm, frames):
    """goblin s06c T26d search, unchanged apart from the player bone names and ranges."""
    ad = arm.data
    up = np.array([0.0, 0.0, 1.0])
    info = {}
    for x in SIDES:
        S = x.upper()
        f = frames[x]
        fwd = np.array(f["fwd"])
        outward = np.array(f["outward"])
        lat = np.cross(fwd, up)
        bank_s = 1.0 if lat @ outward > 0 else -1.0
        dirs = {"fwd": fwd, "lat": lat, "outward": outward}
        V = f["shoe_co"]
        z_rest = float(V[:, 2].min())
        hip = np.array(ad.bones[f"Thigh_{S}"].head_local) + np.array([0.0, 0.0, CLAMP_STANCE_M])
        ankle = np.array(ad.bones[f"Foot_{S}"].head_local)
        leg = ad.bones[f"Thigh_{S}"].length + ad.bones[f"Calf_{S}"].length
        f["before"] = {k: f[k].copy() for k in PIVOT_SEARCH}
        info[x] = {"bank_sign": bank_s, "leg_len": leg}
        for kind, (akey, dkey, (t0, t1), values) in PIVOT_SEARCH.items():
            a, b = dirs[akey], dirs[dkey]
            ts = np.arange(t0, t1 + 1e-9, SEARCH_STEP)
            hs = np.arange(SEARCH_H[0], SEARCH_H[1] + 1e-9, SEARCH_STEP)
            T, H = np.meshgrid(ts, hs, indexing="ij")
            T, H = T.reshape(-1), H.reshape(-1)
            P = np.array(f[kind])[None, :] + T[:, None] * b[None, :] + H[:, None] * up[None, :]
            D = V[None, :, :] - P[:, None, :]
            axD = np.cross(np.broadcast_to(a, D.shape), D)
            DA = ankle[None, :] - P
            axA = np.cross(np.broadcast_to(a, DA.shape), DA)
            aA = DA @ a
            count = np.zeros(len(P), dtype=np.int64)
            alive = np.ones(len(P), dtype=bool)
            maxdev = np.zeros(len(P))
            maxslip = np.zeros(len(P))
            rows = np.arange(len(P))
            aD = D @ a
            for v in values:
                th = math.radians(pivot_angle_deg(kind, float(v), bank_s))
                c, sn = math.cos(th), math.sin(th)
                zr = D[..., 2] * c + axD[..., 2] * sn
                imin = zr.argmin(axis=1)
                zmin = P[:, 2] + zr[rows, imin]
                d, axd, ad_ = D[rows, imin], axD[rows, imin], aD[rows, imin]
                low = P + d * c + axd * sn + np.outer(ad_, a) * (1.0 - c)
                slip = np.linalg.norm(low[:, :2] - V[imin][:, :2], axis=1)
                A2 = P + DA * c + axA * sn + np.outer(aA, a) * (1.0 - c)
                ok = ((zmin >= Z_MIN + SEARCH_Z_MARGIN) & (zmin <= Z_MAX - SEARCH_Z_MARGIN)
                      & (slip <= SLIP_MAX - SEARCH_SLIP_MARGIN)
                      & (np.linalg.norm(A2 - hip, axis=1) <= leg + SEARCH_REACH_TOL))
                alive &= ok
                count += alive
                maxdev = np.where(alive, np.maximum(maxdev, np.abs(zmin - z_rest)), maxdev)
                maxslip = np.where(alive, np.maximum(maxslip, slip), maxslip)
            dist = np.abs(T) + H
            k = int(np.lexsort((dist, maxslip, -count))[0])
            k0 = int(np.argmin(np.abs(T) + np.abs(H)))
            f[kind] = Vector(P[k].tolist())
            info[x][kind] = {"t": float(T[k]), "h": float(H[k]), "range": int(count[k]), "maxdev": float(maxdev[k]),
                             "maxslip": float(maxslip[k]), "n_full": int((count == len(values)).sum()),
                             "cand": int(len(P)), "max_range": len(values), "range_at_start": int(count[k0])}
    return info


def signed_angle(u, v, normal):
    a = u.angle(v)
    if u.cross(v).angle(normal) < 1:
        a = -a
    return a


def pole_angle(base, tip, pole_loc):
    pole_normal = (tip.tail - base.head).cross(pole_loc - base.head)
    projected = pole_normal.cross(base.tail - base.head)
    return signed_angle(base.x_axis, projected, base.tail - base.head)


# ---------------------------------------------------------------- build
def edit_bones(arm, frames):
    vl = bpy.context.view_layer
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    angles, bank_sign, bend = {}, {}, {}
    up = Vector((0.0, 0.0, 1.0))
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            if ebs.get(name) is not None:
                raise RuntimeError(f"bone {name} already exists (expected input pl_r04b.blend)")
        f = frames[x]
        n = names(x)
        for name, parent, src, connect in new_bone_list(x):
            eb = ebs.new(name)
            if src == "foot_ik":
                h = ebs[n["def"][2]].head.copy()
                eb.head, eb.tail = h, h + Vector((0.0, -FOOT_IK_LEN, 0.0))
                eb.align_roll(up)
            elif src in ("heel", "toe", "bank_in", "bank_out"):
                h = f[src]
                eb.head, eb.tail = h, h + f["fwd"] * PIVOT_LEN
                eb.align_roll(up)
            elif src == "pole":
                h = ebs[n["def"][1]].head + POLE_DIR * POLE_DIST
                eb.head, eb.tail, eb.roll = h, h + Vector((0.0, 0.0, POLE_LEN)), 0.0
            else:
                d = ebs[src]
                eb.head, eb.tail, eb.roll = d.head.copy(), d.tail.copy(), d.roll
            eb.use_deform = False
            eb.parent = ebs[parent] if parent else None
            eb.use_connect = bool(connect) and parent is not None
        bend[x] = preferred_bend(ebs, n)
        angles[x] = pole_angle(ebs[n["mch"][0]], ebs[n["mch"][1]], ebs[n["pole"]].head)
        bank_sign[x] = 1.0 if ebs[n["bank_out"]].x_axis.dot(f["outward"]) > 0 else -1.0
    bpy.ops.object.mode_set(mode="OBJECT")
    return angles, bank_sign, bend


def roll_from_x(eb, xvec):
    """goblin s04_skeleton.roll_from_x: roll so that the bone's local X = xvec made perpendicular to the bone."""
    y = (eb.tail - eb.head).normalized()
    xv = (xvec - y * xvec.dot(y)).normalized()
    eb.align_roll(xv.cross(y))


def preferred_bend(ebs, n):
    """MCH IK knee J' = point of the hip-ankle line at the DEF knee station + KNEE_BEND along the in-plane forward
    direction f = unit(axis x DEF Thigh local X) (toward -Y): the MCH chain bends exactly in the hinge plane; MCH thigh tail / calf head = J'; MCH rolls keep the
    DEF local X. The helper bones (rest = DEF Thigh / Calf, children of the MCH bones) keep DEF rest exact."""
    th, ca = ebs[n["mch"][0]], ebs[n["mch"][1]]
    X = ebs[n["def"][0]].x_axis.copy()
    hip, J, ank = th.head.copy(), th.tail.copy(), ca.tail.copy()
    a = (ank - hip).normalized()
    f = a.cross(X).normalized()
    if f.dot(Vector((0.0, -1.0, 0.0))) < 0:
        f = -f
    o = (J - hip) - a * (J - hip).dot(a)
    o_f = o.dot(f)
    Jp = hip + a * (J - hip).dot(a) + f * KNEE_BEND     # in the hinge plane (out-of-plane DEF offset removed)
    delta = Jp - J
    th.tail = Jp
    ca.head = Jp
    for eb, d in ((th, n["def"][0]), (ca, n["def"][1])):
        roll_from_x(eb, ebs[d].x_axis.copy())
    return {"in_plane_offset_def_mm": o_f * 1000.0, "out_of_plane_offset_def_mm": o.dot(X) * 1000.0,
            "delta_mm": [v * 1000.0 for v in delta], "delta_len_mm": delta.length * 1000.0,
            "mch_bend_deg": math.degrees((Jp - hip).angle(ank - Jp)), "forward_dir": list(f)}


def new_driver(fc, arm, prop, expr):
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


def clamp_expr(lo, hi):
    return f"min(max(v, {float(lo)}), {float(hi)})"


def roll_exprs(bank_s, cl):
    c = {k: clamp_expr(*cl[k]) for k in cl}
    return {
        ("heel", 0): ("foot_roll", f"max(-{c['foot_roll']}, 0.0) * {DEG}"),
        ("heel", 2): ("heel_twist", f"{c['heel_twist']} * {DEG}"),
        ("toe", 0): ("foot_roll", f"-max({c['foot_roll']}, 0.0) * {DEG}"),
        ("toe", 2): ("toe_twist", f"{c['toe_twist']} * {DEG}"),
        ("bank_in", 1): ("foot_bank", f"{bank_s} * min({c['foot_bank']}, 0.0) * {DEG}"),
        ("bank_out", 1): ("foot_bank", f"{bank_s} * max({c['foot_bank']}, 0.0) * {DEG}"),
    }


def pose_setup(arm, x, angle, bank_s):
    pbs = arm.pose.bones
    props = pbs["PROPS"]
    n = names(x)
    drivers = {}
    for name in n["fk"] + [n["ik"], n["pole"]]:
        pb = pbs[name]
        pb.rotation_mode = "XYZ"
        pb.lock_scale = (True, True, True)
        if name in n["fk"]:
            pb.lock_location = (True, True, True)
        if name == n["pole"]:
            pb.lock_rotation = (True, True, True)
    for key in ("heel", "toe", "bank_in", "bank_out"):
        pbs[n[key]].rotation_mode = "XYZ"
    for m in n["mch"][:2]:
        pbs[m].ik_stretch = 0.0
    ca = pbs[n["mch"][1]]
    ca.lock_ik_y = ca.lock_ik_z = True
    ca.use_ik_limit_x = True
    ca.ik_min_x = math.radians(KNEE_IK_MIN)
    ca.ik_max_x = math.radians(KNEE_IK_MAX)
    ik = ca.constraints.new("IK")
    ik.name = "IK"
    ik.target = arm
    ik.subtarget = n["roll_foot"]
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
    cr.subtarget = n["roll_foot"]
    cr.target_space = "WORLD"
    cr.owner_space = "WORLD"
    cr.mix_mode = "REPLACE"
    for (key, idx), (pk, expr) in roll_exprs(bank_s, START_CLAMPS).items():
        fc = pbs[n[key]].driver_add("rotation_euler", idx)
        drivers[f"{n[key]}.rotation_euler[{idx}]"] = (new_driver(fc, arm, n["roll_props"][pk], expr), key, idx, pk)
    sp = pbs[n["space"]]
    cur = props[n["prop_space"]]
    for k, (_, cname, tgt) in enumerate(POLE_SPACES):
        c = sp.constraints.new("ARMATURE")
        c.name = cname
        c.use_deform_preserve_volume = False
        c.use_bone_envelopes = False
        c.use_current_location = False
        t = c.targets.new()
        t.target = arm
        t.subtarget = n["ik"] if tgt == "foot_ik" else tgt
        t.weight = 1.0
        c.influence = 1.0 if cur == k else 0.0
        drivers[f"{n['space']}.{cname}"] = (new_driver(c.driver_add("influence"), arm, n["prop_space"],
                                                       f"1.0 if v == {k} else 0.0"), None, None, None)
    ikfk = float(props[n["prop_ikfk"]])
    for fk, mch, d in zip(n["fk"], n["hlp"] + [n["mch"][2]], n["def"]):
        copy_tf(arm, pbs[d], "fk_copy", fk)
        c = copy_tf(arm, pbs[d], "ik_copy", mch, influence=ikfk)
        drivers[f"{d}.ik_copy"] = (new_driver(c.driver_add("influence"), arm, n["prop_ikfk"], "v"), None, None, None)
    return drivers


# ---------------------------------------------------------------- skirt follows thigh (T253, spec d-02 §8 b)
FOLLOW_PROP = "skirt_follow"
FOLLOW_JSON = RIG / "data" / "skirt_follow.json"
SKIRT_AZ = (("F", 0), ("FL", 45), ("L", 90), ("BL", 135), ("B", 180), ("BR", 225), ("R", 270), ("FR", 315))
_DG = math.sqrt(0.5)
# panel -> (source sides, motion, push direction (x, y): horizontal unit vector in the pelvis rest frame, Blender axes)
FOLLOW_SRC = {
    "F": (("l", "r"), "flex (knee forward); the more forward thigh wins", (0.0, -1.0)),
    "FL": (("l",), "flex + abduction (knee toward the FL azimuth)", (_DG, -_DG)),
    "L": (("l",), "abduction (knee outward)", (1.0, 0.0)),
    "BL": (("l",), "extension (knee backward)", (0.0, 1.0)),
    "B": (("l", "r"), "extension (knee backward); the more backward thigh wins", (0.0, 1.0)),
    "BR": (("r",), "extension (knee backward)", (0.0, 1.0)),
    "R": (("r",), "abduction (knee outward)", (-1.0, 0.0)),
    "FR": (("r",), "flex + abduction (knee toward the FR azimuth)", (-_DG, -_DG)),
}
FOLLOW_W = {"F": 1.0, "FL": 1.0, "L": 1.0, "BL": 1.0, "B": 1.0, "BR": 1.0, "R": 1.0, "FR": 1.0}   # tuned T253
FOLLOW_GAP_DEG = 0.0
HINGE_LEN = 0.03
PROBE_LEN = 0.03


def follow_bone_names():
    out = [f"MCH_knee_probe_{x}" for x in SIDES]
    for t, _ in SKIRT_AZ:
        out.append(f"MCH_skirt_hinge_{t}")
    return out


def follow_edit_bones(arm):
    """MCH_knee_probe_x (parent MCH_pelvis): head = DEF Calf_X head (knee), +Z, roll 0 (local X = +X, Y = +Z, Z = -Y).
    MCH_skirt_hinge_T (parent MCH_pelvis): head = DEF Skirt_T_01 head, Y = outward azimuth (horizontal), Z = up, so
    rotation X+ swings the hanging panel outward. DEF Skirt_T_01 copies the hinge's local rotation in its own
    orientation (no rest copy of the DEF bone: an edit-bone roll copy is off by up to 1.7e-5, the local copy is exact
    at hinge 0)."""
    vl = bpy.context.view_layer
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    for n in follow_bone_names():
        if ebs.get(n) is not None:
            raise RuntimeError(f"bone {n} already exists")
    up = Vector((0.0, 0.0, 1.0))
    for x in SIDES:
        eb = ebs.new(f"MCH_knee_probe_{x}")
        h = ebs[f"Calf_{x.upper()}"].head.copy()
        eb.head, eb.tail, eb.roll = h, h + up * PROBE_LEN, 0.0
        eb.parent = ebs["MCH_pelvis"]
    for t, az in SKIRT_AZ:
        d = ebs[f"Skirt_{t}_01"]
        hg = ebs.new(f"MCH_skirt_hinge_{t}")
        out = Vector((math.sin(math.radians(az)), -math.cos(math.radians(az)), 0.0))
        hg.head, hg.tail = d.head.copy(), d.head + out * HINGE_LEN
        hg.align_roll(up)
        hg.parent = ebs["MCH_pelvis"]
    for n in follow_bone_names():
        ebs[n].use_deform = False
        ebs[n].use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")


def follow_geometry(arm):
    """rest knee - hip (DEF Calf_X head - DEF Thigh_X head), armature space."""
    B = arm.data.bones
    return {x: B[f"Calf_{x.upper()}"].head_local - B[f"Thigh_{x.upper()}"].head_local for x in SIDES}


def follow_expr(t, w, geo, gap_deg=FOLLOW_GAP_DEG):
    """Driver expression of MCH_skirt_hinge_T rotation X. Knee vector r = (knee - hip)_rest + (x, -z, y) with (x, y, z)
    = MCH_knee_probe local location (pelvis frame); tilt toward s = atan2(r . s, -r_z); push = tilt - rest tilt - gap;
    panel angle = skirt_follow * w * max(push (max over the sources), 0)."""
    sides, _m, (sx, sy) = FOLLOW_SRC[t]
    terms = []
    for x in sides:
        r0 = geo[x]
        c = r0.x * sx + r0.y * sy
        hz = -r0.z
        a0 = math.atan2(c, hz) + math.radians(gap_deg)
        num = f"{c:.9f}"
        if abs(sx) > 1e-9:
            num += f"{sx:+.9f}*x{x}"
        if abs(sy) > 1e-9:
            num += f"{-sy:+.9f}*z{x}"
        terms.append(f"atan2({num},{hz:.9f}-y{x})-({a0:.9f})")
    push = terms[0] if len(terms) == 1 else f"max({terms[0]},{terms[1]})"
    return f"f*{w:.4f}*max({push},0.0)"


def follow_pose_setup(arm, weights=None, gap_deg=FOLLOW_GAP_DEG):
    pbs = arm.pose.bones
    weights = weights or FOLLOW_W
    geo = follow_geometry(arm)
    for x in SIDES:
        pb = pbs[f"MCH_knee_probe_{x}"]
        c = pb.constraints.new("COPY_LOCATION")
        c.name = "knee_copy"
        c.target = arm
        c.subtarget = f"Calf_{x.upper()}"
        c.head_tail = 0.0
        c.target_space = "WORLD"
        c.owner_space = "WORLD"
    drivers = {}
    for t, _ in SKIRT_AZ:
        hg = pbs[f"MCH_skirt_hinge_{t}"]
        hg.rotation_mode = "XYZ"
        c = copy_tf(arm, pbs[f"Skirt_{t}_01"], "follow_copy", f"MCH_skirt_hinge_{t}")
        c.target_space = "LOCAL_OWNER_ORIENT"
        c.owner_space = "LOCAL"
        fc = hg.driver_add("rotation_euler", 0)
        for m in list(fc.modifiers):
            fc.modifiers.remove(m)
        drv = fc.driver
        drv.type = "SCRIPTED"
        v = drv.variables.new()
        v.name = "f"
        v.type = "SINGLE_PROP"
        v.targets[0].id_type = "OBJECT"
        v.targets[0].id = arm
        v.targets[0].data_path = f'pose.bones["PROPS"]["{FOLLOW_PROP}"]'
        for x in FOLLOW_SRC[t][0]:
            for ch in "XYZ":
                v = drv.variables.new()
                v.name = f"{ch.lower()}{x}"
                v.type = "TRANSFORMS"
                tg = v.targets[0]
                tg.id = arm
                tg.bone_target = f"MCH_knee_probe_{x}"
                tg.transform_type = f"LOC_{ch}"
                tg.transform_space = "LOCAL_SPACE"
        drv.expression = follow_expr(t, weights[t], geo, gap_deg)
        drivers[t] = drv
    return drivers


def apply_follow_weights(arm, weights, gap_deg=FOLLOW_GAP_DEG):
    """Rewrite the follow driver expressions (tuning; used in memory by scratch measurements)."""
    geo = follow_geometry(arm)
    out = {}
    for t, _ in SKIRT_AZ:
        fc = arm.animation_data.drivers.find(f'pose.bones["MCH_skirt_hinge_{t}"].rotation_euler', index=0)
        fc.driver.expression = follow_expr(t, float(weights.get(t, FOLLOW_W[t])), geo, gap_deg)
        out[t] = fc.driver.expression
    arm.update_tag()
    bpy.context.view_layer.update()
    return out


def follow_selftest(arm):
    """FK leg poses on the left thigh: driven hinge angle vs the angle computed here from the DEF knee (pelvis frame)."""
    pbs = arm.pose.bones
    geo = follow_geometry(arm)
    B = arm.data.bones
    rows = []
    set_prop(arm, "leg_ik_fk_l", 0.0)
    f = float(pbs["PROPS"][FOLLOW_PROP])
    for label, ax, deg in (("flex 60", 0, 60.0), ("abduction 30", 2, 30.0), ("extension 30", 0, -30.0)):
        th = pbs["CTRL_thigh_fk_l"]
        th.rotation_euler = (0.0, 0.0, 0.0)
        th.rotation_euler[ax] = math.radians(deg)
        arm.update_tag()
        bpy.context.view_layer.update()
        pel = arm.matrix_world @ pbs["Pelvis"].matrix
        T = B["Pelvis"].matrix_local @ pel.inverted()
        worst = 0.0
        got = {}
        for t, _ in SKIRT_AZ:
            sides, _m, (sx, sy) = FOLLOW_SRC[t]
            pushes = []
            for x in sides:
                S = x.upper()
                r = T @ (arm.matrix_world @ pbs[f"Calf_{S}"].head) - T @ (arm.matrix_world @ pbs[f"Thigh_{S}"].head)
                a = math.atan2(r.x * sx + r.y * sy, -r.z)
                a0 = math.atan2(geo[x].x * sx + geo[x].y * sy, -geo[x].z) + math.radians(FOLLOW_GAP_DEG)
                pushes.append(a - a0)
            exp = f * FOLLOW_W[t] * max(max(pushes), 0.0)
            val = pbs[f"MCH_skirt_hinge_{t}"].rotation_euler[0]
            worst = max(worst, abs(val - exp))
            got[t] = round(math.degrees(val), 3)
        rows.append((label, got, math.degrees(worst)))
        th.rotation_euler = (0.0, 0.0, 0.0)
    set_prop(arm, "leg_ik_fk_l", 1.0)
    arm.update_tag()
    bpy.context.view_layer.update()
    return rows


def write_follow_json(arm, drivers):
    B = arm.data.bones
    geo = follow_geometry(arm)
    panels = {}
    for t, az in SKIRT_AZ:
        sides, motion, s = FOLLOW_SRC[t]
        hg = B[f"MCH_skirt_hinge_{t}"]
        panels[t] = {
            "azimuth_deg": az, "def_bone": f"Skirt_{t}_01",
            "def_constraint": "follow_copy (COPY_TRANSFORMS <- hinge, target LOCAL_OWNER_ORIENT, owner LOCAL)",
            "hinge": f"MCH_skirt_hinge_{t}",
            "hinge_axis_rest": [round(v, 6) for v in hg.matrix_local.col[0][:3]],
            "outward_dir_rest": [round(v, 6) for v in hg.matrix_local.col[1][:3]],
            "sources": [f"Thigh_{x.upper()}" for x in sides], "motion": motion,
            "push_dir_xy": [round(v, 6) for v in s], "combine": "max" if len(sides) > 1 else "single",
            "weight": FOLLOW_W[t], "fraction_at_default": round(FOLLOW_W[t] * 0.5, 4),
            "rest_tilt_deg": {f"Thigh_{x.upper()}": round(math.degrees(math.atan2(geo[x].x * s[0] + geo[x].y * s[1],
                                                                                   -geo[x].z)), 4) for x in sides},
            "expression": drivers[t].expression, "simple_expression": bool(drivers[t].is_simple_expression)}
    doc = {
        "_doc": "Skirt follows thigh (T253, spec d-02 §8 b), generated by p24c_ctrl_legs.py. DEF Skirt_T_01 (parent "
                "DEF Pelvis) copies the local rotation of MCH_skirt_hinge_T (parent MCH_pelvis = DEF Pelvis, head = "
                "Skirt_T_01 head) in its own orientation (Copy Transforms LOCAL_OWNER_ORIENT -> LOCAL): the panel turns "
                "about the hinge axis through its head. "
                "Hinge rotation X (driver) = skirt_follow * weight * max(push, 0) rad, where push = max over the "
                "sources of (tilt - rest tilt - gap); tilt = atan2(r . push_dir, -r_z), r = knee - hip of the source "
                "thigh in the pelvis rest frame (knee = DEF Calf_X head through MCH_knee_probe_x, Copy Location "
                "WORLD, local location read by the driver in LOCAL_SPACE). Hinge X axis = outward x up, so X+ swings "
                "the hanging panel outward along outward_dir. Skirt_T_02 stays a plain child of Skirt_T_01.",
        "prop": {"bone": "PROPS", "name": FOLLOW_PROP, "default": 0.5, "min": 0.0, "max": 1.0,
                 "set_by": ["player.reset_rig (UI default)", "player.ready_pose (UI default)"],
                 "ui": "View3D > Sidebar > Player, box Skirt"},
        "push_only": "max(push, 0): a panel only swings outward (hinge X >= 0) when its source thigh tilts toward the "
                     "panel's push direction beyond its rest tilt (+ gap); the opposite motion gives 0 (never pulled "
                     "inward). Two-source panels (F, B) take the larger push.",
        "gap_deg": FOLLOW_GAP_DEG,
        "zero_rule": "skirt_follow = 0 -> every hinge angle 0 -> DEF Skirt_* = rest relative to DEF Pelvis (pre-follow)",
        "probes": {x: {"bone": f"MCH_knee_probe_{x}", "parent": "MCH_pelvis",
                       "copy_location_from": f"Calf_{x.upper()} head",
                       "local_axes": "X = +X, Y = +Z, Z = -Y (rest, pelvis frame): knee offset = (x, -z, y)",
                       "rest_knee_minus_hip": [round(v, 6) for v in geo[x]]} for x in SIDES},
        "panels": panels,
        "measurement": "P2.8b poke-out (springs off, Blender DEF mesh, pelvis rest frame) per rigtest section; tuning "
                       "iterations in the T253 report",
    }
    FOLLOW_JSON.write_text(json.dumps(doc, indent=1) + "\n", encoding="utf-8")
    return FOLLOW_JSON


def assign_collections(arm):
    ad = arm.data
    colls = {c.name: c for c in ad.collections_all}
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            b = ad.bones[name]
            for c in list(b.collections):
                c.unassign(b)
            colls["MCH" if name.startswith("MCH_") else "CTRL"].assign(b)
    for name in follow_bone_names():
        b = ad.bones[name]
        for c in list(b.collections):
            c.unassign(b)
        colls["MCH"].assign(b)


# ---------------------------------------------------------------- evaluation helpers (s06c)
def set_prop(arm, name, value):
    props = arm.pose.bones["PROPS"]
    props[name] = type(props[name])(value)
    arm.update_tag()


def shoe_coords(mesh_obj, ids):
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    ev = mesh_obj.evaluated_get(dg)
    me = ev.to_mesh()
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    ev.to_mesh_clear()
    return co.reshape(-1, 3)[ids]


def planted_state(arm, mesh_obj, x, ids, prev=None, prev_s=0.0, use_slip=False):
    co = shoe_coords(mesh_obj, ids)
    z = float(co[:, 2].min())
    slip = prev_s
    if prev is not None:
        c = prev[:, 2] <= prev[:, 2].min() + CONTACT_BAND
        slip = prev_s + float(np.linalg.norm(co[c, :2] - prev[c, :2], axis=1).mean())
    pbs = arm.pose.bones
    n = names(x)
    reach = (pbs[n["mch"][1]].tail - pbs[n["roll_foot"]].head).length
    causes = []
    if reach > REACH_TOL:
        causes.append("reach")
    if z < Z_MIN:
        causes.append("penetrate")
    if z > Z_MAX:
        causes.append("float")
    if use_slip and slip > SLIP_MAX:
        causes.append("slip")
    return reach, z, causes, slip, co


def knee_bend_deg(arm, x):
    pbs = arm.pose.bones
    n = names(x)
    a, b = pbs[n["mch"][0]], pbs[n["mch"][1]]
    return math.degrees((a.tail - a.head).angle(b.tail - b.head))


def clamp_sweep(arm, mesh_obj, frames):
    torso = arm.pose.bones["CTRL_torso"]
    res, stances = {}, {}
    for st in STANCES_M:
        torso.location = (0.0, st, 0.0)
        arm.update_tag()
        bpy.context.view_layer.update()
        info = {}
        for x in SIDES:
            ids = frames[x]["shoe_ids"]
            reach0, z0, c0, _, ref = planted_state(arm, mesh_obj, x, ids)
            info[x] = {"knee_bend_deg": knee_bend_deg(arm, x), "reach0": reach0, "z0": z0, "causes0": c0}
            n = names(x)
            for pk, (lo, hi) in START_CLAMPS.items():
                prop = n["roll_props"][pk]
                use_slip = pk in SLIP_PROPS
                r = {"start": [lo, hi], "reach0": reach0, "z0": z0, "causes0": c0, "slip_in_criterion": use_slip}
                rng = []
                for sgn, end in ((-1, lo), (1, hi)):
                    tag = "neg" if sgn < 0 else "pos"
                    last_ok, first_fail, samples = (0 if not c0 else None), None, {}
                    prev, prev_s = ref, 0.0
                    for d in range(1, abs(end) + 1):
                        val = sgn * d
                        set_prop(arm, prop, float(val))
                        st_ = planted_state(arm, mesh_obj, x, ids, prev, prev_s, use_slip)
                        samples[val] = st_[:4]
                        prev, prev_s = st_[4], st_[3]
                        if c0:
                            continue
                        if samples[val][2] and first_fail is None:
                            first_fail = val
                        if first_fail is None:
                            last_ok = val
                    set_prop(arm, prop, 0.0)
                    r[f"last_ok_{tag}"] = last_ok
                    r[f"first_fail_{tag}"] = first_fail
                    r[f"at_last_ok_{tag}"] = samples.get(last_ok, (reach0, z0, c0, 0.0)) if last_ok is not None else None
                    r[f"at_first_fail_{tag}"] = samples.get(first_fail) if first_fail is not None else None
                    r[f"max_slip_all_{tag}"] = max(t[3] for t in samples.values())
                    rng.append(last_ok)
                r["planted"] = None if c0 else rng
                res.setdefault(prop, {"stances": {}})["stances"][st] = r
        stances[st] = info
    torso.location = (0.0, 0.0, 0.0)
    arm.update_tag()
    bpy.context.view_layer.update()
    for prop, d in res.items():
        pl = d["stances"][CLAMP_STANCE_M]["planted"]
        d["final"] = list(pl) if pl is not None else [0, 0]
    return res, stances


def apply_clamps(arm, drivers, bank_sign, clamp_res):
    props = arm.pose.bones["PROPS"]
    for x in SIDES:
        n = names(x)
        cl = {pk: tuple(clamp_res[n["roll_props"][pk]]["final"]) for pk in START_CLAMPS}
        ex = roll_exprs(bank_sign[x], cl)
        for key, (drv, bkey, idx, pk) in drivers.items():
            if bkey is not None and key == f"{n[bkey]}.rotation_euler[{idx}]":
                drv.expression = ex[(bkey, idx)][1]
        for pk, (lo, hi) in cl.items():
            ui = props.id_properties_ui(n["roll_props"][pk])
            ui.update(min=float(lo), max=float(hi), soft_min=float(lo), soft_max=float(hi))
    arm.update_tag()
    bpy.context.view_layer.update()


# ---------------------------------------------------------------- manifest
def write_manifest(angles, bank_sign, bend, clamp_res, stances, pinfo, frames, arm):
    old = json.loads(MANIFEST.read_text(encoding="utf-8"))
    man = dict(old)
    man["_doc"] = dict(old.get("_doc", {}))
    man["_doc"].update(DOC)
    man["_doc"]["p24c_pivots"] = "; ".join(
        f"{x}: " + ", ".join(f"{k} {fmt_v(frames[x][k])} (start {fmt_v(frames[x]['before'][k])}, geometric planted range "
                             f"{pinfo[x][k]['range']} deg, max slip {pinfo[x][k]['maxslip'] * 1000:.2f} mm)"
                             for k in PIVOT_SEARCH) for x in SIDES)
    controls, spaces, ikfk, clamps = {}, {}, {}, {}
    ad = arm.data
    for x in SIDES:
        n = names(x)
        sw = sweeps(x)
        for name in n["fk"] + [n["ik"], n["pole"]]:
            controls[name] = {"owner": OWNER, "side": x.upper(), "part": "leg", "rotation_mode": "XYZ",
                              "sweep": sw[name], "mode": {n["prop_ikfk"]: 0 if name in n["fk"] else 1}}
        cons = [s[1] for s in POLE_SPACES]
        tg = {s[1]: (n["ik"] if s[2] == "foot_ik" else s[2]) for s in POLE_SPACES}
        spaces[n["prop_space"]] = {
            "owner": OWNER, "control": n["pole"], "prop": n["prop_space"], "values": [s[0] for s in POLE_SPACES],
            "mch": n["space"], "constraints": cons, "constraint_type": {c: "ARMATURE" for c in cons}, "targets": tg,
            "influence": {s[1]: f"driver PROPS[prop] == {k}" for k, s in enumerate(POLE_SPACES)},
            "value_constraint": {s[0]: s[1] for s in POLE_SPACES},
            "value_effect": {s[0]: f"full follow of {tg[s[1]]}" for s in POLE_SPACES}}
        off = ad.bones[n["def"][2]].matrix_local.inverted() @ ad.bones[n["ik"]].matrix_local
        ikfk[f"leg_{x}"] = {
            "owner": OWNER, "prop": n["prop_ikfk"], "fk_value": 0, "ik_value": 1,
            "fk": n["fk"], "ik": n["ik"], "pole": n["pole"], "def": n["def"],
            "mch_ik": n["hlp"] + [n["mch"][2]], "ik_chain": n["mch"],
            "fk_to_def": dict(zip(n["fk"], n["def"])), "fk_to_mch_ik": dict(zip(n["fk"], n["hlp"] + [n["mch"][2]])),
            "preferred_bend": bend[x],
            "def_constraints": {"fk": "fk_copy", "ik": "ik_copy"},
            "ik_constraint": {"bone": n["mch"][1], "name": "IK", "chain_count": 2, "use_tail": True,
                              "use_stretch": False, "target": n["roll_foot"], "pole_angle_rad": angles[x],
                              "pole_angle_deg": math.degrees(angles[x]),
                              "hinge_limit": {"lock_ik_y": True, "lock_ik_z": True, "ik_x_deg": [KNEE_IK_MIN,
                                                                                               KNEE_IK_MAX]}},
            "ik_foot_rotation": {"bone": n["mch"][2], "constraint": "IK_rot", "type": "COPY_ROTATION",
                                 "source": n["roll_foot"]},
            "roll_chain": [n["heel"], n["toe"], n["bank_in"], n["bank_out"], n["roll_foot"]],
            "roll_props": list(n["roll_props"].values()), "bank_sign": bank_sign[x],
            "ik_space": {"prop": n["prop_space"], "mch": n["space"]},
            "pole_parent": n["space"], "pole_distance": POLE_DIST,
            "flexion_sign": {"bone": n["fk"][1], "axis": "X", "sign": -1},
            "ik_ctrl_in_def_foot_rest": [list(r) for r in off],
            "snap": {"fk_from_ik": "for fk, mch in fk_to_mch_ik in chain order: fk world matrix = mch world matrix "
                                   "(roll props keep their values), then prop = fk_value",
                     "ik_from_fk": "roll_props to 0; CTRL_foot_ik world = CTRL_foot_fk world @ "
                                   "ik_ctrl_in_def_foot_rest; pole from the FK thigh X axis, corrected until the MCH "
                                   "knee matches the FK knee; prop = ik_value"}}
        for pk in START_CLAMPS:
            clamps[n["roll_props"][pk]] = list(clamp_res[n["roll_props"][pk]]["final"])
    for sec, new in (("controls", controls), ("spaces", spaces), ("ikfk", ikfk)):
        keep = {k: v for k, v in old.get(sec, {}).items() if not (isinstance(v, dict) and v.get("owner") == OWNER)}
        keep.update(new)
        man[sec] = keep
    cl = dict(old.get("clamps", {}))
    cl.update(clamps)
    man["clamps"] = cl
    man["clamps_reference"] = {
        "owner": OWNER, "chosen_stance_m": CLAMP_STANCE_M, "ready_torso_y_m": READY_TORSO_Y,
        "criterion": {"reach_tol_m": REACH_TOL, "z_min_m": Z_MIN, "z_max_m": Z_MAX, "slip_max_m": SLIP_MAX,
                      "slip_props": list(SLIP_PROPS)},
        "stances": {f"{st:+.3f}": {"knee_bend_deg": {x: stances[st][x]["knee_bend_deg"] for x in SIDES},
                                   "planted": {prop: d["stances"][st]["planted"] for prop, d in clamp_res.items()}}
                    for st in STANCES_M}}
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


def reset_new(arm):
    for x in SIDES:
        for name, _, _, _ in new_bone_list(x):
            pb = arm.pose.bones[name]
            pb.location = (0, 0, 0)
            pb.rotation_euler = (0, 0, 0)
            pb.rotation_quaternion = (1, 0, 0, 0)
            pb.scale = (1, 1, 1)
    arm.update_tag()


def main():
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    arm = bpy.data.objects[ARM]
    mesh_obj = bpy.data.objects[MESH]
    canon = json.loads(CANON.read_text(encoding="utf-8"))
    parts = json.loads(PARTS_JSON.read_text(encoding="utf-8"))["part_id"]
    frames = foot_frames(arm, mesh_obj, parts)
    pinfo = search_pivots(arm, frames)
    for x in SIDES:
        f = frames[x]
        for kind in PIVOT_SEARCH:
            i = pinfo[x][kind]
            log(f"PIVOT {kind}_{x}: start {fmt_v(f['before'][kind])} -> {fmt_v(f[kind])} (t {i['t'] * 1000:+.0f} mm along "
                f"{PIVOT_SEARCH[kind][1]}, h {i['h'] * 1000:.0f} mm); geometric planted range {i['range']}/"
                f"{i['max_range']} deg (start point {i['range_at_start']}), max slip {i['maxslip'] * 1000:.3f} mm")
    angles, bank_sign, bend = edit_bones(arm, frames)
    follow_edit_bones(arm)
    for x in SIDES:
        log(f"PREFERRED BEND {x}: {bend[x]}")
    drivers = {}
    for x in SIDES:
        drivers.update(pose_setup(arm, x, angles[x], bank_sign[x]))
    fdrv = follow_pose_setup(arm)
    assign_collections(arm)
    reset_new(arm)
    arm.data.pose_position = "POSE"
    ik = def_vs_canon(arm, canon)
    for x in SIDES:
        set_prop(arm, f"leg_ik_fk_{x}", 0.0)
    fk = def_vs_canon(arm, canon)
    for x in SIDES:
        set_prop(arm, f"leg_ik_fk_{x}", 1.0)
    bpy.context.view_layer.update()
    log(f"G6.2 SELFCHECK leg IK (default) {ik[0]:.3e} ({ik[1]}), leg FK {fk[0]:.3e} ({fk[1]}), threshold 1e-4")
    for t, drv in fdrv.items():
        log(f"FOLLOW {t}: MCH_skirt_hinge_{t} rot X = {drv.expression!r} (valid {drv.is_valid}, simple "
            f"{drv.is_simple_expression}), rest value "
            f"{math.degrees(arm.pose.bones[f'MCH_skirt_hinge_{t}'].rotation_euler[0]):.6f} deg")
    for label, got, err in follow_selftest(arm):
        log(f"FOLLOW SELFTEST CTRL_thigh_fk_l {label} deg (leg FK, skirt_follow "
            f"{float(arm.pose.bones['PROPS'][FOLLOW_PROP])}): hinge deg {got}; |driver - DEF knee tilt| max {err:.6f} deg")
    for x in SIDES:
        log(f"POLE {x}: pole_angle {math.degrees(angles[x]):+.4f} deg, pole head "
            f"{fmt_v(arm.data.bones[names(x)['pole']].head_local)}, bank sign {bank_sign[x]:+.0f}; hinge MCH_calf_ik_{x} "
            f"X [{KNEE_IK_MIN}, {KNEE_IK_MAX}] deg, Y/Z locked")
    clamp_res, stances = clamp_sweep(arm, mesh_obj, frames)
    apply_clamps(arm, drivers, bank_sign, clamp_res)

    def st_txt(t):
        return "-" if t is None else f"reach {t[0] * 1000:.3f} mm, z {t[1] * 1000:+.3f} mm, S {t[3] * 1000:.2f} mm, fail {t[2] or 'none'}"

    for st in STANCES_M:
        for x in SIDES:
            i = stances[st][x]
            log(f"STANCE {st * 1000:+.0f} mm {x}: knee bend {i['knee_bend_deg']:.2f} deg; props 0: reach "
                f"{i['reach0'] * 1000:.3f} mm, shoe z {i['z0'] * 1000:+.3f} mm, fail {i['causes0'] or 'none'}")
        for prop, d in clamp_res.items():
            r = d["stances"][st]
            log(f"PLANTED {st * 1000:+.0f} mm {prop}: planted {r['planted']}; neg last ok {r['last_ok_neg']} "
                f"({st_txt(r['at_last_ok_neg'])}), first fail {r['first_fail_neg']} ({st_txt(r['at_first_fail_neg'])}); "
                f"pos last ok {r['last_ok_pos']} ({st_txt(r['at_last_ok_pos'])}), first fail {r['first_fail_pos']} "
                f"({st_txt(r['at_first_fail_pos'])})")
    log(f"FINAL CLAMPS (stance {CLAMP_STANCE_M * 1000:+.0f} mm): " + ", ".join(f"{p}={d['final']}" for p, d in clamp_res.items()))
    for tag, (drv, *_r) in drivers.items():
        if not drv.is_valid:
            log(f"DRIVER INVALID {tag}: {drv.expression}")
    reset_new(arm)
    after = def_vs_canon(arm, canon)
    log(f"G6.2 after sweep {after[0]:.3e}")
    out = write_manifest(angles, bank_sign, bend, clamp_res, stances, pinfo, frames, arm)
    log(f"MANIFEST {out}")
    log(f"FOLLOW JSON {write_follow_json(arm, fdrv)}")
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
