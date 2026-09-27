"""s06c_ctrl_legs.py - T26: left / right leg FK + IK controls and foot roll (G6.1-G6.7 legs).

Input : rig/work/r04b.blend (s06a torso/head/PROPS, s06b arms), rig/data/pivots.json (heel_x, foot_tip_x),
        rig/data/parts.json (shoe_x part_id)
Output: rig/work/r04c.blend (save copy, save_version 0), rig/data/ctrl_manifest.json (adds owner "s06c" entries and the
        clamps section)

Run:  bl.ps1 -Script s06c_ctrl_legs.py -Blend work/r04b.blend

Per side x (l, r), from its own DEF rest and pivots (no mirroring; all new bones use_deform False):
  FK : CTRL_pelvis > CTRL_thigh_fk_x > CTRL_calf_fk_x > CTRL_foot_fk_x            (rest/roll/connect = DEF)
  IK : CTRL_pelvis > MCH_thigh_ik_x > MCH_calf_ik_x > MCH_foot_ik_x                 (rest/roll/connect = DEF)
       MCH_calf_ik_x: IK -> MCH_roll_foot_x (head = ankle; chain 2, use_tail, no stretch, pole CTRL_knee_pole_x,
       pole_angle from the rest chain); ik_stretch 0.  MCH_foot_ik_x: Copy Rotation WORLD/WORLD <- MCH_roll_foot_x.
  CTRL_root > CTRL_foot_ik_x (head = ankle = DEF foot head, points world -Y, local Z = world +Z)
       > MCH_roll_heel_x (pivot heel_x)  > MCH_roll_toe_x (pivot foot_tip_x)
       > MCH_roll_bank_in_x (shoe inner sole edge) > MCH_roll_bank_out_x (shoe outer sole edge)
       > MCH_roll_foot_x (rest = DEF foot) = IK target + foot rotation.
       Pivot bones: head = pivot, Y = foot forward (heel -> foot_tip, horizontal), Z = world up, X = Y x Z.
       Drivers (degrees, clamped to the final clamp range c(v) = min(max(v, lo), hi)):
         heel  rot X = max(-c(foot_roll), 0), rot Z = c(heel_twist)      (roll - : toes up on the heel)
         toe   rot X = -max(c(foot_roll), 0), rot Z = c(toe_twist)       (roll + : heel up on the toe)
         bank_in  rot Y = s * min(c(foot_bank), 0);  bank_out rot Y = s * max(c(foot_bank), 0)
           (s = sign(X_pivot . outward): bank + = onto the outer edge, - = onto the inner edge)
         twist + = counter-clockwise seen from above.
  MCH_CTRL_knee_pole_x_space (no parent) > CTRL_knee_pole_x at DEF calf head + POLE_DIST * (-Y) (preferred knee bend);
       space constraints (Armature, influence = driver PROPS["knee_pole_space_x"] == k):
       space_foot -> CTRL_foot_ik_x (0), space_world -> CTRL_root (1).
  DEF thigh/calf/foot_x: fk_copy (Copy Transforms WORLD/WORLD, influence 1) <- FK control, then ik_copy (influence =
       driver PROPS["leg_ik_fk_x"]) <- MCH IK bone.  0 = FK, 1 = IK (default 1).
Clamps (T26b, planted criterion): stances CTRL_torso loc Y (= world Z) 0 / -15 / -30 mm, leg IK, all other controls
  identity, foot IK at rest; each roll property alone in 1 deg steps over its whole starting range. Planted = IK reach
  error |MCH_calf_ik tail - MCH_roll_foot head| <= 1 mm and -1 mm <= lowest evaluated shoe vertex z (part_id shoe_x)
  <= +3 mm and (roll / bank only) slip of the lowest material point <= 5 mm (T26d). Planted range = continuous range
  around 0. The -15 mm stance range is written to ctrl_manifest.clamps, the
  PROPS UI min/max and the driver clamps; 0 / -30 mm are reference (manifest clamps_reference).
Self-checks: G6.2 IK (defaults) and FK (leg_ik_fk = 0): DEF pose matrix vs canonical rest.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Vector  # noqa: E402

OUT_BLEND = goblib.WORK / "r04c.blend"
MANIFEST = "ctrl_manifest.json"
OWNER = "s06c"
ARM = "GOB_rig"
MESH = "GOB_mesh"
SIDES = ("l", "r")
CHAIN = ("thigh", "calf", "foot")
POLE_DIST = 0.30
POLE_DIR = Vector((0.0, -1.0, 0.0))  # preferred knee bend (-Y, G4.9)
POLE_LEN = 0.05
FOOT_IK_LEN = 0.15
PIVOT_LEN = 0.05
SOLE_BAND = 0.01  # shoe verts within this height above the shoe's lowest vertex = sole (bank edges)
REACH_TOL = 0.001  # planted: IK reach error (MCH_calf_ik tail -> MCH_roll_foot head) <= 1 mm
Z_MIN = -0.001  # planted: lowest shoe vertex z >= -1 mm (no ground penetration)
Z_MAX = 0.003  # planted: lowest shoe vertex z <= +3 mm (no floating; T26d: low-poly rounded shoe)
SLIP_MAX = 0.005  # planted (roll / bank): cumulative contact slip S <= 5 mm (T26e)
CONTACT_BAND = 0.001  # T26e: contact set = shoe verts with z <= lowest z of the step + 1 mm
SLIP_PROPS = ("foot_roll", "foot_bank")  # twist: slip reported only, clamp from reach + z
STANCES_MM = (0, -15, -30)  # CTRL_torso loc Y (= world Z) offsets, mm
CLAMP_STANCE_MM = -15  # stance whose planted range becomes the final clamp
DEG = 0.017453292519943295
START_CLAMPS = {"foot_roll": (-45, 60), "foot_bank": (-30, 30), "heel_twist": (-45, 45), "toe_twist": (-45, 45)}
POLE_SPACES = [("foot", "space_foot", "foot_ik"), ("world", "space_world", "CTRL_root")]


def names(x):
    return {
        "fk": [f"CTRL_{b}_fk_{x}" for b in CHAIN],
        "mch": [f"MCH_{b}_ik_{x}" for b in CHAIN],
        "def": [f"{b}_{x}" for b in CHAIN],
        "ik": f"CTRL_foot_ik_{x}",
        "heel": f"MCH_roll_heel_{x}",
        "toe": f"MCH_roll_toe_{x}",
        "bank_in": f"MCH_roll_bank_in_{x}",
        "bank_out": f"MCH_roll_bank_out_{x}",
        "roll_foot": f"MCH_roll_foot_{x}",
        "space": f"MCH_CTRL_knee_pole_{x}_space",
        "pole": f"CTRL_knee_pole_{x}",
        "prop_ikfk": f"leg_ik_fk_{x}",
        "prop_space": f"knee_pole_space_{x}",
        "roll_props": {k: f"{k}_{x}" for k in START_CLAMPS},
    }


def new_bone_list(x):
    """(name, parent, rest source, use_connect or None = copy from DEF)"""
    n = names(x)
    out = []
    pf, pm = "CTRL_pelvis", "CTRL_pelvis"
    for b, fk, mch in zip(CHAIN, n["fk"], n["mch"]):
        out.append((fk, pf, f"{b}_{x}", None))
        out.append((mch, pm, f"{b}_{x}", None))
        pf, pm = fk, mch
    out += [(n["ik"], "CTRL_root", "foot_ik", False),
            (n["heel"], n["ik"], "heel", False),
            (n["toe"], n["heel"], "toe", False),
            (n["bank_in"], n["toe"], "bank_in", False),
            (n["bank_out"], n["bank_in"], "bank_out", False),
            (n["roll_foot"], n["bank_out"], f"foot_{x}", False),
            (n["space"], None, "pole", False),
            (n["pole"], n["space"], "pole", False)]
    return out


# ---------------------------------------------------------------- sweep ranges (rom_poses _doc sign convention)
def rot(axis, lo, hi):
    return {"channel": "rot", "axis": axis, "min": lo, "max": hi}


def loc(axis, lo, hi):
    return {"channel": "loc", "axis": axis, "min": lo, "max": hi}


def sweeps(x):
    n = names(x)
    abd = [-45, 15] if x == "l" else [-15, 45]  # abduction = thigh_l Z-, thigh_r Z+
    return {
        n["fk"][0]: [rot("X", -90, 60), rot("Y", -45, 45), rot("Z", *abd)],
        n["fk"][1]: [rot("X", 0, 130)],
        n["fk"][2]: [rot("X", -30, 30), rot("Z", -30, 30)],
        # T28b, DOC s06c_sweep_ranges
        n["ik"]: [loc("X", -0.08, 0.08), loc("Y", -0.12, 0.12), loc("Z", 0.0, 0.08)] + [rot(a, -45, 45) for a in "XYZ"],
        n["pole"]: [loc(a, -0.2, 0.2) for a in "XYZ"],
    }


DOC = {
    "s06c_sweep_ranges": "T28b: CTRL_foot_ik_x loc up (local Z) 0..+0.08 m (ankle 0.124 -> 0.204 m stays 70 mm below "
                         "the hip 0.274 m, so the 150 mm leg bends without folding through; jump tuck / step lift), "
                         "forward/back (local Y) +-0.12 m, side (local X) +-0.08 m, rot +-45; poles +-0.2 m unchanged.",
    "s06c_legs": "Legs (s06c, T26), per side from its own DEF rest and pivots (no mirroring). FK CTRL_thigh/calf/"
                 "foot_fk_x (thigh parent CTRL_pelvis) and MCH IK chain MCH_thigh/calf/foot_ik_x copy DEF rest, roll "
                 "and use_connect, so the rom_poses sign convention holds for the FK controls (thigh X- = hip "
                 "flexion, abduction thigh_l Z- / thigh_r Z+, calf X+ = knee flexion, foot X- = toes up). DEF thigh/"
                 "calf/foot: fk_copy (influence 1) then ik_copy (influence = leg_ik_fk_x; 0 = FK, 1 = IK, default 1). "
                 "IK: no stretch.",
    "s06c_foot_ik": "CTRL_foot_ik_x (parent CTRL_root): head = ankle (DEF foot head), points world -Y, local Z = world "
                    "+Z, so loc X = world -X, loc Y = forward, loc Z = up (z 0 = foot on the ground at rest) and rot X+ "
                    "= toes up about the ankle; world-aligned axes keep ground-plane animation intuitive, the ankle "
                    "pivot matches the anatomical joint and ground contact is handled by the foot roll pivots.",
    "s06c_roll": "Foot roll MCH chain under CTRL_foot_ik_x: MCH_roll_heel_x > MCH_roll_toe_x > MCH_roll_bank_in_x / "
                 "MCH_roll_bank_out_x (pivot heads from the T26c contact search, see s06c_pivots) > MCH_roll_foot_x "
                 "(rest = DEF foot) = IK target (head) and foot rotation. Pivot axes: "
                 "Y = heel->toe horizontal, Z = up. foot_roll - = toes up on the heel, + = heel up on the toe; "
                 "foot_bank + = onto the outer edge, - = onto the inner edge; heel_twist / toe_twist + = counter-"
                 "clockwise seen from above. Degrees; driver expressions clamp to clamps[prop].",
    "s06c_pole": "CTRL_knee_pole_x: DEF calf head + 0.30 m along -Y (preferred knee bend side, G4.9), bone points +Z, "
                 "roll 0, child of MCH_CTRL_knee_pole_x_space. knee_pole_space: foot = CTRL_foot_ik_x, world = "
                 "CTRL_root (chosen over MCH_world so root motion in walk/run carries the pole with the feet, which "
                 "are also in root space; the pole then only ignores foot turns).",
    "s06c_clamps": "clamps.<prop> = [min, max] degrees, written by s06c (T26b/T26d): planted range at the stance "
                   "CTRL_torso loc Y (= world Z) = -15 mm (knees bent), leg IK, all other controls identity, foot IK "
                   "control at rest. Each property alone swept in 1 deg steps over its starting range (roll [-45, 60], "
                   "bank [-30, 30], twist [-45, 45]); planted = IK reach error |MCH_calf_ik tail - MCH_roll_foot "
                   "head| <= 1 mm AND lowest shoe vertex z >= -1 mm (no penetration) AND <= +3 mm (no floating; "
                   "T26d, low-poly rounded shoe) AND, for foot_roll / foot_bank only, cumulative contact slip S <= "
                   "5 mm (T26e): walking from 0 toward each end one 1 deg step at a time, the contact set C_k of step "
                   "k = shoe vertices with z <= (lowest z at step k + 1 mm); the increment = mean horizontal (XY) "
                   "displacement of the C_k vertices from step k to step k+1; S(v) = sum of the increments from 0 to "
                   "v (a rolling contact stays put, only sliding contact accumulates, and a change of the lowest "
                   "vertex does not make S jump). Twist: S reported only (a twist rotates in place), clamp from reach "
                   "+ z. Pivot positions are those of the T26d search (unchanged in T26e). The clamp is the continuous planted range around 0. Stances 0 and -30 mm are "
                   "reference only (clamps_reference). Same range in the PROPS UI min/max and the driver clamps.",
}


# ---------------------------------------------------------------- geometry
def shoe_vertex_ids(mesh_obj, part_id):
    me = mesh_obj.data
    att = me.attributes["part_id"]
    if att.domain != "FACE":
        raise RuntimeError(f"part_id domain {att.domain}")
    pid = np.empty(len(me.polygons), dtype=np.int32)
    att.data.foreach_get("value", pid)
    ids = set()
    for p in me.polygons:
        if pid[p.index] == part_id:
            ids.update(p.vertices)
    return np.array(sorted(ids), dtype=np.int64)


def foot_frames(mesh_obj, pivots, parts):
    """Per side: forward (heel -> tip, horizontal), outward, pivot points heel/toe/bank_in/bank_out."""
    me = mesh_obj.data
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    mw = np.array(mesh_obj.matrix_world)
    co = co.reshape(-1, 3) @ mw[:3, :3].T + mw[:3, 3]
    out = {}
    for x in SIDES:
        heel = Vector(pivots[f"heel_{x}"]["co"])
        tip = Vector(pivots[f"foot_tip_{x}"]["co"])
        fwd = Vector((tip.x - heel.x, tip.y - heel.y, 0.0)).normalized()
        outward = Vector((-fwd.y, fwd.x, 0.0))
        if (outward.x > 0) != (x == "l"):
            outward = -outward
        ids = shoe_vertex_ids(mesh_obj, parts[f"shoe_{x}"])
        sc = co[ids]
        zmin = float(sc[:, 2].min())
        sole = sc[sc[:, 2] <= zmin + SOLE_BAND]
        s = sole[:, :2] @ np.array([outward.x, outward.y])
        v_out, v_in = sole[int(np.argmax(s))], sole[int(np.argmin(s))]
        out[x] = {"fwd": fwd, "outward": outward, "shoe_ids": ids, "shoe_zmin_rest": zmin,
                  "sole_verts": int(len(sole)),
                  "heel": Vector((heel.x, heel.y, 0.0)), "toe": Vector((tip.x, tip.y, 0.0)),
                  "bank_in": Vector((float(v_in[0]), float(v_in[1]), 0.0)),
                  "bank_out": Vector((float(v_out[0]), float(v_out[1]), 0.0)),
                  "bank_in_vert_z": float(v_in[2]), "bank_out_vert_z": float(v_out[2]), "shoe_co": sc}
    return out


# pivot search (T26d): kind -> (rotation axis key, horizontal search direction key, t range m, property values).
# The position along the rotation axis does not change the rotation, so the horizontal search is 1-D across the axis
# (fwd for heel / toe covering the whole shoe length, outward for bank covering the whole shoe width).
PIVOT_SEARCH = {
    "heel": ("lat", "fwd", (-0.02, 0.37), range(-1, START_CLAMPS["foot_roll"][0] - 1, -1)),
    "toe": ("lat", "fwd", (-0.37, 0.02), range(1, START_CLAMPS["foot_roll"][1] + 1)),
    "bank_in": ("fwd", "outward", (-0.03, 0.32), range(-1, START_CLAMPS["foot_bank"][0] - 1, -1)),
    "bank_out": ("fwd", "outward", (-0.32, 0.03), range(1, START_CLAMPS["foot_bank"][1] + 1)),
}
SEARCH_STEP = 0.001  # m, grid step of the pivot search (horizontal t and height h)
SEARCH_H = (0.0, 0.02)  # m, pivot height range (T26d: near the contact plane)
SEARCH_REACH_TOL = 0.0005  # m, geometric reach margin (|hip - ankle'| <= leg length + tol) at the clamp stance
SEARCH_Z_MARGIN = 0.00002  # m, search keeps z_min inside [Z_MIN, Z_MAX] shrunk by this (IK / float32 slack)
SEARCH_SLIP_MARGIN = 0.00005  # m, search keeps slip <= SLIP_MAX - this


def pivot_angle_deg(kind, v, bank_s):
    """Rotation (deg, right hand about the pivot bone's local X = lat for heel/toe, local Y = fwd for bank) that the
    drivers apply for property value v (same formulas as roll_exprs)."""
    if kind == "heel":
        return max(-v, 0.0)
    if kind == "toe":
        return -max(v, 0.0)
    if kind == "bank_in":
        return bank_s * min(v, 0.0)
    return bank_s * max(v, 0.0)


def search_pivots(arm, frames):
    """Per side / pivot: grid over (t across the axis, h height 0..20 mm) around the T26 pivot; rigid shoe rotated
    about the pivot axis for each property value (1 deg steps); planted = z_min in [Z_MIN, Z_MAX], slip of the lowest
    material point <= SLIP_MAX and ankle within leg reach of the hip at the clamp stance. Keeps the pivot with the
    longest continuous planted range from 0 (capped at the start range), ties -> smallest max slip, then smallest
    |t| + h."""
    ad = arm.data
    up = np.array([0.0, 0.0, 1.0])
    info = {}
    for x in SIDES:
        f = frames[x]
        fwd = np.array(f["fwd"])
        outward = np.array(f["outward"])
        lat = np.cross(fwd, up)
        bank_s = 1.0 if lat @ outward > 0 else -1.0
        dirs = {"fwd": fwd, "lat": lat, "outward": outward}
        V = f["shoe_co"]
        z_rest = float(V[:, 2].min())
        hip = np.array(ad.bones[f"thigh_{x}"].head_local) + np.array([0.0, 0.0, CLAMP_STANCE_MM / 1000.0])
        ankle = np.array(ad.bones[f"foot_{x}"].head_local)
        leg = ad.bones[f"thigh_{x}"].length + ad.bones[f"calf_{x}"].length
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
                zr = D[..., 2] * c + axD[..., 2] * sn  # a is horizontal (a_z = 0)
                imin = zr.argmin(axis=1)
                zmin = P[:, 2] + zr[rows, imin]
                d, axd, ad_ = D[rows, imin], axD[rows, imin], aD[rows, imin]
                low = P + d * c + axd * sn + np.outer(ad_, a) * (1.0 - c)  # lowest vertex at this value
                slip = np.linalg.norm(low[:, :2] - V[imin][:, :2], axis=1)  # vs the same vertex at value 0 (rest)
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
                             "cand": int(len(P)), "max_range": len(values), "range_at_t26": int(count[k0])}
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
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    angles, bank_sign = {}, {}
    up = Vector((0.0, 0.0, 1.0))
    for x in SIDES:
        for p in ("CTRL_pelvis", "CTRL_root"):
            if ebs.get(p) is None:
                raise RuntimeError(f"s06a bone {p} missing")
        for name, _, _, _ in new_bone_list(x):
            if ebs.get(name) is not None:
                raise RuntimeError(f"bone {name} already exists (expected input work/r04b.blend)")
        f = frames[x]
        for name, parent, src, connect in new_bone_list(x):
            eb = ebs.new(name)
            if src == "foot_ik":
                h = ebs[f"foot_{x}"].head.copy()
                eb.head, eb.tail = h, h + Vector((0.0, -FOOT_IK_LEN, 0.0))
                eb.align_roll(up)
            elif src in ("heel", "toe", "bank_in", "bank_out"):
                h = f[src]
                eb.head, eb.tail = h, h + f["fwd"] * PIVOT_LEN
                eb.align_roll(up)
            elif src == "pole":
                h = ebs[f"calf_{x}"].head + POLE_DIR * POLE_DIST
                eb.head, eb.tail, eb.roll = h, h + Vector((0.0, 0.0, POLE_LEN)), 0.0
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
        bank_sign[x] = 1.0 if ebs[n["bank_out"]].x_axis.dot(f["outward"]) > 0 else -1.0
    bpy.ops.object.mode_set(mode="OBJECT")
    return angles, bank_sign


def set_driver_var(drv, arm, prop, vname="v"):
    v = drv.variables.new()
    v.name = vname
    v.type = "SINGLE_PROP"
    v.targets[0].id_type = "OBJECT"
    v.targets[0].id = arm
    v.targets[0].data_path = f'pose.bones["PROPS"]["{prop}"]'


def new_driver(fc, arm, prop, expr):
    for m in list(fc.modifiers):
        fc.modifiers.remove(m)
    drv = fc.driver
    drv.type = "SCRIPTED"
    set_driver_var(drv, arm, prop)
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
    """{(bone key, euler index): (prop key, expression)} with clamp ranges cl[prop key] = (lo, hi)."""
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
    # IK chain
    for m in n["mch"][:2]:
        pbs[m].ik_stretch = 0.0
    ik = pbs[n["mch"][1]].constraints.new("IK")
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
    # foot roll drivers (starting clamps; final clamps written after the sweep)
    for (key, idx), (pk, expr) in roll_exprs(bank_s, START_CLAMPS).items():
        fc = pbs[n[key]].driver_add("rotation_euler", idx)
        drivers[f"{n[key]}.rotation_euler[{idx}]"] = (new_driver(fc, arm, n["roll_props"][pk], expr), key, idx, pk)
    # knee pole space
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
    # DEF blend
    ikfk = float(props[n["prop_ikfk"]])
    for fk, mch, d in zip(n["fk"], n["mch"], n["def"]):
        copy_tf(arm, pbs[d], "fk_copy", fk)
        c = copy_tf(arm, pbs[d], "ik_copy", mch, influence=ikfk)
        drivers[f"{d}.ik_copy"] = (new_driver(c.driver_add("influence"), arm, n["prop_ikfk"], "v"), None, None, None)
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


# ---------------------------------------------------------------- evaluation helpers
def set_prop(arm, name, value):
    props = arm.pose.bones["PROPS"]
    props[name] = type(props[name])(value)
    arm.update_tag()


def shoe_coords(mesh_obj, ids):
    """World coordinates (n, 3) of the evaluated shoe vertices (updates the depsgraph first)."""
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    ev = mesh_obj.evaluated_get(dg)
    me = ev.to_mesh()
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    ev.to_mesh_clear()
    mw = np.array(mesh_obj.matrix_world)
    return co.reshape(-1, 3)[ids] @ mw[:3, :3].T + mw[:3, 3]


def planted_state(arm, mesh_obj, x, ids, prev=None, prev_s=0.0, use_slip=False):
    """(reach error m = |MCH_calf_ik tail - MCH_roll_foot head|, lowest shoe vertex z m, failure causes, cumulative
    contact slip S m, shoe coords). T26e: S = prev_s + mean XY displacement, from prev (the previous 1 deg step) to
    now, of the previous step's contact set (shoe verts with z <= its lowest z + CONTACT_BAND); prev None -> S 0."""
    co = shoe_coords(mesh_obj, ids)
    z = float(co[:, 2].min())
    slip = prev_s
    if prev is not None:
        c = prev[:, 2] <= prev[:, 2].min() + CONTACT_BAND
        slip = prev_s + float(np.linalg.norm(co[c, :2] - prev[c, :2], axis=1).mean())
    pbs = arm.pose.bones
    n = names(x)
    mw = arm.matrix_world
    reach = ((mw @ pbs[n["mch"][1]].tail) - (mw @ pbs[n["roll_foot"]].head)).length
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
    a = pbs[n["mch"][0]]
    b = pbs[n["mch"][1]]
    return math.degrees((a.tail - a.head).angle(b.tail - b.head))


def clamp_sweep(arm, mesh_obj, frames):
    """Planted sweep: per stance (CTRL_torso loc Y = world Z offset), leg IK, other controls identity; each roll
    property alone in 1 deg steps over its whole starting range; planted range = continuous range around 0 where
    reach error <= REACH_TOL and Z_MIN <= lowest shoe z <= Z_MAX. Final clamp = CLAMP_STANCE_MM result."""
    torso = arm.pose.bones["CTRL_torso"]
    res = {}
    stances = {}
    for st in STANCES_MM:
        torso.location = (0.0, st / 1000.0, 0.0)
        arm.update_tag()
        bpy.context.view_layer.update()
        info = {}
        for x in SIDES:
            ids = frames[x]["shoe_ids"]
            reach0, z0, c0, _, ref = planted_state(arm, mesh_obj, x, ids)  # value-0 state (all props 0)
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
                    prev, prev_s = ref, 0.0  # cumulative contact slip walks from 0 toward the end
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
                    r[f"at_last_ok_{tag}"] = (samples.get(last_ok, (reach0, z0, c0, 0.0)) if last_ok is not None
                                              else None)
                    r[f"at_first_fail_{tag}"] = samples.get(first_fail) if first_fail is not None else None
                    r[f"at_end_{tag}"] = samples[end]
                    r[f"max_slip_planted_{tag}"] = (max([0.0] + [samples[v][3] for v in samples
                                                                 if last_ok is not None and 0 < v * sgn <= last_ok * sgn])
                                                    if last_ok is not None else None)
                    r[f"max_slip_all_{tag}"] = max(t[3] for t in samples.values())
                    rng.append(last_ok)
                r["planted"] = None if c0 else rng
                res.setdefault(prop, {"stances": {}})["stances"][st] = r
        stances[st] = info
    torso.location = (0.0, 0.0, 0.0)
    arm.update_tag()
    bpy.context.view_layer.update()
    for prop, d in res.items():
        pl = d["stances"][CLAMP_STANCE_MM]["planted"]
        d["final"] = list(pl) if pl is not None else [0, 0]
    return res, stances


def apply_clamps(arm, drivers, bank_sign, clamp_res):
    """Final clamp ranges -> driver expressions and PROPS UI min/max."""
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
def manifest_entries(angles, frames, bank_sign, clamp_res, arm):
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
            "mch": n["space"], "constraints": cons, "constraint_type": {c: "ARMATURE" for c in cons},
            "targets": tg, "influence": {s[1]: f"driver PROPS[prop] == {k}" for k, s in enumerate(POLE_SPACES)},
            "value_constraint": {s[0]: s[1] for s in POLE_SPACES},
            "value_effect": {s[0]: f"full follow of {tg[s[1]]}" for s in POLE_SPACES}}
        foot_rest = ad.bones[f"foot_{x}"].matrix_local
        ik_rest = ad.bones[n["ik"]].matrix_local
        off = foot_rest.inverted() @ ik_rest
        ikfk[f"leg_{x}"] = {
            "owner": OWNER, "prop": n["prop_ikfk"], "fk_value": 0, "ik_value": 1,
            "fk": n["fk"], "ik": n["ik"], "pole": n["pole"], "def": n["def"], "mch_ik": n["mch"],
            "fk_to_def": dict(zip(n["fk"], n["def"])), "fk_to_mch_ik": dict(zip(n["fk"], n["mch"])),
            "def_constraints": {"fk": "fk_copy", "ik": "ik_copy"},
            "ik_constraint": {"bone": n["mch"][1], "name": "IK", "chain_count": 2, "use_tail": True,
                              "use_stretch": False, "target": n["roll_foot"], "pole_angle_rad": angles[x],
                              "pole_angle_deg": math.degrees(angles[x])},
            "ik_foot_rotation": {"bone": n["mch"][2], "constraint": "IK_rot", "type": "COPY_ROTATION",
                                 "source": n["roll_foot"]},
            "roll_chain": [n["heel"], n["toe"], n["bank_in"], n["bank_out"], n["roll_foot"]],
            "roll_props": list(n["roll_props"].values()),
            "ik_space": {"prop": n["prop_space"], "mch": n["space"]},
            "pole_parent": n["space"], "pole_distance": POLE_DIST,
            "ik_ctrl_in_def_foot_rest": [list(r) for r in off],
            "snap": {"fk_from_ik": "for fk, mch in fk_to_mch_ik in chain order: fk world matrix = mch world matrix "
                                   "(roll props keep their values; the FK pose includes the roll result), then prop "
                                   "= fk_value",
                     "ik_from_fk": "set every roll_props value to 0 (arm.update_tag + depsgraph update) so "
                                   "MCH_roll_foot = CTRL_foot_ik * rest offset; CTRL_foot_ik world matrix = DEF/"
                                   "CTRL_foot_fk world matrix @ ik_ctrl_in_def_foot_rest; pole world location = calf "
                                   "head + pole_distance * unit(knee offset from the thigh-head -> foot-head line, FK "
                                   "result); then prop = ik_value"},
        }
        for pk in START_CLAMPS:
            clamps[n["roll_props"][pk]] = list(clamp_res[n["roll_props"][pk]]["final"])
    return controls, spaces, ikfk, clamps


def write_manifest(entries, clamp_res, stances):
    p = goblib.DATA / MANIFEST
    with open(p, "r", encoding="utf-8") as f:
        old = json.load(f)
    man = dict(old)
    man["_doc"] = dict(old.get("_doc", {}))
    man["_doc"].update(DOC)
    controls, spaces, ikfk, clamps = entries
    for sec, new in (("controls", controls), ("spaces", spaces), ("ikfk", ikfk)):
        keep = {k: v for k, v in old.get(sec, {}).items() if not (isinstance(v, dict) and v.get("owner") == OWNER)}
        keep.update(new)
        man[sec] = keep
    cl = dict(old.get("clamps", {}))
    cl.update(clamps)
    man["clamps"] = cl
    man["clamps_reference"] = {
        "owner": OWNER, "chosen_stance_mm": CLAMP_STANCE_MM,
        "criterion": {"reach_tol_m": REACH_TOL, "z_min_m": Z_MIN, "z_max_m": Z_MAX, "slip_max_m": SLIP_MAX,
                      "slip_props": list(SLIP_PROPS)},
        "stances": {str(st): {"knee_bend_deg": {x: stances[st][x]["knee_bend_deg"] for x in SIDES},
                              "planted": {prop: d["stances"][st]["planted"] for prop, d in clamp_res.items()}}
                    for st in STANCES_MM}}
    return goblib.save_json(MANIFEST, man)


# ---------------------------------------------------------------- self-checks / report
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
    arm.update_tag()


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


def g62(arm, canon):
    props = arm.pose.bones["PROPS"]
    arm.data.pose_position = "POSE"
    ik = def_vs_canon(arm, canon)
    infl = {x: arm.pose.bones[f"thigh_{x}"].constraints["ik_copy"].influence for x in SIDES}
    print(f"[s06c] G6.2 SELFCHECK IK (defaults: leg_ik_fk_l={props['leg_ik_fk_l']}, leg_ik_fk_r="
          f"{props['leg_ik_fk_r']}, ik_copy influence l={infl['l']:.3f} r={infl['r']:.3f}, roll props 0, all CTRL "
          f"identity): DEF vs canonical rest max abs diff = {ik[0]:.3e} (bone {ik[1]}), threshold 1e-4")
    for x in SIDES:
        set_prop(arm, f"leg_ik_fk_{x}", 0.0)
    fk = def_vs_canon(arm, canon)
    infl = {x: arm.pose.bones[f"thigh_{x}"].constraints["ik_copy"].influence for x in SIDES}
    print(f"[s06c] G6.2 SELFCHECK FK (leg_ik_fk_l/r = 0, ik_copy influence l={infl['l']:.3f} r={infl['r']:.3f}, all "
          f"CTRL identity): DEF vs canonical rest max abs diff = {fk[0]:.3e} (bone {fk[1]}), threshold 1e-4")
    for x in SIDES:
        set_prop(arm, f"leg_ik_fk_{x}", 1.0)
    after = def_vs_canon(arm, canon)
    print(f"[s06c] restored leg_ik_fk_l={props['leg_ik_fk_l']}, leg_ik_fk_r={props['leg_ik_fk_r']}: DEF vs canonical "
          f"max abs diff = {after[0]:.3e}")
    return ik[0], fk[0]


def report(arm, drivers, angles, frames, bank_sign):
    ad = arm.data
    for x in SIDES:
        f = frames[x]
        print(f"[s06c] FOOT FRAME {x}: fwd={fmt_v(f['fwd'])}, outward={fmt_v(f['outward'])}, shoe verts="
              f"{len(f['shoe_ids'])}, shoe z min (rest)={f['shoe_zmin_rest'] * 1000:+.3f} mm, sole verts (z <= min + "
              f"{SOLE_BAND * 1000:.0f} mm)={f['sole_verts']}; pivots heel={fmt_v(f['heel'])}, toe={fmt_v(f['toe'])}, "
              f"bank_in={fmt_v(f['bank_in'])} (vert z {f['bank_in_vert_z'] * 1000:+.2f} mm), bank_out="
              f"{fmt_v(f['bank_out'])} (vert z {f['bank_out_vert_z'] * 1000:+.2f} mm), bank sign={bank_sign[x]:+.0f}")
        for name, _, _, _ in new_bone_list(x):
            b = ad.bones[name]
            pb = arm.pose.bones[name]
            cons = []
            for c in pb.constraints:
                if c.type == "ARMATURE":
                    cons.append(f"ARMATURE:{c.name}->{','.join(t.subtarget for t in c.targets)}")
                elif c.type == "IK":
                    cons.append(f"IK:{c.name}->{c.subtarget} pole {c.pole_subtarget} angle "
                                f"{math.degrees(c.pole_angle):+.4f} deg chain {c.chain_count} stretch {c.use_stretch}")
                else:
                    cons.append(f"{c.type}:{c.name}->{c.subtarget}")
            print(f"[s06c] BONE {name}: parent={b.parent.name if b.parent else None}, connect={b.use_connect}, "
                  f"head={fmt_v(b.head_local)}, tail={fmt_v(b.tail_local)}, x_axis={fmt_v(b.x_axis)}, "
                  f"z_axis={fmt_v(b.z_axis)}, deform={b.use_deform}, coll={[c.name for c in b.collections]}, "
                  f"constraints={'; '.join(cons) or '-'}")
        for d in names(x)["def"]:
            pb = arm.pose.bones[d]
            print(f"[s06c] DEF {d}: " + "; ".join(f"{c.type}:{c.name}<-{c.subtarget} infl {c.influence:.3f}"
                                                  for c in pb.constraints))
        print(f"[s06c] POLE {x}: pole_angle={angles[x]:+.6f} rad ({math.degrees(angles[x]):+.4f} deg), pole head="
              f"{fmt_v(ad.bones[names(x)['pole']].head_local)}")
    for tag, (drv, *_r) in drivers.items():
        print(f"[s06c] DRIVER {tag}: expr '{drv.expression}', valid={drv.is_valid}, simple={drv.is_simple_expression}")


def pivot_doc(frames, pinfo):
    parts = []
    for x in SIDES:
        f = frames[x]
        parts.append(f"{x}: " + ", ".join(
            f"{k} {fmt_v(f[k])} (T26 {fmt_v(f['before'][k])}, geometric planted range {pinfo[x][k]['range']} deg, "
            f"max slip {pinfo[x][k]['maxslip'] * 1000:.2f} mm)"
            for k in PIVOT_SEARCH))
    return ("T26d pivot placement (T26c high pivots, z 64-290 mm, slid the contact along the ground by about height x "
            "angle, so the pivots are kept near the contact plane): grid search per side and pivot, height h 0..20 mm "
            "and horizontal t across the rotation axis (fwd for heel / toe over the whole shoe length, outward for "
            "bank over the whole shoe width; the position along the axis does not change the rotation), 1 mm steps. "
            "The rigid shoe (its own vertices) is rotated about the pivot axis (lat for roll, fwd for bank) exactly "
            "as the drivers do for each property value in 1 deg steps; a value counts as planted when the lowest "
            "shoe vertex stays in [-1, +3] mm (0.02 mm margin), the slip of the lowest material point stays <= 5 mm "
            "(0.05 mm margin) and the rotated ankle stays within leg length + 0.5 mm of the hip at the -15 mm stance. "
            "Chosen = longest continuous planted range from 0 in the target direction (capped at the start range), "
            "then smallest max slip, then smallest offset, so each pivot sits at the sole contact edge / arc "
            "reached by that roll or bank direction. Twist pivots are the same heel / toe bones (vertical axis "
            "through the new head). " + "; ".join(parts))


def main():
    print(f"[s06c] Blender {bpy.app.version_string}")
    arm = bpy.data.objects[ARM]
    mesh_obj = bpy.data.objects[MESH]
    canon = goblib.load_json("canonical_skeleton.json")
    pivots = goblib.load_json("pivots.json")["pivots"]
    parts = goblib.load_json("parts.json")
    frames = foot_frames(mesh_obj, pivots, parts)
    pinfo = search_pivots(arm, frames)
    for x in SIDES:
        f = frames[x]
        for kind in PIVOT_SEARCH:
            i = pinfo[x][kind]
            print(f"[s06c] PIVOT {kind}_{x}: before {fmt_v(f['before'][kind])} -> after {fmt_v(f[kind])} (t "
                  f"{i['t'] * 1000:+.0f} mm along {PIVOT_SEARCH[kind][1]}, h {i['h'] * 1000:.0f} mm); geometric "
                  f"planted range {i['range']}/{i['max_range']} deg (T26 pivot {i['range_at_t26']}), max |z_min - "
                  f"z_rest| {i['maxdev'] * 1000:.3f} mm, max slip {i['maxslip'] * 1000:.3f} mm, candidates with "
                  f"full range {i['n_full']}/{i['cand']}")
    DOC["s06c_pivots"] = pivot_doc(frames, pinfo)
    angles, bank_sign = edit_bones(arm, frames)
    drivers = {}
    for x in SIDES:
        drivers.update(pose_setup(arm, x, angles[x], bank_sign[x]))
    assign_collections(arm)
    reset_new(arm)
    arm.data.pose_position = "POSE"
    clamp_res, stances = clamp_sweep(arm, mesh_obj, frames)
    apply_clamps(arm, drivers, bank_sign, clamp_res)
    report(arm, drivers, angles, frames, bank_sign)

    def st_txt(t):
        return "-" if t is None else (f"reach {t[0] * 1000:.3f} mm, z {t[1] * 1000:+.3f} mm, S "
                                      f"{t[3] * 1000:.2f} mm, fail {t[2] or 'none'}")

    def mm(v):
        return "-" if v is None else f"{v * 1000:.2f}"

    for st in STANCES_MM:
        for x in SIDES:
            i = stances[st][x]
            print(f"[s06c] STANCE {st:+d} mm {x}: CTRL_torso loc Y = {st / 1000:+.3f} m, knee bend (MCH thigh/calf ik "
                  f"angle) = {i['knee_bend_deg']:.3f} deg; at props 0: reach {i['reach0'] * 1000:.3f} mm, shoe z "
                  f"{i['z0'] * 1000:+.3f} mm, fail {i['causes0'] or 'none'}")
        for prop, d in clamp_res.items():
            r = d["stances"][st]
            print(f"[s06c] PLANTED {st:+d} mm {prop}: start {r['start']}, planted range {r['planted']}; neg: last ok "
                  f"{r['last_ok_neg']} ({st_txt(r['at_last_ok_neg'])}), first fail {r['first_fail_neg']} "
                  f"({st_txt(r['at_first_fail_neg'])}), end {r['start'][0]} ({st_txt(r['at_end_neg'])}); pos: last "
                  f"ok {r['last_ok_pos']} ({st_txt(r['at_last_ok_pos'])}), first fail {r['first_fail_pos']} "
                  f"({st_txt(r['at_first_fail_pos'])}), end {r['start'][1]} ({st_txt(r['at_end_pos'])}); max S "
                  f"in planted range neg/pos {mm(r['max_slip_planted_neg'])}/{mm(r['max_slip_planted_pos'])} mm, "
                  f"over whole sweep {mm(r['max_slip_all_neg'])}/{mm(r['max_slip_all_pos'])} mm, slip in criterion "
                  f"{r['slip_in_criterion']}")
    print(f"[s06c] FINAL CLAMPS (stance {CLAMP_STANCE_MM:+d} mm): "
          + ", ".join(f"{p}={d['final']}" for p, d in clamp_res.items()))
    pr = arm.pose.bones["PROPS"]
    print("[s06c] PROPS UI after clamp: " + ", ".join(
        f"{p}=[{pr.id_properties_ui(p).as_dict()['min']}, {pr.id_properties_ui(p).as_dict()['max']}]"
        for p in clamp_res))
    ik_err, fk_err = g62(arm, canon)
    nondef = [nm for x in SIDES for nm, _, _, _ in new_bone_list(x) if arm.data.bones[nm].use_deform]
    print(f"[s06c] new bones with use_deform True: {nondef}; total bones={len(arm.data.bones)}; collections: "
          + ", ".join(f"{c.name}(visible={c.is_visible}, bones={len(c.bones)})" for c in arm.data.collections_all))
    out = write_manifest(manifest_entries(angles, frames, bank_sign, clamp_res, arm), clamp_res, stances)
    print(f"[s06c] MANIFEST {out}")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s06c] OUTPUT {OUT_BLEND}")
    print(f"[s06c] G6.2 IK max_err={ik_err:.3e}, FK max_err={fk_err:.3e}")


if __name__ == "__main__":
    goblib.run_main(main)
