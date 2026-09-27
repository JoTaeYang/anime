"""s10a_swing_keyposes.py - goblin club swing key poses (T50, spec d-25 A1).

Input  rig/gob_r04_ctrl.blend (read only; the run saves a copy).
Output rig/gob_r06_swing.blend (copy=True, save_version 0): action `attack_swing` (24 fps, frames 1-39) with the 7 key
       poses keyed on CTRL_* pose bones (location, rotation_euler, scale) and PROPS, CONSTANT interpolation (blocking).
       rig/data/swing_keyposes.json  poses (frame, name, intent, CTRL values, props, contact) + contact_ranges + the
                                     self measurement of every pose
       rig/data/swing_cam.json       front camera fitted to the reference framing (f1) + the 3/4 camera
       rig/inspect/A1/               keyposes_sheet.png (rows = poses, columns = reference | front | 3/4) and per pose
                                     f<NN>_<name>_{front,34,row}.png

Pose construction: goblin.reset_rig + goblin.ready_pose (blend text goblin_rig_ui.py registered as a module) give the
base stance; every pose = base CTRL state + the literal CTRL / PROPS values of POSES below (arms FK, legs IK, weapon in
the hand_r space). Values are absolute control-local values (rotation degrees in the controls' XYZ Euler, location m).

Run (repo root):
  powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s10a_swing_keyposes.py -Blend gob_r04_ctrl.blend
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from bpy_extras import anim_utils  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

ARM, MESH, CLUB, PROPS = "GOB_rig", "GOB_mesh", "GOB_club", "PROPS"
TEXT_UI = "goblin_rig_ui.py"
ACTION, FPS, FRAME_START, FRAME_END = "attack_swing", 24, 1, 39
OUT_BLEND = goblib.RIG / "gob_r06_swing.blend"
KEYPOSE_JSON, CAM_JSON = "swing_keyposes.json", "swing_cam.json"
INSP = goblib.INSPECT / "A1"
REF_DIR = goblib.RIG.parent / "ref"
AXES = "XYZ"
EPS = 1e-6

# ---------------------------------------------------------------- safe range (spec d-25 section 2, draft)
SAFE = {
    "upperarm_raise_max": 100.0,      # FK upperarm raise (upperarm_l Z+, upperarm_r Z-)
    "shoulder_up_max": 25.0,          # CTRL_shoulder raise (same sign as the upperarm raise)
    "upperarm_fwd_max": 90.0,         # upperarm X+
    "upperarm_back_min": -45.0,       # upperarm X-
    "elbow_max": 100.0,               # lowerarm X
    "wrist_bend_max": 40.0,           # |hand X|
    "wrist_twist_max": 90.0,          # |hand Y|
    "hip_flex_max": 60.0,             # DEF thigh -X (local to DEF pelvis)
    "knee_max": 90.0,                 # DEF calf X
    "cog_min": -0.08,                 # CTRL_torso loc Y
    "spine_axis_max": 30.0,           # CTRL_spine_01 / CTRL_chest / CTRL_pelvis, each axis
    "ik_ratio_max": 0.97,             # |hip - IK target| / (thigh + calf)
    "club_clear_min_mm": 10.0,
    "reach_mm_max": 1.0,              # planted feet
    "shoe_z_mm": (-1.0, 3.0),         # planted feet: lowest shoe vertex z
    "isect_cover_max": 1,
}

# ---------------------------------------------------------------- key poses
# ctrl: {control: {"loc": [x, y, z] m, "rot": [x, y, z] deg}} absolute local values; controls not listed keep the
# ready_pose value. props: PROPS overrides (defaults = ready_pose state).
POSES = []  # filled in POSE TABLE below


def P(frame, name, intent, ctrl, props=None, contact=("l", "r"), notes=None):
    POSES.append({"frame": frame, "name": name, "intent": intent, "ctrl": ctrl, "props": props or {},
                  "contact": list(contact), "notes": notes or {}})


# POSE TABLE ---------------------------------------------------------------------------------------------------------
# Values were designed with a scratch solver (targets = reference landmarks mapped through the fitted front camera,
# FK arm values from analytic FK + Blender-in-the-loop search on self intersection / club clearance) and are written
# here as literals so the run is reproducible. Design limits re-measured with the VISIBLE self intersection
# (d-25 A1.3 rev. 2026-09-25, T50b): visible knee crease > ~60 deg (COG below -0.02 with vertical shins), visible
# armpit crease when an upper arm is lowered > ~35-40 deg (to ~45 deg with back swing + twist + CTRL_shoulder), head
# yaw visible beyond about +-10 deg; spine_01 back bend and CTRL_shoulder raise are hidden (no longer limits).
# Deeper COG uses CTRL_pelvis X +10 (hips back, hip flexion instead of knee bend). f15 / f24 (T50b): moderate
# torso (lean ~25-27 deg, face toward the camera, head counter-turned -10), the arm swing carries the crossing and the
# club head lands in front of the left foot (~50 mm above the ground at impact).
WIDE_R = [0.065, 0.01, 0.0]          # right foot wide stance (step out 65 mm, 10 mm forward), f13-f26
IMPACT_BASE = {"CTRL_torso": {"loc": [-0.045, -0.045, 0.0]}, "CTRL_pelvis": {"rot": [10.0, 0.0, 0.0]},
               "CTRL_spine_01": {"rot": [18.0, 10.0, -3.0]}, "CTRL_head": {"rot": [-12.0, -10.0, 0.0]},
               "CTRL_foot_ik_r": {"loc": WIDE_R}}


def arm(side, sh, ua, la, hand):
    return {f"CTRL_shoulder_{side}": {"rot": list(sh)}, f"CTRL_upperarm_fk_{side}": {"rot": list(ua)},
            f"CTRL_lowerarm_fk_{side}": {"rot": [la, 0.0, 0.0]}, f"CTRL_hand_fk_{side}": {"rot": list(hand)}}


def merge(*ds):
    out = {}
    for d in ds:
        out.update({k: dict(v) for k, v in d.items()})
    return out


P(1, "idle",
  "Reference idle (club vertical beside the right hip, head up; free arm hanging). ready_pose stance unchanged. "
  "The armpit crease is still a VISIBLE intersection beyond ~40-45 deg of lowering, so the arms hang as far as "
  "that allows: left upper arm lowered 43.5 deg (+ shoulder forward 10), right 40 deg with the shoulder drawn back "
  "18, elbows bent 42 / 60; the right wrist cocks the club upright and slightly outward. Loop pose (f39 = f1).",
  merge(arm("r", (-18.0, 0, -6.0), (6.5, -20.5, 40.0), 60.0, (49.1, 50.7, 15.4)),
        arm("l", (10.0, 0, 0), (-20.0, 19.5, -43.5), 41.5, (1.5, 0.0, 1.5))),
  notes={"reference_frame": "ref/keyframes/key_0001.png"})
P(9, "anticipation",
  "Anticipation peak: body rises 9 mm (COG -0.006, leg IK ratio 0.96 < 0.97) and leans back slightly, twisting "
  "right (spine/chest -8/-10); club swung up and inboard with the head above the right shoulder / head corner "
  "(fist at the reference position); free arm swung out and down.",
  merge({"CTRL_torso": {"loc": [0.0, -0.006, 0.0]}, "CTRL_spine_01": {"rot": [-4.0, -8.0, 0.0]},
         "CTRL_chest": {"rot": [-6.0, -10.0, 2.0]}, "CTRL_head": {"rot": [-5.0, 3.0, 0.0]}},
        arm("r", (0, 0, 0), (61.8, 52.2, 57.0), 56.6, (-5.2, -1.2, -1.5)),
        arm("l", (0, 0, 0), (-43.1, 0.0, -21.9), 47.5, (0.0, 0.0, 0.0))),
  notes={"reference_frame": "ref/keyframes/key_0009.png"})
P(12, "max_windup",
  "Maximum wind-up, adapted: instead of laying the club behind the head, the fist is high beside the right side of "
  "the head (shoulder up 16, upperarm raised) and the club points back-up-out over the right shoulder (head "
  "clearance >= 10 mm); torso leans back and twists right (spine/chest -12/-16); free arm thrown high. Right foot in "
  "the air on its way to the wide stance (35 mm out, 12 mm up, toes up 8 = heel first).",
  merge({"CTRL_torso": {"loc": [0.0, -0.018, 0.0]}, "CTRL_spine_01": {"rot": [-6.0, -12.0, 0.0]},
         "CTRL_chest": {"rot": [-8.0, -16.0, -3.0]}, "CTRL_head": {"rot": [0.0, 4.0, -4.0]},
         "CTRL_foot_ik_r": {"loc": [0.035, 0.01, 0.012], "rot": [8.0, 0.0, 0.0]}},
        arm("r", (10.0, 0, -16.0), (88.0, 64.5, 66.4), 39.6, (-7.0, 87.0, 5.3)),
        arm("l", (0, 0, 0), (-25.2, 0.2, 51.7), 0.0, (0.0, 0.0, 0.0))),
  contact=("l",), notes={"reference_frame": "ref/keyframes/key_0012.png", "right_foot": "in the air (f10-12)"})
P(13, "swing_top",
  "Top of the arc: club straight up over the right shoulder (highest point of the clip), arm high with the elbow "
  "bent (not fully extended), DEF wrist bend <= 40, FK; body starts down and to the left (COG -0.03, hips 20 mm to the "
  "right, lean left, spine/chest start twisting left); right foot lands in the wide stance; free arm out to the side.",
  merge({"CTRL_torso": {"loc": [-0.02, -0.03, 0.0]}, "CTRL_pelvis": {"rot": [5.0, 0.0, 0.0]},
         "CTRL_spine_01": {"rot": [4.0, 4.0, -5.0]}, "CTRL_chest": {"rot": [4.0, 6.0, -6.0]},
         "CTRL_head": {"rot": [8.0, 2.0, -2.0]}, "CTRL_foot_ik_r": {"loc": WIDE_R}},
        arm("r", (0, 0, -4.0), (88.0, 79.0, 66.6), 57.8, (-20.6, 88.2, 17.5)),
        arm("l", (0, 0, 0), (-29.2, 0.0, 55.5), 49.4, (0.0, 0.0, 0.0))),
  notes={"reference_frame": "ref/keyframes/key_0013.png", "right_foot": "lands (wide stance)"})
P(15, "impact",
  "Impact (T50b, less hunched like the reference): club head comes down in front of the left foot and stops ~50 mm "
  "above the ground. The crossing comes from the arm swing (right shoulder protracted 28, straight arm swung "
  "forward-across) plus a moderate torso twist left (spine 10 + chest 25), forward bend 18 + 10 and chest lean "
  "left 6 -> hip-eye lean ~25 deg; head counter-turned -10 and pitched back -12 so the face reads toward the camera "
  "looking down. COG -0.045 with the hips 45 mm to the character's right (counter shift), pelvis X 10 so the knees "
  "stay < 60 deg. Free arm raised out to the left with the elbow bent 60.",
  merge(IMPACT_BASE, {"CTRL_chest": {"rot": [10.0, 25.0, -6.0]}},
        arm("r", (28.0, 0, 0.0), (55.0, 0.0, 21.5), 0.0, (-37.9, -10.7, 17.0)),
        arm("l", (0, 0, 0), (-20.0, -27.0, 40.0), 60.0, (0.0, 0.0, 0.0))),
  notes={"reference_frame": "ref/keyframes/key_0015.png"})
P(24, "follow_extreme",
  "Follow-through extreme: impact pose held and settled - chest leans 3 deg further left (hip-eye lean ~27), club "
  "lifted clear of the ground (~80 mm) and drawn slightly back toward the body by the elbow (45) and upper arm - "
  "the shoulder stays at the impact setting (forward 22, no shrug; T52b); free arm still raised. Start of recovery.",
  merge(IMPACT_BASE, {"CTRL_chest": {"rot": [10.0, 25.0, -9.0]}},
        arm("r", (22.0, 0, 0.0), (64.7, -27.8, -50.4), 45.2, (-18.2, 83.6, 19.9)),
        arm("l", (0, 0, 0), (-20.0, -45.0, 40.0), 60.0, (0.0, 0.0, 0.0))),
  notes={"reference_frame": "ref/keyframes/key_0024.png"})
P(29, "recovery_mid",
  "Recovery mid: body almost upright again (COG -0.007, lean ~2 deg), club swung back up on the right side pointing "
  "up-out, fist beside the right hip; right foot mid-STEP back to the idle spot (lifted 18 mm, halfway), not the "
  "reference slide; free arm dropping.",
  merge({"CTRL_torso": {"loc": [-0.01, -0.007, 0.0]}, "CTRL_spine_01": {"rot": [1.0, -3.0, -2.0]},
         "CTRL_chest": {"rot": [2.0, -4.0, -2.0]}, "CTRL_head": {"rot": [4.0, 2.0, -1.0]},
         "CTRL_foot_ik_r": {"loc": [0.0325, 0.005, 0.018]}},
        arm("r", (0, 0, 0), (-1.5, -16.5, 35.0), 60.0, (38.0, 80.7, 13.8)),
        arm("l", (0, 0, 0), (-33.5, 0.0, -28.7), 58.4, (0.0, 0.0, 0.0))),
  contact=("l",), notes={"reference_frame": "ref/keyframes/key_0029.png", "right_foot": "in the air (f27-31)"})
# --------------------------------------------------------------------------------------------------------------------

CONTACT_RANGES = {
    "l": [[1, 39]],
    "r": [[1, 9], [13, 26], [32, 39]],
}
CONTACT_NOTES = {
    "l": "left foot planted for the whole clip (reference: never moves)",
    "r": "right foot: planted at the idle spot f1-9, lifts f10, in the air f10-12 (f12 = coming down, heel first "
         "about to touch), lands in the wide stance f13, planted f13-26, lifts f27, steps back in the air f27-31 "
         "(a step, not the reference slide), lands on the idle spot f32, planted f32-39",
}

# ---------------------------------------------------------------- render / camera
RES_W, RES_H = 896, 1152            # = ref/keyframes key_NNNN.png (2x of the 448x576 video)
REF_W, REF_H = 448, 576
MESH_RGBA = (0.60, 0.66, 0.50, 1.0)   # light green-gray
CLUB_RGBA = (0.52, 0.40, 0.30, 1.0)   # brown-gray (club readable against the body)
# reference f1 landmarks (metrics.json frame 1, 448x576 px, origin top-left): head top = green_body_top_y,
# belt bbox = hips_belt_bbox, feet = feet_bboxes ([0] = screen-left = character right foot)
REF_F1 = {"head_top_y": 108.0, "belt_bbox": [122, 324, 331, 379],
          "foot_r_bbox": [110, 448, 193, 486], "foot_l_bbox": [255, 448, 341, 486]}
ELEV_GRID = [x * 0.25 for x in range(0, 101)]  # deg, 0..25
TQ_AZ, TQ_EL = 45.0, 15.0               # 3/4 camera: front turned 45 deg toward the character's right, 15 deg up


# ================================================================ rig helpers
def refresh(rig):
    rig.update_tag()
    bpy.context.view_layer.update()


def ctrl_names(rig):
    return [pb.name for pb in rig.pose.bones if pb.name.startswith("CTRL_")]


def prop_names(rig):
    pr = rig.pose.bones[PROPS]
    return [k for k in pr.keys() if isinstance(pr[k], (int, float))]


def snapshot(rig):
    pbs = rig.pose.bones
    c = {n: {"loc": list(pbs[n].location), "rot": [math.degrees(a) for a in pbs[n].rotation_euler]}
         for n in ctrl_names(rig)}
    p = {k: pbs[PROPS][k] for k in prop_names(rig)}
    return c, p


def sweep_table(man):
    return {(c, s["channel"], s["axis"]): (s["min"], s["max"]) for c, e in man["controls"].items() for s in e["sweep"]}


def apply_state(rig, man, base_c, base_p, pose):
    """Base CTRL / PROPS state + pose overrides (sweep / clamp checked), then update."""
    pbs = rig.pose.bones
    sw = sweep_table(man)
    vals = {n: {"loc": list(v["loc"]), "rot": list(v["rot"])} for n, v in base_c.items()}
    for n, v in pose["ctrl"].items():
        if n not in vals:
            raise RuntimeError(f"{pose['name']}: unknown control {n}")
        for ch in ("loc", "rot"):
            if ch in v:
                vals[n][ch] = [float(x) for x in v[ch]]
    for n, v in vals.items():
        for ch in ("loc", "rot"):
            for i, x in enumerate(v[ch]):
                rng = sw.get((n, ch, AXES[i]))
                if rng is None:
                    if abs(x) > (1e-6 if ch == "loc" else 1e-4):
                        raise RuntimeError(f"{pose['name']}: {n} {ch} {AXES[i]} = {x} not in sweep")
                elif not rng[0] - EPS <= x <= rng[1] + EPS:
                    raise RuntimeError(f"{pose['name']}: {n} {ch} {AXES[i]} = {x} outside sweep {rng}")
        pb = pbs[n]
        pb.location = v["loc"]
        pb.rotation_euler = [math.radians(a) for a in v["rot"]]
        pb.scale = (1.0, 1.0, 1.0)
    props = dict(base_p)
    props.update(pose["props"])
    for k, x in props.items():
        if k in man.get("clamps", {}):
            lo, hi = man["clamps"][k]
            if not lo - EPS <= x <= hi + EPS:
                raise RuntimeError(f"{pose['name']}: PROPS {k} = {x} outside clamp {lo, hi}")
        pbs[PROPS][k] = type(pbs[PROPS][k])(x)
    refresh(rig)
    return vals, props


def key_state(rig, f):
    pbs = rig.pose.bones
    for n in ctrl_names(rig):
        pb = pbs[n]
        pb.keyframe_insert("location", frame=f, group=n)
        pb.keyframe_insert("rotation_euler", frame=f, group=n)
        pb.keyframe_insert("scale", frame=f, group=n)
    for k in prop_names(rig):
        pbs[PROPS].keyframe_insert(f'["{k}"]', frame=f, group=PROPS)


def channelbag(rig):
    ad = rig.animation_data
    return anim_utils.action_get_channelbag_for_slot(ad.action, ad.action_slot)


# ================================================================ measurement
class Topo:
    def __init__(self, mesh_obj, club_obj, parts):
        me = mesh_obj.data
        npoly = len(me.polygons)
        pid = np.empty(npoly, dtype=np.int32)
        me.attributes["part_id"].data.foreach_get("value", pid)
        self.part = pid.astype(np.int64)
        self.part_names = {v: k for k, v in parts.items()}
        me.calc_loop_triangles()
        nt = len(me.loop_triangles)
        tv = np.empty(nt * 3, dtype=np.int32)
        tp = np.empty(nt, dtype=np.int32)
        me.loop_triangles.foreach_get("vertices", tv)
        me.loop_triangles.foreach_get("polygon_index", tp)
        self.tri_v = tv.reshape(nt, 3).astype(np.int64)
        self.tri_poly = tp.astype(np.int64)
        self.poly_verts = [list(p.vertices) for p in me.polygons]
        # dominant deform group per vertex -> per polygon (most common among its vertices' dominant groups)
        gnames = [g.name for g in mesh_obj.vertex_groups]
        dom = np.full(len(me.vertices), -1, dtype=np.int64)
        for v in me.vertices:
            best, bw = -1, 0.0
            for g in v.groups:
                if g.weight > bw:
                    best, bw = g.group, g.weight
            dom[v.index] = best
        self.vdom = dom
        self.gnames = gnames
        pdom = np.empty(npoly, dtype=np.int64)
        for i, vs in enumerate(self.poly_verts):
            vals, cnt = np.unique(dom[vs], return_counts=True)
            pdom[i] = vals[np.argmax(cnt)]
        self.pdom = pdom
        # club
        cm = club_obj.data
        cm.calc_loop_triangles()
        ct = np.empty(len(cm.loop_triangles) * 3, dtype=np.int32)
        cm.loop_triangles.foreach_get("vertices", ct)
        self.club_tri = ct.reshape(-1, 3).astype(np.int64)
        cco = np.empty(len(cm.vertices) * 3, dtype=np.float32)
        cm.vertices.foreach_get("co", cco)
        self.club_local = cco.reshape(-1, 3).astype(np.float64)
        # clearance target: every GOB_mesh face except the right hand shell and body faces of the right
        # lowerarm / wrist region (dominant group lowerarm_r, lowerarm_twist_r, hand_r)
        excl_g = {gnames.index(n) for n in ("lowerarm_r", "lowerarm_twist_r", "hand_r") if n in gnames}
        hand_r = parts["hand_r"]
        keep = np.array([(self.part[p] != hand_r) and not (self.part[p] == parts["body"] and self.pdom[p] in excl_g)
                         for p in range(npoly)])
        self.clear_polys = keep
        self.clear_tris = np.nonzero(keep[self.tri_poly])[0]
        self.body_tris = np.nonzero(self.part[self.tri_poly] == parts["body"])[0]
        self.body_sets = {int(p): set(self.poly_verts[p]) for p in np.nonzero(self.part == parts["body"])[0]}
        self.parts = parts

    def poly_label(self, p):
        g = self.gnames[self.pdom[p]] if self.pdom[p] >= 0 else "-"
        return f"{self.part_names[int(self.part[p])]}/{g}"


def eval_world(obj):
    dg = bpy.context.evaluated_depsgraph_get()
    oe = obj.evaluated_get(dg)
    me = oe.to_mesh()
    try:
        n = len(me.vertices)
        co = np.empty(n * 3, dtype=np.float32)
        me.vertices.foreach_get("co", co)
    finally:
        oe.to_mesh_clear()
    mw = np.array(oe.matrix_world, dtype=np.float64)
    return co.reshape(n, 3).astype(np.float64) @ mw[:3, :3].T + mw[:3, 3], mw


def bvh(V, tris):
    return BVHTree.FromPolygons([tuple(p) for p in V.tolist()], tris.tolist(), all_triangles=True)


def club_clearance(topo, V, C):
    """Min mesh-to-mesh distance club <-> clearance target faces (both vertex sets against the other surface), the
    location, and the number of intersecting triangle pairs."""
    tt = topo.tri_v[topo.clear_tris]
    bb = bvh(V, tt)
    bc = bvh(C, topo.club_tri)
    best = (math.inf, None, None, None)
    for i, p in enumerate(C.tolist()):
        loc, _n, idx, d = bb.find_nearest(Vector(p))
        if loc is not None and d < best[0]:
            best = (d, int(topo.tri_poly[topo.clear_tris[idx]]), i, "club_vertex")
    tv = np.unique(tt.ravel())
    for vi in tv.tolist():
        loc, _n, idx, d = bc.find_nearest(Vector(V[vi]))
        if loc is not None and d < best[0]:
            # body side: the polygon of this vertex with the nearest label
            poly = next(int(topo.tri_poly[t]) for t in topo.clear_tris[np.any(tt == vi, axis=1)][:1])
            ci = int(topo.club_tri[idx][0])
            best = (d, poly, ci, "body_vertex")
    pairs = bc.overlap(bb)
    d, poly, ci, how = best
    s = float(topo.club_local[ci][1]) if ci is not None else None
    return {"min_mm": round(d * 1000.0, 2) if not pairs else 0.0, "min_mm_surface": round(d * 1000.0, 2),
            "where": topo.poly_label(poly) if poly is not None else None,
            "club_s_m": round(s, 3) if s is not None else None, "found_by": how,
            "intersect_tri_pairs": len(pairs),
            "intersect_where": sorted({topo.poly_label(int(topo.tri_poly[topo.clear_tris[j]])) for _i, j in pairs})[:8]}


RIGID_PARTS = ("head", "hand_l", "hand_r", "shoe_l", "shoe_r", "belt")
RAY_DIRS = [Vector(d).normalized() for d in ((0.5773, 0.5271, 0.6237), (-0.6428, 0.2819, 0.7124),
                                              (0.2113, -0.8356, -0.5071))]


def _inside(tree, p):
    votes = 0
    for d in RAY_DIRS:
        o, n = p.copy(), 0
        for _ in range(512):
            hit = tree.ray_cast(o, d)
            if hit[0] is None:
                break
            n += 1
            o = hit[0] + d * 1e-6
        votes += n % 2
    return votes >= 2


def _cover(pairs):
    if not pairs:
        return 0
    common = set(pairs[0])
    for pr in pairs[1:]:
        common &= set(pr)
    return 1 if common else ">=2"


def _tri_pair_point(V, tv, ta, tb):
    from mathutils import geometry
    A = [Vector(V[k]) for k in tv[ta]]
    B = [Vector(V[k]) for k in tv[tb]]
    pts = []
    for P_, T in ((A, B), (B, A)):
        for k in range(3):
            p, q = P_[k], P_[(k + 1) % 3]
            d = q - p
            if d.length <= 0.0:
                continue
            hit = geometry.intersect_ray_tri(T[0], T[1], T[2], d, p, True)
            if hit is not None and (hit - p).length <= d.length:
                pts.append(hit)
    if pts:
        s = Vector((0.0, 0.0, 0.0))
        for x in pts:
            s += x
        return s / len(pts)
    best = None
    for P_, T in ((A, B), (B, A)):
        for p in P_:
            c = geometry.closest_point_on_tri(p, T[0], T[1], T[2])
            if best is None or (c - p).length < best[0]:
                best = ((c - p).length, (p + c) * 0.5)
    return best[1]


def _overlap_pairs(topo, V, tris, keep):
    tp = topo.tri_poly[tris]
    b = bvh(V, topo.tri_v[tris])
    out = {}
    for i, j in b.overlap(b):
        a, c = int(tp[i]), int(tp[j])
        if a == c or set(topo.poly_verts[a]) & set(topo.poly_verts[c]) or not keep(a, c):
            continue
        out.setdefault((min(a, c), max(a, c)), set()).add((int(tris[i]), int(tris[j])))
    return out


def self_isect(topo, V):
    """d-25 A1.3 (same definition as check_a_swing): body faces self overlap (G5: same polygon / shared vertex pairs
    dropped) + overlaps between different rigid parts; cover = minimum face set covering every pair (0 / 1 / '>=2').
    Visible filter: a tri pair is hidden when its representative intersection point (mean of the edge x triangle
    hits, fallback closest vertex-triangle midpoint) lies inside a rigid part shell (ray parity, 3 dirs majority)
    other than the parts of the pair's own faces; a face pair is hidden when all its tri pairs are hidden.
    visible_cover = cover over the non-hidden face pairs (the verdict value); totals are report."""
    rigid_ids = {topo.parts[n] for n in RIGID_PARTS}
    rigid_tris = np.nonzero(np.isin(topo.part[topo.tri_poly], list(rigid_ids)))[0]
    body = _overlap_pairs(topo, V, topo.body_tris, lambda a, c: True)
    rigid = _overlap_pairs(topo, V, rigid_tris, lambda a, c: topo.part[a] != topo.part[c])
    allp = dict(body)
    allp.update(rigid)
    shells = {}
    for n in RIGID_PARTS:
        t = np.nonzero(topo.part[topo.tri_poly] == topo.parts[n])[0]
        vs = np.unique(topo.tri_v[t].ravel())
        shells[n] = (bvh(V, topo.tri_v[t]), V[vs].min(0), V[vs].max(0))
    visible = []
    for key, tps in allp.items():
        own = {topo.part_names[int(topo.part[f])] for f in key}
        hidden_all = True
        for ta, tb in tps:
            p = _tri_pair_point(V, topo.tri_v, ta, tb)
            pa = np.array(p)
            hid = False
            for n, (tree, lo, hi) in shells.items():
                if n in own or np.any(pa < lo) or np.any(pa > hi):
                    continue
                if _inside(tree, p):
                    hid = True
                    break
            if not hid:
                hidden_all = False
                break
        if not hidden_all:
            visible.append(key)
    pairs = sorted(allp)
    visible = sorted(visible)

    def groups(ps):
        lab = {}
        for a, c in ps:
            k = "|".join(sorted([topo.poly_label(a), topo.poly_label(c)]))
            lab[k] = lab.get(k, 0) + 1
        return lab
    return {"pairs": len(pairs), "cover": _cover(pairs), "n_body_pairs": len(body), "n_rigid_pairs": len(rigid),
            "visible_pairs": len(visible), "visible_cover": _cover(visible),
            "visible_groups": groups(visible), "pair_groups": groups(pairs)}


OWN_GROUPS = {  # rigid part -> body deform groups it is designed to overlap (G2/G5 designed overlaps)
    "head": {"head", "spine_02", "spine_01", "shoulder_l", "shoulder_r"},
    "hand_l": {"hand_l", "lowerarm_l", "lowerarm_twist_l"},
    "hand_r": {"hand_r", "lowerarm_r", "lowerarm_twist_r"},
    "shoe_l": {"foot_l", "calf_l", "thigh_l"},
    "shoe_r": {"foot_r", "calf_r", "thigh_r"},
    "belt": {"pelvis", "spine_01", "thigh_l", "thigh_r"},
}


def foreign_overlaps(topo, V):
    """Report only: rigid part shells overlapping each other or body faces outside their own region."""
    out = {}
    tris_of = {name: np.nonzero(topo.part[topo.tri_poly] == pid)[0] for name, pid in topo.parts.items()}
    trees = {name: bvh(V, topo.tri_v[t]) for name, t in tris_of.items() if len(t)}
    names = [n for n in topo.parts if n != "body"]
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            n = len(trees[a].overlap(trees[b]))
            if n:
                out[f"{a}~{b}"] = n
        own = {topo.gnames.index(g) for g in OWN_GROUPS[a] if g in topo.gnames}
        bt = tris_of["body"]
        foreign = bt[~np.isin(topo.pdom[topo.tri_poly[bt]], list(own))]
        if len(foreign):
            tf = bvh(V, topo.tri_v[foreign])
            ov = trees[a].overlap(tf)
            if ov:
                groups = sorted({topo.gnames[topo.pdom[topo.tri_poly[foreign[j]]]] for _i, j in ov})
                out[f"{a}~body({','.join(groups)})"] = len(ov)
    return out


def local_euler(rig, name):
    """Pose rotation of a bone relative to its parent's pose (rest-relative, like matrix_basis), XYZ degrees."""
    pb = rig.pose.bones[name]
    b = pb.bone
    if b.parent is not None:
        pp = rig.pose.bones[b.parent.name]
        ref = pp.matrix @ b.parent.matrix_local.inverted() @ b.matrix_local
    else:
        ref = b.matrix_local
    e = (ref.inverted() @ pb.matrix).to_3x3().to_euler("XYZ")
    return [math.degrees(a) for a in e]


def leg_state(rig, topo, V, side):
    pbs = rig.pose.bones
    mw = rig.matrix_world
    calf = pbs[f"MCH_calf_ik_{side}"]
    thigh = pbs[f"MCH_thigh_ik_{side}"]
    tgt = mw @ pbs[f"MCH_roll_foot_{side}"].head
    reach = ((mw @ calf.tail) - tgt).length
    ratio = ((mw @ thigh.head) - tgt).length / (thigh.bone.length + calf.bone.length)
    shoe = np.unique(topo.tri_v[np.nonzero(topo.part[topo.tri_poly] == topo.parts[f"shoe_{side}"])[0]].ravel())
    th = local_euler(rig, f"thigh_{side}")
    ca = local_euler(rig, f"calf_{side}")
    ft = local_euler(rig, f"foot_{side}")
    return {"reach_mm": round(reach * 1000.0, 3), "ik_ratio": round(ratio, 4),
            "shoe_min_z_mm": round(float(V[shoe, 2].min()) * 1000.0, 2),
            "hip_flex": round(-th[0], 1), "hip_abd": round(-th[2] if side == "l" else th[2], 1),
            "hip_twist": round(th[1], 1), "knee": round(ca[0], 1), "ankle_dorsi": round(-ft[0], 1)}


def arm_geo(rig, side):
    """Report only: DEF upperarm direction in the DEF spine_02 (chest) frame - elevation above the chest horizontal
    plane and angle from its rest direction (the G5 'raise' axis is rest-relative)."""
    pbs = rig.pose.bones
    mw = rig.matrix_world
    ch = (mw @ pbs["spine_02"].matrix).to_3x3()
    ch_rest = pbs["spine_02"].bone.matrix_local.to_3x3()
    ua = pbs[f"upperarm_{side}"]
    d = ((mw @ ua.tail) - (mw @ ua.head)).normalized()
    d_rest = (ua.bone.tail_local - ua.bone.head_local).normalized()
    d_rest_now = ch @ ch_rest.inverted() @ d_rest
    up = ch.col[1].normalized()
    return {"upperarm_elev_deg": round(math.degrees(math.asin(max(-1.0, min(1.0, d.dot(up))))), 1),
            "upperarm_from_rest_deg": round(math.degrees(d.angle(d_rest_now)), 1)}


def wrist_def(rig, side):
    """check_a_swing definition: DEF hand relative to DEF lowerarm (parent), swing-twist about local Y:
    wrist_bend = swing deg, wrist_twist = twist deg."""
    pb = rig.pose.bones[f"hand_{side}"]
    b = pb.bone
    rr = b.parent.matrix_local.inverted() @ b.matrix_local
    rp = rig.pose.bones[b.parent.name].matrix.inverted() @ pb.matrix
    M3 = (rr.inverted() @ rp).to_3x3().normalized()
    q = M3.to_quaternion().normalized()
    tw = (math.degrees(2.0 * math.atan2(q.y, q.w)) + 180.0) % 360.0 - 180.0
    y = M3 @ Vector((0.0, 1.0, 0.0))
    sw = math.degrees(math.atan2(y.cross(Vector((0.0, 1.0, 0.0))).length, y.y))
    return {"wrist_bend_def": round(sw, 1), "wrist_twist_def": round(tw, 1)}


class FrontView:
    """Projection of the fitted front camera (448x576 reference pixels) + the eye-line point attached to the DEF
    head bone (reference f1 eye midpoint (0.511, 0.282) mapped onto the head front plane y = -0.26 at f1)."""

    def __init__(self, cam, rig):
        self.s = cam["fit"]["s_px_per_m"]
        self.u0, self.v0 = cam["fit"]["u0"], cam["fit"]["v0"]
        self.d, self.r, self.u = (Vector(cam[k]) for k in ("view_dir", "right", "up"))
        a = (0.511 * REF_W - self.u0) / self.s
        b = (self.v0 - 0.282 * REF_H) / self.s
        base = self.r * a + self.u * b
        eye = base + self.d * ((-0.26 - base.y) / self.d.y)
        self.eye_local = (rig.matrix_world @ rig.pose.bones["head"].matrix).inverted() @ eye

    def px(self, p):
        p = Vector(p)
        return self.s * p.dot(self.r) + self.u0, self.v0 - self.s * p.dot(self.u)


def front_metrics(rig, topo, V, fv):
    """Report: hip -> eye lean in the front image (deg from vertical, + = toward screen right = character left; hips =
    belt vertex centroid, eyes = head-attached eye-line point), and the face direction (DEF head local -Y rest axis =
    front) vs the direction to the front camera: face_to_cam_deg (0 = facing the camera), face_yaw_deg (+ = turned to
    the character's left), face_pitch_deg (+ = looking down)."""
    belt = np.unique(topo.tri_v[np.nonzero(topo.part[topo.tri_poly] == topo.parts["belt"])[0]].ravel())
    hm = rig.matrix_world @ rig.pose.bones["head"].matrix
    eye = hm @ fv.eye_local
    hx, hy = fv.px(V[belt].mean(0))
    ex, ey = fv.px(eye)
    rest = rig.pose.bones["head"].bone.matrix_local.to_3x3()
    front_rest_local = rest.inverted() @ Vector((0.0, -1.0, 0.0))
    f = (hm.to_3x3() @ front_rest_local).normalized()
    to_cam = -fv.d
    return {"lean_hip_eye_deg": round(math.degrees(math.atan2(ex - hx, hy - ey)), 1),
            "eye_px": [round(ex, 1), round(ey, 1)], "hips_px": [round(hx, 1), round(hy, 1)],
            "face_to_cam_deg": round(math.degrees(f.angle(to_cam)), 1),
            "face_yaw_deg": round(math.degrees(math.atan2(f.x, -f.y)), 1),
            "face_pitch_deg": round(math.degrees(math.asin(max(-1.0, min(1.0, -f.z)))), 1)}


def arm_angles(vals, side):
    s = 1.0 if side == "l" else -1.0
    ua = vals[f"CTRL_upperarm_fk_{side}"]["rot"]
    la = vals[f"CTRL_lowerarm_fk_{side}"]["rot"]
    ha = vals[f"CTRL_hand_fk_{side}"]["rot"]
    sh = vals[f"CTRL_shoulder_{side}"]["rot"]
    return {"shoulder_up": round(s * sh[2], 1), "shoulder_fwd": round(sh[0], 1),
            "upperarm_raise": round(s * ua[2], 1), "upperarm_fwd": round(ua[0], 1), "upperarm_twist": round(ua[1], 1),
            "elbow": round(la[0], 1), "wrist_bend": round(ha[0], 1), "wrist_twist": round(ha[1], 1),
            "wrist_side": round(ha[2], 1)}


def measure(rig, topo, man, vals, props, pose, fv=None):
    V, _ = eval_world(bpy.data.objects[MESH])
    C, _ = eval_world(bpy.data.objects[CLUB])
    m = {"club": club_clearance(topo, V, C), "self_isect": self_isect(topo, V),
         "foreign_overlaps_report": foreign_overlaps(topo, V),
         "club_min_z_mm": round(float(C[:, 2].min()) * 1000.0, 1),
         "arms": {s: dict(arm_angles(vals, s), **arm_geo(rig, s), **wrist_def(rig, s)) for s in ("l", "r")},
         "front": front_metrics(rig, topo, V, fv) if fv is not None else None,
         "legs": {s: leg_state(rig, topo, V, s) for s in ("l", "r")},
         "cog_m": round(vals["CTRL_torso"]["loc"][1], 4),
         "torso": {k: [round(x, 1) for x in vals[k]["rot"]] for k in
                   ("CTRL_torso", "CTRL_pelvis", "CTRL_spine_01", "CTRL_chest", "CTRL_head")}}
    out = []
    for s in ("l", "r"):
        a = m["arms"][s]
        if a["upperarm_raise"] > SAFE["upperarm_raise_max"]:
            out.append(f"upperarm_raise_{s} {a['upperarm_raise']} > {SAFE['upperarm_raise_max']}")
        if a["shoulder_up"] > SAFE["shoulder_up_max"]:
            out.append(f"shoulder_up_{s} {a['shoulder_up']} > {SAFE['shoulder_up_max']}")
        if not SAFE["upperarm_back_min"] <= a["upperarm_fwd"] <= SAFE["upperarm_fwd_max"]:
            out.append(f"upperarm_fwd_{s} {a['upperarm_fwd']} outside [-45, 90]")
        if a["elbow"] > SAFE["elbow_max"]:
            out.append(f"elbow_{s} {a['elbow']} > {SAFE['elbow_max']}")
        if a["wrist_bend_def"] > SAFE["wrist_bend_max"]:
            out.append(f"wrist_bend_{s} (DEF swing) {a['wrist_bend_def']} > {SAFE['wrist_bend_max']}")
        if abs(a["wrist_twist_def"]) > SAFE["wrist_twist_max"]:
            out.append(f"wrist_twist_{s} (DEF) {a['wrist_twist_def']} > {SAFE['wrist_twist_max']}")
        g = m["legs"][s]
        if g["hip_flex"] > SAFE["hip_flex_max"]:
            out.append(f"hip_flex_{s} {g['hip_flex']} > {SAFE['hip_flex_max']}")
        if g["knee"] > SAFE["knee_max"]:
            out.append(f"knee_{s} {g['knee']} > {SAFE['knee_max']}")
        if g["ik_ratio"] > SAFE["ik_ratio_max"]:
            out.append(f"ik_ratio_{s} {g['ik_ratio']} > {SAFE['ik_ratio_max']}")
        if s in pose["contact"]:
            if g["reach_mm"] > SAFE["reach_mm_max"]:
                out.append(f"planted {s} reach {g['reach_mm']} mm > 1")
            lo, hi = SAFE["shoe_z_mm"]
            if not lo <= g["shoe_min_z_mm"] <= hi:
                out.append(f"planted {s} shoe z {g['shoe_min_z_mm']} mm outside [{lo}, {hi}]")
    if m["cog_m"] < SAFE["cog_min"]:
        out.append(f"cog {m['cog_m']} < {SAFE['cog_min']}")
    for k in ("CTRL_spine_01", "CTRL_chest", "CTRL_pelvis"):
        for i, x in enumerate(vals[k]["rot"]):
            if abs(x) > SAFE["spine_axis_max"]:
                out.append(f"{k} rot {AXES[i]} {x:.1f} > {SAFE['spine_axis_max']}")
    if m["club"]["min_mm"] < SAFE["club_clear_min_mm"]:
        out.append(f"club clearance {m['club']['min_mm']} mm < {SAFE['club_clear_min_mm']} ({m['club']['where']})")
    cov = m["self_isect"]["visible_cover"]
    if not isinstance(cov, int) or cov > SAFE["isect_cover_max"]:
        out.append(f"visible self intersection cover {cov} ({m['self_isect']['visible_pairs']} pairs)")
    for k, x in props.items():
        if k in man.get("clamps", {}):
            lo, hi = man["clamps"][k]
            if not lo <= x <= hi:
                out.append(f"{k} {x} outside clamp")
    m["outside_safe_range"] = out
    return m


def fmt_measure(pose, m):
    a, g, c = m["arms"], m["legs"], m["club"]
    return (f"f{pose['frame']:02d} {pose['name']:15s} club {c['min_mm']:7.1f} mm @ {c['where']} (s {c['club_s_m']}, "
            f"isect {c['intersect_tri_pairs']}) club_z {m['club_min_z_mm']} | self visible {m['self_isect']['visible_pairs']}p "
            f"cover {m['self_isect']['visible_cover']} (total {m['self_isect']['pairs']}p cover {m['self_isect']['cover']}) | "
            f"front {m['front']} | R ua r{a['r']['upperarm_raise']} f{a['r']['upperarm_fwd']} "
            f"t{a['r']['upperarm_twist']} el {a['r']['elbow']} wrDEF {a['r']['wrist_bend_def']}/{a['r']['wrist_twist_def']} "
            f"(ctrl {a['r']['wrist_bend']}/{a['r']['wrist_twist']}/{a['r']['wrist_side']}) sh {a['r']['shoulder_up']} geo elev {a['r']['upperarm_elev_deg']} from_rest "
            f"{a['r']['upperarm_from_rest_deg']} | L ua r{a['l']['upperarm_raise']} f{a['l']['upperarm_fwd']} "
            f"el {a['l']['elbow']} wrDEF {a['l']['wrist_bend_def']}/{a['l']['wrist_twist_def']} geo elev {a['l']['upperarm_elev_deg']} from_rest {a['l']['upperarm_from_rest_deg']} | "
            f"cog {m['cog_m']} | "
            f"legs l hip {g['l']['hip_flex']} knee {g['l']['knee']} ratio {g['l']['ik_ratio']} reach {g['l']['reach_mm']} "
            f"z {g['l']['shoe_min_z_mm']} ; r hip {g['r']['hip_flex']} knee {g['r']['knee']} ratio {g['r']['ik_ratio']} "
            f"reach {g['r']['reach_mm']} z {g['r']['shoe_min_z_mm']} | contact {''.join(pose['contact'])} | "
            f"outside {m['outside_safe_range']} | visible groups {m['self_isect']['visible_groups']} | "
            f"foreign {m['foreign_overlaps_report']}")


# ================================================================ camera
def view_basis(az_deg, el_deg):
    """Camera looking at the character from the front (-Y side) turned az deg toward the character's right (-X) and
    raised el deg: returns (d = view direction, right, up)."""
    az, el = math.radians(az_deg), math.radians(el_deg)
    pos = Vector((-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), math.sin(el)))
    d = -pos
    right = d.cross(Vector((0, 0, 1))).normalized()
    up = right.cross(d).normalized()
    return d.normalized(), right, up


def part_verts(topo, V, name):
    idx = np.unique(topo.tri_v[np.nonzero(topo.part[topo.tri_poly] == topo.parts[name])[0]].ravel())
    return V[idx]


def fit_front_camera(topo, V):
    """Orthographic front camera fitted to the reference f1 framing: for each elevation (camera looking slightly
    down) least squares of image = s * projection + offset over the landmark extents (head top, belt bbox, both foot
    bboxes); the elevation with the smallest RMS wins."""
    ref = REF_F1
    parts = {k: part_verts(topo, V, k) for k in ("head", "belt", "shoe_l", "shoe_r")}
    best = None
    for el in ELEV_GRID:
        d, right, up = view_basis(0.0, el)
        R, U = np.array(right), np.array(up)
        pu = {k: v @ R for k, v in parts.items()}
        pv = {k: v @ U for k, v in parts.items()}
        rows, rhs, names = [], [], []

        def xr(a, target, nm):
            rows.append([a, 1.0, 0.0]); rhs.append(target); names.append(nm)

        def yr(b, target, nm):  # image y = -s * b + v0
            rows.append([-b, 0.0, 1.0]); rhs.append(target); names.append(nm)
        yr(pv["head"].max(), ref["head_top_y"], "head_top_y")
        xr(pu["belt"].min(), ref["belt_bbox"][0], "belt_x0"); xr(pu["belt"].max(), ref["belt_bbox"][2], "belt_x1")
        yr(pv["belt"].max(), ref["belt_bbox"][1], "belt_y0"); yr(pv["belt"].min(), ref["belt_bbox"][3], "belt_y1")
        for k, rk in (("shoe_r", "foot_r_bbox"), ("shoe_l", "foot_l_bbox")):
            xr(pu[k].min(), ref[rk][0], f"{k}_x0"); xr(pu[k].max(), ref[rk][2], f"{k}_x1")
            yr(pv[k].max(), ref[rk][1], f"{k}_y0"); yr(pv[k].min(), ref[rk][3], f"{k}_y1")
        A, b = np.array(rows), np.array(rhs)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        res = A @ sol - b
        rms = float(np.sqrt((res ** 2).mean()))
        if best is None or rms < best["rms_px"]:
            best = {"elev_deg": el, "s_px_per_m": float(sol[0]), "u0": float(sol[1]), "v0": float(sol[2]),
                    "rms_px": rms, "residual_px": {n: round(float(r), 2) for n, r in zip(names, res)}}
    el = best["elev_deg"]
    d, right, up = view_basis(0.0, el)
    s = best["s_px_per_m"]
    cu = (REF_W / 2.0 - best["u0"]) / s        # world coordinate along right at the image centre
    cv = (best["v0"] - REF_H / 2.0) / s        # along up
    center = right * cu + up * cv
    return make_cam_json("front", d, right, up, center, REF_H / s, best)


def make_cam_json(name, d, right, up, center, ortho_scale, fit):
    dist = 6.0
    loc = center - d * dist
    rot = Matrix((right, up, -d)).transposed()
    mw = Matrix.Translation(loc) @ rot.to_4x4()
    return {"name": name, "type": "orthographic", "resolution": [RES_W, RES_H], "sensor_fit": "VERTICAL",
            "ortho_scale": ortho_scale, "location": list(loc), "view_dir": list(d), "right": list(right),
            "up": list(up), "matrix_world": [list(r) for r in mw], "clip_start": 0.01, "clip_end": 20.0, "fit": fit}


def fit_tq_camera(boxes):
    """3/4 camera: fixed azimuth / elevation, framing the union of all poses' projected mesh + club (5 % pad),
    aspect RES_W:RES_H."""
    d, right, up = view_basis(TQ_AZ, TQ_EL)
    R, U = np.array(right), np.array(up)
    P_ = np.concatenate(boxes)
    u, v = P_ @ R, P_ @ U
    u0, u1, v0, v1 = u.min(), u.max(), v.min(), v.max()
    h = max(v1 - v0, (u1 - u0) * RES_H / RES_W) * 1.10
    center = right * ((u0 + u1) / 2) + up * ((v0 + v1) / 2)
    return make_cam_json("three_quarter", d, right, up, center, h,
                         {"azimuth_deg_toward_char_right": TQ_AZ, "elevation_deg": TQ_EL,
                          "framing": "union of the 7 key poses (GOB_mesh + GOB_club evaluated), 10 % margin"})


# ================================================================ render / images
def render(cam, out_png):
    sc = bpy.context.scene
    cd = bpy.data.cameras.new("_a1_cam")
    cd.type = "ORTHO"
    cd.sensor_fit = "VERTICAL"
    cd.ortho_scale = cam["ortho_scale"]
    cd.clip_start, cd.clip_end = cam["clip_start"], cam["clip_end"]
    co = bpy.data.objects.new("_a1_cam", cd)
    sc.collection.objects.link(co)
    co.matrix_world = Matrix(cam["matrix_world"])
    old_cam = sc.camera
    sc.camera = co
    try:
        bpy.ops.render.render(write_still=False)
        img = bpy.data.images["Render Result"]
        img.save_render(str(out_png))
    finally:
        sc.camera = old_cam
        bpy.data.objects.remove(co, do_unlink=True)
        bpy.data.cameras.remove(cd)
    return load_rgba(out_png)


def setup_render():
    sc = bpy.context.scene
    r = sc.render
    r.engine = "BLENDER_WORKBENCH"
    r.resolution_x, r.resolution_y, r.resolution_percentage = RES_W, RES_H, 100
    r.pixel_aspect_x = r.pixel_aspect_y = 1.0
    r.film_transparent = True
    r.use_file_extension = False
    r.image_settings.file_format = "PNG"
    r.image_settings.color_mode = "RGBA"
    r.image_settings.color_depth = "8"
    sc.view_settings.view_transform = "Standard"
    sc.view_settings.look = "None"
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "OBJECT"
    sh.show_object_outline = True
    sh.object_outline_color = (0.25, 0.27, 0.22)
    sh.show_cavity = False
    sh.show_shadows = False
    sh.show_xray = False
    sh.show_specular_highlight = False
    bpy.data.objects[MESH].color = MESH_RGBA
    bpy.data.objects[CLUB].color = CLUB_RGBA


def load_rgba(path):
    img = bpy.data.images.load(str(path), check_existing=False)
    try:
        w, h = img.size
        buf = np.empty(w * h * img.channels, dtype=np.float32)
        img.pixels.foreach_get(buf)
        px = buf.reshape(h, w, img.channels)
        if img.channels == 3:
            px = np.concatenate([px, np.ones((h, w, 1), np.float32)], axis=2)
    finally:
        bpy.data.images.remove(img)
    return np.ascontiguousarray(px[::-1])  # row 0 = top


def save_rgb(arr, path):
    h, w = arr.shape[:2]
    img = bpy.data.images.new("_a1_out", w, h, alpha=True)
    try:
        rgba = np.ones((h, w, 4), np.float32)
        rgba[:, :, :3] = arr[:, :, :3]
        img.pixels.foreach_set(np.ascontiguousarray(rgba[::-1]).ravel())
        img.filepath_raw = str(path)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)


def over_white(rgba):
    a = rgba[:, :, 3:4]
    return rgba[:, :, :3] * a + (1.0 - a)  # workbench PNG = straight alpha


def half(a):
    h, w = a.shape[0] // 2 * 2, a.shape[1] // 2 * 2
    a = a[:h, :w]
    return (a[0::2, 0::2] + a[1::2, 0::2] + a[0::2, 1::2] + a[1::2, 1::2]) / 4.0


def grid(a):
    """0.1 normalized grid like ref/keyframes (vertical cyan, horizontal magenta), light."""
    a = a.copy()
    h, w = a.shape[:2]
    for k in range(1, 10):
        x = int(round(k * w / 10))
        a[:, x:x + 1] = a[:, x:x + 1] * 0.55 + np.array([0.0, 0.8, 0.9]) * 0.45
        y = int(round(k * h / 10))
        a[y:y + 1, :] = a[y:y + 1, :] * 0.55 + np.array([0.9, 0.0, 0.9]) * 0.45
    return a


# ================================================================ main
def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    quick = "--quick" in args          # debug: skip renders / save
    rig = bpy.data.objects[ARM]
    man = goblib.load_json("ctrl_manifest.json")
    parts = goblib.load_json("parts.json")
    sc = bpy.context.scene
    if bpy.context.object and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.objects.active = rig
    rig.data.pose_position = "POSE"

    mod = bpy.data.texts[TEXT_UI].as_module()
    mod.register()
    ts = sc.tool_settings
    autokey0 = ts.use_keyframe_insert_auto
    ts.use_keyframe_insert_auto = False
    for op in ("reset_rig", "ready_pose"):
        r = getattr(bpy.ops.goblin, op)()
        if "FINISHED" not in r:
            raise RuntimeError(f"goblin.{op} returned {r}")
    refresh(rig)
    base_c, base_p = snapshot(rig)
    print(f"[s10a] base = goblin.reset_rig + goblin.ready_pose (blend text {TEXT_UI} as module); non-identity: "
          + ", ".join(f"{n} loc {[round(x, 4) for x in v['loc']]} rot {[round(x, 2) for x in v['rot']]}"
                      for n, v in base_c.items() if any(abs(x) > 1e-6 for x in v["loc"] + v["rot"])))

    # action
    old = bpy.data.actions.get(ACTION)
    if old is not None:
        bpy.data.actions.remove(old)
    ad = rig.animation_data or rig.animation_data_create()
    act = bpy.data.actions.new(ACTION)
    act.use_fake_user = True
    slot = act.slots.new(id_type="OBJECT", name=ARM)
    ad.action = act
    ad.action_slot = slot
    sc.render.fps, sc.render.fps_base = FPS, 1.0
    sc.frame_start, sc.frame_end = FRAME_START, FRAME_END
    act.use_frame_range = True
    act.frame_start, act.frame_end = FRAME_START, FRAME_END

    states = {}
    for pose in POSES:
        sc.frame_set(pose["frame"])
        vals, props = apply_state(rig, man, base_c, base_p, pose)
        key_state(rig, pose["frame"])
        states[pose["frame"]] = (vals, props)
    fcs = channelbag(rig).fcurves
    nkeys = 0
    for fc in fcs:
        for kp in fc.keyframe_points:
            kp.interpolation = "CONSTANT"
            nkeys += 1
        fc.update()
    ts.use_keyframe_insert_auto = autokey0
    print(f"[s10a] ACTION {act.name} slot {slot.identifier!r}: {len(fcs)} fcurves, {nkeys} keys, CONSTANT, frames "
          f"{[p['frame'] for p in POSES]}, scene {sc.frame_start}-{sc.frame_end} @ {sc.render.fps} fps")

    # measure (from the action: frame_set evaluates the keys)
    topo = Topo(bpy.data.objects[MESH], bpy.data.objects[CLUB], parts)
    def_names = [b["name"] for b in goblib.load_json("canonical_skeleton.json")["bones"]]
    meas, boxes, f1_V, defs = {}, [], None, {}
    sc.frame_set(FRAME_START)
    refresh(rig)
    f1_V, _ = eval_world(bpy.data.objects[MESH])
    front = fit_front_camera(topo, f1_V)
    fv = FrontView(front, rig)
    for pose in POSES:
        f = pose["frame"]
        sc.frame_set(f)
        refresh(rig)
        vals, props = states[f]
        cur, curp = snapshot(rig)
        drift = max(max(abs(a - b) for a, b in zip(cur[n]["loc"] + cur[n]["rot"], vals[n]["loc"] + vals[n]["rot"]))
                    for n in vals)
        m = measure(rig, topo, man, vals, props, pose, fv)
        m["key_readback_max_diff"] = drift
        meas[f] = m
        defs[f] = {n: [[round(float(x), 7) for x in row] for row in (rig.matrix_world @ rig.pose.bones[n].matrix)]
                   for n in def_names}
        print("[s10a] MEASURE " + fmt_measure(pose, m))
        V, _ = eval_world(bpy.data.objects[MESH])
        C, _ = eval_world(bpy.data.objects[CLUB])
        boxes.append(np.concatenate([V, C]))
    if quick:
        return rig, topo, meas

    # keypose json
    doc = {
        "_doc": {
            "owner": "scripts/s10a_swing_keyposes.py (T50, spec d-25 A1)",
            "values": "ctrl = full CTRL state of the pose (absolute local values: loc m, rot deg XYZ Euler in the "
                      "control's rotation_mode; = ready_pose base + the pose's overrides); ctrl_overrides = only the "
                      "values the pose sets; props = full PROPS state. Arms FK (arm_ik_fk 0), legs IK (leg_ik_fk 1), "
                      "weapon_space 0 (hand_r), head_space 0 (chest).",
            "measure": "self measurement of the pose (evaluated from the action attack_swing): club = min mesh-to-mesh "
                       "distance GOB_club <-> GOB_mesh faces except the hand_r shell and body faces whose dominant "
                       "deform group is lowerarm_r / lowerarm_twist_r / hand_r (club vertices -> target BVH nearest and "
                       "target vertices -> club BVH nearest; 0 if any triangle pair intersects); where = part/dominant "
                       "group of the nearest target face; club_s_m = club local axis coordinate of the nearest club "
                       "point (grip = 0, head +). self_isect = G5 definition over all body faces (BVH overlap, same "
                       "polygon / shared vertex pairs dropped; cover = minimum face set covering every pair). "
                       "foreign_overlaps_report = report only (rigid shells vs each other / body faces outside their "
                       "designed region). legs: reach = |MCH_calf_ik tail - MCH_roll_foot head|, ik_ratio = |MCH_thigh_ik "
                       "head - MCH_roll_foot head| / (thigh + calf), shoe_min_z = lowest evaluated shoe vertex, "
                       "hip_flex / hip_abd / knee / ankle_dorsi = DEF local rotation to the parent pose (rom_poses sign "
                       "convention). arms: CTRL FK values with the rom_poses signs (raise = upperarm_l Z+ / "
                       "upperarm_r Z-, shoulder_up the same sign on CTRL_shoulder); upperarm_elev_deg / "
                       "upperarm_from_rest_deg = report only, DEF upperarm direction in the DEF spine_02 frame "
                       "(elevation above the chest horizontal plane / angle from the rest direction).",
            "def": "def = world matrix (4x4, row-major, rig.matrix_world @ pose_bone.matrix) of the 24 DEF bones "
                   "(canonical_skeleton.json) at the key frame, evaluated from the action.",
            "contact_ranges": "frame ranges (inclusive) in which a foot is fully still (planted) over the whole clip "
                              "(A2 / A2.5); contact per pose = feet planted in that key pose.",
            "safe_range": SAFE,
        },
        "action": ACTION, "fps": FPS, "frame_range": [FRAME_START, FRAME_END],
        "loop": "f39 = f1 (A2); f1 is the loop idle pose",
        "base": "goblin.reset_rig + goblin.ready_pose",
        "poses": [],
        "contact_ranges": [{"foot": ft, "frames": rng} for ft in ("l", "r") for rng in CONTACT_RANGES[ft]],
        "contact_notes": CONTACT_NOTES,
    }
    for pose in POSES:
        f = pose["frame"]
        vals, props = states[f]
        doc["poses"].append({
            "frame": f, "name": pose["name"], "intent": pose["intent"], "notes": pose["notes"],
            "contact": pose["contact"],
            "ctrl_overrides": pose["ctrl"],
            "ctrl": {n: {"loc": [round(x, 6) for x in v["loc"]], "rot": [round(x, 4) for x in v["rot"]],
                         "scale": [1.0, 1.0, 1.0]} for n, v in vals.items()},
            "props": props,
            "def": defs[f],
            "measure": meas[f]})
    goblib.save_json(KEYPOSE_JSON, doc)

    # blend (before renders; renders add nothing to the file)
    sc.frame_set(FRAME_START)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s10a] SAVED {OUT_BLEND.name} (copy=True, save_version {bpy.context.preferences.filepaths.save_version})")

    # cameras
    tq = fit_tq_camera(boxes)
    goblib.save_json(CAM_JSON, {
        "_doc": "A1 render cameras (s10a). front: orthographic, fitted to the reference framing on the f1 pose "
                "(fit: elevation grid 0-25 deg step 0.25, per elevation linear least squares of the reference f1 "
                "landmark extents in 448x576 px = s * projection + (u0, v0); landmarks = head top, belt bbox, both "
                "foot bboxes from ref/metrics.json frame 1; image = 896x1152 = 2x the reference). three_quarter: "
                "fixed azimuth / elevation framing all key poses. Blender camera looks along local -Z, up = local +Y.",
        "reference_f1_px": REF_F1, "front": front, "three_quarter": tq})
    print(f"[s10a] CAMERA front ortho elev {front['fit']['elev_deg']} deg, {front['fit']['s_px_per_m']:.1f} px/m "
          f"(448x576), ortho_scale {front['ortho_scale']:.4f} m, rms {front['fit']['rms_px']:.2f} px, residuals "
          f"{front['fit']['residual_px']}")
    print(f"[s10a] CAMERA 3/4 ortho az {TQ_AZ} el {TQ_EL} ortho_scale {tq['ortho_scale']:.4f} m")

    # renders
    INSP.mkdir(parents=True, exist_ok=True)
    colors0 = {n: tuple(bpy.data.objects[n].color) for n in (MESH, CLUB)}
    setup_render()
    rows = []
    for pose in POSES:
        f = pose["frame"]
        sc.frame_set(f)
        refresh(rig)
        tag = f"f{f:02d}_{pose['name']}"
        fr = over_white(render(front, INSP / f"{tag}_front.png"))
        tqr = over_white(render(tq, INSP / f"{tag}_34.png"))
        save_rgb(fr, INSP / f"{tag}_front.png")
        save_rgb(tqr, INSP / f"{tag}_34.png")
        refp = REF_DIR / "keyframes" / f"key_{f:04d}.png"
        ref = load_rgba(refp)[:, :, :3]
        row = np.concatenate([ref, grid(fr), tqr], axis=1)
        save_rgb(row, INSP / f"{tag}_row.png")
        rows.append(half(row))
        print(f"[s10a] RENDER {tag}: front / 3/4 / row (ref {refp.name} | front + 0.1 grid | 3/4)")
    sep = np.full((6, rows[0].shape[1], 3), 1.0, np.float32)
    sheet = np.concatenate(sum(([r, sep] for r in rows), [])[:-1], axis=0)
    save_rgb(sheet, INSP / "keyposes_sheet.png")
    print(f"[s10a] SHEET {goblib._rel(INSP / 'keyposes_sheet.png')} {sheet.shape[1]}x{sheet.shape[0]} "
          f"(rows {[p['frame'] for p in POSES]}, columns reference | ours front | ours 3/4)")
    for n, c in colors0.items():
        bpy.data.objects[n].color = c


if __name__ == "__main__":
    goblib.run_main(main)
