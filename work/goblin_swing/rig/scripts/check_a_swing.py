"""check_a_swing - A1 KEYPOSE / A2 ANIM checker for the goblin club swing (design doc d-25 section 2 + 4).

A1.5 and A2.6 are [U] render items (production side) and are not measured here.

Inputs (independent of the production script s10a_swing_keyposes.py; only blend data + data json):
  rig/gob_r06_swing.blend           GOB_rig (DEF 24 + CTRL/MCH/PROPS) with action attack_swing, GOB_mesh
                                    (face INT part_id, Armature modifier), GOB_club (bone parent weapon_socket_r)
  rig/data/swing_keyposes.json      key poses + contact sections (schema below, read tolerantly)
  rig/data/canonical_skeleton.json  DEF bone names (24)
  rig/data/ctrl_manifest.json       ikfk chains (def / mch_ik / ik_constraint), clamps
  rig/data/parts.json               part_id values

swing_keyposes.json (accepted forms; anything else is reported under A1.1):
  poses: list (or dict name/frame -> pose) of
    {frame: int, name: str, contact: ["l","r"] | {"l": bool, "r": bool} | "l"/"r"/"both"/"none",
     ctrl|ctrls|controls: {CTRL_bone: {loc|location: [3] m, rot|rot_deg|rotation_euler_deg: [3] deg (XYZ),
                                       rotation_euler: [3] rad, rotation_quaternion|quat: [4] wxyz, scale: [3]}},
     props: {PROPS key: value}  (a "PROPS" entry inside ctrl is read as props),
     def (optional snapshot): {DEF bone: 4x4 world matrix (row-major) | {"matrix": 4x4}}}
  contact_ranges: list of {foot|side: "l"/"r", frames|range: [a, b]} or {start, end}; or {"l": [[a, b], ...]}.
  Foot names l / r / foot_l / shoe_l / left / L are accepted.

Pose source: action attack_swing evaluated with scene.frame_set(f) (assigned in memory when not active).
Without the action, A1 poses are the JSON CTRL values applied in memory (action detached, every CTRL_* /
PROPS bone reset to identity, PROPS at the contract defaults, then the listed values); A2 is blocked.
Everything is changed in memory only and restored at the end; nothing is saved.

Modes:  -- --gate A1  (key-pose frames 1, 9, 12, 13, 15, 24, 29)   -- --gate A2  (all frames 1..39)
        no --gate: both, A2 blocked automatically when the action keys only the 7 key-pose frames.
Debug:  -- --keyposes <json> (override input), --debug-out <dir> (evidence written there, not rig/inspect).

Run:    bl.ps1 -Script check_a_swing.py -Blend gob_r06_swing.blend [-- --gate A1|A2]
Output: rig/inspect/A/check_a_swing.json
Exit:   0 when the evidence was written; 2 on a script error (run_main) or a missing input (evidence
        still written with blocked criteria).
Coordinates: front -Y, up +Z, character left (_l) +X, ground z=0.  Lengths in m, reported in mm.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import json  # noqa: E402
import math  # noqa: E402
import re  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Euler, Matrix, Quaternion, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

GATE = "A"
CHECKER = "check_a_swing"
BLEND = goblib.RIG / "gob_r06_swing.blend"
KEYPOSES_JSON = goblib.DATA / "swing_keyposes.json"
CANON_JSON = goblib.DATA / "canonical_skeleton.json"
MANIFEST_JSON = goblib.DATA / "ctrl_manifest.json"
PARTS_JSON = goblib.DATA / "parts.json"

ARM = "GOB_rig"
MESH = "GOB_mesh"
CLUB = "GOB_club"
PROPS = "PROPS"
ACTION = "attack_swing"
SIDES = ("l", "r")

KEY_FRAMES = (1, 9, 12, 13, 15, 24, 29)   # A1.1
FRAME_RANGE = (1, 39)                     # A2.1
FPS = 24.0
POSE_TOL = 1e-4                           # A2.1 / A2.2 max abs matrix element (DEF) or channel value
ROOT_TOL = 1e-7                           # A2.2 CTRL_root motion "0" (float noise only)
CLEAR_MIN = 0.010                         # A1.3 / A2.3 club clearance (m)
COVER_MAX = 1                             # A1.3 / A2.3
PLANT_REACH = 0.001                       # A1.4 (m), = G6.7
PLANT_Z = (-0.001, 0.003)                 # A1.4 lowest shoe z (m), = G6.7
FOOT_MOVE = 0.001                         # A2.5 (m)
FLIP_MIN_DEG = 2.0                        # A2.4 = G6.5b
FOREARM_BONES = ("lowerarm_r", "lowerarm_twist_r", "hand_r")
FOREARM_W = 0.5                           # face excluded when mean vertex weight on FOREARM_BONES > 0.5
CLUB_EXCLUDED_PARTS = ("hand_r",)
SHELL_PARTS = ("body", "head", "hand_l", "shoe_l", "shoe_r", "belt")   # inside tests (hand_r excluded)
RIGID_PARTS = ("head", "hand_l", "hand_r", "shoe_l", "shoe_r", "belt")
SECTIONS = (("idle", 1, 5), ("windup", 6, 12), ("swing", 13, 15), ("hold", 16, 23),
            ("recovery", 24, 34), ("settle", 35, 39))
CONTRACT_DEFAULTS = {   # = check_g6_ctrl (plan contract)
    "arm_ik_fk_l": 0.0, "arm_ik_fk_r": 0.0, "leg_ik_fk_l": 1.0, "leg_ik_fk_r": 1.0,
    "head_space": 0, "hand_ik_space_l": 0, "hand_ik_space_r": 0, "weapon_space": 0,
    "knee_pole_space_l": 0, "knee_pole_space_r": 0,
    "foot_roll_l": 0.0, "foot_roll_r": 0.0, "foot_bank_l": 0.0, "foot_bank_r": 0.0,
    "heel_twist_l": 0.0, "heel_twist_r": 0.0, "toe_twist_l": 0.0, "toe_twist_r": 0.0,
}
# spec d-25 section 2 (draft limits): item -> (op, limit)
SAFE_LIMITS = {
    "upperarm_raise": ("<=", 100.0), "upperarm_fwd_max": ("<=", 90.0), "upperarm_fwd_min": (">=", -45.0),
    "shoulder_raise": ("<=", 25.0), "elbow_flex": ("<=", 100.0), "wrist_bend": ("<=", 40.0),
    "wrist_twist_abs": ("<=", 90.0), "hip_flex": ("<=", 60.0), "knee_flex": ("<=", 90.0),
    "cog_dz_m": (">=", -0.08), "spine_ctrl_abs": ("<=", 30.0), "ik_ratio": ("<=", 0.97),
}
SAFE_NOTE = (
    "DEF local rotation = (parent_rest^-1 @ rest)^-1 @ (parent_pose^-1 @ pose) of the evaluated DEF bone "
    "(armature space, 3x3 normalized), Euler XYZ deg (= the FK control local rotation, rom_poses _doc signs). "
    "upperarm_raise = upperarm_l Z / -upperarm_r Z (parent shoulder); upperarm_fwd = upperarm X (fwd +); "
    "shoulder_raise = shoulder_l Z / -shoulder_r Z (parent spine_02); elbow_flex = lowerarm X; wrist = hand "
    "relative to lowerarm, swing-twist about local Y: wrist_bend = swing deg, wrist_twist_abs = |twist| deg; "
    "hip_flex = -thigh X (parent pelvis); knee_flex = calf X; cog_dz_m = CTRL_torso head world z - rest; "
    "spine_ctrl_abs = |CTRL_spine_01 / CTRL_chest rotation_euler X, Y, Z| deg (matrix_basis Euler XYZ if not "
    "XYZ mode); ik_ratio = |IK subtarget head - MCH chain root head| / (rest length of the 2 IK bones), judged "
    "only when the chain's ik_fk prop > 0; foot props vs manifest clamps. CTRL_torso / pelvis / head Euler and "
    "DEF spine / head local Euler are report only (not in the section 2 table). Club clearance is A1.3. "
    "Wrist bend / twist = DEF hand angle relative to DEF lowerarm (its parent); this checker's definition is the "
    "reference.")
CLEAR_NOTE = (
    "Evaluated GOB_club vs evaluated GOB_mesh (world). Excluded: part hand_r and body faces whose mean vertex "
    "weight on lowerarm_r + lowerarm_twist_r + hand_r > 0.5 (grip / fist hole). Distance = min of (club verts -> "
    "BVH nearest on target tris) and (target verts inside club AABB + current min -> BVH nearest on club tris). "
    "Penetration: ray parity (3 dirs, majority) of club verts inside each closed part shell (body, head, "
    "hand_l, shoe_l, shoe_r, belt; a hit whose nearest shell tri is an excluded face is ignored), target verts "
    "inside the club shell, and club x target tri BVH overlap; clearance = -max depth when inside, 0 when only "
    "tri overlap, else the min distance. Location: part, dominant weight bone of the nearest face / vertex, "
    "club_s_mm = club local Y (grip origin, + = club head).")
ISECT_NOTE = (
    "G5 definition: body faces (part_id body) BVH self overlap, same polygon / shared-vertex pairs dropped; plus "
    "tri overlaps between different rigid parts (head, hand_l, hand_r, shoe_l, shoe_r, belt). Designed body-rigid "
    "overlaps and same-rigid-part pairs are not counted. cover = minimum face set covering every pair "
    "(0 / 1 / '>=2'). Visible filter (d-25 A1.3, user decision 2026-09-25): per intersecting tri pair a "
    "representative point = mean of the edge x triangle intersection points (midpoint of the intersection "
    "segment; fallback: midpoint of the closest vertex-to-triangle pair); the tri pair is hidden when that point "
    "is inside a rigid part shell (head, hand_l, hand_r, shoe_l, shoe_r, belt; ray parity, 3 dirs majority, "
    "shell AABB prefilter) other than the parts of the pair's own faces; a face pair is hidden when all its tri "
    "pairs are hidden. Verdict = visible_cover (cover over the non-hidden pairs) <= 1; totals are report.")
PLANT_NOTE = ("reach = |MCH_calf_ik tail - IK subtarget (MCH_roll_foot) head| (manifest ikfk.leg_x ik_constraint); "
              "shoe_min_z = lowest evaluated shoe_x vertex z (= G6.7). Contact feet from the JSON pose contact.")

_BONE_RE = re.compile(r'pose\.bones\["(.+?)"\]')
RAY_DIRS = [Vector(d).normalized() for d in ((0.5773, 0.5271, 0.6237),
                                              (-0.6428, 0.2819, 0.7124),
                                              (0.2113, -0.8356, -0.5071))]
RAY_EPS = 1e-6
LIST_CAP = 10


# ---------------------------------------------------------------- helpers
class Blocked(Exception):
    pass


def mm(x):
    if x is None:
        return None
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=4):
    if x is None:
        return None
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def sci(x):
    return None if x is None else float(f"{float(x):.3e}")


def rel(p):
    return goblib._rel(p)


def mat_max_abs(a, b):
    return max(abs(a[i][j] - b[i][j]) for i in range(4) for j in range(4))


def rot_np(m):
    """Column-normalized float64 3x3 rotation of a mathutils matrix."""
    R = np.array(m.to_3x3(), dtype=np.float64)
    return R / np.linalg.norm(R, axis=0)


def rot_diff_deg(ma, mb):
    """Rotation angle (deg) of ma^T mb in float64 via atan2(|axial vector|, (trace - 1) / 2); robust at
    small angles (mathutils quaternions are float32: 2*acos(|q1.q2|) gives ~0.05 deg for identical input)."""
    R = rot_np(ma).T @ rot_np(mb)
    s = 0.5 * math.sqrt((R[2, 1] - R[1, 2]) ** 2 + (R[0, 2] - R[2, 0]) ** 2 + (R[1, 0] - R[0, 1]) ** 2)
    c = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    return math.degrees(math.atan2(s, c))


def fcurves_of(action):
    """All F-curves of a (layered, Blender 4.4+) action; legacy action.fcurves as fallback (= check_g7)."""
    out = []
    for layer in getattr(action, "layers", ()):
        for strip in layer.strips:
            for cb in getattr(strip, "channelbags", ()):
                out.extend(cb.fcurves)
    if not out and hasattr(action, "fcurves"):
        out.extend(action.fcurves)
    return out


def twist_angle_deg(M3):
    """Swing-twist decomposition about local Y of a 3x3 rotation: (twist deg, swing deg) (= check_g5)."""
    q = M3.to_quaternion().normalized()
    tw = math.degrees(2.0 * math.atan2(q.y, q.w))
    tw = (tw + 180.0) % 360.0 - 180.0
    y = M3 @ Vector((0.0, 1.0, 0.0))
    swing = math.degrees(math.atan2(y.cross(Vector((0.0, 1.0, 0.0))).length, y.y))
    return tw, swing


def side_of(v):
    s = str(v).strip().lower()
    for pre in ("foot_", "shoe_", "leg_"):
        if s.startswith(pre):
            s = s[len(pre):]
    return {"l": "l", "left": "l", "r": "r", "right": "r"}.get(s)


def parse_contact(c):
    """-> set of sides or None when unparseable."""
    if c is None:
        return None
    if isinstance(c, str):
        s = c.strip().lower()
        if s in ("both", "lr", "l+r"):
            return {"l", "r"}
        if s in ("none", ""):
            return set()
        x = side_of(s)
        return {x} if x else None
    if isinstance(c, dict):
        out = set()
        for k, v in c.items():
            x = side_of(k)
            if x is None:
                return None
            if v:
                out.add(x)
        return out
    if isinstance(c, (list, tuple)):
        out = set()
        for k in c:
            x = side_of(k)
            if x is None:
                return None
            out.add(x)
        return out
    return None


def parse_ranges(cr):
    """contact_ranges -> (list of {"foot", "a", "b"}, issues)."""
    out, issues = [], []
    if cr is None:
        return out, ["contact_ranges missing"]
    items = []
    if isinstance(cr, dict):
        for k, v in cr.items():
            x = side_of(k)
            if x is None:
                issues.append(f"contact_ranges key {k!r} is not a foot")
                continue
            vs = v if isinstance(v, list) and v and isinstance(v[0], (list, tuple, dict)) else [v]
            for r in vs:
                items.append((x, r))
    elif isinstance(cr, list):
        for r in cr:
            if not isinstance(r, dict):
                issues.append(f"contact_ranges entry {r!r} is not an object")
                continue
            x = side_of(r.get("foot", r.get("side", "")))
            if x is None:
                issues.append(f"contact_ranges entry {r!r}: foot/side missing")
                continue
            items.append((x, r))
    else:
        return out, ["contact_ranges is neither list nor object"]
    for x, r in items:
        ab = None
        if isinstance(r, dict):
            fr = r.get("frames", r.get("range"))
            if isinstance(fr, (list, tuple)) and len(fr) == 2:
                ab = fr
            elif "start" in r and "end" in r:
                ab = (r["start"], r["end"])
        elif isinstance(r, (list, tuple)) and len(r) == 2:
            ab = r
        try:
            a, b = int(ab[0]), int(ab[1])
        except (TypeError, ValueError, IndexError):
            issues.append(f"contact range {r!r} ({x}): no [a, b]")
            continue
        if not (FRAME_RANGE[0] <= a <= b <= FRAME_RANGE[1]):
            issues.append(f"contact range {x} [{a}, {b}] outside {list(FRAME_RANGE)} or a > b")
            continue
        out.append({"foot": x, "a": a, "b": b})
    return out, issues


def num_list(v, n):
    if not isinstance(v, (list, tuple)) or len(v) != n:
        return None
    try:
        return [float(x) for x in v]
    except (TypeError, ValueError):
        return None


def parse_channels(ch):
    """JSON control entry -> ({loc, euler (rad), quat, scale}, issues)."""
    out, issues = {}, []
    if not isinstance(ch, dict):
        return out, [f"control value is not an object: {ch!r}"]
    for k, v in ch.items():
        if k in ("loc", "location"):
            x = num_list(v, 3)
            key = "loc"
        elif k in ("rot", "rot_deg", "rotation_euler_deg", "euler_deg"):
            x = num_list(v, 3)
            x = [math.radians(a) for a in x] if x else None
            key = "euler"
        elif k == "rotation_euler":
            x = num_list(v, 3)
            key = "euler"
        elif k in ("rotation_quaternion", "quat", "quaternion"):
            x = num_list(v, 4)
            key = "quat"
        elif k == "scale":
            x = num_list(v, 3)
            key = "scale"
        elif k in ("rotation_mode", "mode"):
            continue
        else:
            issues.append(f"unknown channel {k!r}")
            continue
        if x is None:
            issues.append(f"channel {k!r} unparseable: {v!r}")
            continue
        out[key] = x
    return out, issues


# ---------------------------------------------------------------- mesh topology
class Topo:
    """Topology of a mesh (indices shared with its evaluated mesh; = check_g5 Topo subset)."""

    def __init__(self, obj, part_names=None):
        me = obj.data
        self.nv, self.npoly = len(me.vertices), len(me.polygons)
        ls = np.empty(self.npoly, dtype=np.int32)
        lt = np.empty(self.npoly, dtype=np.int32)
        me.polygons.foreach_get("loop_start", ls)
        me.polygons.foreach_get("loop_total", lt)
        lv = np.empty(len(me.loops), dtype=np.int32)
        me.loops.foreach_get("vertex_index", lv)
        self.loop_vert = lv.astype(np.int64)
        self.poly_verts = [self.loop_vert[s:s + n].tolist() for s, n in zip(ls.tolist(), lt.tolist())]
        me.calc_loop_triangles()
        nt = len(me.loop_triangles)
        tv = np.empty(nt * 3, dtype=np.int32)
        tp = np.empty(nt, dtype=np.int32)
        me.loop_triangles.foreach_get("vertices", tv)
        me.loop_triangles.foreach_get("polygon_index", tp)
        self.tri_v = tv.reshape(nt, 3).astype(np.int64)
        self.tri_poly = tp.astype(np.int64)
        self.part = None
        att = me.attributes.get("part_id")
        if att is not None and att.domain == "FACE" and att.data_type == "INT":
            pv = np.empty(self.npoly, dtype=np.int32)
            att.data.foreach_get("value", pv)
            self.part = pv.astype(np.int64)
        self.part_names = part_names or {}
        self.pid_name = {int(v): k for k, v in self.part_names.items()}

    def polys_of(self, name):
        pid = self.part_names.get(name)
        if pid is None or self.part is None:
            return np.zeros(0, dtype=np.int64)
        return np.nonzero(self.part == pid)[0]

    def verts_of_polys(self, polys):
        if not len(polys):
            return np.zeros(0, dtype=np.int64)
        return np.unique(np.concatenate([np.array(self.poly_verts[int(p)], dtype=np.int64)
                                         for p in polys.tolist()]))

    def tris_of_polys(self, polys):
        m = np.zeros(self.npoly, dtype=bool)
        m[polys] = True
        return np.nonzero(m[self.tri_poly])[0]


def eval_verts(obj):
    """World coords (nv,3) of the depsgraph-evaluated mesh of obj (= check_g5)."""
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
    return co.reshape(n, 3).astype(np.float64) @ mw[:3, :3].T + mw[:3, 3], oe.matrix_world.copy()


def bvh_of(V, tri_v):
    return BVHTree.FromPolygons([tuple(p) for p in V.tolist()], tri_v.tolist(), all_triangles=True)


def _inside(bvh, p):
    """Ray parity majority of 3 directions (= check_g5)."""
    votes = 0
    for d in RAY_DIRS:
        o, n = p.copy(), 0
        for _ in range(512):
            hit = bvh.ray_cast(o, d)
            if hit[0] is None:
                break
            n += 1
            o = hit[0] + d * RAY_EPS
        votes += n % 2
    return votes >= 2


def pairs_overlap(topo, V, tris, sets, keep):
    """Self overlap of tris -> (sorted unique poly pairs, {poly pair: [(global tri, global tri), ...]})
    (same poly / shared vertex dropped, keep(a, b))."""
    if not len(tris):
        return [], {}
    tp = topo.tri_poly[tris]
    bvh = bvh_of(V, topo.tri_v[tris])
    pairs = {}
    for i, j in bvh.overlap(bvh):
        a, b = int(tp[i]), int(tp[j])
        if a == b or sets[a] & sets[b] or not keep(a, b):
            continue
        key = (min(a, b), max(a, b))
        tt = (int(tris[i]), int(tris[j]))
        lst = pairs.setdefault(key, [])
        if tt not in lst and tt[::-1] not in lst:
            lst.append(tt)
    return sorted(pairs), pairs


def tri_pair_point(V, tri_v, ta, tb):
    """Representative intersection point of 2 triangles: mean of the edge x triangle hits (midpoint of the
    intersection segment); fallback midpoint of the closest vertex-to-triangle pair."""
    from mathutils import geometry
    A = [Vector(V[k]) for k in tri_v[ta]]
    B = [Vector(V[k]) for k in tri_v[tb]]
    pts = []
    for P, T in ((A, B), (B, A)):
        for k in range(3):
            p, q = P[k], P[(k + 1) % 3]
            d = q - p
            L = d.length
            if L <= 0.0:
                continue
            hit = geometry.intersect_ray_tri(T[0], T[1], T[2], d, p, True)
            if hit is not None and (hit - p).length <= L:
                pts.append(hit)
    if pts:
        s = Vector((0.0, 0.0, 0.0))
        for x in pts:
            s += x
        return s / len(pts), "segment"
    best = None
    for P, T in ((A, B), (B, A)):
        for p in P:
            c = geometry.closest_point_on_tri(p, T[0], T[1], T[2])
            dd = (c - p).length
            if best is None or dd < best[0]:
                best = (dd, (p + c) * 0.5)
    return best[1], "closest"


def isect_cover(pairs):
    """Minimum face set covering every pair: 0 no pairs; 1 one face in every pair; '>=2' (= check_g5)."""
    if not pairs:
        return 0, None
    common = set(pairs[0])
    for pr in pairs[1:]:
        common &= set(pr)
        if not common:
            return ">=2", None
    return 1, min(common)


# ---------------------------------------------------------------- context
class Ctx:
    def __init__(self):
        self.miss = []
        self.rig = self.mesh = self.club = None
        self.kp = self.canon = self.man = self.parts = None
        self.kp_path = KEYPOSES_JSON
        self.def_names = []
        self.act = None
        self.act_info = {}
        self.saved = None
        self.ctrl_set = []
        self.prop_defaults = {}
        self.poses = {}          # frame -> normalized pose
        self.kp_issues = []
        self.ranges = []
        self.cache = {}          # (source, frame) -> measurement
        self.notes = []


# ---------------------------------------------------------------- state
def props_bone(ctx):
    return ctx.rig.pose.bones.get(PROPS)


def evaluate(ctx):
    ctx.rig.update_tag()
    bpy.context.view_layer.update()


def assign_action(ctx):
    """In memory only (= check_g7 assign_action)."""
    rig, act = ctx.rig, ctx.act
    ad = rig.animation_data or rig.animation_data_create()
    if ad.action != act:
        ad.action = act
    if hasattr(ad, "action_slot") and ad.action_slot is None:
        from bpy_extras import anim_utils
        slot = anim_utils.action_get_first_suitable_slot(act, "OBJECT")
        if slot is not None:
            ad.action_slot = slot
    slot = getattr(ad, "action_slot", None)
    return getattr(slot, "identifier", None) if slot is not None else None


def detach_action(ctx):
    ad = ctx.rig.animation_data
    if ad is not None and ad.action is not None:
        ad.action = None


def save_state(ctx):
    rig = ctx.rig
    pose = [(pb.name, pb.rotation_mode, tuple(pb.location), tuple(pb.rotation_quaternion),
             tuple(pb.rotation_euler), tuple(pb.rotation_axis_angle), tuple(pb.scale)) for pb in rig.pose.bones]
    props = {}
    h = props_bone(ctx)
    if h is not None:
        for k in h.keys():
            v = h[k]
            props[k] = v.to_list() if hasattr(v, "to_list") else (v.to_dict() if hasattr(v, "to_dict") else v)
    ad = rig.animation_data
    ctx.saved = {"pose": pose, "props": props, "frame": bpy.context.scene.frame_current,
                 "action": ad.action if ad else None,
                 "slot": getattr(ad, "action_slot", None) if ad else None,
                 "autokey": bpy.context.scene.tool_settings.use_keyframe_insert_auto}


def restore_state(ctx):
    s = ctx.saved
    if not s:
        return
    rig = ctx.rig
    ad = rig.animation_data
    if ad is not None:
        ad.action = s["action"]
        if s["action"] is not None and s["slot"] is not None and hasattr(ad, "action_slot"):
            try:
                ad.action_slot = s["slot"]
            except Exception:
                pass
    for name, mode, loc, q, e, aa, sc in s["pose"]:
        pb = rig.pose.bones.get(name)
        if pb is None:
            continue
        pb.rotation_mode = mode
        pb.location, pb.rotation_quaternion, pb.rotation_euler = loc, q, e
        pb.rotation_axis_angle, pb.scale = aa, sc
    h = props_bone(ctx)
    if h is not None:
        for k, v in s["props"].items():
            try:
                h[k] = v
            except Exception:
                pass
    bpy.context.scene.tool_settings.use_keyframe_insert_auto = s["autokey"]
    bpy.context.scene.frame_set(s["frame"])
    evaluate(ctx)


def prepare(ctx):
    rig = ctx.rig
    bpy.context.scene.tool_settings.use_keyframe_insert_auto = False
    rig.data.pose_position = "POSE"
    names = {b.name for b in rig.data.bones if b.name.startswith("CTRL_") or b.name == PROPS}
    ctx.ctrl_set = sorted(names)
    h = props_bone(ctx)
    ctx.prop_defaults = {}
    if h is not None:
        for k in h.keys():
            if k.startswith("_"):
                continue
            v = h[k]
            if not isinstance(v, (int, float)) or isinstance(v, bool):
                continue
            ud = None
            try:
                d = h.id_properties_ui(k).as_dict().get("default")
                ud = d if isinstance(d, (int, float)) and not isinstance(d, bool) else None
            except Exception:
                pass
            ctx.prop_defaults[k] = CONTRACT_DEFAULTS.get(k, ud if ud is not None else v)


def set_prop(ctx, k, v):
    h = props_bone(ctx)
    cur = h[k]
    h[k] = int(round(float(v))) if isinstance(cur, int) and not isinstance(cur, bool) else float(v)


def apply_json_pose(ctx, pose):
    """Action detached; CTRL_* / PROPS identity, PROPS defaults, then the JSON values; evaluated."""
    detach_action(ctx)
    for n in ctx.ctrl_set:
        pb = ctx.rig.pose.bones[n]
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        pb.rotation_euler = (0.0, 0.0, 0.0)
        pb.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
        pb.scale = (1.0, 1.0, 1.0)
    for k, v in ctx.prop_defaults.items():
        set_prop(ctx, k, v)
    for b, ch in pose["ctrl"].items():
        pb = ctx.rig.pose.bones.get(b)
        if pb is None:
            continue
        if "loc" in ch:
            pb.location = ch["loc"]
        if "scale" in ch:
            pb.scale = ch["scale"]
        q = None
        if "quat" in ch:
            q = Quaternion(ch["quat"])
        elif "euler" in ch:
            if pb.rotation_mode not in ("QUATERNION", "AXIS_ANGLE"):
                pb.rotation_euler = Euler(ch["euler"], "XYZ").to_matrix().to_euler(pb.rotation_mode) \
                    if pb.rotation_mode != "XYZ" else Euler(ch["euler"], "XYZ")
            else:
                q = Euler(ch["euler"], "XYZ").to_quaternion()
        if q is not None:
            if pb.rotation_mode == "QUATERNION":
                pb.rotation_quaternion = q
            elif pb.rotation_mode == "AXIS_ANGLE":
                ax, ang = q.to_axis_angle()
                pb.rotation_axis_angle = (ang, ax.x, ax.y, ax.z)
            else:
                pb.rotation_euler = q.to_euler(pb.rotation_mode)
    h = props_bone(ctx)
    for k, v in pose["props"].items():
        if h is not None and k in h.keys():
            set_prop(ctx, k, v)
    evaluate(ctx)


def goto_frame(ctx, f):
    bpy.context.scene.frame_set(int(f))
    evaluate(ctx)


# ---------------------------------------------------------------- input parsing
def load_inputs(ctx, kp_path):
    def lj(p):
        if not Path(p).exists():
            ctx.miss.append(f"missing input: {rel(p)}")
            return None
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)

    if not bpy.data.filepath:
        ctx.miss.append("missing input: no blend loaded")
    ctx.kp = lj(kp_path)
    ctx.canon = lj(CANON_JSON)
    ctx.man = lj(MANIFEST_JSON)
    ctx.parts = lj(PARTS_JSON)
    for name, attr in ((ARM, "rig"), (MESH, "mesh"), (CLUB, "club")):
        o = bpy.data.objects.get(name)
        if o is None:
            ctx.miss.append(f"missing input: object {name} in {rel(bpy.data.filepath or '?')}")
        setattr(ctx, attr, o)
    if ctx.canon is not None:
        ctx.def_names = [b["name"] for b in ctx.canon.get("bones", [])]
        if ctx.rig is not None:
            lost = [n for n in ctx.def_names if n not in ctx.rig.pose.bones]
            if lost:
                ctx.miss.append(f"DEF bones missing in {ARM}: {lost}")
    ctx.act = bpy.data.actions.get(ACTION)


def parse_keyposes(ctx):
    kp = ctx.kp
    issues = []
    raw = kp.get("poses") if isinstance(kp, dict) else None
    items = []
    if isinstance(raw, list):
        items = list(raw)
    elif isinstance(raw, dict):
        for k, v in raw.items():
            if isinstance(v, dict):
                v = dict(v)
                v.setdefault("name", k if not str(k).isdigit() else v.get("name"))
                if str(k).isdigit():
                    v.setdefault("frame", int(k))
                items.append(v)
    else:
        issues.append("poses missing or not a list/object")
    rig_bones = ctx.rig.pose.bones
    h = props_bone(ctx)
    prop_keys = set(h.keys()) if h is not None else set()
    per_pose = {}
    for p in items:
        pi = []
        if not isinstance(p, dict):
            issues.append(f"pose entry is not an object: {p!r}"[:200])
            continue
        try:
            f = int(p.get("frame"))
        except (TypeError, ValueError):
            issues.append(f"pose {p.get('name')!r}: frame missing / not int")
            continue
        name = p.get("name")
        if not isinstance(name, str) or not name.strip():
            pi.append("name missing")
        contact = parse_contact(p.get("contact"))
        if contact is None:
            pi.append(f"contact missing / unparseable: {p.get('contact')!r}")
        craw = p.get("ctrl", p.get("ctrls", p.get("controls")))
        ctrl, props = {}, {}
        if not isinstance(craw, dict) or not craw:
            pi.append("ctrl values missing / empty")
            craw = {}
        for b, ch in craw.items():
            if b == PROPS and isinstance(ch, dict) and not any(k in ch for k in ("loc", "rot", "location")):
                props.update(ch)
                continue
            if b not in rig_bones:
                pi.append(f"ctrl bone {b} not in {ARM}")
                continue
            if not (b.startswith("CTRL_") or b == PROPS):
                pi.append(f"ctrl entry {b} is not a CTRL_*/PROPS bone")
                continue
            chans, ci = parse_channels(ch)
            pi.extend(f"{b}: {x}" for x in ci)
            ctrl[b] = chans
        praw = p.get("props", p.get("PROPS", {}))
        if isinstance(praw, dict):
            props.update(praw)
        elif praw:
            pi.append("props is not an object")
        pv = {}
        for k, v in props.items():
            if k not in prop_keys:
                pi.append(f"prop {k} not on {PROPS}")
                continue
            try:
                pv[k] = float(v)
            except (TypeError, ValueError):
                pi.append(f"prop {k} value {v!r} not numeric")
        dsnap = None
        if p.get("def") is not None:
            dsnap, bad = {}, []
            for b, m in (p["def"].items() if isinstance(p["def"], dict) else []):
                mm_ = m.get("matrix") if isinstance(m, dict) else m
                try:
                    M = Matrix(mm_)
                    assert len(M) == 4 and len(M[0]) == 4
                    dsnap[b] = M
                except Exception:
                    bad.append(b)
            if bad or not dsnap:
                pi.append(f"def snapshot unparseable for {bad or 'all'}")
        if f in ctx.poses:
            issues.append(f"frame {f} defined more than once")
        ctx.poses[f] = {"frame": f, "name": name, "contact": contact, "ctrl": ctrl, "props": pv,
                        "def": dsnap, "n_ctrl": len(ctrl)}
        per_pose[f] = pi
    ctx.ranges, ri = parse_ranges(kp.get("contact_ranges") if isinstance(kp, dict) else None)
    issues.extend(ri)
    ctx.kp_issues = issues
    return per_pose


def action_info(ctx):
    act = ctx.act
    if act is None:
        return {"action": None}
    fcs = fcurves_of(act)
    keyed, non_ctrl, bones = set(), [], set()
    for fc in fcs:
        for k in fc.keyframe_points:
            keyed.add(int(round(k.co.x)))
        m = _BONE_RE.search(fc.data_path)
        b = m.group(1) if m else None
        if b is None or not (b.startswith("CTRL_") or b == PROPS):
            non_ctrl.append(f"{fc.data_path}[{fc.array_index}]")
        if b is not None:
            bones.add(b)
    f0, f1 = act.frame_range
    sc = bpy.context.scene
    ad = ctx.rig.animation_data
    return {"action": act.name, "active_before": ad.action.name if ad and ad.action else None,
            "n_fcurves": len(fcs), "keyed_frames": sorted(keyed), "non_ctrl_paths": non_ctrl,
            "keyed_bones": sorted(bones), "frame_range": [rnd(f0, 4), rnd(f1, 4)],
            "use_frame_range": bool(getattr(act, "use_frame_range", False)),
            "scene_frame_start_end": [sc.frame_start, sc.frame_end],
            "fps": rnd(sc.render.fps / (sc.render.fps_base or 1.0), 6),
            "nla_tracks": len(ad.nla_tracks) if ad else 0,
            "root_fcurves": [f"{fc.data_path}[{fc.array_index}]" for fc in fcs
                             if _BONE_RE.search(fc.data_path) and _BONE_RE.search(fc.data_path).group(1) == "CTRL_root"]}


# ---------------------------------------------------------------- static geometry
class Geo:
    def __init__(self, ctx):
        mo = ctx.mesh
        self.topo = t = Topo(mo, ctx.parts)
        self.ctopo = Topo(ctx.club)
        if t.part is None:
            raise Blocked(f"{MESH} face INT attribute part_id missing")
        # weights
        names = {vg.index: vg.name for vg in mo.vertex_groups}
        fw = np.zeros(t.nv)
        self.vbone = [None] * t.nv
        self.vw = [dict() for _ in range(t.nv)]
        for v in mo.data.vertices:
            d = {names.get(g.group, f"#{g.group}"): float(g.weight) for g in v.groups}
            self.vw[v.index] = d
            fw[v.index] = sum(d.get(b, 0.0) for b in FOREARM_BONES)
            if d:
                self.vbone[v.index] = max(d, key=d.get)
        body = t.polys_of("body")
        excl = np.zeros(t.npoly, dtype=bool)
        for pn in CLUB_EXCLUDED_PARTS:
            excl[t.polys_of(pn)] = True
        n_body_ex = 0
        for p in body.tolist():
            if float(np.mean(fw[t.poly_verts[p]])) > FOREARM_W:
                excl[p] = True
                n_body_ex += 1
        self.excl = excl
        self.target_polys = np.nonzero(~excl)[0]
        self.target_tris = t.tris_of_polys(self.target_polys)
        self.target_verts = t.verts_of_polys(self.target_polys)
        self.shell_tris = {pn: t.tris_of_polys(t.polys_of(pn)) for pn in SHELL_PARTS}
        self.shell_verts = {pn: t.verts_of_polys(t.polys_of(pn)) for pn in SHELL_PARTS}
        self.vpart = np.full(t.nv, -1, dtype=np.int64)
        for p in range(t.npoly):
            self.vpart[t.poly_verts[p]] = t.part[p]
        self.excl_info = {"parts": list(CLUB_EXCLUDED_PARTS), "forearm_bones": list(FOREARM_BONES),
                          "forearm_weight_gt": FOREARM_W, "n_body_faces_excluded": n_body_ex,
                          "n_faces_excluded_total": int(excl.sum()), "n_target_faces": int(len(self.target_polys))}
        # self intersection regions
        self.body_polys = body
        self.body_tris = t.tris_of_polys(body)
        rig_p = np.concatenate([t.polys_of(pn) for pn in RIGID_PARTS])
        self.rigid_tris = t.tris_of_polys(rig_p)
        self.sets = {p: set(t.poly_verts[p]) for p in range(t.npoly)}
        # club local s (local Y) of each club vertex
        cv = np.empty(len(ctx.club.data.vertices) * 3, dtype=np.float64)
        ctx.club.data.vertices.foreach_get("co", cv)
        self.club_s = cv.reshape(-1, 3)[:, 1]

    def part_name(self, pid):
        return self.topo.pid_name.get(int(pid), f"#{int(pid)}")

    def face_bone(self, poly):
        acc = {}
        for v in self.topo.poly_verts[poly]:
            for b, w in self.vw[v].items():
                acc[b] = acc.get(b, 0.0) + w
        return max(acc, key=acc.get) if acc else None


# ---------------------------------------------------------------- per frame metrics
def clearance(ctx, geo, V, C, cmw):
    t, ct = geo.topo, geo.ctopo
    bvh_t = bvh_of(V, t.tri_v[geo.target_tris])
    bvh_c = bvh_of(C, ct.tri_v)
    best = {"d": float("inf")}
    # (a) club verts -> target tris
    for i, p in enumerate(C.tolist()):
        loc, _n, ti, d = bvh_t.find_nearest(Vector(p))
        if loc is not None and d < best["d"]:
            poly = int(t.tri_poly[geo.target_tris[ti]])
            best = {"d": d, "dir": "club->body", "part": geo.part_name(t.part[poly]), "bone": geo.face_bone(poly),
                    "face": poly, "club_s_mm": mm(geo.club_s[i]), "point_mm": [mm(x) for x in loc]}
    # (b) target verts near the club -> club tris
    lo, hi = C.min(axis=0) - best["d"], C.max(axis=0) + best["d"]
    tv = geo.target_verts
    near = tv[np.all((V[tv] >= lo) & (V[tv] <= hi), axis=1)]
    cinv = cmw.inverted()
    for vi in near.tolist():
        loc, _n, _ti, d = bvh_c.find_nearest(Vector(V[vi]))
        if loc is not None and d < best["d"]:
            best = {"d": d, "dir": "body->club", "part": geo.part_name(geo.vpart[vi]), "bone": geo.vbone[vi],
                    "vert": vi, "club_s_mm": mm((cinv @ loc).y), "point_mm": [mm(x) for x in V[vi]]}
    # penetration
    depth, n_cin, n_bin, where = 0.0, 0, 0, None
    for pn, tris in geo.shell_tris.items():
        if not len(tris):
            continue
        sv = V[geo.shell_verts[pn]]
        slo, shi = sv.min(axis=0), sv.max(axis=0)
        cand = np.nonzero(np.all((C >= slo) & (C <= shi), axis=1))[0]
        if not len(cand):
            continue
        bvh_s = bvh_of(V, t.tri_v[tris])
        for i in cand.tolist():
            pv = Vector(C[i])
            if not _inside(bvh_s, pv):
                continue
            loc, _n, ti, d = bvh_s.find_nearest(pv)
            poly = int(t.tri_poly[tris[ti]])
            if geo.excl[poly]:
                continue
            n_cin += 1
            if d > depth:
                depth, where = d, {"dir": "club vert inside", "part": pn, "bone": geo.face_bone(poly),
                                   "club_s_mm": mm(geo.club_s[i]), "point_mm": [mm(x) for x in C[i]]}
    clo, chi = C.min(axis=0), C.max(axis=0)
    candb = tv[np.all((V[tv] >= clo) & (V[tv] <= chi), axis=1)]
    for vi in candb.tolist():
        pv = Vector(V[vi])
        if not _inside(bvh_c, pv):
            continue
        n_bin += 1
        loc, _n, _ti, d = bvh_c.find_nearest(pv)
        if d > depth:
            depth, where = d, {"dir": "body vert inside club", "part": geo.part_name(geo.vpart[vi]),
                               "bone": geo.vbone[vi], "club_s_mm": mm((cinv @ loc).y),
                               "point_mm": [mm(x) for x in V[vi]]}
    ov = bvh_c.overlap(bvh_t)
    ov_at = None
    if ov:
        poly = int(t.tri_poly[geo.target_tris[ov[0][1]]])
        ov_at = {"part": geo.part_name(t.part[poly]), "bone": geo.face_bone(poly), "face": poly}
    if n_cin or n_bin:
        clear = -depth
    elif ov:
        clear = 0.0
    else:
        clear = best["d"]
    at = where if (n_cin or n_bin) else (ov_at if ov else {k: v for k, v in best.items() if k != "d"})
    return {"clearance_mm": mm(clear), "min_dist_mm": mm(best["d"]), "penetration": bool(n_cin or n_bin or ov),
            "max_depth_mm": mm(depth), "n_club_verts_inside": n_cin, "n_body_verts_inside_club": n_bin,
            "n_tri_overlaps": len(ov), "at": at, "_clear": clear}


def self_isect(ctx, geo, V):
    t = geo.topo
    body_pairs, body_tt = pairs_overlap(t, V, geo.body_tris, geo.sets, lambda a, b: True)
    rigid_pairs, rigid_tt = pairs_overlap(t, V, geo.rigid_tris, geo.sets, lambda a, b: t.part[a] != t.part[b])
    allp = body_pairs + rigid_pairs
    cov, face = isect_cover(allp)
    rp_parts = sorted({f"{geo.part_name(t.part[a])}-{geo.part_name(t.part[b])}" for a, b in rigid_pairs})
    # visible filter: representative point inside a rigid shell (other than the pair's own parts) -> hidden
    shells = {}
    for pn in RIGID_PARTS:
        tris = t.tris_of_polys(t.polys_of(pn))
        if not len(tris):
            continue
        sv = V[t.verts_of_polys(t.polys_of(pn))]
        shells[pn] = (tris, sv.min(axis=0), sv.max(axis=0), None)
    hidden_by, visible, n_fallback = {}, [], 0
    for key in allp:
        tt = body_tt.get(key) or rigid_tt.get(key) or []
        own = {geo.part_name(t.part[key[0]]), geo.part_name(t.part[key[1]])}
        pair_hidden_by = None
        for ta, tb in tt:
            p, how = tri_pair_point(V, t.tri_v, ta, tb)
            n_fallback += how != "segment"
            hb = None
            for pn, (tris, lo, hi, bvh) in shells.items():
                if pn in own or not all(lo[k] <= p[k] <= hi[k] for k in range(3)):
                    continue
                if bvh is None:
                    bvh = bvh_of(V, t.tri_v[tris])
                    shells[pn] = (tris, lo, hi, bvh)
                if _inside(bvh, p):
                    hb = pn
                    break
            if hb is None:
                pair_hidden_by = None
                break
            pair_hidden_by = hb
        if tt and pair_hidden_by is not None:
            hidden_by[pair_hidden_by] = hidden_by.get(pair_hidden_by, 0) + 1
        else:
            visible.append(key)
    vcov, vface = isect_cover(visible)
    return {"total_pairs": len(allp), "total_cover": cov, "total_common_face": face,
            "visible_pairs": len(visible), "visible_cover": vcov, "visible_common_face": vface,
            "hidden_pairs_by_part": hidden_by, "n_hidden_pairs": sum(hidden_by.values()),
            "n_tri_pairs_point_fallback": n_fallback,
            "n_body_pairs": len(body_pairs),
            "n_rigid_pairs": len(rigid_pairs), "rigid_pair_parts": rp_parts,
            "visible_pair_examples": visible[:LIST_CAP],
            "visible_pair_bones": sorted({geo.face_bone(f) or "?" for pr in visible for f in pr})[:LIST_CAP],
            "body_pair_examples": body_pairs[:LIST_CAP],
            "body_pair_bones": sorted({geo.face_bone(f) or "?" for pr in body_pairs for f in pr})[:LIST_CAP]}


def local_rot(rig, oe, name):
    b = rig.data.bones[name]
    pb = oe.pose.bones[name]
    if b.parent is not None:
        rr = b.parent.matrix_local.inverted() @ b.matrix_local
        rp = oe.pose.bones[b.parent.name].matrix.inverted() @ pb.matrix
    else:
        rr, rp = b.matrix_local, pb.matrix
    return (rr.inverted() @ rp).to_3x3().normalized()


def e_deg(m3):
    return [math.degrees(a) for a in m3.to_euler("XYZ")]


def ctrl_euler_deg(pb):
    if pb.rotation_mode == "XYZ":
        return [math.degrees(a) for a in pb.rotation_euler]
    return [math.degrees(a) for a in pb.matrix_basis.to_3x3().normalized().to_euler("XYZ")]


def ik_chains(ctx):
    out = {}
    for key, ch in (ctx.man.get("ikfk") or {}).items():
        mch = ch.get("mch_ik") or []
        ikc = ch.get("ik_constraint") or {}
        bone = ikc.get("bone") or (mch[1] if len(mch) > 1 else None)
        pb = ctx.rig.pose.bones.get(bone) if bone else None
        tgt = ikc.get("target")
        if pb is not None and tgt is None:
            for c in pb.constraints:
                if c.type == "IK":
                    tgt = c.subtarget or None
                    break
        out[key] = {"prop": ch.get("prop"), "def": ch.get("def") or [], "mch": mch, "bone": bone, "target": tgt}
    return out


def measure(ctx, geo, heavy=True):
    """Metrics of the current evaluated state."""
    rig = ctx.rig
    dg = bpy.context.evaluated_depsgraph_get()
    oe = rig.evaluated_get(dg)
    mw = oe.matrix_world
    pbs = oe.pose.bones
    r = {}
    r["def_world"] = {n: (mw @ pbs[n].matrix).copy() for n in ctx.def_names}
    r["def_head_tail"] = {n: ((mw @ pbs[n].head).copy(), (mw @ pbs[n].tail).copy())
                          for n in ("foot_l", "foot_r") if n in pbs}
    ctrl = {}
    for n in ctx.ctrl_set:
        pb = pbs[n]
        ctrl[n] = {"loc": list(pb.location), "euler": list(pb.rotation_euler), "quat": list(pb.rotation_quaternion),
                   "scale": list(pb.scale), "mode": pb.rotation_mode, "basis": pb.matrix_basis.copy()}
    r["ctrl"] = ctrl
    r["root_world"] = (mw @ pbs["CTRL_root"].matrix).copy() if "CTRL_root" in pbs else None
    hp = pbs.get(PROPS)
    props = {}
    if hp is not None:
        for k in hp.keys():
            v = hp[k]
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                props[k] = float(v)
    r["props"] = props
    # bend theta (G6.5b)
    th = {}
    for key, ch in ctx.chains.items():
        d = ch["def"]
        if len(d) >= 2 and d[0] in pbs and d[1] in pbs:
            pu, pl = pbs[d[0]], pbs[d[1]]
            u = ((mw @ pu.tail) - (mw @ pu.head)).normalized()
            ll = ((mw @ pl.tail) - (mw @ pl.head)).normalized()
            x = (mw.to_3x3() @ pu.matrix.to_3x3()).col[0].normalized()
            th[key] = math.degrees(math.atan2(u.cross(ll).dot(x), u.dot(ll)))
    r["theta"] = th
    # safe range values
    s = {}
    for x in SIDES:
        sg = 1.0 if x == "l" else -1.0
        ua = e_deg(local_rot(rig, oe, f"upperarm_{x}"))
        sh = e_deg(local_rot(rig, oe, f"shoulder_{x}"))
        la = e_deg(local_rot(rig, oe, f"lowerarm_{x}"))
        hm = local_rot(rig, oe, f"hand_{x}")
        tw, sw = twist_angle_deg(hm)
        th_ = e_deg(local_rot(rig, oe, f"thigh_{x}"))
        ca = e_deg(local_rot(rig, oe, f"calf_{x}"))
        s[f"upperarm_raise_{x}"] = sg * ua[2]
        s[f"upperarm_fwd_{x}"] = ua[0]
        s[f"upperarm_twist_Y_{x} (report)"] = ua[1]
        s[f"shoulder_raise_{x}"] = sg * sh[2]
        s[f"shoulder_fwd_{x} (report)"] = sh[0]
        s[f"elbow_flex_{x}"] = la[0]
        s[f"wrist_bend_{x}"] = sw
        s[f"wrist_twist_{x}"] = tw
        s[f"wrist_euler_XZ_{x} (report)"] = [e_deg(hm)[0], e_deg(hm)[2]]
        s[f"hip_flex_{x}"] = -th_[0]
        s[f"hip_abduct_{x} (report)"] = -th_[2] if x == "l" else th_[2]
        s[f"knee_flex_{x}"] = ca[0]
    tb = rig.data.bones.get("CTRL_torso")
    s["cog_dz_m"] = ((mw @ pbs["CTRL_torso"].head).z - (mw @ tb.head_local).z) if tb else None
    for c in ("CTRL_spine_01", "CTRL_chest"):
        s[f"{c}_euler"] = ctrl_euler_deg(pbs[c]) if c in pbs else None
    for c in ("CTRL_torso", "CTRL_pelvis", "CTRL_head"):
        s[f"{c}_euler (report)"] = ctrl_euler_deg(pbs[c]) if c in pbs else None
    for n in ("spine_01", "spine_02", "head"):
        s[f"DEF_{n}_local_euler (report)"] = e_deg(local_rot(rig, oe, n))
    ikr = {}
    for key, ch in ctx.chains.items():
        mch = ch["mch"]
        if len(mch) < 2 or ch["target"] not in pbs or mch[0] not in pbs:
            ikr[key] = None
            continue
        L = rig.data.bones[mch[0]].length + rig.data.bones[mch[1]].length
        d = ((mw @ pbs[ch["target"]].head) - (mw @ pbs[mch[0]].head)).length
        ikr[key] = {"ratio": d / L if L > 0 else None, "ik_fk": props.get(ch["prop"]),
                    "target": ch["target"], "def_ratio (report)": (
                        ((mw @ pbs[ch["def"][2]].head) - (mw @ pbs[ch["def"][0]].head)).length / L
                        if len(ch["def"]) >= 3 and L > 0 else None)}
    s["ik"] = ikr
    s["foot_props"] = {k: props.get(k) for k in (ctx.man.get("clamps") or {})}
    r["safe"] = s
    # planted
    pl = {}
    for x in SIDES:
        ch = ctx.chains.get(f"leg_{x}")
        reach = None
        if ch and ch["bone"] in pbs and ch["target"] in pbs:
            reach = ((mw @ pbs[ch["bone"]].tail) - (mw @ pbs[ch["target"]].head)).length
        pl[x] = {"reach": reach, "leg_ik_fk": props.get(f"leg_ik_fk_{x}")}
    if heavy:
        V, _ = eval_verts(ctx.mesh)
        if len(V) != geo.topo.nv:
            raise RuntimeError(f"evaluated {MESH} has {len(V)} verts, obj.data has {geo.topo.nv}")
        C, cmw = eval_verts(ctx.club)
        r["clear"] = clearance(ctx, geo, V, C, cmw)
        r["isect"] = self_isect(ctx, geo, V)
        for x in SIDES:
            sv = geo.shoe_v[x]
            pl[x]["shoe_min_z"] = float(V[sv, 2].min()) if len(sv) else None
    r["planted"] = pl
    return r


def safe_out_of_range(ctx, s):
    """-> (values dict for the evidence, list of out-of-range items)."""
    out, vals = [], {}

    def chk(item, v, key):
        op, lim = SAFE_LIMITS[key]
        vals[item] = rnd(v, 4)
        if v is None:
            return
        bad = v > lim if op == "<=" else v < lim
        if bad:
            out.append({"item": item, "value": rnd(v, 3), "limit": f"{op} {lim}"})

    for x in SIDES:
        chk(f"upperarm_raise_{x}", s[f"upperarm_raise_{x}"], "upperarm_raise")
        v = s[f"upperarm_fwd_{x}"]
        chk(f"upperarm_fwd_{x}", v, "upperarm_fwd_max")
        if v < SAFE_LIMITS["upperarm_fwd_min"][1]:
            out.append({"item": f"upperarm_fwd_{x}", "value": rnd(v, 3), "limit": ">= -45.0"})
        chk(f"shoulder_raise_{x}", s[f"shoulder_raise_{x}"], "shoulder_raise")
        chk(f"elbow_flex_{x}", s[f"elbow_flex_{x}"], "elbow_flex")
        chk(f"wrist_bend_{x}", s[f"wrist_bend_{x}"], "wrist_bend")
        chk(f"wrist_twist_abs_{x}", abs(s[f"wrist_twist_{x}"]), "wrist_twist_abs")
        chk(f"hip_flex_{x}", s[f"hip_flex_{x}"], "hip_flex")
        chk(f"knee_flex_{x}", s[f"knee_flex_{x}"], "knee_flex")
    if s["cog_dz_m"] is not None:
        chk("cog_dz_m", s["cog_dz_m"], "cog_dz_m")
    for c in ("CTRL_spine_01", "CTRL_chest"):
        e = s.get(f"{c}_euler")
        if e is None:
            continue
        for ax, a in zip("XYZ", e):
            chk(f"{c}_abs_{ax}", abs(a), "spine_ctrl_abs")
    for key, v in s["ik"].items():
        if v is None or v["ratio"] is None:
            vals[f"ik_ratio_{key}"] = None
            continue
        if (v["ik_fk"] or 0.0) > 0.0:
            chk(f"ik_ratio_{key}", v["ratio"], "ik_ratio")
        else:
            vals[f"ik_ratio_{key} (FK, report)"] = rnd(v["ratio"], 4)
    for k, v in s["foot_props"].items():
        cl = (ctx.man.get("clamps") or {}).get(k)
        vals[k] = rnd(v, 4)
        if v is not None and cl and not (float(cl[0]) <= v <= float(cl[1])):
            out.append({"item": k, "value": rnd(v, 3), "limit": f"clamp {cl}"})
    for k, v in s.items():
        if k.endswith("(report)") and k not in vals:
            vals[k] = [rnd(a, 3) for a in v] if isinstance(v, list) else rnd(v, 4)
    vals["wrist_twist_signed"] = {x: rnd(s[f"wrist_twist_{x}"], 3) for x in SIDES}
    vals["ik_detail"] = {k: None if v is None else {kk: (rnd(vv, 4) if isinstance(vv, float) else vv)
                                                     for kk, vv in v.items()} for k, v in s["ik"].items()}
    return vals, out


# ---------------------------------------------------------------- evidence helpers
def blocked(ev, ids, reason):
    for i in ids:
        ev.criterion(i, f"blocked: {reason}", THRESHOLDS[i], False, "")


THRESHOLDS = {
    "A1.1": f"7 poses at frames {list(KEY_FRAMES)} in swing_keyposes.json, each with name, contact and CTRL values "
            f"(existing CTRL_*/PROPS bones, parseable channels, props on {PROPS}); contact_ranges parseable in 1..39",
    "A1.2": "report: section 2 draft limits " + json.dumps(SAFE_LIMITS),
    "A1.3": {"clearance_mm": ">= 10", "visible_isect_cover": "<= 1 (pairs hidden inside a rigid shell excluded; totals report)"},
    "A1.4": {"reach_mm": "<= 1", "shoe_min_z_mm": "[-1, +3]", "feet": "JSON contact feet"},
    "A2.1": {"fps": 24, "frames": "1..39", "keys": "CTRL_*/PROPS only",
             "key_pose_match": "DEF snapshot max abs <= 1e-4, else CTRL/PROPS channel |diff| <= 1e-4"},
    "A2.2": {"def_world_f39_vs_f1_max_abs": "<= 1e-4", "ctrl_root_motion": "0 (<= 1e-7 float)"},
    "A2.3": {"clearance_mm": ">= 10 every frame", "visible_isect_cover": "<= 1 every frame (A1.3 definition)"},
    "A2.4": {"branch_flips": 0},
    "A2.5": {"contact_foot_def_move_mm": "<= 1"},
}


def frame_measure(ctx, geo, f, source):
    key = (source, f)
    if key not in ctx.cache:
        if source == "action":
            goto_frame(ctx, f)
        else:
            apply_json_pose(ctx, ctx.poses[f])
        ctx.cache[key] = measure(ctx, geo)
        m = ctx.cache[key]
        print(f"[{GATE}/{CHECKER}] {source} f{f}: clearance_mm={m['clear']['clearance_mm']} "
              f"at={m['clear']['at'].get('part') if m['clear']['at'] else None} cover={m['isect']['total_cover']} visible_cover={m['isect']['visible_cover']}")
        sys.stdout.flush()
    return ctx.cache[key]


# ---------------------------------------------------------------- A1
def run_a1(ev, ctx, geo, per_pose_issues, source):
    # A1.1
    frames = sorted(ctx.poses)
    missing = [f for f in KEY_FRAMES if f not in ctx.poses]
    extra = [f for f in frames if f not in KEY_FRAMES]
    bad = {f: v for f, v in per_pose_issues.items() if v}
    ai = ctx.act_info
    keyed = set(ai.get("keyed_frames") or [])
    meas = {"frames_in_json": frames, "missing_frames": missing, "extra_frames": extra,
            "n_poses": len(ctx.poses), "per_pose_issues": bad, "file_issues": ctx.kp_issues,
            "poses": {f: {"name": p["name"], "contact": sorted(p["contact"]) if p["contact"] is not None else None,
                          "n_ctrl": p["n_ctrl"], "n_props": len(p["props"]), "def_snapshot": p["def"] is not None}
                      for f, p in sorted(ctx.poses.items())},
            "contact_ranges": ctx.ranges,
            "action_keys_at_key_frames (report)": {f: f in keyed for f in KEY_FRAMES} if ctx.act else None,
            "pose_source": source}
    ok = not missing and not extra and not bad and not ctx.kp_issues and len(ctx.poses) == len(KEY_FRAMES)
    ev.criterion("A1.1", meas, THRESHOLDS["A1.1"], ok,
                 "JSON schema read tolerantly (see checker docstring); rot = degrees Euler XYZ.")
    # A1.2 .. A1.4
    per2, outs, per3, per4 = {}, [], {}, {}
    ok3, ok4 = True, True
    worst = None
    for f in KEY_FRAMES:
        if source == "json" and f not in ctx.poses:
            per2[f] = per3[f] = per4[f] = "blocked: pose missing in JSON and no action"
            ok3 = ok4 = False
            continue
        m = frame_measure(ctx, geo, f, source)
        vals, out = safe_out_of_range(ctx, m["safe"])
        name = ctx.poses.get(f, {}).get("name")
        per2[f] = {"name": name, "values": vals}
        outs.extend(dict(o, frame=f, name=name) for o in out)
        c, i = m["clear"], m["isect"]
        per3[f] = {"name": name, **{k: v for k, v in c.items() if not k.startswith("_")}, "isect": i}
        lok = c["_clear"] >= CLEAR_MIN and i["visible_cover"] in (0, 1)
        ok3 = ok3 and lok
        if worst is None or c["_clear"] < worst[0]:
            worst = (c["_clear"], f)
        contact = ctx.poses.get(f, {}).get("contact")
        feet = {}
        for x in SIDES:
            p = m["planted"][x]
            planted = (p["reach"] is not None and p["reach"] <= PLANT_REACH and p["shoe_min_z"] is not None
                       and PLANT_Z[0] <= p["shoe_min_z"] <= PLANT_Z[1])
            is_c = None if contact is None else (x in contact)
            feet[x] = {"contact": is_c, "reach_mm": mm(p["reach"]), "shoe_min_z_mm": mm(p["shoe_min_z"]),
                       "leg_ik_fk": p["leg_ik_fk"], "planted": planted}
            if is_c is None or (is_c and not planted):
                ok4 = False
        per4[f] = {"name": name, "feet": feet}
    ev.criterion("A1.2", {"per_pose": per2, "out_of_range": outs, "n_out_of_range": len(outs)},
                 THRESHOLDS["A1.2"], not outs, SAFE_NOTE)
    covers = [v["isect"]["total_cover"] for v in per3.values() if isinstance(v, dict)]
    vcovers = [v["isect"]["visible_cover"] for v in per3.values() if isinstance(v, dict)]
    ev.criterion("A1.3", {"min_clearance_mm": mm(worst[0]) if worst else None,
                          "min_at_frame": worst[1] if worst else None,
                          "max_total_cover": max(covers, key=lambda c: 2 if c == ">=2" else c) if covers else None,
                          "max_visible_cover": max(vcovers, key=lambda c: 2 if c == ">=2" else c) if vcovers else None,
                          "exclusion": geo.excl_info, "per_pose": per3},
                 THRESHOLDS["A1.3"], ok3, CLEAR_NOTE + " " + ISECT_NOTE)
    ev.criterion("A1.4", {"per_pose": per4}, THRESHOLDS["A1.4"], ok4,
                 PLANT_NOTE + " Non-contact feet recorded (not judged); contact None = JSON contact missing.")


# ---------------------------------------------------------------- A2
def run_a2(ev, ctx, geo):
    ai = ctx.act_info
    frames = list(range(FRAME_RANGE[0], FRAME_RANGE[1] + 1))
    M = {f: frame_measure(ctx, geo, f, "action") for f in frames}
    # A2.1
    fr = ai["frame_range"]
    fps_ok = ai["fps"] is not None and abs(float(ai["fps"]) - FPS) < 1e-6
    fr_ok = fr[0] == FRAME_RANGE[0] and fr[1] == FRAME_RANGE[1]
    keys_ok = not ai["non_ctrl_paths"] and ai["n_fcurves"] > 0
    match, match_ok = {}, True
    for f in KEY_FRAMES:
        p = ctx.poses.get(f)
        m = M[f]
        if p is None:
            match[f] = "missing in JSON"
            match_ok = False
            continue
        if p["def"]:
            worst, wb = 0.0, None
            for b, S in p["def"].items():
                if b not in m["def_world"]:
                    continue
                d = mat_max_abs(m["def_world"][b], S)
                if d > worst:
                    worst, wb = d, b
            miss_b = [b for b in ctx.def_names if b not in p["def"]]
            r = {"mode": "def_snapshot", "max_abs": sci(worst), "bone": wb, "def_bones_not_in_snapshot": miss_b}
            lok = worst <= POSE_TOL and not miss_b
        else:
            worst, wc = 0.0, None
            for b, ch in p["ctrl"].items():
                c = m["ctrl"].get(b)
                if c is None:
                    continue
                diffs = []
                if "loc" in ch:
                    diffs += [abs(a - b_) for a, b_ in zip(ch["loc"], c["loc"])]
                if "scale" in ch:
                    diffs += [abs(a - b_) for a, b_ in zip(ch["scale"], c["scale"])]
                if "euler" in ch:
                    if c["mode"] == "XYZ":
                        diffs += [abs(a - b_) for a, b_ in zip(ch["euler"], c["euler"])]
                    else:
                        diffs.append(math.radians(rot_diff_deg(Euler(ch["euler"], "XYZ").to_matrix(), c["basis"])))
                if "quat" in ch:
                    diffs.append(math.radians(rot_diff_deg(Quaternion(ch["quat"]).normalized().to_matrix(),
                                                           c["basis"])))
                d = max(diffs) if diffs else 0.0
                if d > worst:
                    worst, wc = d, b
            for k, v in p["props"].items():
                d = abs(v - m["props"].get(k, float("nan")))
                if not d <= worst:
                    worst, wc = d, k
            unl = sorted(n for n, c in m["ctrl"].items() if n not in p["ctrl"]
                         and mat_max_abs(c["basis"], Matrix.Identity(4)) > POSE_TOL)
            r = {"mode": "ctrl_values (no DEF snapshot)", "max_abs": sci(worst), "at": wc,
                 "unlisted_ctrl_non_identity (report)": unl}
            lok = worst <= POSE_TOL
        # report: JSON pose applied in memory vs action DEF
        try:
            apply_json_pose(ctx, p)
            dg = bpy.context.evaluated_depsgraph_get()
            oe = ctx.rig.evaluated_get(dg)
            dj = max(mat_max_abs(oe.matrix_world @ oe.pose.bones[n].matrix, m["def_world"][n]) for n in ctx.def_names)
            r["json_applied_vs_action_def_max_abs (report)"] = sci(dj)
        finally:
            assign_action(ctx)
        match[f] = r
        match_ok = match_ok and lok
    ev.criterion("A2.1", {"fps": ai["fps"], "action_frame_range": fr, "scene_frame_start_end": ai["scene_frame_start_end"],
                          "use_frame_range": ai["use_frame_range"], "n_fcurves": ai["n_fcurves"],
                          "non_ctrl_paths": ai["non_ctrl_paths"][:20], "n_non_ctrl_paths": len(ai["non_ctrl_paths"]),
                          "keyed_bones": ai["keyed_bones"], "keyed_frames": ai["keyed_frames"],
                          "nla_tracks": ai["nla_tracks"], "key_pose_match": match},
                 THRESHOLDS["A2.1"], fps_ok and fr_ok and keys_ok and match_ok,
                 "DEF world = matrix_world @ pose_bone.matrix of the evaluated rig after scene.frame_set(f). "
                 "CTRL mode: listed channels only (euler rad, loc m, scale, props units); unlisted non-identity "
                 "controls reported. json_applied = JSON pose applied in memory (unlisted CTRL identity, PROPS "
                 "contract defaults), report only.")
    # A2.2
    f0, f1 = FRAME_RANGE
    worst, wb, pos, rot = 0.0, None, 0.0, 0.0
    for n in ctx.def_names:
        a, b = M[f1]["def_world"][n], M[f0]["def_world"][n]
        d = mat_max_abs(a, b)
        if d > worst:
            worst, wb = d, n
        pos = max(pos, (a.translation - b.translation).length)
        rot = max(rot, rot_diff_deg(a, b))
    rb, rw = 0.0, 0.0
    r0 = M[f0]["root_world"]
    for f in frames:
        c = M[f]["ctrl"].get("CTRL_root")
        if c is not None:
            rb = max(rb, mat_max_abs(c["basis"], Matrix.Identity(4)))
        if r0 is not None and M[f]["root_world"] is not None:
            rw = max(rw, mat_max_abs(M[f]["root_world"], r0))
    ev.criterion("A2.2", {"def_world_f39_vs_f1_max_abs": sci(worst), "bone": wb, "pos_mm": mm(pos),
                          "rot_deg": rnd(rot, 5), "ctrl_root_basis_vs_identity_max_abs": sci(rb),
                          "ctrl_root_world_vs_f1_max_abs": sci(rw), "ctrl_root_fcurves": ai["root_fcurves"]},
                 THRESHOLDS["A2.2"], worst <= POSE_TOL and rb <= ROOT_TOL and rw <= ROOT_TOL,
                 "max abs element of the 4x4 world matrices over the 24 DEF bones; CTRL_root basis every frame.")
    # A2.3
    per, fails, wc, wcov = {}, [], None, 0
    for f in frames:
        c, i = M[f]["clear"], M[f]["isect"]
        per[f] = {"clearance_mm": c["clearance_mm"], "part": (c["at"] or {}).get("part"),
                  "bone": (c["at"] or {}).get("bone"), "club_s_mm": (c["at"] or {}).get("club_s_mm"),
                  "penetration": c["penetration"], "total_pairs": i["total_pairs"], "total_cover": i["total_cover"],
                  "visible_pairs": i["visible_pairs"], "visible_cover": i["visible_cover"],
                  "hidden_pairs_by_part": i["hidden_pairs_by_part"],
                  "body_pairs": i["n_body_pairs"], "rigid_pairs": i["n_rigid_pairs"]}
        if c["_clear"] < CLEAR_MIN or i["visible_cover"] not in (0, 1):
            fails.append(f)
        if wc is None or c["_clear"] < wc[0]:
            wc = (c["_clear"], f, c["at"])
        cv = 2 if i["visible_cover"] == ">=2" else i["visible_cover"]
        wcov = max(wcov, cv)
    ev.criterion("A2.3", {"min_clearance_mm": mm(wc[0]), "min_at_frame": wc[1], "min_at": wc[2],
                          "max_visible_cover": ">=2" if wcov == 2 else wcov, "failing_frames": fails,
                          "exclusion": geo.excl_info, "per_frame": per},
                 THRESHOLDS["A2.3"], not fails, CLEAR_NOTE + " " + ISECT_NOTE)
    # A2.4
    flips, trange = [], {}
    for k in ctx.chains:
        ths = [M[f]["theta"].get(k) for f in frames]
        vals = [t for t in ths if t is not None]
        trange[k] = [rnd(min(vals), 3), rnd(max(vals), 3)] if vals else None
        for a, b, fa in zip(ths[:-1], ths[1:], frames[:-1]):
            if a is None or b is None:
                continue
            if (a > 0) != (b > 0) and abs(a) > FLIP_MIN_DEG and abs(b) > FLIP_MIN_DEG:
                flips.append({"chain": k, "frames": [fa, fa + 1], "theta_deg": [rnd(a, 3), rnd(b, 3)]})
    sec = {}
    for f in frames[1:]:
        name = next((s for s, a, b in SECTIONS if a <= f <= b), "?")
        mx, bone = 0.0, None
        for n in ctx.def_names:
            d = rot_diff_deg(M[f - 1]["def_world"][n], M[f]["def_world"][n])
            if d > mx:
                mx, bone = d, n
        s = sec.setdefault(name, {"max_deg": 0.0, "bone": None, "frames": None})
        if mx >= s["max_deg"]:
            s.update(max_deg=rnd(mx, 3), bone=bone, frames=[f - 1, f])
    ev.criterion("A2.4", {"n_flips": len(flips), "flips": flips[:30], "theta_deg_min_max": trange,
                          "chains": {k: v["def"][:2] for k, v in ctx.chains.items()},
                          "sections": {s: [a, b] for s, a, b in SECTIONS}, "max_adjacent_rot_per_section": sec},
                 THRESHOLDS["A2.4"], not flips,
                 "G6.5b: theta = atan2(dot(cross(u, l), X_upper), dot(u, l)) of the DEF chain (upper, lower); flip = "
                 "sign change between adjacent frames with |theta| > 2 deg on both. Adjacent change = max over the 24 "
                 "DEF bones of the world rotation angle (float64 matrices, f-1 -> f), pair assigned to the section of f (d-25 section 1).")
    # A2.5
    rng = []
    ok5 = bool(ctx.ranges)
    for r in ctx.ranges:
        bn = f"foot_{r['foot']}"
        h0, t0 = M[r["a"]]["def_head_tail"][bn]
        mx, at = 0.0, r["a"]
        for f in range(r["a"], r["b"] + 1):
            h, t = M[f]["def_head_tail"][bn]
            d = max((h - h0).length, (t - t0).length)
            if d > mx:
                mx, at = d, f
        rng.append({"foot": r["foot"], "frames": [r["a"], r["b"]], "max_move_mm": mm(mx), "at_frame": at})
        ok5 = ok5 and mx <= FOOT_MOVE
    ev.criterion("A2.5", {"ranges": rng, "n_ranges": len(rng)}, THRESHOLDS["A2.5"], ok5,
                 "DEF foot_x world head and tail displacement from the range's first frame, max over the range. "
                 "No contact_ranges -> ok False.")


# ---------------------------------------------------------------- main
def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {"gate": None, "keyposes": None, "debug_out": None}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("--gate", "--keyposes", "--debug-out") and i + 1 < len(argv):
            out[a[2:].replace("-", "_")] = argv[i + 1]
            i += 2
            continue
        raise SystemExit(f"{CHECKER}: unexpected argument {a!r}")
    if out["gate"] is not None:
        out["gate"] = out["gate"].upper()
        if out["gate"] not in ("A1", "A2"):
            raise SystemExit(f"{CHECKER}: --gate must be A1 or A2")
    return out


def write(ev, debug_out):
    if not debug_out:
        return ev.write()
    d = Path(debug_out)
    d.mkdir(parents=True, exist_ok=True)
    p = d / f"{CHECKER}_debug.json"
    doc = {"gate": ev.gate, "checker": ev.checker, "inputs": ev.inputs, "criteria": ev.criteria,
           "renders": ev.renders}
    with open(p, "w", encoding="utf-8") as f:
        json.dump(goblib._jsonable(doc), f, indent=1, ensure_ascii=False)
    for c in ev.criteria:
        print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']}")
    print(f"[{GATE}/{CHECKER}] wrote {p}")
    return p


def main():
    args = parse_args()
    ctx = Ctx()
    kp_path = Path(args["keyposes"]).resolve() if args["keyposes"] else KEYPOSES_JSON
    ctx.kp_path = kp_path
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(os.path.abspath(__file__))
    if bpy.data.filepath and Path(bpy.data.filepath).exists():
        ev.add_input(bpy.data.filepath)
    for p in (kp_path, CANON_JSON, MANIFEST_JSON, PARTS_JSON):
        if Path(p).exists():
            ev.add_input(p)
    ev.add_stage_inputs("s10")
    if bpy.data.filepath and Path(bpy.data.filepath).resolve() != BLEND.resolve():
        ctx.notes.append(f"blend is {rel(bpy.data.filepath)}, not {rel(BLEND)}")

    gates = [args["gate"]] if args["gate"] else ["A1", "A2"]
    ids = {"A1": ["A1.1", "A1.2", "A1.3", "A1.4"], "A2": ["A2.1", "A2.2", "A2.3", "A2.4", "A2.5"]}
    load_inputs(ctx, kp_path)
    if ctx.miss:
        for g in gates:
            blocked(ev, ids[g], "; ".join(ctx.miss))
        write(ev, args["debug_out"])
        print(f"[{GATE}/{CHECKER}] " + "; ".join(ctx.miss))
        sys.stdout.flush()
        sys.exit(2)

    save_state(ctx)
    try:
        prepare(ctx)
        ctx.chains = ik_chains(ctx)
        per_pose_issues = parse_keyposes(ctx)
        geo = Geo(ctx)
        geo.shoe_v = {x: geo.topo.verts_of_polys(geo.topo.polys_of(f"shoe_{x}")) for x in SIDES}
        if ctx.act is not None:
            slot = assign_action(ctx)
            ctx.act_info = action_info(ctx)
            ctx.act_info["slot"] = slot
        else:
            ctx.act_info = {"action": None}
        source = "action" if ctx.act is not None else "json"
        keyed = set(ctx.act_info.get("keyed_frames") or [])
        a2_block = None
        if ctx.act is None:
            a2_block = f"action {ACTION} missing in {rel(bpy.data.filepath)}"
        elif args["gate"] is None and keyed and keyed <= set(KEY_FRAMES):
            a2_block = (f"action keys only at key-pose frames {sorted(keyed)} (A1 stage); "
                        f"run -- --gate A2 to force")
        if "A1" in gates:
            run_a1(ev, ctx, geo, per_pose_issues, source)
        if "A2" in gates:
            if a2_block:
                blocked(ev, ids["A2"], a2_block)
            else:
                run_a2(ev, ctx, geo)
        info = {"action": {k: v for k, v in ctx.act_info.items() if k != "keyed_bones"},
                "pose_source_A1": source, "notes": ctx.notes, "keyposes_json": rel(kp_path)}
        ev.criteria.insert(0, {"id": "info", "measured": goblib._jsonable(info), "threshold": None, "ok": True,
                               "note": "report only (run context)"})
    finally:
        restore_state(ctx)
    write(ev, args["debug_out"])


if __name__ == "__main__":
    goblib.run_main(main)
