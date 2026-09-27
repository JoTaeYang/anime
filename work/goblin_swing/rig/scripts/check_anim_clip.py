"""check_anim_clip - clip-agnostic animation checker (key poses + animation), metric code from check_a_swing.

Common criteria (C-ids), emitted under the clip's gate ids (gate_id mapping, see below):
  C1  key poses present in the clip JSON (name, contact, CTRL values; optional required key_frames)
  C2  safe-range report per key pose (d-25 section 2 limits)
  C3  per key pose: club clearance >= 10 mm, visible self-intersection cover <= 1 (d-25 A1.3 definition)
  C4  per key pose: contact feet planted (reach <= 1 mm, lowest shoe z in [-1, +3] mm)
  C5  action: fps, frame range, keys on CTRL_*/PROPS only; key poses match (DEF snapshot, else CTRL values) <= 1e-4
  C6  loop: DEF world at the last frame = first frame (<= 1e-4)
  C7  CTRL_root motion 0 over the clip
  C8  every frame: clearance >= 10 mm, visible cover <= 1
  C9  every frame: branch flip 0 (G6.5b) + per-section max adjacent DEF rotation (report)
  C10 contact_ranges: contact foot DEF world movement <= 1 mm.  A range with "mode": "pivot" (T132, d-29 R3 revision:
      toes roll about the heel / heel_twist splay with the heel planted) is judged with the G6.7 planted-roll rule
      instead (check_g6_ctrl c_foot, same constants): per frame the contact set = shoe_x vertices (evaluated GOB_mesh,
      part_id shoe_x) with z <= that frame's lowest shoe z + 1 mm; slip S += mean XY displacement of the contact set of
      frame k from k to k+1 over the range; S <= 5 mm and the lowest shoe z in [-1, +3] mm on every frame of the range.
      Every range reports mode, S and the lowest shoe z min/max; a pivot range reports the foot DEF movement only.
  C11 (only when the JSON gate_id maps C11) ground contact: every frame the lowest evaluated GOB_mesh and
      GOB_club vertex z >= -P mm (penetration); in JSON ground_rest_ranges the lowest body z <= +R mm (resting);
      P / R from JSON ground, a missing value = calibration run (measured only, component ok True); phase 1
      adds a per_pose block (key poses) to the same row (spec d-28 section 4)
  C12 (report only, 2026-09-26 HR2) per frame: PROPS mouth_open and the evaluated GOB_mesh mouth_open shape-key
      value (None when absent); emitted under the id the JSON gate_id maps C12 to, else "C12"
  M1  (optional, JSON match_first_frame) DEF world at the first frame = reference clip frame (<= 1e-4)
  M2  (optional, JSON targets) motion amplitude relative to the first frame (report)
Phases: 1 = C1..C4 (+ C11 per_pose) (key poses), 2 = C5..C11, M1, M2 (animation).

Arguments (after --):
  --action <name>      action to check (required)
  --json <path>        clip JSON (required), e.g. rig/data/clips/idle.json or rig/data/swing_keyposes.json
  --gate I1|I2|A1|A2|1|2|both   phase 1, phase 2 or both (default both; with both, phase 2 is blocked
                       automatically when the action keys only the key-pose frames)
  --out <path>         evidence path (default rig/inspect/<json stem>/check_anim_clip.json)
  --blend <path>       input blend (opened when it differs from the loaded one; bl.ps1 -Blend also works)
  --stage <sNN>        production scripts hashed = rig/scripts/sNN*.py up to NN (default s99 = all)
  Relative paths: absolute, else existing relative to cwd, else 'rig/...' relative to work/goblin_swing,
  else relative to rig/.

Clip JSON (swing_keyposes.json schema, read tolerantly):
  poses: list (or dict name/frame -> pose) of
    {frame: int, name: str, contact: ["l","r"] | {"l": bool, "r": bool} | "l"/"r"/"both"/"none",
     ctrl|ctrls|controls: {CTRL_bone: {loc|location: [3] m, rot|rot_deg|rotation_euler_deg: [3] deg (XYZ),
                                       rotation_euler: [3] rad, rotation_quaternion|quat: [4] wxyz, scale: [3]}},
     props: {PROPS key: value}  (a "PROPS" entry inside ctrl is read as props),
     def (optional snapshot): {DEF bone: 4x4 world matrix (row-major) | {"matrix": 4x4}}}
  contact_ranges: list of {foot|side: "l"/"r", frames|range: [a, b]} or {start, end}; or {"l": [[a, b], ...]}.
  An object entry may add "mode": "pivot" (C10 planted-roll rule); no mode = the 1 mm foot DEF rule.
  optional: fps (default 24), frame_range [a, b] (default: action frame range), key_frames [..] (required
  key-pose frames; default = the pose frames), sections [{name, frames: [a, b]}] (C9 report; default =
  preset sections or one section "all"),
  match_first_frame {blend, action, frame} (M1; reference rig + action appended in memory from that blend),
  targets {head_rise_mm: [lo, hi], fist_rise_mm: [lo, hi], fist_out_mm: [lo, hi], club_tip_move_mm_max: x}
  (M2 report), ground {penetration_mm: P, rest_max_mm: R} (C11 thresholds, each optional),
  ground_rest_ranges [[a, b], ...] (C11 resting frames, inclusive),
  gate_id: preset name "A" | "I" or {C-id: id | {id, report: bool}} (default: preset "A" when
  the action is attack_swing, else "I").  report: true = the component's ok does not enter the row's ok.
  Rows sharing one gate id are merged: measured / threshold = {C-id: ...}, ok = all non-report components.

Pose source (phase 1): the action evaluated with scene.frame_set(f); without the action, the JSON CTRL values
applied in memory (action detached, CTRL_* / PROPS identity, PROPS contract defaults, then the listed values).
Everything is changed in memory only and restored at the end; nothing is saved.

Run:    bl.ps1 -Script check_anim_clip.py -Blend <anim.blend> -- --action <clip> --json rig/data/clips/<clip>.json
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

GATE = "clip"            # evidence "gate" field = clip name (set in main)
CHECKER = "check_anim_clip"
CANON_JSON = goblib.DATA / "canonical_skeleton.json"
MANIFEST_JSON = goblib.DATA / "ctrl_manifest.json"
PARTS_JSON = goblib.DATA / "parts.json"

ARM = "GOB_rig"
MESH = "GOB_mesh"
CLUB = "GOB_club"
PROPS = "PROPS"
ACTION = None            # set in main (--action)
SIDES = ("l", "r")

KEY_FRAMES = ()                           # C1 (set in main from the clip JSON)
FRAME_RANGE = (1, 39)                     # C5 (set in main from the clip JSON / action)
FPS = 24.0                                # C5 (clip JSON fps)
POSE_TOL = 1e-4                           # A2.1 / A2.2 max abs matrix element (DEF) or channel value
ROOT_TOL = 1e-7                           # A2.2 CTRL_root motion "0" (float noise only)
CLEAR_MIN = 0.010                         # A1.3 / A2.3 club clearance (m)
COVER_MAX = 1                             # A1.3 / A2.3
PLANT_REACH = 0.001                       # A1.4 (m), = G6.7
PLANT_Z = (-0.001, 0.003)                 # A1.4 lowest shoe z (m), = G6.7
FOOT_MOVE = 0.001                         # A2.5 (m)
PIVOT_SLIP = 0.005                        # C10 pivot range: cumulative contact slip (m), = G6.7 PLANT_SLIP
CONTACT_TOL = 0.001                       # C10 pivot range: contact set z <= lowest z + this (m), = G6.7
FLIP_MIN_DEG = 2.0                        # A2.4 = G6.5b
FOREARM_BONES = ("lowerarm_r", "lowerarm_twist_r", "hand_r")
FOREARM_W = 0.5                           # face excluded when mean vertex weight on FOREARM_BONES > 0.5
CLUB_EXCLUDED_PARTS = ("hand_r",)
SHELL_PARTS = ("body", "head", "hand_l", "shoe_l", "shoe_r", "belt")   # inside tests (hand_r excluded)
RIGID_PARTS = ("head", "hand_l", "hand_r", "shoe_l", "shoe_r", "belt")
SECTIONS = ()             # C9 (set in main)
SWING_SECTIONS = (("idle", 1, 5), ("windup", 6, 12), ("swing", 13, 15), ("hold", 16, 23),
                  ("recovery", 24, 34), ("settle", 35, 39))   # d-25 section 1
PRESETS = {
    "A": {"ids": {"C1": "A1.1", "C2": "A1.2", "C3": "A1.3", "C4": "A1.4", "C5": "A2.1", "C6": "A2.2",
                  "C7": "A2.2", "C8": "A2.3", "C9": "A2.4", "C10": "A2.5", "M1": "A2.2", "M2": "A2.amp"},
          "report": {"M2", "C12"}, "sections": SWING_SECTIONS},
    "I": {"ids": {"C1": "I1.1", "C2": "I1.2", "C3": "I1.2", "C4": "I1.2", "C5": "I2.1", "C6": "I2.2",
                  "C7": "I2.2", "M1": "I2.2", "C8": "I2.3", "C9": "I2.3", "C10": "I2.3", "M2": "I2.4"},
          "report": {"C2", "M2", "C12"}, "sections": None},
}
PHASE1 = ("C1", "C2", "C3", "C4")
PHASE2 = ("C5", "C6", "C7", "C8", "C9", "C10", "C11", "C12", "M1", "M2")
MOUTH_PROP = "mouth_open"   # C12 report (hr2_contract.md)
MOUTH_KEY = "mouth_open"
GROUND = {"penetration_mm": None, "rest_max_mm": None, "rest": [], "issues": []}   # C11 (set in main)
GROUND_CAL = "calibration run: threshold not set (spec d-28 \u00a74, measure first)"
GROUND_NOTE = ("Ground z = 0. body = lowest world z over the evaluated GOB_mesh vertices (the clearance / "
               "self-intersection array), part = part_id of a face using that vertex, bone = its dominant weight "
               "vertex group; club = lowest world z over the evaluated GOB_club vertices (club_s_mm = club local "
               "Y). Penetration: min(body, club) >= -P mm every frame; resting: body <= +R mm in "
               "ground_rest_ranges.")
CONTRACT_DEFAULTS = {   # = check_g6_ctrl (plan contract)
    "arm_ik_fk_l": 0.0, "arm_ik_fk_r": 0.0, "leg_ik_fk_l": 1.0, "leg_ik_fk_r": 1.0,
    "head_space": 0, "hand_ik_space_l": 0, "hand_ik_space_r": 0, "weapon_space": 0,
    "knee_pole_space_l": 0, "knee_pole_space_r": 0,
    "foot_roll_l": 0.0, "foot_roll_r": 0.0, "foot_bank_l": 0.0, "foot_bank_r": 0.0,
    "heel_twist_l": 0.0, "heel_twist_r": 0.0, "toe_twist_l": 0.0, "toe_twist_r": 0.0,
    "mouth_open": 0.0,   # 2026-09-26 HR2 (hr2_contract.md)
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
        mode = r.get("mode") if isinstance(r, dict) else None
        if mode is not None and str(mode).strip().lower() != "pivot":
            issues.append(f"contact range {x} [{a}, {b}]: unknown mode {mode!r} (only 'pivot')")
            continue
        out.append({"foot": x, "a": a, "b": b, "mode": "pivot" if mode is not None else None})
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
        self.kp_path = None
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
        self.c11 = False         # C11 mapped in the JSON gate_id
        self.c11_pose = None     # C11 phase-1 per_pose block


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
        # M2: club tip = club vertices within 1 mm of the max local Y (club head end); left fist verts
        self.club_tip_idx = np.nonzero(self.club_s >= self.club_s.max() - 0.001)[0]
        self.hand_l_v = t.verts_of_polys(t.polys_of("hand_l"))
        self.head_v = t.verts_of_polys(t.polys_of("head"))   # M2 eye point candidates

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
    r["mouth"] = {"prop": props.get(MOUTH_PROP), "key": mouth_key_value(ctx, dg)}
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
        r["amp"] = {"club_tip": C[geo.club_tip_idx].mean(axis=0), "club_grip": np.array(cmw.translation),
                    "hand_l_center": V[geo.hand_l_v].mean(axis=0) if len(geo.hand_l_v) else None,
                    "head_v": V[geo.head_v].copy() if len(geo.head_v) else None}
        bi, ci = int(np.argmin(V[:, 2])), int(np.argmin(C[:, 2]))
        r["ground"] = {"body_z": float(V[bi, 2]), "body_vi": bi, "club_z": float(C[ci, 2]), "club_vi": ci}
        for x in SIDES:
            sv = geo.shoe_v[x]
            pl[x]["shoe_min_z"] = float(V[sv, 2].min()) if len(sv) else None
            pl[x]["shoe_w"] = V[sv].copy() if len(sv) else None      # C10 pivot ranges (T132)
    r["planted"] = pl
    return r


def mouth_key_value(ctx, dg):
    """C12 report: evaluated GOB_mesh shape-key value mouth_open (None when the mesh has no such key)."""
    key = ctx.mesh.data.shape_keys if ctx.mesh is not None else None
    if key is None or key.key_blocks.get(MOUTH_KEY) is None:
        return None
    try:
        return float(key.evaluated_get(dg).key_blocks[MOUTH_KEY].value)
    except Exception:
        return float(key.key_blocks[MOUTH_KEY].value)


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
def thresholds():
    return {
        "C1": f"key poses at frames {list(KEY_FRAMES)} in the clip JSON, each with name, contact and CTRL values "
              f"(existing CTRL_*/PROPS bones, parseable channels, props on {PROPS}); contact_ranges parseable in "
              f"{FRAME_RANGE[0]}..{FRAME_RANGE[1]}",
        "C2": "report: d-25 section 2 draft limits " + json.dumps(SAFE_LIMITS),
        "C3": {"clearance_mm": ">= 10", "visible_isect_cover": "<= 1 (pairs hidden inside a rigid shell excluded; "
                                                               "totals report)"},
        "C4": {"reach_mm": "<= 1", "shoe_min_z_mm": "[-1, +3]", "feet": "JSON contact feet"},
        "C5": {"fps": FPS, "frames": f"{FRAME_RANGE[0]}..{FRAME_RANGE[1]}", "keys": "CTRL_*/PROPS only",
               "key_pose_match": "DEF snapshot max abs <= 1e-4, else CTRL/PROPS channel |diff| <= 1e-4"},
        "C6": {"def_world_last_vs_first_max_abs": "<= 1e-4"},
        "C7": {"ctrl_root_motion": "0 (<= 1e-7 float)"},
        "C8": {"clearance_mm": ">= 10 every frame", "visible_isect_cover": "<= 1 every frame (C3 definition)"},
        "C9": {"branch_flips": 0},
        "C10": {"contact_foot_def_move_mm": "<= 1 (ranges without mode)",
                "pivot_ranges": f"cumulative contact slip <= {PIVOT_SLIP * 1000:g} mm and lowest shoe z in "
                                f"[{PLANT_Z[0] * 1000:g}, {PLANT_Z[1] * 1000:g}] mm every frame (G6.7 rule)"},
        "C11": {"body_and_club_min_z_mm": (f">= -{GROUND['penetration_mm']} every frame"
                                           if GROUND["penetration_mm"] is not None else GROUND_CAL),
                "rest_body_min_z_mm": (f"<= +{GROUND['rest_max_mm']} in ground_rest_ranges "
                                       f"{[[a, b] for a, b in GROUND['rest']]}"
                                       if GROUND["rest_max_mm"] is not None else GROUND_CAL)},
        "C12": "report only: per frame PROPS mouth_open and evaluated GOB_mesh mouth_open shape-key value",
        "M1": {"def_world_first_vs_reference_max_abs": "<= 1e-4"},
        "M2": "report: JSON targets (amplitude relative to the first frame)",
    }


class Rows:
    """C-level results, merged into gate-id rows by emit()."""

    def __init__(self):
        self.c = {}

    def add(self, cid, measured, ok, note=""):
        self.c[cid] = {"measured": measured, "ok": bool(ok), "note": note}


def blocked(rows, cids, reason):
    for c in cids:
        rows.add(c, f"blocked: {reason}", False, "")


def resolve_ids(kp, action):
    g = kp.get("gate_id") if isinstance(kp, dict) else None
    if isinstance(g, dict):
        ids, rep = {}, set()
        for k, v in g.items():
            if isinstance(v, dict):
                ids[k] = str(v.get("id", k))
                if v.get("report"):
                    rep.add(k)
            else:
                ids[k] = str(v)
        return {"source": "clip JSON gate_id mapping", "ids": ids, "report": rep, "sections": None}
    name = g if isinstance(g, str) and g in PRESETS else ("A" if action == "attack_swing" else "I")
    pr = PRESETS[name]
    return {"source": f"preset {name}" + (" (clip JSON gate_id)" if g == name else " (default)"),
            "ids": dict(pr["ids"]), "report": set(pr["report"]), "sections": pr["sections"]}


def emit(ev, rows, idmap):
    th = thresholds()
    groups = {}
    for cid in PHASE1 + PHASE2:
        if cid not in rows.c or (cid == "C11" and cid not in idmap["ids"]):
            continue
        gid = idmap["ids"].get(cid, cid)
        groups.setdefault(gid, []).append(cid)
    for gid, cids in groups.items():
        comp_ok = {c: rows.c[c]["ok"] for c in cids}
        judged = [c for c in cids if c not in idmap["report"]]
        ok = all(comp_ok[c] for c in judged) if judged else all(comp_ok.values())
        tags = ", ".join(f"{c}{' (report)' if c in idmap['report'] else ''}" for c in cids)
        if len(cids) == 1:
            c = cids[0]
            ev.criterion(gid, rows.c[c]["measured"], th[c], ok,
                         f"[components: {tags}; component ok {comp_ok}] " + rows.c[c]["note"])
        else:
            meas = {c: rows.c[c]["measured"] for c in cids}
            meas["_component_ok"] = comp_ok
            ev.criterion(gid, meas, {c: th[c] for c in cids}, ok,
                         f"[components: {tags}; ok = all non-report components] "
                         + " | ".join(f"{c}: {rows.c[c]['note']}" for c in cids if rows.c[c]["note"]))


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
              f"at={m['clear']['at'].get('part') if m['clear']['at'] else None} "
              f"cover={m['isect']['total_cover']} visible_cover={m['isect']['visible_cover']}")
        sys.stdout.flush()
    return ctx.cache[key]


def ground_entry(geo, m, f):
    """C11 values of one measured frame -> (entry, in_rest, penetration ok, rest ok); ok None = not judged."""
    g = m["ground"]
    bi = g["body_vi"]
    e = {"body_z_mm": mm(g["body_z"]), "part": geo.part_name(geo.vpart[bi]), "bone": geo.vbone[bi],
         "club_z_mm": mm(g["club_z"]), "club_s_mm": mm(geo.club_s[g["club_vi"]])}
    in_rest = any(a <= f <= b for a, b in GROUND["rest"])
    P, R = GROUND["penetration_mm"], GROUND["rest_max_mm"]
    pen_ok = None if P is None else min(g["body_z"], g["club_z"]) * 1000.0 >= -P
    rest_ok = None if (R is None or not in_rest) else g["body_z"] * 1000.0 <= R
    return e, in_rest, pen_ok, rest_ok


def ground_cfg_ok():
    """Resting judged (R set) needs parseable, non-empty ground_rest_ranges."""
    return GROUND["rest_max_mm"] is None or (bool(GROUND["rest"]) and not GROUND["issues"])


def ground_notes():
    n = [GROUND_NOTE]
    if GROUND["penetration_mm"] is None:
        n.append("penetration: " + GROUND_CAL)
    if GROUND["rest_max_mm"] is None:
        n.append("resting: " + GROUND_CAL)
    return " ".join(n)


# ---------------------------------------------------------------- phase 1 (C1 .. C4)
def run_phase1(rows, ctx, geo, per_pose_issues, source, required_given):
    frames = sorted(ctx.poses)
    missing = [f for f in KEY_FRAMES if f not in ctx.poses]
    extra = [f for f in frames if f not in KEY_FRAMES]
    outside = [f for f in frames if not (FRAME_RANGE[0] <= f <= FRAME_RANGE[1])]
    bad = {f: v for f, v in per_pose_issues.items() if v}
    ai = ctx.act_info
    keyed = set(ai.get("keyed_frames") or [])
    meas = {"frames_in_json": frames, "required_key_frames": list(KEY_FRAMES),
            "required_from": "clip JSON key_frames" if required_given else "pose frames",
            "missing_frames": missing, "extra_frames": extra, "frames_outside_range": outside,
            "n_poses": len(ctx.poses), "per_pose_issues": bad, "file_issues": ctx.kp_issues,
            "poses": {f: {"name": p["name"], "contact": sorted(p["contact"]) if p["contact"] is not None else None,
                          "n_ctrl": p["n_ctrl"], "n_props": len(p["props"]), "def_snapshot": p["def"] is not None}
                      for f, p in sorted(ctx.poses.items())},
            "contact_ranges": ctx.ranges,
            "action_keys_at_key_frames (report)": {f: f in keyed for f in KEY_FRAMES} if ctx.act else None,
            "pose_source": source}
    ok = bool(ctx.poses) and not missing and not extra and not outside and not bad and not ctx.kp_issues
    rows.add("C1", meas, ok, "JSON schema read tolerantly (see checker docstring); rot = degrees Euler XYZ.")
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
        ok3 = ok3 and c["_clear"] >= CLEAR_MIN and i["visible_cover"] in (0, 1)
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
    rows.add("C2", {"per_pose": per2, "out_of_range": outs, "n_out_of_range": len(outs)}, not outs, SAFE_NOTE)
    covers = [v["isect"]["total_cover"] for v in per3.values() if isinstance(v, dict)]
    vcovers = [v["isect"]["visible_cover"] for v in per3.values() if isinstance(v, dict)]
    rows.add("C3", {"min_clearance_mm": mm(worst[0]) if worst else None,
                    "min_at_frame": worst[1] if worst else None,
                    "max_total_cover": max(covers, key=lambda c: 2 if c == ">=2" else c) if covers else None,
                    "max_visible_cover": max(vcovers, key=lambda c: 2 if c == ">=2" else c) if vcovers else None,
                    "exclusion": geo.excl_info, "per_pose": per3},
             ok3, CLEAR_NOTE + " " + ISECT_NOTE)
    rows.add("C4", {"per_pose": per4}, ok4,
             PLANT_NOTE + " Non-contact feet recorded (not judged); contact None = JSON contact missing.")
    if ctx.c11:   # C11 per_pose block (cached key-pose measurements, no extra evaluation)
        per11, ok11 = {}, True
        for f in KEY_FRAMES:
            if source == "json" and f not in ctx.poses:
                per11[f] = "blocked: pose missing in JSON and no action"
                ok11 = False
                continue
            e, in_rest, pen_ok, rest_ok = ground_entry(geo, frame_measure(ctx, geo, f, source), f)
            per11[f] = {"name": ctx.poses.get(f, {}).get("name"), **e, "in_rest_range": in_rest,
                        "penetration_ok": pen_ok, "rest_ok": rest_ok}
            ok11 = ok11 and pen_ok is not False and rest_ok is not False
        ctx.c11_pose = per11
        rows.add("C11", {"per_pose": per11, "range_issues": GROUND["issues"]}, ok11 and ground_cfg_ok(),
                 ground_notes())


# ---------------------------------------------------------------- phase 2 (C5 .. C10, M1, M2)
def match_first(ctx, cfg, M, ev):
    """M1: DEF world of this clip's first frame vs a reference clip frame (reference GOB_rig + action appended
    in memory from the reference blend, evaluated, removed again)."""
    f0 = FRAME_RANGE[0]
    try:
        ref_path = resolve_path(cfg["blend"], base=goblib.RIG)
        ref_action, ref_frame = str(cfg["action"]), int(cfg.get("frame", 1))
    except (KeyError, TypeError, ValueError) as e:
        return {"blocked": f"match_first_frame config invalid: {e}"}, False
    if not ref_path.exists():
        return {"blocked": f"missing input: {rel(ref_path)}"}, False
    ev.add_input(ref_path)
    with bpy.data.libraries.load(str(ref_path), link=False) as (src, dst):
        found = ARM in src.objects and ref_action in src.actions
        if found:
            dst.objects = [ARM]
            dst.actions = [ref_action]
    if not found:
        return {"blocked": f"{ARM} or action {ref_action} missing in {rel(ref_path)}"}, False
    ref, ract = dst.objects[0], dst.actions[0]
    sc = bpy.context.scene
    try:
        sc.collection.objects.link(ref)
        ad = ref.animation_data or ref.animation_data_create()
        ad.action = ract
        if hasattr(ad, "action_slot") and ad.action_slot is None:
            from bpy_extras import anim_utils
            slot = anim_utils.action_get_first_suitable_slot(ract, "OBJECT")
            if slot is not None:
                ad.action_slot = slot
        sc.frame_set(ref_frame)
        ref.update_tag()
        bpy.context.view_layer.update()
        dg = bpy.context.evaluated_depsgraph_get()
        re_ = ref.evaluated_get(dg)
        worst, wb, pos, rot, lost = 0.0, None, 0.0, 0.0, []
        for n in ctx.def_names:
            if n not in re_.pose.bones:
                lost.append(n)
                continue
            a = re_.matrix_world @ re_.pose.bones[n].matrix
            b = M[f0]["def_world"][n]
            d = mat_max_abs(a, b)
            if d > worst:
                worst, wb = d, n
            pos = max(pos, (a.translation - b.translation).length)
            rot = max(rot, rot_diff_deg(a, b))
        meas = {"reference": {"blend": rel(ref_path), "action": ref_action, "frame": ref_frame,
                              "appended_as": [ref.name, ract.name]},
                "clip_frame": f0, "def_world_max_abs": sci(worst), "bone": wb, "pos_mm": mm(pos),
                "rot_deg": rnd(rot, 5), "def_bones_missing_in_reference": lost}
        return meas, worst <= POSE_TOL and not lost
    finally:
        arm_data = ref.data
        bpy.data.objects.remove(ref, do_unlink=True)
        if arm_data is not None and arm_data.users == 0:
            bpy.data.armatures.remove(arm_data)
        if ract.users == 0:
            bpy.data.actions.remove(ract)
        goto_frame(ctx, f0)


def amplitude(ctx, M, frames, targets):
    """M2: world displacement relative to the first frame (report)."""
    f0 = frames[0]
    hl = ctx.rig.data.bones["hand_l"].length

    def tail(mw):
        return mw @ Vector((0.0, hl, 0.0))

    head0 = M[f0]["def_world"]["head"].translation
    ht0 = tail(M[f0]["def_world"]["hand_l"])
    a0 = M[f0]["amp"]
    # eye point: head-part vertices selected once at the first frame (fixed index set), mean world position
    eye_sel, eye_info = None, None
    H0 = a0.get("head_v")
    if H0 is not None and len(H0):
        ymin, ymax = float(H0[:, 1].min()), float(H0[:, 1].max())
        zmin, zmax = float(H0[:, 2].min()), float(H0[:, 2].max())
        y_lim = ymin + EYE_FRONT_FRAC * (ymax - ymin)
        z_lo, z_hi = zmin + EYE_Z_BAND[0] * (zmax - zmin), zmin + EYE_Z_BAND[1] * (zmax - zmin)
        eye_sel = np.nonzero((H0[:, 1] <= y_lim) & (H0[:, 2] >= z_lo) & (H0[:, 2] <= z_hi))[0]
        eye0 = H0[eye_sel].mean(axis=0) if len(eye_sel) else None
        eye_info = {"rule": (f"head part vertices at frame {f0} (world) with y <= ymin + {EYE_FRONT_FRAC} * (ymax - "
                             f"ymin) (front -Y quarter) and z in [zmin + {EYE_Z_BAND[0]} h, zmin + {EYE_Z_BAND[1]} h] "
                             f"(h = head z extent); the same vertex indices are followed in every frame"),
                    "n_verts": int(len(eye_sel)), "n_head_verts": int(len(H0)),
                    "head_bbox_first_frame_mm": {"y": [mm(ymin), mm(ymax)], "z": [mm(zmin), mm(zmax)]},
                    "band_mm": {"y_max": mm(y_lim), "z": [mm(z_lo), mm(z_hi)]},
                    "eye_point_first_frame_mm": [mm(x) for x in eye0] if eye0 is not None else None,
                    "x_range_mm": [mm(H0[eye_sel, 0].min()), mm(H0[eye_sel, 0].max())] if len(eye_sel) else None}
        if not len(eye_sel):
            eye_sel = None
    series = {"head_rise_mm": [], "head_bone_rise_mm": [], "fist_tail_rise_mm": [], "fist_tail_out_mm": [], "fist_center_rise_mm": [],
              "fist_center_out_mm": [], "club_tip_move_mm": [], "club_grip_move_mm": []}
    for f in frames:
        m = M[f]
        if eye_sel is not None:
            series["head_rise_mm"].append(mm(m["amp"]["head_v"][eye_sel, 2].mean() - H0[eye_sel, 2].mean()))
        series["head_bone_rise_mm"].append(mm(m["def_world"]["head"].translation.z - head0.z))
        ht = tail(m["def_world"]["hand_l"])
        series["fist_tail_rise_mm"].append(mm(ht.z - ht0.z))
        series["fist_tail_out_mm"].append(mm(ht.x - ht0.x))
        if a0["hand_l_center"] is not None:
            series["fist_center_rise_mm"].append(mm(m["amp"]["hand_l_center"][2] - a0["hand_l_center"][2]))
            series["fist_center_out_mm"].append(mm(m["amp"]["hand_l_center"][0] - a0["hand_l_center"][0]))
        series["club_tip_move_mm"].append(mm(np.linalg.norm(m["amp"]["club_tip"] - a0["club_tip"])))
        series["club_grip_move_mm"].append(mm(np.linalg.norm(m["amp"]["club_grip"] - a0["club_grip"])))

    def mx(k):
        v = [x for x in series[k] if isinstance(x, (int, float))]
        if not v:
            return None
        i = max(range(len(v)), key=lambda j: v[j])
        return {"max": v[i], "at_frame": frames[i], "min": min(v)}

    summary = {k: mx(k) for k in series}
    checks, ok = {}, True
    t = targets if isinstance(targets, dict) else {}
    for key, src in (("head_rise_mm", "head_rise_mm"), ("fist_rise_mm", "fist_center_rise_mm"),
                     ("fist_out_mm", "fist_center_out_mm")):
        rngt = t.get(key)
        if isinstance(rngt, (list, tuple)) and len(rngt) == 2 and summary[src]:
            v = summary[src]["max"]
            checks[key] = {"measured_max": v, "target": list(rngt), "within": rngt[0] <= v <= rngt[1]}
            ok = ok and checks[key]["within"]
    cm = t.get("club_tip_move_mm_max")
    if isinstance(cm, (int, float)) and summary["club_tip_move_mm"]:
        v = summary["club_tip_move_mm"]["max"]
        checks["club_tip_move_mm_max"] = {"measured_max": v, "target": cm, "within": v <= cm}
        ok = ok and checks["club_tip_move_mm_max"]["within"]
    return {"summary": summary, "targets_check": checks, "targets": t, "per_frame": series,
            "n_club_tip_verts": int(len(ctx.geo.club_tip_idx)), "eye_point": eye_info}, ok


EYE_FRONT_FRAC = 0.25          # M2 eye point: front (-Y) quarter of the head part's y extent
EYE_Z_BAND = (0.40, 0.65)      # M2 eye point: upper-middle band of the head part's z extent (eye rings)
AMP_NOTE = ("Relative to the first frame, world: head_rise = z rise of the eye point (mean of a fixed head-part "
            "vertex set, selection rule in measured.eye_point; d-26 section 2 basis, targets.head_rise_mm); "
            "head_bone_rise = DEF head head z (report); fist_tail = DEF hand_l tail; fist_center = "
            "centroid of the evaluated hand_l shell vertices (targets use the centroid); out = +X (character left, "
            "outward for the left fist); club_tip = centroid of the GOB_club vertices within 1 mm of the max club "
            "local Y (head end), movement = 3D distance; club_grip = GOB_club origin. Report only.")


def run_phase2(rows, ctx, geo, ev):
    ai = ctx.act_info
    frames = list(range(FRAME_RANGE[0], FRAME_RANGE[1] + 1))
    M = {f: frame_measure(ctx, geo, f, "action") for f in frames}
    # C5
    fr = ai["frame_range"]
    fps_ok = ai["fps"] is not None and abs(float(ai["fps"]) - FPS) < 1e-6
    fr_ok = fr[0] == FRAME_RANGE[0] and fr[1] == FRAME_RANGE[1]
    keys_ok = not ai["non_ctrl_paths"] and ai["n_fcurves"] > 0
    match, match_ok = {}, True
    for f in KEY_FRAMES:
        p = ctx.poses.get(f)
        m = M.get(f)
        if p is None or m is None:
            match[f] = "missing in JSON" if p is None else "key frame outside the frame range"
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
    rows.add("C5", {"fps": ai["fps"], "action_frame_range": fr, "scene_frame_start_end": ai["scene_frame_start_end"],
                    "use_frame_range": ai["use_frame_range"], "n_fcurves": ai["n_fcurves"],
                    "non_ctrl_paths": ai["non_ctrl_paths"][:20], "n_non_ctrl_paths": len(ai["non_ctrl_paths"]),
                    "keyed_bones": ai["keyed_bones"], "keyed_frames": ai["keyed_frames"],
                    "nla_tracks": ai["nla_tracks"], "key_pose_match": match},
             fps_ok and fr_ok and keys_ok and match_ok,
             "DEF world = matrix_world @ pose_bone.matrix of the evaluated rig after scene.frame_set(f). "
             "CTRL mode: listed channels only (euler rad, loc m, scale, props units); unlisted non-identity "
             "controls reported. json_applied = JSON pose applied in memory (unlisted CTRL identity, PROPS "
             "contract defaults), report only.")
    # C6
    f0, f1 = FRAME_RANGE
    worst, wb, pos, rot = 0.0, None, 0.0, 0.0
    for n in ctx.def_names:
        a, b = M[f1]["def_world"][n], M[f0]["def_world"][n]
        d = mat_max_abs(a, b)
        if d > worst:
            worst, wb = d, n
        pos = max(pos, (a.translation - b.translation).length)
        rot = max(rot, rot_diff_deg(a, b))
    rows.add("C6", {"frames": [f1, f0], "def_world_last_vs_first_max_abs": sci(worst), "bone": wb,
                    "pos_mm": mm(pos), "rot_deg": rnd(rot, 5)}, worst <= POSE_TOL,
             "max abs element of the 4x4 world matrices over the 24 DEF bones.")
    # C7
    rb, rw = 0.0, 0.0
    r0 = M[f0]["root_world"]
    for f in frames:
        c = M[f]["ctrl"].get("CTRL_root")
        if c is not None:
            rb = max(rb, mat_max_abs(c["basis"], Matrix.Identity(4)))
        if r0 is not None and M[f]["root_world"] is not None:
            rw = max(rw, mat_max_abs(M[f]["root_world"], r0))
    rows.add("C7", {"ctrl_root_basis_vs_identity_max_abs": sci(rb), "ctrl_root_world_vs_first_max_abs": sci(rw),
                    "ctrl_root_fcurves": ai["root_fcurves"]}, rb <= ROOT_TOL and rw <= ROOT_TOL,
             "CTRL_root basis vs identity and world vs the first frame, every frame.")
    # C8
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
    rows.add("C8", {"min_clearance_mm": mm(wc[0]), "min_at_frame": wc[1], "min_at": wc[2],
                    "max_visible_cover": ">=2" if wcov == 2 else wcov, "failing_frames": fails,
                    "exclusion": geo.excl_info, "per_frame": per},
             not fails, CLEAR_NOTE + " " + ISECT_NOTE)
    # C9
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
    rows.add("C9", {"n_flips": len(flips), "flips": flips[:30], "theta_deg_min_max": trange,
                    "chains": {k: v["def"][:2] for k, v in ctx.chains.items()},
                    "sections": {s: [a, b] for s, a, b in SECTIONS}, "sections_from": ctx.sections_from,
                    "max_adjacent_rot_per_section": sec},
             not flips,
             "G6.5b: theta = atan2(dot(cross(u, l), X_upper), dot(u, l)) of the DEF chain (upper, lower); flip = "
             "sign change between adjacent frames with |theta| > 2 deg on both. Adjacent change = max over the 24 "
             "DEF bones of the world rotation angle (float64 matrices, f-1 -> f), pair assigned to the section "
             "of f.")
    # C10
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
        S, zs, wp, zp = 0.0, [], None, None
        for f in range(r["a"], r["b"] + 1):
            w = M[f]["planted"][r["foot"]].get("shoe_w")
            if w is None:
                zs = None
                break
            z = float(w[:, 2].min())
            zs.append(z)
            if wp is not None:
                c_k = wp[:, 2] <= zp + CONTACT_TOL          # contact set of frame k (G6.7 definition)
                S += float(np.mean(np.hypot(w[c_k, 0] - wp[c_k, 0], w[c_k, 1] - wp[c_k, 1])))
            wp, zp = w, z
        zmm = [mm(min(zs)), mm(max(zs))] if zs else None
        if r.get("mode") == "pivot":
            z_ok = bool(zs) and all(PLANT_Z[0] <= z <= PLANT_Z[1] for z in zs)
            rok = zs is not None and S <= PIVOT_SLIP and z_ok
            rng.append({"foot": r["foot"], "frames": [r["a"], r["b"]], "mode": "pivot", "slip_cum_mm": mm(S),
                        "shoe_min_z_mm_min_max": zmm, "z_in_range_every_frame": z_ok,
                        "max_move_mm (report)": mm(mx), "at_frame (report)": at, "ok": rok})
            ok5 = ok5 and rok
        else:
            rng.append({"foot": r["foot"], "frames": [r["a"], r["b"]], "max_move_mm": mm(mx), "at_frame": at,
                        "mode": "def_move", "slip_cum_mm (report)": mm(S) if zs is not None else None,
                        "shoe_min_z_mm_min_max (report)": zmm})
            ok5 = ok5 and mx <= FOOT_MOVE
    rows.add("C10", {"ranges": rng, "n_ranges": len(rng)}, ok5,
             "DEF foot_x world head and tail displacement from the range's first frame, max over the range. "
             "No contact_ranges -> ok False. mode pivot (T132): G6.7 planted-roll rule (cumulative slip of the "
             f"contact set z <= lowest + {CONTACT_TOL * 1000:g} mm, lowest shoe z range) instead of the DEF movement; "
             "slip / z are reported for every range.")
    # C11
    if ctx.c11:
        per, pen_fail, rest_fail = {}, [], []
        for f in frames:
            e, in_rest, pen_ok, rest_ok = ground_entry(geo, M[f], f)
            per[f] = e
            if pen_ok is False:
                pen_fail.append(f)
            if rest_ok is False:
                rest_fail.append(f)

        def ext(k, fn):
            f = fn(frames, key=lambda x: per[x][k])
            return {"mm": per[f][k], "frame": f}

        bmin, cmin = ext("body_z_mm", min), ext("club_z_mm", min)
        bmin.update(part=per[bmin["frame"]]["part"], bone=per[bmin["frame"]]["bone"])
        cmin["club_s_mm"] = per[cmin["frame"]]["club_s_mm"]
        rest = []
        for a, b in GROUND["rest"]:
            vals = {f: per[f]["body_z_mm"] for f in range(a, b + 1)}
            fmax = max(vals, key=vals.get)
            rest.append({"frames": [a, b], "body_z_mm": vals, "max_mm": vals[fmax], "max_at_frame": fmax,
                         "min_mm": min(vals.values())})
        pen_ok_all = not pen_fail
        rest_ok_all = not rest_fail and ground_cfg_ok()
        rows.add("C11", {"body_min_z": bmin, "body_max_z": ext("body_z_mm", max), "club_min_z": cmin,
                         "club_max_z": ext("club_z_mm", max),
                         "thresholds_mm": {"penetration": GROUND["penetration_mm"], "rest_max": GROUND["rest_max_mm"]},
                         "component_ok": {"penetration": pen_ok_all, "resting": rest_ok_all},
                         "penetration_failing_frames": pen_fail, "rest_failing_frames": rest_fail,
                         "rest_ranges": rest, "range_issues": GROUND["issues"], "per_frame": per,
                         "per_pose": ctx.c11_pose},
                 pen_ok_all and rest_ok_all, ground_notes())
    # C12 (report only, HR2 2026-09-26)
    per, pv, kv, dk = {}, [], [], []
    for f in frames:
        mo_ = M[f]["mouth"]
        p, k = mo_["prop"], mo_["key"]
        per[f] = {"prop": None if p is None else rnd(p, 6), "key": None if k is None else rnd(k, 6)}
        if p is not None:
            pv.append(p)
        if k is not None:
            kv.append(k)
        if p is not None and k is not None:
            dk.append(abs(k - p))
    rows.add("C12", {"per_frame": per, "prop_min_max": [rnd(min(pv), 6), rnd(max(pv), 6)] if pv else None,
                     "key_min_max": [rnd(min(kv), 6), rnd(max(kv), 6)] if kv else None,
                     "max_abs_key_minus_prop": sci(max(dk)) if dk else None,
                     "frames_key_nonzero": [f for f in frames if (per[f]["key"] or 0.0) != 0.0]},
             True, "report only (2026-09-26 HR2): prop = evaluated PROPS pose-bone value, key = "
                   "GOB_mesh.data.shape_keys.evaluated_get(depsgraph).key_blocks['mouth_open'].value after "
                   "scene.frame_set(f); None = absent.")
    # M1
    cfg = ctx.kp.get("match_first_frame") if isinstance(ctx.kp, dict) else None
    if cfg:
        meas, ok = match_first(ctx, cfg, M, ev)
        rows.add("M1", meas, ok, "DEF world (rig.matrix_world @ pose_bone.matrix) of the clip's first frame vs the "
                                 "reference rig (GOB_rig appended in memory from the reference blend, its action "
                                 "assigned, scene.frame_set(frame)); max abs 4x4 element over the 24 DEF bones.")
    # M2
    if isinstance(ctx.kp, dict) and ctx.kp.get("targets") is not None:
        meas, ok = amplitude(ctx, M, frames, ctx.kp.get("targets"))
        rows.add("M2", meas, ok, AMP_NOTE)


# ---------------------------------------------------------------- main
def resolve_path(p, base=None):
    """absolute; existing relative to cwd; 'rig/...' relative to work/goblin_swing; else relative to base/RIG."""
    q = Path(p)
    if q.is_absolute():
        return q
    if q.exists():
        return q.resolve()
    if q.parts and q.parts[0] == "rig":
        return (goblib.RIG.parent / q).resolve()
    return ((base or goblib.RIG) / q).resolve()


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {"gate": "both", "json": None, "action": None, "out": None, "blend": None, "stage": "s99"}
    i = 0
    while i < len(argv):
        a = argv[i]
        k = a[2:] if a.startswith("--") else None
        if k in out and i + 1 < len(argv):
            out[k] = argv[i + 1]
            i += 2
            continue
        print(f"{CHECKER}: unexpected argument {a!r}")
        sys.exit(2)
    g = str(out["gate"]).upper()
    phases = {"I1": ["1"], "A1": ["1"], "1": ["1"], "I2": ["2"], "A2": ["2"], "2": ["2"], "BOTH": ["1", "2"]}
    if g not in phases:
        print(f"{CHECKER}: --gate must be I1|I2|A1|A2|1|2|both")
        sys.exit(2)
    out["phases"], out["auto_block"] = phases[g], g == "BOTH"
    if not out["action"] or not out["json"]:
        print(f"{CHECKER}: --action and --json are required")
        sys.exit(2)
    return out


def write(ev, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"gate": ev.gate, "checker": ev.checker, "inputs": ev.inputs, "criteria": ev.criteria,
           "renders": ev.renders}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(goblib._jsonable(doc), f, indent=1, ensure_ascii=False)
    for c in ev.criteria:
        ms = json.dumps(goblib._jsonable(c["measured"]), ensure_ascii=False)
        print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']} measured={ms[:300]}{'...' if len(ms) > 300 else ''}")
    print(f"[{GATE}/{CHECKER}] wrote {rel(path)}")
    sys.stdout.flush()
    return path


def main():
    global GATE, ACTION, KEY_FRAMES, FRAME_RANGE, FPS, SECTIONS
    args = parse_args()
    ACTION = args["action"]
    kp_path = resolve_path(args["json"])
    GATE = kp_path.stem
    out_path = resolve_path(args["out"]) if args["out"] else goblib.INSPECT / kp_path.stem / f"{CHECKER}.json"
    ctx = Ctx()
    ctx.kp_path = kp_path
    if args["blend"]:
        bp = resolve_path(args["blend"])
        if not bp.exists():
            ctx.miss.append(f"missing input: {rel(bp)}")
        elif not bpy.data.filepath or Path(bpy.data.filepath).resolve() != bp:
            bpy.ops.wm.open_mainfile(filepath=str(bp))
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(os.path.abspath(__file__))
    if bpy.data.filepath and Path(bpy.data.filepath).exists():
        ev.add_input(bpy.data.filepath)
    for p in (kp_path, CANON_JSON, MANIFEST_JSON, PARTS_JSON):
        if Path(p).exists():
            ev.add_input(p)
    ev.add_stage_inputs(args["stage"])
    rows = Rows()
    all_ids = {"1": list(PHASE1), "2": ["C5", "C6", "C7", "C8", "C9", "C10", "C11"]}   # C11 emitted only if mapped

    if not ctx.miss:
        load_inputs(ctx, kp_path)
    idmap = resolve_ids(ctx.kp, ACTION)
    if ctx.miss:
        for ph in args["phases"]:
            blocked(rows, all_ids[ph], "; ".join(ctx.miss))
        emit(ev, rows, idmap)
        write(ev, out_path)
        print(f"[{GATE}/{CHECKER}] " + "; ".join(ctx.miss))
        sys.stdout.flush()
        sys.exit(2)

    kp = ctx.kp if isinstance(ctx.kp, dict) else {}
    try:
        FPS = float(kp.get("fps", 24))
    except (TypeError, ValueError):
        FPS = 24.0
    fr = kp.get("frame_range")
    if isinstance(fr, (list, tuple)) and len(fr) == 2:
        FRAME_RANGE = (int(fr[0]), int(fr[1]))
    elif ctx.act is not None:
        a, b = ctx.act.frame_range
        FRAME_RANGE = (int(round(a)), int(round(b)))
    secs = kp.get("sections")
    if isinstance(secs, list) and secs:
        SECTIONS = tuple((str(x.get("name")), int(x["frames"][0]), int(x["frames"][1])) for x in secs)
        ctx.sections_from = "clip JSON sections"
    elif idmap["sections"]:
        SECTIONS = tuple(idmap["sections"])
        ctx.sections_from = "preset sections"
    else:
        SECTIONS = (("all", FRAME_RANGE[0], FRAME_RANGE[1]),)
        ctx.sections_from = "default (one section)"
    ctx.c11 = "C11" in idmap["ids"]
    if ctx.c11:   # C11 config (spec d-28 section 4): missing thresholds = calibration run
        gc = kp.get("ground")
        if gc is not None and not isinstance(gc, dict):
            GROUND["issues"].append("ground is not an object")
        gc = gc if isinstance(gc, dict) else {}
        for k in ("penetration_mm", "rest_max_mm"):
            v = gc.get(k)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                GROUND[k] = float(v)
            elif v is not None:
                GROUND["issues"].append(f"ground.{k} not a number: {v!r}")
        rr = kp.get("ground_rest_ranges")
        if rr is not None and not isinstance(rr, list):
            GROUND["issues"].append("ground_rest_ranges is not a list")
        for r in (rr if isinstance(rr, list) else []):
            try:
                a, b = int(r[0]), int(r[1])
            except (TypeError, ValueError, IndexError, KeyError):
                GROUND["issues"].append(f"ground_rest_ranges entry {r!r}: no [a, b]")
                continue
            if not (FRAME_RANGE[0] <= a <= b <= FRAME_RANGE[1]):
                GROUND["issues"].append(f"ground_rest_ranges [{a}, {b}] outside {list(FRAME_RANGE)} or a > b")
                continue
            GROUND["rest"].append((a, b))

    save_state(ctx)
    try:
        prepare(ctx)
        ctx.chains = ik_chains(ctx)
        per_pose_issues = parse_keyposes(ctx)
        kf = kp.get("key_frames")
        required_given = isinstance(kf, list) and bool(kf)
        KEY_FRAMES = tuple(sorted(int(x) for x in kf)) if required_given else tuple(sorted(ctx.poses))
        geo = Geo(ctx)
        ctx.geo = geo
        geo.shoe_v = {x: geo.topo.verts_of_polys(geo.topo.polys_of(f"shoe_{x}")) for x in SIDES}
        if ctx.act is not None:
            slot = assign_action(ctx)
            ctx.act_info = action_info(ctx)
            ctx.act_info["slot"] = slot
        else:
            ctx.act_info = {"action": None}
        source = "action" if ctx.act is not None else "json"
        keyed = set(ctx.act_info.get("keyed_frames") or [])
        p2_block = None
        if ctx.act is None:
            p2_block = f"action {ACTION} missing in {rel(bpy.data.filepath)}"
        elif args["auto_block"] and keyed and keyed <= set(KEY_FRAMES):
            p2_block = (f"action keys only at key-pose frames {sorted(keyed)} (key-pose stage); "
                        f"run -- --gate 2 to force")
        if "1" in args["phases"]:
            run_phase1(rows, ctx, geo, per_pose_issues, source, required_given)
        if "2" in args["phases"]:
            if p2_block:
                blocked(rows, all_ids["2"], p2_block)
                if ctx.c11_pose is not None:   # keep the phase-1 C11 per_pose block (D1.2 report)
                    rows.c["C11"]["measured"] = {"phase2": rows.c["C11"]["measured"], "per_pose": ctx.c11_pose}
            else:
                run_phase2(rows, ctx, geo, ev)
        info = {"action": {k: v for k, v in ctx.act_info.items() if k != "keyed_bones"},
                "pose_source_phase1": source, "notes": ctx.notes, "clip_json": rel(kp_path),
                "gate_id": {"source": idmap["source"], "ids": idmap["ids"], "report": sorted(idmap["report"])},
                "frame_range": list(FRAME_RANGE), "fps": FPS, "key_frames": list(KEY_FRAMES),
                "sections": [list(x) for x in SECTIONS], "phases": args["phases"],
                "stage_inputs": args["stage"]}
        emit(ev, rows, idmap)
        ev.criteria.insert(0, {"id": "info", "measured": goblib._jsonable(info), "threshold": None, "ok": True,
                               "note": "report only (run context)"})
    finally:
        restore_state(ctx)
    write(ev, out_path)


if __name__ == "__main__":
    goblib.run_main(main)
