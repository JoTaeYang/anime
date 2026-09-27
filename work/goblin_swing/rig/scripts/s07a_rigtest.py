"""s07a_rigtest.py - T31 (G7.1): test clip `rigtest` on GOB_rig -> gob_r05_rigtest.blend + data/rigtest_manifest.json.

Input : rig/gob_r04_ctrl.blend (G6 rig, read only), rig/data/ctrl_manifest.json (read only, sweep ranges / clamps),
        rig/data/canonical_skeleton.json (read only, rest check)
Output: rig/gob_r05_rigtest.blend (save copy, save_version 0), rig/data/rigtest_manifest.json

Run:  bl.ps1 -Script s07a_rigtest.py -Blend gob_r04_ctrl.blend

- Action `rigtest`, 24 fps, keys only on CTRL_* pose bones (location, Euler rotation, scale) and on the PROPS custom
  properties. DEF / MCH are never keyed (they follow through the G6 constraints and drivers).
- Every key frame keys the full control state (all CTRL_* transforms + all PROPS properties), so a later frame_set
  beyond the last key returns exactly the last pose (constant extrapolation) and each key is a complete pose.
- Space switches / snaps / ready pose / reset use the T28 operators (blend text goblin_rig_ui.py imported as a module)
  with auto keying on; the operator keys are then completed by the full-state key at the same frame.
  Switch / snap technique: last key at f-1 = old space / old mode, operator at f, key at f.
- PROPS interpolation: int spaces CONSTANT; ik_fk CONSTANT except the keys that start an IK/FK blend (BEZIER);
  foot_* BEZIER (sweeps). CTRL keys: BEZIER, auto-clamped handles (holds stay flat).
- Every value written by this script is asserted inside ctrl_manifest sweep / clamps; values produced by the operators
  (snap / switch compensation) are reported only.
- Self-checks (stdout): keyed bones, frame range, segment table, last frame DEF vs canonical rest, DEF foot movement
  in foot_lock segments, DEF discontinuity f-1 -> f at every switch / snap.
"""
import math
import os
import re
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402
from bpy_extras import anim_utils  # noqa: E402

OUT_BLEND = goblib.RIG / "gob_r05_rigtest.blend"
MANIFEST_OUT = "rigtest_manifest.json"
ARM = "GOB_rig"
ACTION = "rigtest"
FPS = 24
FRAME_START = 1
TEXT_UI = "goblin_rig_ui.py"
PROPS = "PROPS"
OPS = ("snap_ikfk", "switch_space", "reset_rig", "ready_pose")
REST_TOL = 1e-4
EPS = 1e-6
AXES = "XYZ"


def rot_path(pb):
    return {"QUATERNION": "rotation_quaternion", "AXIS_ANGLE": "rotation_axis_angle"}.get(pb.rotation_mode,
                                                                                         "rotation_euler")


# ---------------------------------------------------------------- clip builder
class Clip:
    def __init__(self, arm, man):
        self.arm, self.man = arm, man
        self.pbs = arm.pose.bones
        self.props = self.pbs[PROPS]
        self.ctrls = [pb.name for pb in self.pbs if pb.name.startswith("CTRL_")]
        self.prop_names = [k for k in self.props.keys() if isinstance(self.props[k], (int, float))]
        self.sweep = {(c, s["channel"], s["axis"]): (s["min"], s["max"])
                      for c, e in man["controls"].items() for s in e["sweep"]}
        self.f = None  # last keyed frame
        self.events = []
        self.blend_starts = defaultdict(set)
        self.segments = []
        self._seg = None
        self.marks = {}  # named frame ranges inside segments (self-check only)

    # -- low level
    def goto(self, f):
        bpy.context.scene.frame_set(f)

    def refresh(self):
        self.arm.update_tag()
        bpy.context.view_layer.update()

    def key_all(self, f):
        for n in self.ctrls:
            pb = self.pbs[n]
            pb.keyframe_insert("location", frame=f, group=n)
            pb.keyframe_insert(rot_path(pb), frame=f, group=n)
            pb.keyframe_insert("scale", frame=f, group=n)
        for k in self.prop_names:
            self.props.keyframe_insert(f'["{k}"]', frame=f, group=PROPS)
        self.f = f

    def _read(self, c, ch, ax):
        pb = self.pbs[c]
        i = AXES.index(ax)
        return pb.location[i] if ch == "loc" else math.degrees(pb.rotation_euler[i])

    def _write(self, c, ch, ax, v):
        rng = self.sweep.get((c, ch, ax))
        if rng is None:
            raise RuntimeError(f"{c} {ch} {ax}: channel not in ctrl_manifest sweep")
        if not rng[0] - EPS <= v <= rng[1] + EPS:
            raise RuntimeError(f"{c} {ch} {ax} = {v} outside sweep {rng}")
        pb = self.pbs[c]
        i = AXES.index(ax)
        if ch == "loc":
            pb.location[i] = v
        else:
            if pb.rotation_mode not in ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"):
                raise RuntimeError(f"{c}: rotation_mode {pb.rotation_mode} is not Euler")
            pb.rotation_euler[i] = math.radians(v)

    def _prop_range(self, k):
        if k in self.man.get("clamps", {}):
            return tuple(self.man["clamps"][k])
        for sp in self.man["spaces"].values():
            if sp["prop"] == k:
                return (0, len(sp["values"]) - 1)
        for e in self.man["ikfk"].values():
            if e["prop"] == k:
                return (0.0, 1.0)
        raise RuntimeError(f"PROPS {k}: no range in ctrl_manifest")

    def _identity(self, c):
        pb = self.pbs[c]
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_euler = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        pb.scale = (1.0, 1.0, 1.0)

    # -- keys
    def start(self):
        self.goto(FRAME_START)
        self.refresh()
        self.key_all(FRAME_START)

    def pose(self, dt, set=(), add=(), zero=(), props=None, blend=()):
        """Key frame last + dt: current (evaluated) state + changes. blend = ik_fk props whose smooth transition
        starts at this key."""
        f = self.f + dt
        self.goto(f)
        for c in zero:
            self._identity(c)
        for c, ch, ax, v in set:
            self._write(c, ch, ax, float(v))
        for c, ch, ax, d in add:
            self._write(c, ch, ax, self._read(c, ch, ax) + d)
        for k, v in (props or {}).items():
            lo, hi = self._prop_range(k)
            if not lo - EPS <= v <= hi + EPS:
                raise RuntimeError(f"PROPS {k} = {v} outside {lo, hi}")
            self.props[k] = type(self.props[k])(v)
        for k in blend:
            self.blend_starts[k].add(f)
        self.refresh()
        self.key_all(f)

    def hold(self, dt):
        self.pose(dt)

    def op(self, name, dt=1, **kw):
        """Operator at last + dt (dt 0 = second operator on the same frame), then full-state key."""
        f = self.f + dt
        if dt:
            self.goto(f)
        r = getattr(bpy.ops.goblin, name)(**kw)
        if "FINISHED" not in r:
            raise RuntimeError(f"goblin.{name}({kw}) returned {r}")
        self.refresh()
        self.key_all(f)
        kind = {"switch_space": "switch", "snap_ikfk": "snap"}.get(name, name)
        self.events.append({"frame": f, "kind": kind, "op": f"goblin.{name}", "args": dict(kw),
                            "segment": self._seg["name"] if self._seg else None})

    # -- segments
    def seg(self, name, covers, foot_lock):
        self._seg = {"name": name, "start": self.f, "end": None, "covers": list(covers), "foot_lock": list(foot_lock)}

    def end(self):
        self._seg["end"] = self.f
        self.segments.append(self._seg)
        self._seg = None


def both(fmt, ch, ax, vl, vr=None):
    vr = vl if vr is None else vr
    return [(fmt.format("l"), ch, ax, vl), (fmt.format("r"), ch, ax, vr)]


def build(c):
    R, T, H, CH, S1, W = "CTRL_root", "CTRL_torso", "CTRL_head", "CTRL_chest", "CTRL_spine_01", "CTRL_weapon"
    SH, UA, LA, HF = "CTRL_shoulder_{}", "CTRL_upperarm_fk_{}", "CTRL_lowerarm_fk_{}", "CTRL_hand_fk_{}"
    HI, EP = "CTRL_hand_ik_{}", "CTRL_elbow_pole_{}"
    TH, CA, FT = "CTRL_thigh_fk_{}", "CTRL_calf_fk_{}", "CTRL_foot_fk_{}"
    ARMP = ["arm_ik_fk_l", "arm_ik_fk_r"]
    arm_fk_all = [n.format(x) for n in (SH, UA, LA, HF) for x in "lr"]
    c.start()

    # 1 root move + rotation (feet follow the root: foot IK controls are children of CTRL_root)
    c.seg("root", ["root"], [])
    c.pose(6, set=[(R, "loc", "Z", 0.3)])
    c.pose(6, set=[(R, "loc", "X", 0.2), (R, "rot", "Y", 45)])
    c.pose(6, set=[(R, "loc", "Z", -0.2), (R, "rot", "Y", -60)])
    c.pose(6, set=[(R, "loc", "X", -0.15), (R, "rot", "Y", 20)])
    c.pose(6, zero=[R])
    c.end()

    # 2 COG (CTRL_torso) move / rotation, legs IK, feet fixed; down first so the 150 mm legs keep slack (hips
    #   x +-0.205 m: yaw / lean lift one hip, values chosen so hip -> ankle stays <= 139 mm < leg 151 mm)
    c.seg("cog", ["cog"], ["l", "r"])
    c.pose(6, set=[(T, "loc", "Y", -0.05)])
    c.pose(6, set=[(T, "loc", "X", 0.04)])
    c.pose(6, set=[(T, "loc", "X", -0.04), (T, "loc", "Z", 0.04)])
    c.pose(6, set=[(T, "loc", "X", 0.0), (T, "loc", "Z", -0.04), (T, "rot", "X", 20)])
    c.pose(6, set=[(T, "loc", "Z", 0.0), (T, "rot", "X", -15), (T, "rot", "Y", 20)])
    c.pose(6, set=[(T, "rot", "X", 0), (T, "rot", "Y", -20)])
    c.pose(6, set=[(T, "rot", "Y", 0), (T, "rot", "Z", 10)])
    c.pose(6, set=[(T, "rot", "Z", -10)])
    c.pose(6, zero=[T])
    c.end()

    # 2b (T31c) pelvis about the waist pivot (T24c: CTRL_pelvis head = waist z 0.5777, bone points -Z; rot X+ = hips
    #   swing back, rot Y = yaw about the vertical, rot Z = hip sway, loc Y = down, loc Z = back). Legs IK, feet fixed.
    #   The hips sit about 0.30 m below the pivot, so rot swings them 5 mm/deg: COG (CTRL_torso loc Y) goes down 50 mm
    #   first and stays there, then rot X +-12, rot Y +-15, rot Z +-8 (hip -> ankle <= 136 mm planned, leg 151 mm),
    #   then a small loc (+-0.015 m; loc moves the waist itself), then COG back up. Spine controls stay identity.
    P = "CTRL_pelvis"
    c.seg("pelvis", ["pelvis"], ["l", "r"])
    c.pose(6, set=[(T, "loc", "Y", -0.05)])
    r0 = c.f
    c.pose(6, set=[(P, "rot", "X", 12)])
    c.pose(6, set=[(P, "rot", "X", -12)])
    c.pose(6, set=[(P, "rot", "X", 0), (P, "rot", "Y", 15)])
    c.pose(6, set=[(P, "rot", "Y", -15)])
    c.pose(6, set=[(P, "rot", "Y", 0), (P, "rot", "Z", 8)])
    c.pose(6, set=[(P, "rot", "Z", -8)])
    c.pose(6, set=[(P, "rot", "Z", 0)])
    c.marks["pelvis_rot"] = (r0, c.f)
    r1 = c.f
    c.pose(6, set=[(P, "loc", "X", 0.015), (P, "loc", "Z", 0.015)])
    c.pose(6, set=[(P, "loc", "X", -0.015), (P, "loc", "Y", 0.015), (P, "loc", "Z", -0.015)])
    c.pose(6, zero=[P])
    c.marks["pelvis_loc"] = (r1, c.f)
    c.pose(6, zero=[T])
    c.end()

    # 3 arm FK ROM, both arms (rom_poses sign convention: raise upperarm_l Z+ / upperarm_r Z-, X+ forward)
    c.seg("arm_fk", ["arm_fk_l", "arm_fk_r"], ["l", "r"])
    c.pose(6, set=both(SH, "rot", "Z", 15, -15) + both(UA, "rot", "Z", 80, -80))
    c.pose(6, set=both(SH, "rot", "Z", 0) + both(SH, "rot", "X", 15) + both(UA, "rot", "Z", 0)
           + both(UA, "rot", "X", 60))
    c.pose(6, set=both(UA, "rot", "X", 30) + both(UA, "rot", "Y", 45, -45))
    c.pose(6, set=both(UA, "rot", "Y", -45, 45) + both(LA, "rot", "X", 100))
    c.pose(6, set=both(SH, "rot", "X", 0) + both(UA, "rot", "X", 0) + both(UA, "rot", "Y", 0)
           + both(LA, "rot", "X", 60) + both(HF, "rot", "X", 45))
    c.pose(6, set=both(HF, "rot", "X", -45) + both(HF, "rot", "Y", 60, -60))
    c.pose(6, set=both(HF, "rot", "Y", -60, 60) + both(HF, "rot", "Z", 20, -20))
    c.pose(6, set=both(HF, "rot", "X", 0) + both(HF, "rot", "Y", 0) + both(HF, "rot", "Z", -20, 20)
           + both(LA, "rot", "X", 0))
    c.pose(6, zero=arm_fk_all)
    c.end()

    # 4 arm IK move, IK/FK blend (0 -> 1 -> 0 transitions), snap TO_FK and TO_IK (both arms)
    c.seg("arm_ik_fk", ["arm_ik_l", "arm_ik_r", "ikfk_blend_l", "ikfk_blend_r", "snap_arm_l", "snap_arm_r"],
          ["l", "r"])
    c.pose(6, set=both(UA, "rot", "X", 40) + both(LA, "rot", "X", 50), blend=ARMP)
    c.pose(10, props={p: 1.0 for p in ARMP})
    c.pose(8, set=both(HI, "loc", "Y", -0.12) + both(HI, "loc", "Z", -0.10) + both(HI, "rot", "X", 20))
    c.hold(6)
    c.op("snap_ikfk", chain="arm_l", direction="TO_FK")
    c.op("snap_ikfk", dt=0, chain="arm_r", direction="TO_FK")
    c.pose(8, add=both(LA, "rot", "X", 20) + both(HF, "rot", "X", 15))
    c.hold(6)
    c.op("snap_ikfk", chain="arm_l", direction="TO_IK")
    c.op("snap_ikfk", dt=0, chain="arm_r", direction="TO_IK")
    c.pose(8, add=both(HI, "loc", "X", 0.06))
    c.pose(6, blend=ARMP)
    c.pose(10, props={p: 0.0 for p in ARMP})
    c.pose(8, zero=[n.format(x) for n in (UA, LA, HF, HI, EP) for x in "lr"])
    c.end()

    # 5 head space chest -> world -> chest, pose held before / after each switch
    c.seg("head_space", ["head_space"], ["l", "r"])
    c.pose(6, set=[(H, "rot", "X", 15), (CH, "rot", "Z", 10)])
    c.hold(6)
    c.op("switch_space", prop="head_space", value=1)
    c.hold(6)
    c.pose(8, set=[(CH, "rot", "Z", -15), (CH, "rot", "X", 15), (S1, "rot", "Y", 15)])
    c.hold(6)
    c.op("switch_space", prop="head_space", value=0)
    c.hold(6)
    c.pose(8, zero=[H, CH, S1])
    c.end()

    # 6 left hand IK in weapon space (3) while the weapon moves (right arm FK + socket offset in hand_r space);
    #   switches happen with the weapon at rest so the compensated CTRL_hand_ik_l values stay in the sweep range
    #   (the left hand then moves rigidly with the club, about 1.4 m lever from the grip, so rotations are small)
    c.seg("hand_ik_space_l_weapon", ["hand_ik_space_l_weapon", "weapon_offset", "snap_arm_l"], ["l", "r"])
    c.op("snap_ikfk", chain="arm_l", direction="TO_IK")
    c.pose(8, set=[(HI.format("l"), "loc", "Y", -0.10), (HI.format("l"), "loc", "Z", -0.10)])
    c.hold(6)
    c.op("switch_space", prop="hand_ik_space_l", value=3)
    c.hold(6)
    c.pose(8, set=[(W, "loc", "X", 0.08), (W, "loc", "Z", 0.05), (W, "rot", "X", 10), (UA.format("r"), "rot", "X", 20)])
    c.pose(8, set=[(W, "loc", "X", -0.05), (W, "loc", "Z", 0.0), (W, "rot", "X", -10), (UA.format("r"), "rot", "X", 0),
                   (UA.format("r"), "rot", "Z", -15)])
    c.pose(8, zero=[W, UA.format("r")])
    c.hold(6)
    c.op("switch_space", prop="hand_ik_space_l", value=0)
    c.hold(6)
    c.pose(8, zero=[HI.format("l"), EP.format("l")])
    c.op("snap_ikfk", chain="arm_l", direction="TO_FK")
    c.end()

    # 7 weapon world space: the club stays in the world while the hand moves away, moves on its own (socket
    #   offset), then switches back to hand_r at the rest arm and returns to the socket (CTRL_weapon -> identity)
    c.seg("weapon_world", ["weapon_world", "weapon_return"], ["l", "r"])
    c.op("switch_space", prop="weapon_space", value=3)
    c.hold(6)
    c.pose(8, set=[(UA.format("r"), "rot", "X", 45), (LA.format("r"), "rot", "X", 60),
                   (W, "loc", "X", 0.15), (W, "rot", "X", 45)])
    c.pose(8, set=[(W, "loc", "Y", -0.2), (W, "rot", "Z", 30)])
    c.pose(8, zero=[UA.format("r"), LA.format("r")])
    c.hold(6)
    c.op("switch_space", prop="weapon_space", value=0)
    c.hold(6)
    c.pose(8, zero=[W])
    c.end()

    # 8 foot roll / bank sweep inside the clamps from the ready pose (G6.7 stance), one foot at a time;
    #   no foot_lock (main T31 addition: the sweep moves the ankle by design; planted quality is G6.7)
    cl = c.man["clamps"]
    c.seg("foot_roll_bank_l", ["ready_pose", "foot_roll_l", "foot_bank_l"], [])
    c.op("ready_pose", dt=8)
    for p, v in (("foot_roll_l", -21), ("foot_roll_l", 23), ("foot_roll_l", 0),
                 ("foot_bank_l", -13), ("foot_bank_l", 27), ("foot_bank_l", 0)):
        assert v == 0 or cl[p][0] < v < cl[p][1]
        c.pose(6, props={p: v})
    c.end()
    c.seg("foot_roll_bank_r", ["foot_roll_r", "foot_bank_r"], [])
    for p, v in (("foot_roll_r", -24), ("foot_roll_r", 26), ("foot_roll_r", 0),
                 ("foot_bank_r", -11), ("foot_bank_r", 16), ("foot_bank_r", 0)):
        assert v == 0 or cl[p][0] < v < cl[p][1]
        c.pose(6, props={p: v})
    c.pose(8, zero=[T, LA.format("l"), LA.format("r")])
    c.end()

    # 9 leg FK (leg_ik_fk = 0 via snap TO_FK), thigh / calf / foot FK motion, back to IK via snap TO_IK
    c.seg("leg_fk", ["leg_fk_l", "leg_fk_r", "snap_leg_l", "snap_leg_r"], [])
    c.op("snap_ikfk", chain="leg_l", direction="TO_FK")
    c.op("snap_ikfk", dt=0, chain="leg_r", direction="TO_FK")
    c.pose(8, set=[(TH.format("l"), "rot", "X", -45), (CA.format("l"), "rot", "X", 60), (FT.format("l"), "rot", "X", -20),
                   (TH.format("r"), "rot", "X", 30), (TH.format("r"), "rot", "Z", 20), (CA.format("r"), "rot", "X", 20),
                   (FT.format("r"), "rot", "X", 15), (FT.format("r"), "rot", "Z", 10)])
    c.pose(8, set=[(TH.format("l"), "rot", "X", 20), (TH.format("l"), "rot", "Y", 20), (CA.format("l"), "rot", "X", 30),
                   (FT.format("l"), "rot", "X", 0), (FT.format("l"), "rot", "Z", -15),
                   (TH.format("r"), "rot", "X", -60), (TH.format("r"), "rot", "Z", 0), (CA.format("r"), "rot", "X", 80),
                   (FT.format("r"), "rot", "X", -20), (FT.format("r"), "rot", "Z", 0)])
    c.pose(8, zero=[n.format(x) for n in (TH, CA, FT) for x in "lr"])
    c.op("snap_ikfk", chain="leg_l", direction="TO_IK")
    c.op("snap_ikfk", dt=0, chain="leg_r", direction="TO_IK")
    c.end()

    # 10 rest: reset_rig, held to the last frame
    c.seg("rest", ["rest"], [])
    c.op("reset_rig", dt=6)
    c.op("reset_rig", dt=6)
    c.end()


# ---------------------------------------------------------------- post / checks
def channelbag(arm):
    ad = arm.animation_data
    return anim_utils.action_get_channelbag_for_slot(ad.action, ad.action_slot)


def set_interpolation(c, fcs):
    ints = {k for k in c.prop_names if isinstance(c.props[k], int)}
    ikfk = {e["prop"] for e in c.man["ikfk"].values()}
    n = defaultdict(int)
    for fc in fcs:
        m = re.fullmatch(r'pose\.bones\["PROPS"\]\["([^"]+)"\]', fc.data_path)
        if not m:
            continue
        k = m.group(1)
        for kp in fc.keyframe_points:
            f = int(round(kp.co[0]))
            if k in ints or (k in ikfk and f not in c.blend_starts.get(k, ())):
                kp.interpolation = "CONSTANT"
            else:
                kp.interpolation = "BEZIER"
            n[kp.interpolation] += 1
        fc.update()
    return dict(n)


def keyed_report(fcs):
    bones, other = defaultdict(set), []
    for fc in fcs:
        m = re.match(r'pose\.bones\["([^"]+)"\]\.?(.*)', fc.data_path)
        if m:
            bones[m.group(1)].add(m.group(2) or fc.data_path)
        else:
            other.append(fc.data_path)
    bad = sorted(b for b in bones if not (b.startswith("CTRL_") or b == PROPS))
    return bones, bad, other


def range_report(c, fcs):
    out = []
    for fc in fcs:
        m = re.fullmatch(r'pose\.bones\["(CTRL_[^"]+)"\]\.(location|rotation_euler|scale)', fc.data_path)
        if not m:
            continue
        name, path = m.groups()
        ax = AXES[fc.array_index]
        for kp in fc.keyframe_points:
            f, v = int(round(kp.co[0])), kp.co[1]
            if path == "scale":
                if abs(v - 1.0) > 1e-4:
                    out.append((name, "scale", ax, f, v, (1, 1)))
                continue
            ch = "loc" if path == "location" else "rot"
            val = v if ch == "loc" else math.degrees(v)
            rng = c.sweep.get((name, ch, ax))
            if rng is None:
                if abs(val) > (1e-4 if ch == "loc" else 0.01):
                    out.append((name, ch, ax, f, val, "not in sweep"))
            elif not rng[0] - 1e-4 <= val <= rng[1] + 1e-4:
                out.append((name, ch, ax, f, val, rng))
    return out


def sample_def(arm, names, frames):
    out = {}
    for f in frames:
        bpy.context.scene.frame_set(f)
        out[f] = {n: (arm.matrix_world @ arm.pose.bones[n].matrix).copy() for n in names}
    return out


def canon_len(canon, name):
    b = next(x for x in canon["bones"] if x["name"] == name)
    return math.dist(b["head"], b["tail"])


def mdiff(a, b):
    """(mm, deg); angle = 2 atan2(|v|, |w|) of the relative quaternion (2 acos(|q1.q2|) reads ~0.08 deg of float32
    noise for identical rotations)."""
    q = a.to_quaternion().conjugated() @ b.to_quaternion()
    ang = 2.0 * math.atan2(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z), abs(q.w))
    return (a.translation - b.translation).length * 1000.0, math.degrees(ang)


# ---------------------------------------------------------------- main
def main():
    arm = bpy.data.objects[ARM]
    man = goblib.load_json("ctrl_manifest.json")
    canon = goblib.load_json("canonical_skeleton.json")
    def_names = [b["name"] for b in canon["bones"]]
    scene = bpy.context.scene
    if bpy.context.object and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.objects.active = arm
    arm.data.pose_position = "POSE"

    # operators: blend text goblin_rig_ui.py as a module (factory startup does not auto-run blend scripts)
    mod = bpy.data.texts[TEXT_UI].as_module()
    mod.register()
    reg = {}
    for o in OPS:
        try:
            getattr(bpy.ops.goblin, o).get_rna_type()
            reg[o] = True
        except Exception:
            reg[o] = False
    print(f"[s07a] OPERATORS from blend text {TEXT_UI}: {reg}")
    if not all(reg.values()):
        raise RuntimeError("goblin operators not registered")

    # action + slot (Blender 5.1 slotted actions)
    prev = arm.animation_data.action.name if arm.animation_data and arm.animation_data.action else None
    old = bpy.data.actions.get(ACTION)
    if old is not None:
        bpy.data.actions.remove(old)
    ad = arm.animation_data or arm.animation_data_create()
    act = bpy.data.actions.new(ACTION)
    act.use_fake_user = True
    slot = act.slots.new(id_type="OBJECT", name=ARM)
    ad.action = act
    ad.action_slot = slot
    print(f"[s07a] ACTION {act.name}: previous action on {ARM} = {prev}; slot {slot.identifier!r} "
          f"(handle {slot.handle}) assigned: animation_data.action_slot = {ad.action_slot.identifier!r}; "
          f"drivers kept {len(ad.drivers)}")

    scene.render.fps = FPS
    scene.render.fps_base = 1.0
    ts = scene.tool_settings
    autokey0 = ts.use_keyframe_insert_auto
    ts.use_keyframe_insert_auto = True

    c = Clip(arm, man)
    build(c)
    frame_end = c.f
    ts.use_keyframe_insert_auto = autokey0
    scene.frame_start, scene.frame_end = FRAME_START, frame_end
    act.use_frame_range = True
    act.frame_start, act.frame_end = FRAME_START, frame_end

    fcs = channelbag(arm).fcurves
    interp = set_interpolation(c, fcs)
    print(f"[s07a] PROPS interpolation: {interp}; ik_fk blend start keys {dict((k, sorted(v)) for k, v in c.blend_starts.items())}")

    # ---- self-checks
    bones, bad, other = keyed_report(fcs)
    print(f"[s07a] CHECK keyed bones: {len(bones)} ({sum(1 for b in bones if b.startswith('CTRL_'))} CTRL_* + "
          f"{'PROPS' if PROPS in bones else 'no PROPS'}); other bones keyed: {bad}; non-bone fcurves: {other}; "
          f"fcurves {len(fcs)}; key frames {len({int(round(k.co[0])) for fc in fcs for k in fc.keyframe_points})}")
    print(f"[s07a] CHECK frame range {FRAME_START}..{frame_end} ({frame_end - FRAME_START + 1} frames @ {FPS} fps), "
          f"action frame_range {tuple(act.frame_range)}")
    print("[s07a] SEGMENTS name | frames | covers | foot_lock")
    for s in c.segments:
        print(f"[s07a]   {s['name']} | {s['start']}-{s['end']} | {','.join(s['covers'])} | {','.join(s['foot_lock']) or '-'}")
    rr = range_report(c, fcs)
    print(f"[s07a] CHECK keyed CTRL values outside manifest sweep (operator results; own writes asserted): {len(rr)}"
          + "".join(f"\n[s07a]   {n} {ch} {ax} f{f}: {v:+.4f} (sweep {r})" for n, ch, ax, f, v, r in rr[:20]))

    frames = list(range(FRAME_START, frame_end + 1))
    S = sample_def(arm, def_names, frames)
    scene.frame_set(frame_end)
    worst, wn = 0.0, None
    for b in canon["bones"]:
        m = arm.pose.bones[b["name"]].matrix
        r = b["rest_matrix"]
        e = max(abs(m[i][j] - r[i][j]) for i in range(4) for j in range(4))
        if e > worst:
            worst, wn = e, b["name"]
    print(f"[s07a] CHECK last frame {frame_end}: DEF pose matrix vs canonical rest max abs diff {worst:.3e} "
          f"(bone {wn}; target <= {REST_TOL:g})")

    for s in c.segments:
        for x in s["foot_lock"]:
            n = f"foot_{x}"
            m0 = S[s["start"]][n]
            dp, dr = zip(*(mdiff(S[f][n], m0) for f in range(s["start"], s["end"] + 1)))
            print(f"[s07a] CHECK foot_lock {s['name']} {n}: max DEF world move {max(dp):.4f} mm / {max(dr):.4f} deg "
                  f"(frames {s['start']}-{s['end']}, vs start frame)")
    for s in c.segments:
        if s["name"] != "pelvis":
            continue
        pbs = arm.pose.bones
        mw = arm.matrix_world
        leg = {x: canon_len(canon, f"thigh_{x}") + canon_len(canon, f"calf_{x}") for x in "lr"}
        for key, (a, b) in (("rotation part", c.marks["pelvis_rot"]), ("loc part", c.marks["pelvis_loc"])):
            scene.frame_set(a)
            w0 = mw @ pbs["spine_01"].head
            mv = []
            for f in range(a, b + 1):
                scene.frame_set(f)
                mv.append((mw @ pbs["spine_01"].head - w0).length * 1000.0)
            print(f"[s07a] CHECK pelvis {key} frames {a}-{b} (COG held at -50 mm): DEF spine_01 head (waist) world "
                  f"move max {max(mv):.4f} mm (frame {a + mv.index(max(mv))}; vs frame {a})")
        ha = {x: [] for x in "lr"}
        for f in range(s["start"], s["end"] + 1):
            scene.frame_set(f)
            for x in "lr":
                ha[x].append((mw @ pbs[f"thigh_{x}"].head - mw @ pbs[f"foot_{x}"].head).length * 1000.0)
        print(f"[s07a] CHECK pelvis segment {s['start']}-{s['end']}: DEF hip (thigh head) -> ankle (foot head) max "
              + ", ".join(f"{x} {max(ha[x]):.2f} mm (frame {s['start'] + ha[x].index(max(ha[x]))}, canonical "
                          f"thigh+calf {leg[x] * 1000:.2f} mm)" for x in "lr"))
    for e in c.events:
        if e["kind"] not in ("switch", "snap"):
            print(f"[s07a] EVENT f{e['frame']} {e['op']} ({e['segment']})")
            continue
        f = e["frame"]
        worst_p = max((mdiff(S[f][n], S[f - 1][n]) for n in def_names), key=lambda t: t[0])
        worst_r = max((mdiff(S[f][n], S[f - 1][n]) for n in def_names), key=lambda t: t[1])
        wpn = max(def_names, key=lambda n: mdiff(S[f][n], S[f - 1][n])[0])
        wrn = max(def_names, key=lambda n: mdiff(S[f][n], S[f - 1][n])[1])
        print(f"[s07a] EVENT f{f} {e['op']} {e['args']} ({e['segment']}): DEF f{f - 1}->f{f} max "
              f"{worst_p[0]:.4f} mm ({wpn}) / {worst_r[1]:.4f} deg ({wrn})")

    # ---- manifest
    out = {
        "_doc": {
            "clip": f"Action '{ACTION}' on {ARM} (source gob_r04_ctrl.blend, script s07a_rigtest.py). Keys only on "
                    "CTRL_* pose bones (location, rotation_euler, scale) and PROPS custom properties; every key frame "
                    "keys the full control state. DEF / MCH are never keyed.",
            "action_slot": f"Blender 5.1 slotted action: act.slots.new(id_type='OBJECT', name='{ARM}') -> identifier "
                           f"'{slot.identifier}'; arm.animation_data.action = act; arm.animation_data.action_slot = "
                           "slot. F-curves live in the slot's channelbag (bpy_extras.anim_utils."
                           "action_get_channelbag_for_slot(act, slot).fcurves). use_fake_user True, use_frame_range "
                           "True (frame_start..frame_end).",
            "frames": "frame_start / frame_end inclusive; segments share their boundary frame (end of one = start of "
                      "the next); segment start/end are key frames.",
            "covers": "item keys tested inside the segment (G7.1 checklist).",
            "foot_lock": "a side is listed only when, for the whole segment, that leg is IK and its CTRL_foot_ik_x "
                         "transform and all four roll props (foot_roll / foot_bank / heel_twist / toe_twist) are "
                         "constant (CTRL_root also constant), so its DEF foot must stay fixed in the world; foot roll / "
                         "bank sweep segments are never marked (the ankle moves by design, planted quality = G6.7).",
            "events": "operator calls: kind switch / snap (last key at frame-1 in the old space / mode, operator at "
                      "frame, full key at frame) plus ready_pose / reset_rig. Two events on the same frame = both "
                      "chains at once.",
            "interpolation": "PROPS: int spaces CONSTANT; ik_fk CONSTANT except the keys that start a blend (BEZIER, "
                             "arm_ik_fk_x 0 -> 1 and 1 -> 0 in arm_ik_fk); foot_* BEZIER. CTRL: BEZIER auto-clamped.",
            "ranges": "values written by the script are inside ctrl_manifest sweep / clamps (asserted); snap / switch "
                      "compensation values come from the operators. COG down -0.05 m (>= -0.08), foot IK not lifted.",
            "segments": "root: CTRL_root forward/side move + yaw (feet follow the root). cog: CTRL_torso down 50 mm, "
                        "side / fore-aft 40 mm, rot X +20/-15, Y +-20, Z +-10 (hip -> ankle <= 139 mm). pelvis (T31c, "
                        "waist pivot of T24c): CTRL_torso loc Y -0.05 first (held), CTRL_pelvis rot X +12/-12 (hips "
                        "back / forward), rot Y +15/-15 (yaw), rot Z +8/-8 (hip sway), then loc (X +0.015, Z +0.015) "
                        "and (X -0.015, Y +0.015 down, Z -0.015), pelvis identity, COG back up; spine controls "
                        "identity; planned hip -> ankle <= 136 mm (leg 151 mm). The rotations keep the waist (DEF "
                        "spine_01 head) still; the loc keys move it by design. "
                        "arm_fk: "
                        "shoulder, upperarm raise / forward / twist, elbow, hand X/Y/Z both arms. arm_ik_fk: FK pose, blend 0 -> 1 onto the IK rest, IK hand "
                        "move, snap TO_FK, FK move, snap TO_IK, IK move, blend 1 -> 0, back to rest. head_space: "
                        "chest rot with head in chest space, hold, switch to world, hold, chest / spine move (head "
                        "rotation stays), hold, switch to chest, hold, rest. hand_ik_space_l_weapon: left arm snap "
                        "TO_IK, left IK hand move, switch to weapon space (3) with the club at rest, CTRL_weapon "
                        "socket offset + right arm FK move carry the left hand, club back to rest, switch back to "
                        "world, rest, snap TO_FK. "
                        "weapon_world: weapon_space -> world (3), right arm moves away, club moves in the world, arm "
                        "back to rest, weapon_space -> hand_r (0), CTRL_weapon -> identity (club back in the socket). "
                        "foot_roll_bank_l / _r: ready_pose (CTRL_torso -15 mm, elbows 15 deg), foot_roll / "
                        "foot_bank sweep of one foot about 90 % of its clamp each way, then back to rest. leg_fk: "
                        "snap TO_FK both legs, thigh / calf / foot FK motion, rest, snap TO_IK. rest: reset_rig at "
                        "the last two keys (last frame = canonical rest).",
        },
        "action": ACTION,
        "armature": ARM,
        "slot": slot.identifier,
        "fps": FPS,
        "frame_start": FRAME_START,
        "frame_end": frame_end,
        "segments": c.segments,
        "events": c.events,
    }
    p = goblib.save_json(MANIFEST_OUT, out)
    print(f"[s07a] OUTPUT {p}")

    scene.frame_set(FRAME_START)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s07a] OUTPUT {OUT_BLEND}")


if __name__ == "__main__":
    goblib.run_main(main)
