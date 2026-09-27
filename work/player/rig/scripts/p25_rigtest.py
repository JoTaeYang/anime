"""p25_rigtest.py - T250 (P2.6): test clip `rigtest` on PL_rig -> pl_r05_rigtest.blend + data/rigtest_manifest.json.

Input : work/player/rig/pl_r04_ctrl.blend (P2c rig; opened as the main file, never saved over),
        work/player/rig/data/ctrl_manifest.json (sweep ranges / clamps), canonical_skeleton.json (rest check)
Output: work/player/rig/pl_r05_rigtest.blend (save copy), work/player/rig/data/rigtest_manifest.json

CLI:  blender --background --factory-startup work/player/rig/pl_r04_ctrl.blend --python p25_rigtest.py

Copied / adapted from work/goblin_swing/rig/scripts/s07a_rigtest.py (T31; goblin files unchanged): Clip builder (every
key frame keys the full control state: all CTRL_* location / Euler rotation / scale + all PROPS), operators from the
blend text (player.* namespace, T240 addon), switch / snap technique (last key at f-1 = old space / mode, operator at
f, full key at f), PROPS interpolation (int spaces CONSTANT, ik_fk CONSTANT except blend starts, foot_* BEZIER), value
asserts against ctrl_manifest sweeps / clamps, self-checks (keyed bones, range, last frame vs canonical rest, DEF foot
movement in foot_lock segments, DEF jump f-1 -> f at every switch / snap).
Player adaptations: bone names (Foot_L, Spine_01 ...), player FK signs (thigh X+ = hip flexion, calf X- = knee flexion,
abduction thigh_l Z+ / thigh_r Z-), two weapon controls (CTRL_weapon_r / _l with weapon_space_r / _l = hand / torso /
world), left hand IK weapon space -> CTRL_weapon_r, a locomotion-like leg section (IK steps, COG bob / sway / fast yaw)
to excite the skirt springs, and the root locked outside the first segment (root: CTRL_root only moves in segment 1 and
is identity afterwards). render_frames (Unity capture) are listed in the manifest.
"""
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import bpy
from bpy_extras import anim_utils

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
IN_BLEND = RIG / "pl_r04_ctrl.blend"
OUT_BLEND = RIG / "pl_r05_rigtest.blend"
MANIFEST_OUT = RIG / "data" / "rigtest_manifest.json"
CTRL_MAN = RIG / "data" / "ctrl_manifest.json"
CANON = RIG / "data" / "canonical_skeleton.json"
ARM = "PL_rig"
ACTION = "rigtest"
FPS = 24
FRAME_START = 1
TEXT_UI = "player_rig_ui.py"
PROPS = "PROPS"
OPS = ("snap_ikfk", "switch_space", "reset_rig", "ready_pose")
REST_TOL = 1e-4
EPS = 1e-6
AXES = "XYZ"


def log(msg):
    print(f"[p25] {msg}")
    sys.stdout.flush()


def rot_path(pb):
    return {"QUATERNION": "rotation_quaternion", "AXIS_ANGLE": "rotation_axis_angle"}.get(pb.rotation_mode,
                                                                                         "rotation_euler")


class Clip:
    def __init__(self, arm, man):
        self.arm, self.man = arm, man
        self.pbs = arm.pose.bones
        self.props = self.pbs[PROPS]
        self.ctrls = [pb.name for pb in self.pbs if pb.name.startswith("CTRL_")]
        self.prop_names = [k for k in self.props.keys() if isinstance(self.props[k], (int, float))]
        self.sweep = {(c, s["channel"], s["axis"]): (s["min"], s["max"])
                      for c, e in man["controls"].items() for s in e["sweep"]}
        self.f = None
        self.events = []
        self.blend_starts = defaultdict(set)
        self.segments = []
        self._seg = None
        self.marks = {}

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

    def start(self):
        self.goto(FRAME_START)
        self.refresh()
        self.key_all(FRAME_START)

    def pose(self, dt, set=(), add=(), zero=(), props=None, blend=()):
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
        f = self.f + dt
        if dt:
            self.goto(f)
        r = getattr(bpy.ops.player, name)(**kw)
        if "FINISHED" not in r:
            raise RuntimeError(f"player.{name}({kw}) returned {r}")
        self.refresh()
        self.key_all(f)
        kind = {"switch_space": "switch", "snap_ikfk": "snap"}.get(name, name)
        self.events.append({"frame": f, "kind": kind, "op": f"player.{name}", "args": dict(kw),
                            "segment": self._seg["name"] if self._seg else None})

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
    R, T, H, CH, S1, P = "CTRL_root", "CTRL_torso", "CTRL_head", "CTRL_chest", "CTRL_spine_01", "CTRL_pelvis"
    WR, WL = "CTRL_weapon_r", "CTRL_weapon_l"
    SH, UA, LA, HF = "CTRL_shoulder_{}", "CTRL_upperarm_fk_{}", "CTRL_lowerarm_fk_{}", "CTRL_hand_fk_{}"
    HI, EP = "CTRL_hand_ik_{}", "CTRL_elbow_pole_{}"
    TH, CA, FT, FI = "CTRL_thigh_fk_{}", "CTRL_calf_fk_{}", "CTRL_foot_fk_{}", "CTRL_foot_ik_{}"
    ARMP = ["arm_ik_fk_l", "arm_ik_fk_r"]
    arm_fk_all = [n.format(x) for n in (SH, UA, LA, HF) for x in "lr"]
    render = []
    c.start()

    # 1 root move + yaw (feet follow the root: the foot IK controls are children of CTRL_root)
    c.seg("root", ["root"], [])
    c.pose(6, set=[(R, "loc", "Z", 0.3)])
    c.pose(6, set=[(R, "loc", "X", 0.2), (R, "rot", "Y", 45)])
    c.pose(6, set=[(R, "loc", "Z", -0.2), (R, "rot", "Y", -60)])
    c.pose(6, set=[(R, "loc", "X", -0.15), (R, "rot", "Y", 20)])
    c.pose(6, zero=[R])
    c.end()

    # 2 COG, legs IK, feet fixed; down 50 mm first (legs 2 x 0.160 m; hip -> ankle then <= 0.30 m)
    c.seg("cog", ["cog"], ["l", "r"])
    c.pose(6, set=[(T, "loc", "Y", -0.05)])
    c.pose(6, set=[(T, "loc", "X", 0.04)])
    c.pose(6, set=[(T, "loc", "X", -0.04), (T, "loc", "Z", 0.04)])
    render.append(c.f)
    c.pose(6, set=[(T, "loc", "X", 0.0), (T, "loc", "Z", -0.04), (T, "rot", "X", 20)])
    c.pose(6, set=[(T, "loc", "Z", 0.0), (T, "rot", "X", -15), (T, "rot", "Y", 20)])
    c.pose(6, set=[(T, "rot", "X", 0), (T, "rot", "Y", -20)])
    c.pose(6, set=[(T, "rot", "Y", 0), (T, "rot", "Z", 10)])
    c.pose(6, set=[(T, "rot", "Z", -10)])
    c.pose(6, zero=[T])
    c.end()

    # 2b pelvis about the waist pivot (CTRL_pelvis head = waist z 0.517; hips 0.092 m below: 1.6 mm / deg)
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

    # 3 arm FK ROM, both arms (raise upperarm_l Z+ / upperarm_r Z-, X+ forward, lowerarm X+ flexion)
    c.seg("arm_fk", ["arm_fk_l", "arm_fk_r"], ["l", "r"])
    c.pose(6, set=both(SH, "rot", "Z", 15, -15) + both(UA, "rot", "Z", 80, -80))
    render.append(c.f)
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

    # 4 arm IK move, IK/FK blend, snaps (both arms)
    c.seg("arm_ik_fk", ["arm_ik_l", "arm_ik_r", "ikfk_blend_l", "ikfk_blend_r", "snap_arm_l", "snap_arm_r"],
          ["l", "r"])
    c.pose(6, set=both(UA, "rot", "X", 40) + both(LA, "rot", "X", 50), blend=ARMP)
    c.pose(10, props={p: 1.0 for p in ARMP})
    c.pose(8, set=both(HI, "loc", "Y", -0.12) + both(HI, "loc", "Z", 0.08) + both(HI, "rot", "X", 20))
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

    # 5 head space chest -> world -> chest
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

    # 6 left hand IK in weapon space (3 -> CTRL_weapon_r) while the right weapon moves
    c.seg("hand_ik_space_l_weapon", ["hand_ik_space_l_weapon", "weapon_r_offset", "snap_arm_l"], ["l", "r"])
    c.op("snap_ikfk", chain="arm_l", direction="TO_IK")
    c.pose(8, set=[(HI.format("l"), "loc", "Y", -0.10), (HI.format("l"), "loc", "Z", -0.10)])
    c.hold(6)
    c.op("switch_space", prop="hand_ik_space_l", value=3)
    c.hold(6)
    c.pose(8, set=[(WR, "loc", "X", 0.08), (WR, "loc", "Z", 0.05), (WR, "rot", "X", 10), (UA.format("r"), "rot", "X", 20)])
    c.pose(8, set=[(WR, "loc", "X", -0.05), (WR, "loc", "Z", 0.0), (WR, "rot", "X", -10), (UA.format("r"), "rot", "X", 0),
                   (UA.format("r"), "rot", "Z", -15)])
    c.pose(8, zero=[WR, UA.format("r")])
    c.hold(6)
    c.op("switch_space", prop="hand_ik_space_l", value=0)
    c.hold(6)
    c.pose(8, zero=[HI.format("l"), EP.format("l")])
    c.op("snap_ikfk", chain="arm_l", direction="TO_FK")
    c.end()

    # 7 right weapon in world space (2), left weapon in torso space (1)
    c.seg("weapon_spaces", ["weapon_r_world", "weapon_l_torso", "weapon_return"], ["l", "r"])
    c.op("switch_space", prop="weapon_space_r", value=2)
    c.op("switch_space", dt=0, prop="weapon_space_l", value=1)
    c.hold(6)
    c.pose(8, set=[(UA.format("r"), "rot", "X", 45), (LA.format("r"), "rot", "X", 60), (WR, "loc", "X", 0.15),
                   (WR, "rot", "X", 45), (UA.format("l"), "rot", "Z", 40), (WL, "loc", "Z", 0.10), (WL, "rot", "Y", 30)])
    c.pose(8, set=[(WR, "loc", "Y", -0.2), (WR, "rot", "Z", 30)])
    c.pose(8, zero=[UA.format("r"), LA.format("r"), UA.format("l")])
    c.hold(6)
    c.op("switch_space", prop="weapon_space_r", value=0)
    c.op("switch_space", dt=0, prop="weapon_space_l", value=0)
    c.hold(6)
    c.pose(8, zero=[WR, WL])
    c.end()

    # 8 foot roll / bank inside the clamps from the ready pose, one foot at a time (ankle moves by design)
    cl = c.man["clamps"]
    c.seg("foot_roll_bank_l", ["ready_pose", "foot_roll_l", "foot_bank_l"], [])
    c.op("ready_pose", dt=8)
    render.append(c.f)
    for p, v in (("foot_roll_l", -35), ("foot_roll_l", 45), ("foot_roll_l", 0),
                 ("foot_bank_l", -22), ("foot_bank_l", 22), ("foot_bank_l", 0)):
        assert v == 0 or cl[p][0] < v < cl[p][1]
        c.pose(6, props={p: v})
    c.end()
    c.seg("foot_roll_bank_r", ["foot_roll_r", "foot_bank_r"], [])
    for p, v in (("foot_roll_r", -35), ("foot_roll_r", 45), ("foot_roll_r", 0),
                 ("foot_bank_r", -22), ("foot_bank_r", 22), ("foot_bank_r", 0)):
        assert v == 0 or cl[p][0] < v < cl[p][1]
        c.pose(6, props={p: v})
    c.pose(8, zero=[T, LA.format("l"), LA.format("r")])
    c.end()

    # 9 leg FK (snap TO_FK), thigh / calf / foot FK (player signs), back to IK (snap TO_IK)
    c.seg("leg_fk", ["leg_fk_l", "leg_fk_r", "snap_leg_l", "snap_leg_r"], [])
    c.op("snap_ikfk", chain="leg_l", direction="TO_FK")
    c.op("snap_ikfk", dt=0, chain="leg_r", direction="TO_FK")
    c.pose(8, set=[(TH.format("l"), "rot", "X", 45), (CA.format("l"), "rot", "X", -60), (FT.format("l"), "rot", "X", -20),
                   (TH.format("r"), "rot", "X", -30), (TH.format("r"), "rot", "Z", -20), (CA.format("r"), "rot", "X", -20),
                   (FT.format("r"), "rot", "X", 15), (FT.format("r"), "rot", "Z", 10)])
    render.append(c.f)
    c.pose(8, set=[(TH.format("l"), "rot", "X", -20), (TH.format("l"), "rot", "Y", 20), (CA.format("l"), "rot", "X", -30),
                   (FT.format("l"), "rot", "X", 0), (FT.format("l"), "rot", "Z", -15),
                   (TH.format("r"), "rot", "X", 60), (TH.format("r"), "rot", "Z", 0), (CA.format("r"), "rot", "X", -80),
                   (FT.format("r"), "rot", "X", -20), (FT.format("r"), "rot", "Z", 0)])
    c.pose(8, zero=[n.format(x) for n in (TH, CA, FT) for x in "lr"])
    c.op("snap_ikfk", chain="leg_l", direction="TO_IK")
    c.op("snap_ikfk", dt=0, chain="leg_r", direction="TO_IK")
    c.end()

    # 10 locomotion-like section (legs IK): COG down 30 mm, alternating steps (foot lift / forward / plant), COG bob
    #    and sway with the steps, then fast COG yaw swings and a stop to excite the skirt springs
    c.seg("locomotion", ["locomotion", "skirt_excitation"], [])
    L, Rr = FI.format("l"), FI.format("r")
    c.pose(6, set=[(T, "loc", "Y", -0.03)])
    for i in range(2):
        c.pose(4, set=[(L, "loc", "Z", 0.05), (L, "loc", "Y", 0.03), (Rr, "loc", "Y", -0.03), (T, "loc", "X", -0.02),
                       (T, "loc", "Y", -0.02), (T, "rot", "Y", 8)])
        c.pose(4, set=[(L, "loc", "Z", 0.0), (L, "loc", "Y", 0.07), (Rr, "loc", "Y", -0.05), (T, "loc", "Y", -0.035),
                       (T, "loc", "Z", 0.02)])
        if i == 0:
            render.append(c.f)
        c.pose(4, set=[(Rr, "loc", "Z", 0.05), (Rr, "loc", "Y", 0.03), (L, "loc", "Y", -0.03), (T, "loc", "X", 0.02),
                       (T, "loc", "Y", -0.02), (T, "rot", "Y", -8), (T, "loc", "Z", 0.0)])
        c.pose(4, set=[(Rr, "loc", "Z", 0.0), (Rr, "loc", "Y", 0.07), (L, "loc", "Y", -0.05), (T, "loc", "Y", -0.035),
                       (T, "loc", "Z", 0.02)])
    c.pose(6, set=[(L, "loc", "Y", 0.0), (Rr, "loc", "Y", 0.0), (T, "loc", "X", 0.0), (T, "loc", "Z", 0.0),
                   (T, "loc", "Y", -0.03), (T, "rot", "Y", 0)])
    for v in (30, -30, 25, -20, 0):
        c.pose(3, set=[(T, "rot", "Y", v)])
    render.append(c.f)
    c.pose(4, set=[(T, "rot", "Z", 12), (T, "loc", "X", 0.03)])
    c.pose(3, set=[(T, "rot", "Z", -12), (T, "loc", "X", -0.03)])
    c.pose(3, set=[(T, "rot", "Z", 0), (T, "loc", "X", 0.0)])
    c.hold(18)
    c.pose(6, zero=[T, L, Rr])
    c.end()

    # 11 rest: reset_rig, held to the last frame
    c.seg("rest", ["rest"], [])
    c.op("reset_rig", dt=6)
    c.op("reset_rig", dt=6)
    c.end()
    return render


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


def mdiff(a, b):
    q = a.to_quaternion().conjugated() @ b.to_quaternion()
    ang = 2.0 * math.atan2(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z), abs(q.w))
    return (a.translation - b.translation).length * 1000.0, math.degrees(ang)


def main():
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    arm = bpy.data.objects[ARM]
    man = json.loads(CTRL_MAN.read_text(encoding="utf-8"))
    canon = json.loads(CANON.read_text(encoding="utf-8"))
    def_names = [b["name"] for b in canon["bones"]]
    scene = bpy.context.scene
    bpy.context.view_layer.objects.active = arm
    arm.data.pose_position = "POSE"
    mod = bpy.data.texts[TEXT_UI].as_module()
    mod.register()
    reg = {}
    for o in OPS:
        try:
            getattr(bpy.ops.player, o).get_rna_type()
            reg[o] = True
        except Exception:
            reg[o] = False
    log(f"OPERATORS from blend text {TEXT_UI}: {reg}")
    if not all(reg.values()):
        raise RuntimeError("player operators not registered")
    old = bpy.data.actions.get(ACTION)
    if old is not None:
        bpy.data.actions.remove(old)
    ad = arm.animation_data or arm.animation_data_create()
    act = bpy.data.actions.new(ACTION)
    act.use_fake_user = True
    slot = act.slots.new(id_type="OBJECT", name=ARM)
    ad.action = act
    ad.action_slot = slot
    log(f"ACTION {act.name}: slot {slot.identifier!r}; drivers kept {len(ad.drivers)}")
    scene.render.fps = FPS
    scene.render.fps_base = 1.0
    ts = scene.tool_settings
    autokey0 = ts.use_keyframe_insert_auto
    ts.use_keyframe_insert_auto = True
    c = Clip(arm, man)
    render_frames = build(c)
    frame_end = c.f
    ts.use_keyframe_insert_auto = autokey0
    scene.frame_start, scene.frame_end = FRAME_START, frame_end
    act.use_frame_range = True
    act.frame_start, act.frame_end = FRAME_START, frame_end
    fcs = channelbag(arm).fcurves
    interp = set_interpolation(c, fcs)
    log(f"PROPS interpolation: {interp}; ik_fk blend start keys {dict((k, sorted(v)) for k, v in c.blend_starts.items())}")

    bones, bad, other = keyed_report(fcs)
    log(f"CHECK keyed bones: {len(bones)} ({sum(1 for b in bones if b.startswith('CTRL_'))} CTRL_* + "
        f"{'PROPS' if PROPS in bones else 'no PROPS'}); other bones keyed: {bad}; non-bone fcurves: {other}; fcurves "
        f"{len(fcs)}; key frames {len({int(round(k.co[0])) for fc in fcs for k in fc.keyframe_points})}")
    log(f"CHECK frame range {FRAME_START}..{frame_end} ({frame_end - FRAME_START + 1} frames @ {FPS} fps)")
    log("SEGMENTS name | frames | covers | foot_lock")
    for s in c.segments:
        log(f"  {s['name']} | {s['start']}-{s['end']} | {','.join(s['covers'])} | {','.join(s['foot_lock']) or '-'}")
    rr = range_report(c, fcs)
    log(f"CHECK keyed CTRL values outside manifest sweep (operator results; own writes asserted): {len(rr)}"
        + "".join(f"\n[p25]   {n} {ch} {ax} f{f}: {v:+.4f} (sweep {r})" for n, ch, ax, f, v, r in rr[:20]))
    frames = list(range(FRAME_START, frame_end + 1))
    S = {}
    root_moves = []
    for f in frames:
        scene.frame_set(f)
        S[f] = {n: (arm.matrix_world @ arm.pose.bones[n].matrix).copy() for n in def_names}
    scene.frame_set(frame_end)
    worst, wn = 0.0, None
    for b in canon["bones"]:
        m = arm.pose.bones[b["name"]].matrix
        r = b["rest_matrix"]
        e = max(abs(m[i][j] - r[i][j]) for i in range(4) for j in range(4))
        if e > worst:
            worst, wn = e, b["name"]
    log(f"CHECK last frame {frame_end}: DEF vs canonical rest max abs diff {worst:.3e} ({wn}; target <= {REST_TOL:g})")
    root_seg = c.segments[0]
    for f in frames:
        if f > root_seg["end"]:
            root_moves.append(mdiff(S[f]["Root"], S[FRAME_START]["Root"])[0])
    log(f"CHECK root locked after segment root (frames {root_seg['end'] + 1}..{frame_end}): DEF Root world move max "
        f"{max(root_moves):.6f} mm")
    for s in c.segments:
        for x in s["foot_lock"]:
            n = f"Foot_{x.upper()}"
            m0 = S[s["start"]][n]
            dp, dr = zip(*(mdiff(S[f][n], m0) for f in range(s["start"], s["end"] + 1)))
            log(f"CHECK foot_lock {s['name']} {n}: max DEF world move {max(dp):.4f} mm / {max(dr):.4f} deg")
    for e in c.events:
        if e["kind"] not in ("switch", "snap"):
            log(f"EVENT f{e['frame']} {e['op']} ({e['segment']})")
            continue
        f = e["frame"]
        wp = max(def_names, key=lambda n: mdiff(S[f][n], S[f - 1][n])[0])
        wr = max(def_names, key=lambda n: mdiff(S[f][n], S[f - 1][n])[1])
        log(f"EVENT f{f} {e['op']} {e['args']} ({e['segment']}): DEF f{f - 1}->f{f} max "
            f"{mdiff(S[f][wp], S[f - 1][wp])[0]:.4f} mm ({wp}) / {mdiff(S[f][wr], S[f - 1][wr])[1]:.4f} deg ({wr})")
    out = {
        "_doc": {
            "clip": f"Action '{ACTION}' on {ARM} (source pl_r04_ctrl.blend, script p25_rigtest.py, adapted from goblin "
                    "s07a). Keys only on CTRL_* pose bones and PROPS; every key frame keys the full control state.",
            "frames": "frame_start / frame_end inclusive; segments share their boundary frame.",
            "foot_lock": "a side is listed when that leg is IK and its foot IK control, roll props and CTRL_root are "
                         "constant for the whole segment (DEF foot must stay fixed).",
            "events": "operator calls (switch / snap: last key at frame-1 old space / mode, operator at frame).",
            "root": "CTRL_root moves only in segment 'root' and is identity afterwards (root locked).",
            "render_frames": "frames captured in Unity (front + three_quarter, springs on); chosen inside in-place "
                             "segments (cog, arm_fk, ready pose, leg_fk, locomotion step, after the yaw swings).",
        },
        "action": ACTION, "armature": ARM, "slot": slot.identifier, "fps": FPS,
        "frame_start": FRAME_START, "frame_end": frame_end,
        "segments": c.segments, "events": c.events, "render_frames": render_frames,
    }
    MANIFEST_OUT.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    log(f"OUTPUT {MANIFEST_OUT}; render frames {render_frames}")
    scene.frame_set(FRAME_START)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    log(f"OUTPUT {OUT_BLEND}")
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
