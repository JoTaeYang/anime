"""check_player_clip - Player P3 per-clip Blender checker, C-rows (task T301; spec work/player/d-03-player-anim-tools.md
section 3 rows C1 .. C7; checker side).  Raw evidence only: main judges the gate.

Arguments (after "--"):
  --clip <name>        clip name (required); defaults below are derived from it
  --prefix <X>         criterion id prefix (the clip's spec picks it): ids "<X>.C1" .. ; default: plain "C1" ..
  --work               work/player/rig/anim/pl_a_<clip>.blend   PL_rig + action <clip> + PL_mesh (p30_clip_anim.py)
  --action             <clip>                                   action name in the work blend
  --clip-json          work/player/rig/data/clips/<clip>.json   clip JSON (source of truth for the checker)
  --resolved           work/player/rig/anim/<clip>_resolved.json  production resolved JSON (optional; cross-checked)
  --manifest           work/player/rig/data/ctrl_manifest.json   controls.<CTRL>.sweep, props.spec, clamps, ikfk
  --parts / --loops / --canon   parts.json, retopo_loops.json, canonical_skeleton.json
  --arm PL_rig  --mesh PL_mesh
  --out                work/player/inspect/clips/<clip>/check_clip.json
Clip fields read (resolved JSON first when present, else clip JSON; the source of each is recorded in clip_info):
  fps (default 30), frame_range [a, b] (else frame_start / frame_end, else the action range), loop (bool),
  loop_seam "first_eq_last" | "first_eq_last_plus_1" (default first_eq_last = goblin check_anim_clip C6 convention),
  root_motion "locked" | "in_place_cycle" | "root" (or {"policy": ...}), sweep_overrides {"_reason", CTRL: [{channel,
  axis, min, max}]}, contacts {planted: [{foot "l"|"r", frames [a, b]}] | {"l": [[a, b], ..]}, ground_ranges [[a, b]]}.
  Effective sweeps are derived here (manifest sweep + clip JSON sweep_overrides, widening only, goblin
  s12_clip_anim.apply_sweep_overrides rule); the resolved JSON "sweeps" {CTRL: [..]} is compared with them.
Rows (per frame = every integer frame a..b of the clip, evaluated with scene.frame_set; nothing is saved):
  C1 (blocking) every CTRL_* channel inside its effective sweep (rot = Euler deg in XYZ, loc = m), channels without a
     sweep entry at 0, scale 1; unlisted CTRL_* at identity; PROPS inside clamps / props.spec / UI range; tol 1e-6
  C2 (blocking: branch flips 0 and sub-frame pops 0; the k ratio is report) per DEF bone parent-relative rotation step
     s(f) = angle(L(f-1), L(f)); ratio r = s(f) / max(s(f-1), s(f+1), floor) for s(f) >= MIN_STEP (distribution, draft
     k); branch flip = G6.5b theta sign change with |theta| > 2 deg on both frames (goblin check_anim_clip.py:1757-1768,
     check_p2_ctrl.py:1124-1132); pop = G6.5 bisection of every frame interval whose max DEF step > POP_MIN_DEG, 4
     levels toward the larger half, pop when jump(h/16) > 0.5 jump(h) (check_p2_ctrl.py:1101-1102, 1174-1199)
  C3 (blocking) planted ranges: G6.7 rule (goblin check_anim_clip.py:1776-1800): contact set = shoe_x vertices with z <=
     lowest shoe z + 1 mm; cumulative slip = sum of the mean XY displacement of frame k's contact set k -> k+1 <= 5 mm;
     lowest shoe z in [-1, +3] mm every frame of the range; DEF Foot_x head / tail movement reported
  C4 (blocking) ground: lowest evaluated PL_mesh vertex z >= -1 mm on every frame of contacts.ground_ranges (goblin
     C11 penetration); no ground_ranges declared -> every clip frame; all frames reported
  C5 (blocking) root motion: locked / in_place_cycle -> DEF Root world vs rest and CTRL_root basis vs identity <= 1e-7
     (goblin ROOT_TOL) every frame; in_place_cycle also the loop seam (declared convention) on the 41 DEF world matrices
     <= 0.01 mm / 0.01 deg, every CTRL_* basis <= 0.01 mm / 0.01 deg, PROPS |diff| <= 1e-4; root -> Root path report
  C6 (report) skirt poke-out springs off per frame: check_p2_deform.poke_out on the evaluated PL_mesh in the Pelvis
     rest frame, geometry from check_p2_skirt.Geo (the P2.8b rule, check_p2_skirt.py:231-240); leg vertices split by
     rest z vs the hem (min rest z of tunic hem_outer, check_p2_skirt.py:359): knee = below, thigh = above
  C7 (report) visible self-intersection per frame with the P2.2 rules (check_p2_deform.cross_isect, ShellSet over every
     shell, T223 ceiling rule): fist_x vs head, fist_x vs torso (tunic + belt + pouch), sleeve_x vs scarf (+ scarf_tail),
     arm_x vs tunic; rest (undeformed mesh) beside
  C8 (report, T311; only when the clip JSON has "weapon" {side: name}) weapon vs body: the weapon mesh (appended in
     memory from --weapons-dir/<name>.blend, object from the contract "object" / "mesh" or WPN_sword_placeholder) placed
     at the evaluated WeaponSocket_<side> world @ the --weapon-contract offset (rig/data/weapon_socket_contract.json),
     BVH-overlap triangle pairs vs head / torso / scarf / arms / legs (holding fist excluded), blade clearance mm; draft
     threshold 0 pairs (T321: blocking from Sword_Attack_01 on); a missing weapons blend / contract blocks C8 only.  --weapon-box 1 = self-test seam
     (synthetic in-memory grip + blade box instead of the blend)
  C9 (report, blocking candidate, T321; only when the clip JSON has "match_pose" {clip, frame, at ["first", "last"]})
     DEF world of the 41 bones at the clip's first / last frame vs the reference clip frame (rig/anim/pl_a_<clip>.blend,
     --match-blend) <= 0.01 mm / 0.01 deg, PROPS <= 1e-4
  C10 (report, T321; only when the clip JSON has "events" [{name, frame}]) names hit_start / hit_end once each, inside
     the range, hit_start < hit_end; blade tip (max local Z of the C8 weapon mesh) world speed m/s in the window +- 2
     frames and the peak frame
  C11 (report, calibration, T325) arm vs body: arm parts sleeve_x / arm_x / fist_x (both sides) vs body parts tunic,
     belt, pouch, scarf, scarf_tail, head with the P2.2 visible rule (check_p2_deform.cross_isect, ShellSet over every
     shell) per frame and per (arm part, body part): visible pairs, max visible depth mm, excess over the baseline
     (T327: an approved posed frame = clip JSON c11_baseline {clip, frame}, else match_pose {clip, frame}, else the
     clip's own first frame; the undeformed-rest comparison kept as info); outline-inside: arm part vertices inside
     the torso volume (tunic virtually capped + scarf / scarf_tail closed shells, 3-ray parity,
     check_p2_deform.verts_inside) that are not inside at the baseline; blocking candidate "no visible penetration
     beyond the baseline"
  C12 (blocking, T332: the d-06 section 4 thresholds on every frame; T331 measure; d-06 section 3: sleeve-cap
     contact with scarf / tunic / belt = cloth bunching, report only; only real penetration blocks) penetration over
     the C11 baseline: (a) arm tube and fist (L, R) x tunic, belt, pouch, head excess visible pairs / depth (the C11 pair measure); (b) sleeve (L, R) x head,
     same; (c) arm tube and fist vertices newly inside the tunic volume only (tunic virtually capped, scarf excluded)
     and inside the head volume; per measure the per-frame series and max + frame, and the frames with any measure > 0
Methods reused by import (unchanged): check_p1 (rang, dist, assign_action, fcurves_of), check_p2_deform (Topo, eval_verts,
  poke_out, cross_isect, ShellSet, Evidence helpers), check_p2_skirt (Geo, parts_map).
Run:    blender --background --factory-startup --python check_player_clip.py -- --clip <name> [--prefix X] [overrides]
Exit:   0 evidence written; 2 script error or missing required input (evidence still written with blocked rows).
Coordinates: front -Y, up +Z, character left (_L) +X, ground z = 0.  Lengths in m, reported in mm.
"""
import sys

sys.dont_write_bytecode = True   # no __pycache__ next to the scripts (set before the local imports)

import json  # noqa: E402
import math  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import check_p1 as p1  # noqa: E402
import check_p2_deform as p2d  # noqa: E402
import check_p2_skirt as p2s  # noqa: E402

PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
CHECKER, TASK = "check_player_clip", "T301"
CALIB = p2d.CALIB
rel, jsonable, mm, rnd = p2d.rel, p2d.jsonable, p2d.mm, p2d.rnd

P = {"clip": None, "prefix": None, "work": None, "action": None, "clip_json": None, "resolved": None,
     "manifest": PLAYER_RIG / "data" / "ctrl_manifest.json", "parts": PLAYER_RIG / "data" / "parts.json",
     "loops": PLAYER_RIG / "data" / "retopo_loops.json", "canon": PLAYER_RIG / "data" / "canonical_skeleton.json",
     "arm": "PL_rig", "mesh": "PL_mesh", "out": None,
     "weapons_dir": PLAYER_RIG / "weapons", "weapon_contract": PLAYER_RIG / "data" / "weapon_socket_contract.json",
     "weapon_box": None,   # T311 C8; --weapon-box 1 = self-test seam: synthetic in-memory box instead of the blend
     "match_blend": None}  # T321 C9: reference anim blend (default rig/anim/pl_a_<match_pose.clip>.blend)
PATH_KEYS = ("work", "clip_json", "resolved", "manifest", "parts", "loops", "canon", "out", "weapons_dir",
             "weapon_contract", "match_blend")

FPS_DEFAULT = 30.0
VAL_TOL = 1e-5              # C1 channel / prop tolerance (deg, m, prop units); T302 main decision
C1_NOTE = ("T302 tolerance 1e-5 (main decision 2026-09-27): the T301 rigtest self-test measured snap float noise of "
           "about 2e-6 (f242 lower-arm FK rot Y/Z); 1e-5 = 5x that and far below any visible value")
MIN_STEP_DEG = 0.5          # C2 ratio counted only for steps >= this (noise floor)
NEIGH_FLOOR_DEG = 0.05      # C2 neighbour step floor in the ratio
K_DRAFT = 3.0               # C2 draft k (calibration)
POP_MIN_DEG = 0.1           # C2 bisect intervals whose max DEF step exceeds this
REFINE_LEVELS = 4           # G6.5 (check_p2_ctrl.py:1101)
POP_RATIO = 0.5             # G6.5 (check_p2_ctrl.py:1102)
FLIP_MIN_DEG = 2.0          # G6.5b (check_p2_ctrl.py:1121)
PLANT_SLIP = 0.005          # C3 G6.7 (check_p2_ctrl.py:1273)
PLANT_Z = (-0.001, 0.003)   # C3 G6.7 (check_p2_ctrl.py:1272)
CONTACT_TOL = 0.001         # C3 G6.7 (check_p2_ctrl.py:1275)
GROUND_PEN = 0.001          # C4 goblin C11 penetration (data/clips/death.json ground.penetration_mm)
ROOT_TOL = 1e-7             # C5 goblin check_anim_clip.py:102
SEAM_POS = 1e-5             # C5 0.01 mm
SEAM_ROT = 0.01             # C5 deg
SEAM_PROP = 1e-4            # C5 PROPS (goblin POSE_TOL)
LIST_CAP = 20
C7_ITEMS = (("fist_vs_head", ("fist",), ("head",)), ("fist_vs_torso", ("fist",), ("tunic", "belt", "pouch")),
            ("sleeve_vs_scarf", ("sleeve",), ("scarf", "scarf_tail")), ("arm_vs_tunic", ("arm",), ("tunic",)))
POLICIES = ("locked", "in_place_cycle", "root")
SEAMS = ("first_eq_last", "first_eq_last_plus_1")


class Blocked(Exception):
    pass


def tb_tail(n=3):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


# ---------------------------------------------------------------- clip definition
def pick(res, clip, key, default=None):
    """(value, source) resolved JSON first, else clip JSON, else default."""
    if isinstance(res, dict) and key in res:
        return res[key], "resolved"
    if isinstance(clip, dict) and key in clip:
        return clip[key], "clip_json"
    return default, "default"


def side_of(v):
    s = str(v).strip().lower()
    for pre in ("foot_", "shoe_", "leg_"):
        if s.startswith(pre):
            s = s[len(pre):]
    return {"l": "l", "left": "l", "r": "r", "right": "r"}.get(s)


def ab_of(r):
    if isinstance(r, dict):
        fr = r.get("frames", r.get("range"))
        if isinstance(fr, (list, tuple)) and len(fr) == 2:
            r = fr
        elif "start" in r and "end" in r:
            r = (r["start"], r["end"])
    if isinstance(r, (list, tuple)) and len(r) == 2:
        try:
            return int(r[0]), int(r[1])
        except (TypeError, ValueError):
            return None
    return None


def parse_planted(pl, f0, f1):
    """contacts.planted -> ([{foot, a, b}], issues); goblin parse_ranges forms (check_anim_clip.py:304-354)."""
    out, issues, items = [], [], []
    if pl is None:
        return out, ["contacts.planted missing"]
    if isinstance(pl, dict):
        for k, v in pl.items():
            x = side_of(k)
            if x is None:
                issues.append(f"planted key {k!r} is not a foot")
                continue
            vs = v if isinstance(v, list) and v and isinstance(v[0], (list, tuple, dict)) else [v]
            items += [(x, r) for r in vs]
    elif isinstance(pl, list):
        for r in pl:
            x = side_of((r or {}).get("foot", (r or {}).get("side", ""))) if isinstance(r, dict) else None
            if x is None:
                issues.append(f"planted entry {r!r}: foot / side missing")
                continue
            items.append((x, r))
    else:
        return out, ["contacts.planted is neither list nor object"]
    for x, r in items:
        ab = ab_of(r)
        if ab is None:
            issues.append(f"planted range {r!r} ({x}): no [a, b]")
            continue
        a, b = ab
        if not (f0 <= a <= b <= f1):
            issues.append(f"planted range {x} [{a}, {b}] outside [{f0}, {f1}] or a > b")
            continue
        out.append({"foot": x, "a": a, "b": b})
    return out, issues


def parse_ground(gr, f0, f1):
    out, issues = [], []
    if gr is None:
        return None, []
    if not isinstance(gr, list):
        return [], [f"ground_ranges {gr!r} is not a list"]
    for r in gr:
        ab = ab_of(r)
        if ab is None or not (f0 <= ab[0] <= ab[1] <= f1):
            issues.append(f"ground range {r!r} unparseable or outside [{f0}, {f1}]")
            continue
        out.append(ab)
    return out, issues


def derive_sweeps(man, overrides):
    """manifest controls.<CTRL>.sweep patched by sweep_overrides (widening only, in memory; goblin
    s12_clip_anim.apply_sweep_overrides rule) -> ({CTRL: [{channel, axis, min, max}]}, applied, issues)."""
    sw = {c: [dict(s) for s in (spec.get("sweep") or [])] for c, spec in ((man or {}).get("controls") or {}).items()}
    applied, issues = [], []
    if not overrides:
        return sw, applied, issues
    if not isinstance(overrides, dict):
        return sw, applied, [f"sweep_overrides is not an object: {overrides!r}"]
    if not str(overrides.get("_reason", "")).strip():
        issues.append("sweep_overrides: _reason missing")
    for c, lst in overrides.items():
        if c.startswith("_"):
            continue
        if c not in sw:
            issues.append(f"sweep_overrides: unknown control {c}")
            continue
        for o in lst if isinstance(lst, list) else []:
            s = next((x for x in sw[c] if x["channel"] == o.get("channel") and x["axis"] == o.get("axis")), None)
            if s is None:
                issues.append(f"sweep_overrides: {c} has no sweep entry {o.get('channel')} {o.get('axis')}")
                continue
            lo, hi = float(o["min"]), float(o["max"])
            if lo > float(s["min"]) + 1e-12 or hi < float(s["max"]) - 1e-12:
                issues.append(f"sweep_overrides: {c} {o['channel']} {o['axis']} [{lo}, {hi}] narrows "
                              f"[{s['min']}, {s['max']}]")
            s["min"], s["max"] = min(lo, float(s["min"])), max(hi, float(s["max"]))
            applied.append({"control": c, "channel": s["channel"], "axis": s["axis"], "min": s["min"], "max": s["max"]})
    return sw, applied, issues


def compare_sweeps(derived, resolved):
    if not isinstance(resolved, dict):
        return None
    diffs = []
    for c in sorted(set(derived) | set(resolved)):
        a = {(s["channel"], s["axis"]): (float(s["min"]), float(s["max"])) for s in derived.get(c, [])}
        try:
            b = {(s["channel"], s["axis"]): (float(s["min"]), float(s["max"])) for s in resolved.get(c) or []}
        except (TypeError, KeyError, ValueError):
            diffs.append({"control": c, "resolved": "unparseable"})
            continue
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b or abs(a[k][0] - b[k][0]) > 1e-9 or abs(a[k][1] - b[k][1]) > 1e-9:
                diffs.append({"control": c, "channel": list(k), "derived": a.get(k), "resolved": b.get(k)})
    return diffs


def prop_ranges(man, holder):
    """PROPS key -> ([lo, hi], source): manifest clamps, else props.spec, else the PROPS UI range."""
    out = {}
    for s in ((man or {}).get("props") or {}).get("spec") or []:
        if isinstance(s, dict) and "name" in s and s.get("min") is not None and s.get("max") is not None:
            out[s["name"]] = ([float(s["min"]), float(s["max"])], "props.spec")
    for k, cl in ((man or {}).get("clamps") or {}).items():
        out[k] = ([float(cl[0]), float(cl[1])], "clamps")
    if holder is not None:
        for k in holder.keys():
            if k in out or k.startswith("_"):
                continue
            try:
                ui = holder.id_properties_ui(k).as_dict()
            except Exception:
                continue
            if isinstance(ui.get("min"), (int, float)) and isinstance(ui.get("max"), (int, float)):
                out[k] = ([float(ui["min"]), float(ui["max"])], "PROPS UI range")
    return out


# ---------------------------------------------------------------- sampling
class Ctx:
    def __init__(self):
        self.man = self.clip = self.res = None
        self.names = self.parent = None
        self.arm = self.mo = self.act = None
        self.geo = None
        self.info = {}


def open_work(ctx):
    p1.open_blend(P["work"])
    arm = bpy.data.objects.get(P["arm"])
    mo = bpy.data.objects.get(P["mesh"])
    if arm is None or arm.type != "ARMATURE" or mo is None or mo.type != "MESH":
        raise Blocked(f"{rel(P['work'])}: {P['arm']} / {P['mesh']} missing")
    act = bpy.data.actions.get(P["action"])
    if act is None:
        raise Blocked(f"{rel(P['work'])}: action {P['action']!r} missing ({[a.name for a in bpy.data.actions]})")
    slot = p1.assign_action(arm, act)
    bpy.context.scene.tool_settings.use_keyframe_insert_auto = False
    arm.data.pose_position = "POSE"
    miss = [n for n in ctx.names if n not in arm.pose.bones]
    if miss:
        raise Blocked(f"{P['arm']} lacks DEF bones {miss}")
    ctx.arm, ctx.mo, ctx.act = arm, mo, act
    return slot


def ctrl_names(arm):
    return sorted(pb.name for pb in arm.pose.bones if pb.name.startswith("CTRL_"))


def ctrl_state(pb):
    """-> (loc [3] m, euler XYZ [3] deg, scale [3], mode, basis 4x4 np)."""
    if pb.rotation_mode == "XYZ":
        e = [math.degrees(a) for a in pb.rotation_euler]
    else:
        e = [math.degrees(a) for a in pb.matrix_basis.to_3x3().normalized().to_euler("XYZ")]
    return (list(pb.location), e, list(pb.scale), pb.rotation_mode, np.array(pb.matrix_basis, dtype=np.float64))


def theta_of(mw, pbs, upper, lower):
    """G6.5b bend angle (copied from check_p2_ctrl.bend_theta, check_p2_ctrl.py:1124-1132)."""
    pu, pl = pbs[upper], pbs[lower]
    u = ((mw @ pu.tail) - (mw @ pu.head)).normalized()
    ll = ((mw @ pl.tail) - (mw @ pl.head)).normalized()
    x = (mw.to_3x3() @ pu.matrix.to_3x3()).col[0].normalized()
    return math.degrees(math.atan2(u.cross(ll).dot(x), u.dot(ll)))


def local_rots(W, names, parent):
    """(B, 3, 3) parent-relative rotations from the (B, 4, 4) world matrices (canonical parents)."""
    idx = {n: i for i, n in enumerate(names)}
    out = np.zeros((len(names), 3, 3))
    for i, n in enumerate(names):
        R = p1.rot3n(W[i])
        pa = parent.get(n)
        if pa in idx:
            R = p1.rot3n(W[idx[pa]]).T @ R
        out[i] = R
    return out


def max_jump(La, Lb, names):
    mx, at = 0.0, None
    for i in range(len(names)):
        d = p1.rang(La[i], Lb[i])
        if d > mx:
            mx, at = d, names[i]
    return mx, at


def def_world_now(ctx):
    dg = bpy.context.evaluated_depsgraph_get()
    oe = ctx.arm.evaluated_get(dg)
    mw = oe.matrix_world
    return np.array([np.array(mw @ oe.pose.bones[n].matrix, dtype=np.float64) for n in ctx.names])


def sample(ctx, frames, extra_frames, chains, mesh_sets):
    """one pass over the frames: DEF world, control / PROPS values, theta, mesh metrics."""
    sc = bpy.context.scene
    arm, mo, g = ctx.arm, ctx.mo, ctx.geo
    mw = arm.matrix_world
    ctrls = ctrl_names(arm)
    pel_rest = np.array(mw @ arm.data.bones["Pelvis"].matrix_local, dtype=np.float64)
    root_rest = np.array(mw @ arm.data.bones["Root"].matrix_local, dtype=np.float64)
    S = {"W": {}, "ctrl": {}, "props": {}, "theta": {}, "shoe": {}, "foot_ht": {}, "zmin": {}, "poke": {},
         "c7": {}, "ctrls": ctrls, "root_rest": root_rest}
    t = g.topo
    for f in list(frames) + list(extra_frames):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = arm.evaluated_get(dg)
        pbs = oe.pose.bones
        W = np.array([np.array(mw @ pbs[n].matrix, dtype=np.float64) for n in ctx.names])
        S["W"][f] = W
        S["ctrl"][f] = {c: ctrl_state(pbs[c]) for c in ctrls}
        hp = pbs.get("PROPS")
        S["props"][f] = {k: float(hp[k]) for k in hp.keys()
                         if isinstance(hp[k], (int, float)) and not isinstance(hp[k], bool)} if hp is not None else {}
        S["theta"][f] = {k: theta_of(mw, pbs, u, l_) for k, (u, l_) in chains.items()}
        S["foot_ht"][f] = {x: (np.array(mw @ pbs[f"Foot_{x.upper()}"].head), np.array(mw @ pbs[f"Foot_{x.upper()}"].tail))
                           for x in ("l", "r")}
        if f not in frames:
            continue
        V = p2d.eval_verts(mo)
        if len(V) != t.nv:
            raise Blocked(f"evaluated {mo.name} has {len(V)} verts, obj.data {t.nv}")
        S["shoe"][f] = {x: V[mesh_sets["shoe"][x]] for x in ("l", "r")}
        vi = int(np.argmin(V[:, 2]))
        S["zmin"][f] = (float(V[vi, 2]), vi)
        # C6: P2.8b rule in the Pelvis rest frame (check_p2_skirt.py:231-240)
        Minv = np.linalg.inv(np.array(mw @ pbs["Pelvis"].matrix, dtype=np.float64) @ np.linalg.inv(pel_rest))
        Vp = V @ Minv[:3, :3].T + Minv[:3, 3]
        pk = {}
        for x in ("l", "r"):
            for zone in ("knee", "thigh"):
                r = p2d.poke_out(g.pctx, Vp, mesh_sets["leg"][x][zone], g.walls)
                pk[f"{zone}_{x}"] = (int(r.get("n_poke_out", 0)), r.get("max_poke_out_mm"), r.get("worst_point_m"))
        S["poke"][f] = pk
        # C7: P2.2 rules on the world mesh
        ss = p2d.ShellSet(t, V, t.shells)
        S["c7"][f] = c7_metrics(ctx, V, ss, mesh_sets)
        S.setdefault("c11", {})[f] = c11_metrics(ctx, V, ss, mesh_sets)   # T325 C11
        S.setdefault("c12", {})[f] = c12_metrics(ctx, V, ss, mesh_sets)   # T331 C12
        if getattr(ctx, "weapons", None):   # T311 C8
            S.setdefault("c8", {})[f] = c8_metrics(ctx, V, W)
        if len(S["zmin"]) % 50 == 0:
            print(f"[{CHECKER}] sampled {len(S['zmin'])}/{len(frames)} frames")
            sys.stdout.flush()
    return S


def c7_metrics(ctx, V, ss, mesh_sets):
    out = {}
    for item, _a, _b in C7_ITEMS:
        for x in ("l", "r"):
            pa, pb_ = mesh_sets["c7"][item][x]
            r = p2d.cross_isect(ctx.c7ctx, V, pa, pb_, ss, with_depth=True)
            out[f"{item}_{x}"] = {"pairs": r.get("pairs", 0), "visible": r.get("visible", 0),
                                  "hidden": r.get("hidden", 0),
                                  "visible_max_depth_mm": r.get("visible_max_depth_mm"),
                                  "visible_by_other_part": r.get("visible_by_other_part"),
                                  "visible_example": (r.get("visible_examples") or [None])[0]}
    return out


# ---------------------------------------------------------------- rows
def c1(ctx, S, frames, sweeps, sweep_info, pranges):
    man_ctrls = set(sweeps)
    viol, per_ctrl, worst = [], {}, {}
    n_checked = 0
    for f in frames:
        for c, (loc, eul, scl, mode, basis) in S["ctrl"][f].items():
            if c not in man_ctrls:
                d = float(np.abs(basis - np.eye(4)).max())
                if d > VAL_TOL:
                    viol.append({"f": f, "control": c, "issue": "unlisted control not at identity", "max_abs": d})
                continue
            decl = {(s["channel"], s["axis"]): (float(s["min"]), float(s["max"])) for s in sweeps[c]}
            for chn, vals in (("loc", loc), ("rot", eul)):
                for i, ax in enumerate("XYZ"):
                    v = float(vals[i])
                    lo, hi = decl.get((chn, ax), (0.0, 0.0))
                    n_checked += 1
                    key = f"{c}.{chn}{ax}"
                    w = worst.setdefault(key, [v, v])
                    w[0], w[1] = min(w[0], v), max(w[1], v)
                    if v < lo - VAL_TOL or v > hi + VAL_TOL:
                        viol.append({"f": f, "control": c, "channel": f"{chn}{ax}", "value": rnd(v, 6),
                                     "range": [lo, hi], "declared": (chn, ax) in decl,
                                     "mode": mode})
            for i, ax in enumerate("XYZ"):
                if abs(scl[i] - 1.0) > VAL_TOL:
                    viol.append({"f": f, "control": c, "channel": f"scale{ax}", "value": rnd(scl[i], 6),
                                 "range": [1.0, 1.0]})
        for k, v in S["props"][f].items():
            if k not in pranges:
                continue
            (lo, hi), _src = pranges[k]
            key = f"PROPS.{k}"
            w = worst.setdefault(key, [v, v])
            w[0], w[1] = min(w[0], v), max(w[1], v)
            if v < lo - VAL_TOL or v > hi + VAL_TOL:
                viol.append({"f": f, "prop": k, "value": rnd(v, 6), "range": [lo, hi]})
    for key, (lo, hi) in worst.items():
        if abs(lo) > VAL_TOL or abs(hi) > VAL_TOL:
            per_ctrl[key] = [rnd(lo, 5), rnd(hi, 5)]
    rmatch = sweep_info.get("resolved_vs_derived")
    ok = not viol and not sweep_info["issues"] and (rmatch is None or not rmatch)
    return {"n_frames": len(frames), "n_channel_samples": n_checked, "n_violations": len(viol),
            "violations_first": viol[:LIST_CAP], "violating_channels": sorted({
                v.get("control", "PROPS") + "." + str(v.get("channel", v.get("prop"))) for v in viol}),
            "used_range_nonzero_[min,max]": per_ctrl, "sweeps": sweep_info,
            "props_ranges": {k: {"range": r, "source": s} for k, (r, s) in pranges.items()},
            "unlisted_ctrl_bones": sorted(set(S["ctrls"]) - man_ctrls),
            "manifest_ctrls_missing_on_rig": sorted(man_ctrls - set(S["ctrls"]))}, ok


def c2(ctx, S, frames, chains, loop_pair):
    names = ctx.names
    L = {f: local_rots(S["W"][f], names, ctx.parent) for f in S["W"]}
    steps = {}
    for f0_, f1_ in zip(frames[:-1], frames[1:]):
        steps[f1_] = np.array([p1.rang(L[f0_][i], L[f1_][i]) for i in range(len(names))])
    ratios, keys, top = [], [], []
    fs = frames[1:]
    for j, f in enumerate(fs):
        nb = [steps[fs[j - 1]]] if j > 0 else []
        if j + 1 < len(fs):
            nb.append(steps[fs[j + 1]])
        if not nb:
            continue
        ref = np.maximum(np.max(nb, axis=0), NEIGH_FLOOR_DEG)
        for i, n in enumerate(names):
            s = steps[f][i]
            if s < MIN_STEP_DEG:
                continue
            r = float(s / ref[i])
            ratios.append(r)
            keys.append([f - 1, f, n])
            top.append((r, f, n, float(s), float(ref[i])))
    top.sort(reverse=True)
    # branch flips (G6.5b)
    flips, trange = [], {}
    for k in chains:
        ths = [S["theta"][f][k] for f in frames]
        trange[k] = [rnd(min(ths), 3), rnd(max(ths), 3)]
        for a, b, fa in zip(ths[:-1], ths[1:], frames[:-1]):
            if (a > 0) != (b > 0) and abs(a) > FLIP_MIN_DEG and abs(b) > FLIP_MIN_DEG:
                flips.append({"chain": k, "frames": [fa, fa + 1], "theta_deg": [rnd(a, 3), rnd(b, 3)]})
    # G6.5 sub-frame bisection of every interval with a max DEF step > POP_MIN_DEG
    sc = bpy.context.scene
    cands, n_eval = [], 0
    t0 = time.time()
    for fa, fb in zip(frames[:-1], frames[1:]):
        j, bone = max_jump(L[fa], L[fb], names)
        if j <= POP_MIN_DEG:
            continue
        a, b, La, Lb = 0.0, 1.0, L[fa], L[fb]
        seq, bones = [j], [bone]
        for _lvl in range(REFINE_LEVELS):
            m = 0.5 * (a + b)
            sc.frame_set(fa, subframe=m)
            Lm = local_rots(def_world_now(ctx), names, ctx.parent)
            n_eval += 1
            jl, bl = max_jump(La, Lm, names)
            jr, br = max_jump(Lm, Lb, names)
            if jl >= jr:
                b, Lb = m, Lm
                seq.append(jl)
                bones.append(bl)
            else:
                a, La = m, Lm
                seq.append(jr)
                bones.append(br)
        cls = "pop" if seq[-1] > POP_RATIO * seq[0] else "continuous"
        cands.append({"interval": [fa, fb], "final_subframe": [rnd(fa + a, 5), rnd(fa + b, 5)],
                      "jump_deg_h_to_h16": [rnd(x, 4) for x in seq], "bones": bones, "class": cls})
    sc.frame_set(frames[0])
    pops = [c for c in cands if c["class"] == "pop"]
    seam = None
    if frames[-1] + 1 in L:
        j, bone = max_jump(L[frames[-1]], L[frames[-1] + 1], names)
        seam = {"step_deg_last_to_last_plus_1": rnd(j, 4), "bone": bone,
                "declared_seam_pair": list(loop_pair) if loop_pair else None}
    per_bone_max = {n: rnd(max((steps[f][i] for f in steps), default=0.0), 4) for i, n in enumerate(names)}
    ok = not flips and not pops
    return {"rule_ratio": f"r = s(f) / max(s(f-1), s(f+1), {NEIGH_FLOOR_DEG}) for s(f) >= {MIN_STEP_DEG} deg; "
                          "s = parent-relative DEF rotation step (canonical parents, world matrices)",
            "k_draft (report)": K_DRAFT,
            "ratio_distribution (report)": p1.dist(ratios, 1.0, 4, keys=keys),
            "n_ratio_over_k_draft (report)": sum(1 for r in ratios if r > K_DRAFT),
            "ratio_top (report)": [{"frames": [f - 1, f], "bone": n, "ratio": rnd(r, 3), "step_deg": rnd(s, 4),
                                    "neighbour_max_deg": rnd(rf, 4)} for r, f, n, s, rf in top[:LIST_CAP]],
            "step_deg_distribution": p1.dist([float(x) for v in steps.values() for x in v], 1.0, 4),
            "step_deg_max_per_bone": per_bone_max,
            "branch_flips": len(flips), "flips": flips[:LIST_CAP], "theta_deg_min_max": trange,
            "chains": {k: list(v) for k, v in chains.items()},
            "bisection": {"n_intervals": len(cands), "n_pops": len(pops), "pops": pops[:LIST_CAP],
                          "worst_continuous": sorted((c for c in cands if c["class"] == "continuous"),
                                                     key=lambda c: -c["jump_deg_h_to_h16"][-1] /
                                                     max(c["jump_deg_h_to_h16"][0], 1e-12))[:5],
                          "n_subframe_evals": n_eval, "seconds": rnd(time.time() - t0, 1),
                          "rule": f"G6.5: bisect toward the larger half {REFINE_LEVELS} levels; pop when "
                                  f"jump(h/16) > {POP_RATIO} jump(h); intervals with max step > {POP_MIN_DEG} deg"},
            "loop_wrap_step (report)": seam}, ok


def c3(ctx, S, planted, issues, declared_empty=False):
    rows, ok = [], (bool(planted) or declared_empty) and not issues
    for r in planted:
        x, a, b = r["foot"], r["a"], r["b"]
        Sc, zs, wp, zp = 0.0, [], None, None
        for f in range(a, b + 1):
            w = S["shoe"][f][x]
            z = float(w[:, 2].min())
            zs.append(z)
            if wp is not None:
                ck = wp[:, 2] <= zp + CONTACT_TOL      # contact set of frame k (G6.7)
                Sc += float(np.mean(np.hypot(w[ck, 0] - wp[ck, 0], w[ck, 1] - wp[ck, 1])))
            wp, zp = w, z
        h0, t0 = S["foot_ht"][a][x]
        mv = max(max(float(np.linalg.norm(S["foot_ht"][f][x][0] - h0)), float(np.linalg.norm(S["foot_ht"][f][x][1] - t0)))
                 for f in range(a, b + 1))
        z_ok = all(PLANT_Z[0] - 1e-12 <= z <= PLANT_Z[1] + 1e-12 for z in zs)
        rok = Sc <= PLANT_SLIP and z_ok
        bad = [f for f, z in zip(range(a, b + 1), zs) if not (PLANT_Z[0] <= z <= PLANT_Z[1])]
        rows.append({"foot": x, "frames": [a, b], "slip_cum_mm": mm(Sc), "shoe_min_z_mm_min_max": [mm(min(zs)), mm(max(zs))],
                     "z_out_of_band_frames": bad[:LIST_CAP], "def_foot_move_mm (report)": mm(mv), "ok": rok})
        ok = ok and rok
    return {"ranges": rows, "n_ranges": len(rows), "issues": issues,
            "declared_empty": declared_empty}, ok


def c4(ctx, S, frames, ground, issues):
    t = ctx.geo.topo
    judged = [f for a, b in ground for f in range(a, b + 1)] if ground else list(frames)
    per = {f: mm(S["zmin"][f][0]) for f in frames}

    def part_of(vi):
        pl = t.loop_poly[t.loop_vert == vi]
        return p2d.pname(t, int(pl[0])) if len(pl) else None
    fmin = min(frames, key=lambda f: S["zmin"][f][0])
    jmin = min(judged, key=lambda f: S["zmin"][f][0]) if judged else None
    fail = [f for f in judged if S["zmin"][f][0] < -GROUND_PEN - 1e-12]
    out_fail = [f for f in frames if f not in set(judged) and S["zmin"][f][0] < -GROUND_PEN]
    ok = bool(judged) and not fail and not issues
    return {"judged_frames": ("contacts.ground_ranges " + json.dumps(ground)) if ground
            else "no ground_ranges declared: every clip frame", "n_judged": len(judged),
            "judged_min_z_mm": {"mm": per.get(jmin), "frame": jmin, "part": part_of(S["zmin"][jmin][1]) if jmin else None},
            "failing_frames": fail[:LIST_CAP], "n_failing": len(fail),
            "all_frames_min_z_mm": {"mm": per[fmin], "frame": fmin, "part": part_of(S["zmin"][fmin][1])},
            "all_frames_max_of_min_z_mm (report)": max(per.values()),
            "outside_ranges_below_-1mm_frames (report)": out_fail[:LIST_CAP], "issues": issues,
            "per_frame_min_z_mm": per}, ok


def c5(ctx, S, frames, policy, seam_conv, loop_pair):
    ri = ctx.names.index("Root")
    rest = S["root_rest"]
    rw, rb, rw_at = 0.0, 0.0, None
    path = []
    for f in frames:
        W = S["W"][f][ri]
        d = float(np.abs(W - rest).max())
        if d > rw:
            rw, rw_at = d, f
        c = S["ctrl"][f].get("CTRL_root")
        if c is not None:
            rb = max(rb, float(np.abs(c[4] - np.eye(4)).max()))
        R = p1.rot3n(W)
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        path.append({"f": f, "pos_mm": [mm(v) for v in W[:3, 3]], "yaw_deg": rnd(yaw, 4)})
    m = {"policy": policy, "root_def_world_vs_rest_max_abs": float(f"{rw:.3e}"), "at_frame": rw_at,
         "ctrl_root_basis_vs_identity_max_abs": float(f"{rb:.3e}")}
    if policy not in POLICIES:
        m["issue"] = f"unknown root_motion {policy!r} (expected one of {POLICIES})"
        return m, False
    if policy == "root":
        p0 = np.array(path[0]["pos_mm"])
        pe = np.array(path[-1]["pos_mm"])
        sp = [float(np.linalg.norm(np.array(b["pos_mm"]) - np.array(a["pos_mm"]))) for a, b in zip(path[:-1], path[1:])]
        m.update({"root_path": path, "displacement_first_to_last_mm": [rnd(v, 4) for v in (pe - p0)],
                  "step_mm_distribution": p1.dist(sp, 1.0, 4),
                  "note": "policy root: path reported (d-03 C5); Root pitch / roll visible in the path matrices only"})
        return m, True
    ok = rw <= ROOT_TOL and rb <= ROOT_TOL
    if policy == "in_place_cycle":
        fa = frames[0]
        fb = loop_pair[1] if loop_pair is not None else frames[-1]
        m["loop_seam"] = {"convention": seam_conv, "compared": [fa, fb]}
        both = {}
        for conv, fb_ in (("first_eq_last", frames[-1]), ("first_eq_last_plus_1", frames[-1] + 1)):
            if fb_ not in S["W"]:
                both[conv] = "not sampled"
                continue
            Pm = [float(np.linalg.norm(S["W"][fa][i][:3, 3] - S["W"][fb_][i][:3, 3])) for i in range(len(ctx.names))]
            Rd = [p1.rang(S["W"][fa][i], S["W"][fb_][i]) for i in range(len(ctx.names))]
            cp, cr, cs, cw = 0.0, 0.0, 0.0, None
            for c, st in S["ctrl"][fa].items():
                o = S["ctrl"][fb_].get(c)
                if o is None:
                    continue
                dp = float(np.linalg.norm(st[4][:3, 3] - o[4][:3, 3]))
                dr = p1.rang(st[4], o[4])
                cs = max(cs, float(np.abs(np.array(st[2]) - np.array(o[2])).max()))
                if dp > cp or dr > cr:
                    cw = c
                cp, cr = max(cp, dp), max(cr, dr)
            pd = {k: abs(v - S["props"][fb_].get(k, float("nan"))) for k, v in S["props"][fa].items()}
            pw = max(pd, key=lambda k: pd[k] if pd[k] == pd[k] else 1e9) if pd else None
            both[conv] = {"frames": [fa, fb_], "def_pos_mm_max": mm(max(Pm)), "def_rot_deg_max": rnd(max(Rd), 5),
                          "def_worst_bone": ctx.names[int(np.argmax(Rd))],
                          "ctrl_pos_mm_max": mm(cp), "ctrl_rot_deg_max": rnd(cr, 5), "ctrl_worst": cw,
                          "ctrl_scale_abs_diff_max": rnd(cs, 7),
                          "props_abs_diff_max": rnd(pd[pw], 7) if pw else None, "props_worst": pw,
                          "ok": (max(Pm) <= SEAM_POS and max(Rd) <= SEAM_ROT and cp <= SEAM_POS and cr <= SEAM_ROT
                                 and cs <= SEAM_PROP and (not pd or all(v <= SEAM_PROP for v in pd.values())))}
        m["loop_seam"]["both_conventions"] = both
        judged = both.get(seam_conv)
        ok = ok and isinstance(judged, dict) and judged["ok"]
    return m, ok


def c6(ctx, S, frames):
    per, tot = {}, []
    for f in frames:
        pk = S["poke"][f]
        n = sum(v[0] for v in pk.values())
        mx = max((v[1] or 0.0) for v in pk.values())
        per[f] = {"n": n, "max_mm": mx, **{k: [v[0], v[1]] for k, v in pk.items() if v[0]}}
        tot.append((n, mx, f))
    split = {}
    for zone in ("knee", "thigh"):
        for x in ("l", "r"):
            k = f"{zone}_{x}"
            vals = [(S["poke"][f][k][0], S["poke"][f][k][1] or 0.0, f) for f in frames]
            w = max(vals, key=lambda v: (v[0], v[1]))
            split[k] = {"max_verts": w[0], "max_mm": max(v[1] for v in vals), "worst_frame": w[2] if w[0] else None,
                        "frames_gt0": sum(1 for v in vals if v[0] > 0),
                        "worst_point_m": S["poke"][w[2]][k][2] if w[0] else None}
    worst = sorted(tot, key=lambda v: (-v[0], -v[1]))[:10]
    g = ctx.geo
    return {"max_verts": max(v[0] for v in tot), "max_mm": max(v[1] for v in tot),
            "frames_gt0": sum(1 for v in tot if v[0] > 0),
            "worst_frames": [{"f": f, "n": n, "max_mm": m_} for n, m_, f in worst if n > 0],
            "split_by_rest_z_vs_hem": split, "hem_z_rest_mm": mm(ctx.hem_z),
            "n_leg_verts": {k: int(len(v)) for x in ("l", "r") for k, v in
                            ((f"knee_{x}", ctx.leg_sets[x]["knee"]), (f"thigh_{x}", ctx.leg_sets[x]["thigh"]))},
            "rule": "check_p2_deform.poke_out on the evaluated PL_mesh (springs off) in the Pelvis rest frame, walls = "
                    f"tunic faces below the belt bottom without the {g.n_ceiling} ceiling faces (check_p2_skirt.Geo); "
                    "leg_x vertices with rest z below tunic.ceiling_z, knee = rest z < hem z (min rest z of "
                    "hem_outer), thigh = rest z >= hem z",
            "per_frame": {f: v for f, v in per.items() if v["n"]}}, True


def c7(ctx, S, frames, rest):
    summ = {}
    for key in rest:
        vals = [(S["c7"][f][key]["visible"], S["c7"][f][key]["visible_max_depth_mm"] or 0.0, f) for f in frames]
        w = max(vals, key=lambda v: (v[0], v[1]))
        summ[key] = {"max_visible_pairs": w[0], "worst_frame": w[2] if w[0] else None,
                     "max_visible_depth_mm": max(v[1] for v in vals),
                     "frames_visible_gt0": sum(1 for v in vals if v[0] > 0),
                     "max_hidden_pairs": max(S["c7"][f][key]["hidden"] for f in frames),
                     "worst_frame_detail": S["c7"][w[2]][key] if w[0] else None,
                     "rest": {k: rest[key][k] for k in ("pairs", "visible", "hidden", "visible_max_depth_mm")}}
    per = {}
    for f in frames:
        v = {k: [r["visible"], r["visible_max_depth_mm"]] for k, r in S["c7"][f].items() if r["visible"]}
        if v:
            per[f] = v
    return {"items": summ, "per_frame_visible": per,
            "rule": "check_p2_deform.cross_isect (P2.2d / T223): intersecting polygon pairs of the two part sets; hidden "
                    "= every crossing point inside a closed or virtually capped shell of a third part (ShellSet over "
                    "every shell); leg x skirt-ceiling = designed; depth = max inside depth of either polygon's "
                    "vertices in the other part; torso = tunic + belt + pouch; scarf = scarf + scarf_tail",
            "rest_source": "undeformed PL_mesh data (rest)"}, True


# ---------------------------------------------------------------- C8 weapon vs body (T311)
C8_GROUPS = (("head", ("head", "eye")), ("torso", ("tunic", "belt", "pouch")), ("scarf", ("scarf", "scarf_tail")),
             ("arms", ("arm", "sleeve", "fist")), ("legs", ("leg", "cuff", "shoe")))
C8_DRAFT_PAIRS = 0
C8_NOTE = ("T321: blocking at 0 pairs from Sword_Attack_01 on; basis: Sword_Idle measured 0 pairs on every frame "
           "(d-04 section 2 / section 6, d-05 section 2 SA1.C8)")
C8_OBJECT_DEFAULT = "WPN_sword_placeholder"
C8_OFFSET_KEYS = ("offset_matrix", "matrix", "pivot", "offset", "location", "rotation_euler_deg", "quaternion",
                  "axes_matrix")


def contract_entry(con, side, wname):
    """socket side ("R" / "L") -> (entry dict, key) from the contract, tolerant of the layout."""
    if not isinstance(con, dict):
        return None, None
    keys = (side, side.lower(), f"WeaponSocket_{side}", wname)
    tops = (con, con.get("sockets"), con.get("weapons"), con.get(wname))
    for top in tops:
        if not isinstance(top, dict):
            continue
        for k in keys:
            if isinstance(top.get(k), dict):
                return top[k], k
    if any(k in con for k in C8_OFFSET_KEYS):
        return con, "(top level)"
    return None, None


def contract_offset(ent):
    """entry -> (4x4 socket-local -> weapon-local offset, source).  offset_matrix / matrix (4x4 row-major), else
    pivot | offset | location [3] (m, socket local) + rotation_euler_deg [3] XYZ | quaternion [4] wxyz | axes_matrix
    [3x3] (columns = weapon axes in socket space); none given = identity (pivot at the socket head, weapon axes =
    socket axes, d-04 section 1)."""
    from mathutils import Euler, Quaternion
    for k in ("offset_matrix", "matrix"):
        m = ent.get(k)
        if isinstance(m, list) and len(m) == 4:
            return np.array(m, dtype=np.float64), k
    T = np.eye(4)
    src = []
    for k in ("pivot", "offset", "location"):
        v = ent.get(k)
        if isinstance(v, (list, tuple)) and len(v) == 3:
            T[:3, 3] = [float(x) for x in v]
            src.append(k)
            break
    re_, q = ent.get("rotation_euler_deg"), ent.get("quaternion")
    if isinstance(re_, (list, tuple)) and len(re_) == 3:
        T[:3, :3] = np.array(Euler([math.radians(float(a)) for a in re_], "XYZ").to_matrix())
        src.append("rotation_euler_deg")
    elif isinstance(q, (list, tuple)) and len(q) == 4:
        T[:3, :3] = np.array(Quaternion([float(a) for a in q]).normalized().to_matrix())
        src.append("quaternion")
    elif isinstance(ent.get("axes_matrix"), list) and len(ent["axes_matrix"]) == 3:
        T[:3, :3] = np.array(ent["axes_matrix"], dtype=np.float64)
        src.append("axes_matrix")
    return T, ("+".join(src) if src else "identity (no offset keys)")


def weapon_box():
    """self-test seam: grip box y -0.08 .. 0.02 and blade box y 0.02 .. 0.36 (m, weapon local = socket axes)."""
    V, F = [], []
    for (x0, x1, y0, y1, z0, z1) in ((-0.012, 0.012, -0.08, 0.02, -0.012, 0.012),
                                     (-0.02, 0.02, 0.02, 0.36, -0.004, 0.004)):
        b = len(V)
        V += [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
        for q in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
            F += [(b + q[0], b + q[1], b + q[2]), (b + q[0], b + q[2], b + q[3])]
    return (np.array(V, dtype=np.float64), np.array(F, dtype=np.int64), np.arange(8, 16),
            "synthetic box (grip + blade, --weapon-box)")


def weapon_mesh(path, obj_name):
    """append the weapon object in memory (never saved) -> (verts local, tris, blade vertex ids, info)."""
    with bpy.data.libraries.load(str(path), link=False) as (src, dst):
        names = list(src.objects)
        want = [obj_name] if obj_name in names else ([n for n in names if n.startswith("WPN_")] or names)
        dst.objects = want
    objs = [o for o in dst.objects if o is not None and o.type == "MESH"]
    if len(objs) != 1:
        raise Blocked(f"{rel(path)}: expected one weapon mesh object ({obj_name}), found "
                      f"{[o.name for o in objs]} of {names}")
    o = objs[0]
    me = o.data
    V = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    V = V.reshape(-1, 3)
    me.calc_loop_triangles()
    tv = np.empty(len(me.loop_triangles) * 3, dtype=np.int32)
    me.loop_triangles.foreach_get("vertices", tv)
    blade, bsrc = None, "whole weapon (no 'blade' vertex group / material)"
    vg = next((g for g in o.vertex_groups if "blade" in g.name.lower()), None)
    if vg is not None:
        blade = np.array([v.index for v in me.vertices if any(g.group == vg.index and g.weight > 0 for g in v.groups)],
                         dtype=np.int64)
        bsrc = f"vertex group {vg.name}"
    else:
        mi = [i for i, m in enumerate(me.materials) if m is not None and "blade" in m.name.lower()]
        if mi:
            blade = np.unique([v for p in me.polygons if p.material_index in mi for v in p.vertices]).astype(np.int64)
            bsrc = f"material {[me.materials[i].name for i in mi]}"
    if blade is None or not len(blade):
        blade = np.arange(len(V))
    info = {"object": o.name, "blade_selection": bsrc, "verts": len(V), "tris": int(len(tv) // 3),
            "object_matrix_world_in_blend (not used)": [[rnd(x, 5) for x in r] for r in o.matrix_world]}
    return V, tv.reshape(-1, 3).astype(np.int64), blade, info


def load_weapons(ctx, weapon):
    """clip JSON weapon {side: name} -> ctx.weapons [{side, socket, fist, V, F, blade, T, info}] + body groups."""
    if not isinstance(weapon, dict) or not weapon:
        raise Blocked(f"clip JSON weapon {weapon!r} is not a non-empty object {{side: name}}")
    cpath = Path(P["weapon_contract"])
    if not cpath.exists():
        raise Blocked(f"missing input {rel(cpath)}")
    con = p2d.load_json(cpath)
    out = []
    for side, wname in weapon.items():
        sd = str(side).strip().upper()[:1]
        if sd not in ("L", "R"):
            raise Blocked(f"weapon side {side!r} is not L / R")
        socket = f"WeaponSocket_{sd}"
        if socket not in ctx.names:
            raise Blocked(f"{socket} is not a DEF bone")
        ent, key = contract_entry(con, sd, str(wname))
        if ent is None:
            raise Blocked(f"{rel(cpath)}: no entry for side {sd} / {wname} (top keys {sorted(con)[:20]})")
        T, tsrc = contract_offset(ent)
        if P["weapon_box"]:
            V, F, blade, msrc = weapon_box()
            info = {"mesh": msrc}
        else:
            bp = Path(P["weapons_dir"]) / f"{wname}.blend"
            if not bp.exists():
                raise Blocked(f"missing input {rel(bp)}")
            V, F, blade, info = weapon_mesh(bp, str(ent.get("object") or ent.get("mesh") or C8_OBJECT_DEFAULT))
            info["blend"] = rel(bp)
        info.update({"side": sd, "weapon": wname, "socket": socket, "contract": rel(cpath), "contract_key": key,
                     "offset_source": tsrc, "offset_matrix": [[rnd(x, 6) for x in r] for r in T],
                     "contract_entry": ent, "holding_fist_excluded": f"fist_{sd.lower()}"})
        out.append({"side": sd, "socket": socket, "fist": f"fist_{sd.lower()}", "V": V, "F": F, "blade": blade,
                    "T": T, "info": info})
    t = ctx.geo.topo
    ctx.c8_groups = {}
    for w in out:
        for gname, fams in C8_GROUPS:
            polys = np.array([q for q in t.polys_of_fams(fams).tolist() if p2d.pname(t, q) != w["fist"]],
                             dtype=np.int64)
            ctx.c8_groups[(w["side"], gname)] = (t.tris_of_polys(polys), t.verts_of_polys(polys))
    ctx.weapons = out


def c8_metrics(ctx, V, W):
    """per weapon: weapon x body-group intersecting triangle pairs (BVH overlap) and blade clearance."""
    from mathutils import Vector
    t = ctx.geo.topo
    out = {}
    for w in ctx.weapons:
        M = W[ctx.names.index(w["socket"])] @ w["T"]
        Vw = w["V"] @ M[:3, :3].T + M[:3, 3]
        bw = p2d.bvh_of(Vw, w["F"])
        bfaces = w["F"][np.all(np.isin(w["F"], w["blade"]), axis=1)]
        bl = p2d.bvh_of(Vw, bfaces) if len(bfaces) and len(bfaces) < len(w["F"]) else bw
        rec = {}
        for gname, _f in C8_GROUPS:
            tris, verts = ctx.c8_groups[(w["side"], gname)]
            if not len(tris):
                rec[gname] = {"pairs": 0, "blade_clearance_mm": None, "parts_hit": []}
                continue
            bb = p2d.bvh_of(V, t.tri_v[tris])
            pairs = bw.overlap(bb)
            clr = float("inf")
            for p in Vw[w["blade"]].tolist():
                hit = bb.find_nearest(Vector(p))
                if hit[0] is not None:
                    clr = min(clr, hit[3])
            for p in V[verts].tolist():
                hit = bl.find_nearest(Vector(p))
                if hit[0] is not None:
                    clr = min(clr, hit[3])
            rec[gname] = {"pairs": len(pairs),
                          "blade_clearance_mm": (0.0 if pairs else mm(clr)) if clr != float("inf") else None,
                          "parts_hit": sorted({p2d.pname(t, int(t.tri_poly[tris[j]])) for _i, j in pairs})}
        out[w["side"]] = rec
    return out


def c8(ctx, S, frames):
    per, summ = {}, {}
    for w in ctx.weapons:
        sd = w["side"]
        g = {}
        for gname, _f in C8_GROUPS:
            vals = [(S["c8"][f][sd][gname]["pairs"], S["c8"][f][sd][gname]["blade_clearance_mm"], f) for f in frames]
            cl = [(v[1], v[2]) for v in vals if v[1] is not None]
            wp = max(vals, key=lambda v: v[0])
            mc = min(cl) if cl else (None, None)
            g[gname] = {"max_pairs": wp[0], "worst_frame": wp[2] if wp[0] else None,
                        "frames_pairs_gt0": sum(1 for v in vals if v[0] > 0),
                        "min_blade_clearance_mm": mc[0], "min_clearance_frame": mc[1],
                        "parts_hit": sorted({p for f in frames for p in S["c8"][f][sd][gname]["parts_hit"]})}
        summ[sd] = {"groups": g,
                    "max_pairs_per_frame_all_groups": max(sum(S["c8"][f][sd][k]["pairs"] for k, _f in C8_GROUPS)
                                                          for f in frames),
                    "weapon": w["info"]}
        for f in frames:
            v = {k: r["pairs"] for k, r in S["c8"][f][sd].items() if r["pairs"]}
            if v:
                per.setdefault(f, {})[sd] = v
    meets = all(s_["max_pairs_per_frame_all_groups"] <= C8_DRAFT_PAIRS for s_ in summ.values())
    return {"weapons": summ, "per_frame_pairs": per, "draft_threshold_pairs": C8_DRAFT_PAIRS,
            "meets_threshold": meets,
            "rule": "weapon placed at socket_world (evaluated WeaponSocket_x pose bone, Blender world) @ contract offset "
                    "@ weapon mesh data coordinates (placeholder object in the anim blend not used); pairs = "
                    "BVHTree.overlap triangle pairs weapon x body group; groups head (head, eyes), torso (tunic, belt, "
                    "pouch), scarf (scarf, scarf_tail), arms (arm, sleeve, fist), legs (leg, cuff, shoe); the holding "
                    "fist (fist_x of the socket side) excluded; blade clearance = min of blade vertex -> group surface "
                    "and group vertex -> blade surface distances (0 when pairs > 0)"}, meets


# ---------------------------------------------------------------- C9 match pose / C10 events (T321)
C9_POS = 1e-5               # m (0.01 mm)
C9_ROT = 0.01               # deg
C9_PROP = 1e-4              # PROPS abs diff (C5 SEAM_PROP)
C9_NOTE = ("T333 blocking (d-05 section 8): 0.01 mm / 0.01 deg, PROPS 1e-4; basis: 0 mm / 0 deg measured on "
           "Sword_Attack_01's first and last frames vs Sword_Idle f1; needed for clean chaining")
C10_NAMES = ("hit_start", "hit_end")
C10_PAD = 2                 # frames around the window


def match_ref(ctx, mp):
    """reference DEF world (41) + PROPS at match_pose.frame of match_pose.clip, from its anim blend (read-only, in
    memory).  Opening the blend drops the work blend: C9 runs after every row that needs the work rig."""
    ref = str(mp.get("clip"))
    fr = int(mp.get("frame", 1))
    path = Path(P["match_blend"]) if P["match_blend"] else PLAYER_RIG / "anim" / f"pl_a_{ref}.blend"
    if not path.exists():
        raise Blocked(f"missing input {rel(path)}")
    p1.open_blend(path)
    arm = bpy.data.objects.get(P["arm"])
    act = bpy.data.actions.get(ref)
    if arm is None or act is None:
        raise Blocked(f"{rel(path)}: {P['arm']} / action {ref!r} missing")
    p1.assign_action(arm, act)
    arm.data.pose_position = "POSE"
    bpy.context.scene.frame_set(fr)
    dg = bpy.context.evaluated_depsgraph_get()
    oe = arm.evaluated_get(dg)
    mw = oe.matrix_world
    W = np.array([np.array(mw @ oe.pose.bones[n].matrix, dtype=np.float64) for n in ctx.names])
    hp = oe.pose.bones.get("PROPS")
    props = {k: float(hp[k]) for k in hp.keys()
             if isinstance(hp[k], (int, float)) and not isinstance(hp[k], bool)} if hp is not None else {}
    return W, props, {"blend": rel(path), "clip": ref, "action": act.name, "frame": fr,
                      "fps": p1.fps_of(bpy.context.scene)}


def c9(ctx, S, frames, mp):
    at = mp.get("at") or ["first", "last"]
    at = [at] if isinstance(at, str) else list(at)
    Wr, pr, info = match_ref(ctx, mp)
    rows, cand = {}, True
    for a in at:
        f = {"first": frames[0], "last": frames[-1]}.get(str(a), a if isinstance(a, int) else None)
        if f not in S["W"]:
            rows[str(a)] = f"frame {a!r} not sampled"
            cand = False
            continue
        W = S["W"][f]
        pe = [float(np.linalg.norm(W[i][:3, 3] - Wr[i][:3, 3])) for i in range(len(ctx.names))]
        re_ = [p1.rang(W[i], Wr[i]) for i in range(len(ctx.names))]
        pd = {k: abs(S["props"][f].get(k, float("nan")) - v) for k, v in pr.items()}
        extra = sorted(set(S["props"][f]) - set(pr))
        pbad = {k: rnd(v, 7) for k, v in pd.items() if not v <= C9_PROP}
        ok = max(pe) <= C9_POS and max(re_) <= C9_ROT and not pbad and not extra
        rows[str(a)] = {"frame": f, "def_pos_mm_max": mm(max(pe)), "def_pos_worst": ctx.names[int(np.argmax(pe))],
                        "def_rot_deg_max": rnd(max(re_), 5), "def_rot_worst": ctx.names[int(np.argmax(re_))],
                        "per_bone_[pos_mm,rot_deg]_over_0.001": {ctx.names[i]: [mm(pe[i]), rnd(re_[i], 5)]
                                                                  for i in range(len(ctx.names))
                                                                  if pe[i] > 1e-6 or re_[i] > 0.001},
                        "props_abs_diff_max": rnd(max((v for v in pd.values() if v == v), default=0.0), 7),
                        "props_over_tol": pbad, "props_only_in_clip": extra, "within_tol": ok}
        cand = cand and ok
    return {"match_pose": mp, "reference": info, "at": rows, "blocking_candidate_ok": cand,
            "tol": {"pos_mm": C9_POS * 1000, "rot_deg": C9_ROT, "props_abs": C9_PROP},
            "rule": "DEF world = matrix_world @ pose_bone.matrix (evaluated) of the 41 canonical bones; clip frames from "
                    "the work blend, reference frame from the reference anim blend with its action assigned in "
                    "memory; PROPS = evaluated PROPS pose-bone numeric values"}, cand


def weapon_tip(V):
    z = V[:, 2]
    m = z >= z.max() - 1e-6
    return V[m].mean(axis=0), int(m.sum())


def c10(ctx, S, frames, events, fps):
    issues, ev_ok = [], isinstance(events, list)
    evs = []
    for e in events if isinstance(events, list) else []:
        if not isinstance(e, dict) or "name" not in e or not isinstance(e.get("frame"), (int, float)):
            issues.append(f"event {e!r}: needs name and frame")
            continue
        evs.append((str(e["name"]), int(e["frame"])))
    unknown = [n for n, _f in evs if n not in C10_NAMES]
    outside = [[n, f] for n, f in evs if not frames[0] <= f <= frames[-1]]
    fb = {n: [f for m, f in evs if m == n] for n in C10_NAMES}
    dup = {n: v for n, v in fb.items() if len(v) != 1}
    order_ok = all(len(fb[n]) == 1 for n in C10_NAMES) and fb["hit_start"][0] < fb["hit_end"][0]
    checks_ok = ev_ok and not issues and not unknown and not outside and not dup and order_ok
    speed = None
    if getattr(ctx, "weapons", None):
        speed = {}
        dt = 1.0 / float(fps)
        allf = sorted(S["W"])
        for w in ctx.weapons:
            tip, _n = weapon_tip(w["V"])
            si = ctx.names.index(w["socket"])
            Pt = {}
            for f in allf:
                Mw = S["W"][f][si] @ w["T"]
                Pt[f] = Mw[:3, :3] @ tip + Mw[:3, 3]
            sp = {}
            for f in frames:
                a, b = (f - 1 if f - 1 in Pt else f), (f + 1 if f + 1 in Pt else f)
                sp[f] = float(np.linalg.norm(Pt[b] - Pt[a])) / ((b - a) * dt) if b > a else 0.0
            cpk = max(sp, key=sp.get)
            rec = {"socket": w["socket"], "tip_local_m": [rnd(x, 5) for x in tip],
                   "clip_peak": {"frame": cpk, "m_s": rnd(sp[cpk], 4)}}
            if order_ok:
                a0, b0 = fb["hit_start"][0], fb["hit_end"][0]
                win = [f for f in frames if a0 - C10_PAD <= f <= b0 + C10_PAD]
                inw = [f for f in frames if a0 <= f <= b0]
                pk = max(win, key=sp.get)
                rec.update({"window": [a0, b0], "reported_frames": [win[0], win[-1]],
                            "speed_m_s": {f: rnd(sp[f], 4) for f in win},
                            "window_peak": {"frame": pk, "m_s": rnd(sp[pk], 4), "inside_hit_window": a0 <= pk <= b0},
                            "window_min_inside_m_s": rnd(min(sp[f] for f in inw), 4),
                            "clip_peak_inside_hit_window": a0 <= cpk <= b0})
            speed[w["side"]] = rec
    return {"events": events, "fps": fps, "known_names": list(C10_NAMES), "issues": issues,
            "unknown_names": unknown, "frames_outside_range": outside, "not_exactly_once": dup,
            "hit_start_lt_hit_end": order_ok, "checks_ok": checks_ok,
            "tip_speed": speed if speed is not None else "no weapon placed (clip JSON 'weapon' absent or C8 blocked)",
            "rule": "tip = mean of the weapon mesh vertices at max local Z (U5 definition) placed as in C8 (socket world "
                    "@ contract offset); speed(f) = |tip(f+1) - tip(f-1)| / (2 / fps) (one-sided at the ends, f1+1 "
                    f"sampled); window = [hit_start - {C10_PAD}, hit_end + {C10_PAD}]"}, checks_ok


# ---------------------------------------------------------------- C11 arm vs body (T325)
C11_ARM = ("sleeve", "arm", "fist")
C11_BODY = ("tunic", "belt", "pouch", "scarf", "scarf_tail", "head")
C11_TORSO_VOL = ("tunic", "scarf", "scarf_tail")
C11_DEPTH_EPS = 0.001       # mm: depth excess counted when > this (float noise)
C11_NOTE = ("T325 / T327 calibration run (report). Baseline (T327 main decision) = an approved posed frame: clip JSON "
            "c11_baseline {clip, frame}, else the match_pose reference {clip, frame}, else the clip's own first frame; "
            "the undeformed-rest comparison is kept as info. Blocking candidate (d-05 section 5): no visible "
            "penetration beyond the baseline = for every (arm part, body part) excess visible pairs <= 0 and excess "
            "visible depth <= 0, and 0 arm vertices newly inside the torso volume, on every frame; the threshold is "
            "fixed from this distribution")


def c11_baseline(ctx, S, ms, cfg, own_clip, f0):
    """T327: (baseline metrics, record).  cfg = {clip, frame} or None (own first frame).  A reference clip other
    than this one is evaluated from rig/anim/pl_a_<clip>.blend (PL_mesh with its action, in memory; opening it
    drops the work blend, so C11 runs last)."""
    clip = str(cfg.get("clip") or own_clip) if isinstance(cfg, dict) else own_clip
    fr = int(cfg.get("frame", f0)) if isinstance(cfg, dict) else f0
    if clip == own_clip and fr in S.get("c11", {}):
        return S["c11"][fr], {"clip": clip, "frame": fr, "source": "this clip's sampled frame"}
    path = PLAYER_RIG / "anim" / f"pl_a_{clip}.blend"
    if not path.exists():
        raise Blocked(f"C11 baseline: missing input {rel(path)}")
    p1.open_blend(path)
    arm, mo, act = bpy.data.objects.get(P["arm"]), bpy.data.objects.get(P["mesh"]), bpy.data.actions.get(clip)
    if arm is None or mo is None or act is None:
        raise Blocked(f"C11 baseline {rel(path)}: {P['arm']} / {P['mesh']} / action {clip!r} missing")
    p1.assign_action(arm, act)
    arm.data.pose_position = "POSE"
    bpy.context.scene.frame_set(fr)
    V = p2d.eval_verts(mo)
    t = ctx.geo.topo
    if len(V) != t.nv:
        raise Blocked(f"C11 baseline {rel(path)}: {len(V)} verts, work mesh {t.nv}")
    return c11_metrics(ctx, V, p2d.ShellSet(t, V, t.shells), ms), \
        {"clip": clip, "frame": fr, "source": rel(path), "action": act.name}


def c11_row(ctx, S, frames, ms, rest, cfg, cfg_src, own_clip):
    base, rec = c11_baseline(ctx, S, ms, cfg, own_clip, frames[0])
    label = f"{rec['clip']} f{rec['frame']} (posed, {cfg_src})"
    m, ok = c11(ctx, S, frames, base, base_label=label)
    r, _ = c11(ctx, S, frames, rest)
    m["baseline"] = {**rec, "selected_by": cfg_src}
    m["info_vs_undeformed_rest"] = {
        "pairs_worst": {k: {kk: v[kk] for kk in ("baseline_[visible,depth_mm]", "max_excess_pairs",
                                                   "max_excess_pairs_frame", "max_excess_depth_mm",
                                                   "max_excess_depth_frame")} for k, v in r["pairs_worst"].items()},
        "torso_volume_inside": {ap: {kk: v[kk] for kk in ("inside_at_baseline", "max_new_inside_verts", "worst_frame",
                                                          "max_new_inside_depth_mm")}
                                for ap, v in r["torso_volume_inside"].items()},
        "n_frames_with_excess": len(r["frames_with_excess"])}
    return m, ok


def c11_sets(t):
    arm = {x: {f"{a}_{x}": t.polys_of_fams((a,), x) for a in C11_ARM} for x in ("l", "r")}
    return {"arm": arm, "arm_verts": {x: {k: t.verts_of_polys(v) for k, v in arm[x].items()} for x in arm},
            "body": np.concatenate([t.polys_of_fams((b,)) for b in C11_BODY]),
            "torso_shells": [sh for sh in t.shells if sh["part"] in C11_TORSO_VOL]}


def c11_metrics(ctx, V, ss, ms):
    """per arm part: visible pairs / max visible depth per body part (P2.2 rule) and the inside depth of its vertices
    in the torso volume."""
    c = ms["c11"]
    out = {"pairs": {}, "hidden": {}, "inside": {}}
    for x in ("l", "r"):
        for ap, polys in c["arm"][x].items():
            r = p2d.cross_isect(ctx.c7ctx, V, polys, c["body"], ss, with_depth=True)
            vb = r.get("visible_by_other_part") or {}
            for bp in C11_BODY:
                e = vb.get(bp) or {}
                out["pairs"][f"{ap}~{bp}"] = (int(e.get("n", 0)), float(e.get("max_depth_mm") or 0.0))
            out["hidden"][ap] = int(r.get("hidden", 0))
            _rec, best = p2d.verts_inside(ctx.c7ctx, V, c["arm_verts"][x][ap], c["torso_shells"], ss)
            out["inside"][ap] = best
    return out


def c11(ctx, S, frames, rest, base_label="rest (undeformed PL_mesh)"):
    keys = list(rest["pairs"])
    worst, series, excess_frames = {}, {}, {}
    for k in keys:
        rv, rd = rest["pairs"][k]
        best_p, best_d = (0, None), (0.0, None)
        for f in frames:
            v, d = S["c11"][f]["pairs"][k]
            ep, ed = v - rv, d - rd
            if ep > best_p[0]:
                best_p = (ep, f)
            if ed > best_d[0]:
                best_d = (ed, f)
            if v or ep > 0 or ed > C11_DEPTH_EPS:
                series.setdefault(f, {})[k] = [v, rnd(d, 3), ep, rnd(ed, 3)]
            if ep > 0 or ed > C11_DEPTH_EPS:
                excess_frames.setdefault(f, []).append(k)
        worst[k] = {"baseline_[visible,depth_mm]": [rv, rnd(rd, 3)],
                    "max_visible": max(S["c11"][f]["pairs"][k][0] for f in frames),
                    "max_depth_mm": rnd(max(S["c11"][f]["pairs"][k][1] for f in frames), 3),
                    "max_excess_pairs": best_p[0], "max_excess_pairs_frame": best_p[1],
                    "max_excess_depth_mm": rnd(best_d[0], 3), "max_excess_depth_frame": best_d[1]}
    inside = {}
    for ap, r0 in rest["inside"].items():
        ins0 = r0 > 0
        per, mx = {}, (0, None, 0.0)
        for f in frames:
            b = S["c11"][f]["inside"][ap]
            new = (b > 0) & ~ins0
            n = int(new.sum())
            dep = float(b[new].max()) * 1000.0 if n else 0.0
            if n:
                per[f] = [n, rnd(dep, 3)]
                excess_frames.setdefault(f, []).append(f"{ap}~torso_volume")
            if n > mx[0]:
                mx = (n, f, dep)
        inside[ap] = {"n_verts": int(len(r0)), "inside_at_baseline": int(ins0.sum()),
                      "max_new_inside_verts": mx[0], "worst_frame": mx[1], "worst_frame_max_depth_mm": rnd(mx[2], 3),
                      "max_new_inside_depth_mm": rnd(max((v[1] for v in per.values()), default=0.0), 3),
                      "per_frame_[new_inside_verts,max_depth_mm]": per}
    top = sorted(((w["max_excess_depth_mm"], w["max_excess_pairs"], k) for k, w in worst.items()), reverse=True)
    return {"pairs_worst": {k: v for k, v in worst.items() if v["max_visible"] or v["baseline_[visible,depth_mm]"][0]},
            "pairs_never_visible": [k for k, v in worst.items() if not v["max_visible"]
                                    and not v["baseline_[visible,depth_mm]"][0]],
            "top_excess_pairs": [{"pair": k, "max_excess_depth_mm": d, "max_excess_pairs": n,
                                  "frame_depth": worst[k]["max_excess_depth_frame"],
                                  "frame_pairs": worst[k]["max_excess_pairs_frame"]} for d, n, k in top[:10] if d > 0 or n > 0],
            "torso_volume_inside": inside,
            "torso_volume_shells": [{"shell": sh["label"], "closed": sh["closed"], "virtual": bool(sh.get("virtual")),
                                     "used": bool(sh["closed"] or sh.get("virtual"))}
                                    for sh in ctx.geo.topo.shells if sh["part"] in C11_TORSO_VOL],
            "frames_with_excess": sorted(excess_frames),
            "frames_with_excess_detail": {f: sorted(set(v)) for f, v in sorted(excess_frames.items())},
            "hidden_pairs_max_per_arm_part": {ap: max(S["c11"][f]["hidden"][ap] for f in frames) for ap in rest["hidden"]},
            "per_frame_series_[visible,depth_mm,excess_pairs,excess_depth_mm]": series,
            "rule": "P2.2 visible rule (check_p2_deform.cross_isect with depth, ShellSet over every shell of the posed "
                    "mesh): pairs grouped by the body part; baseline = " + base_label + "; excess = value - baseline value "
                    f"(depth counted when > {C11_DEPTH_EPS} mm); torso volume = tunic (virtually capped) + scarf + "
                    "scarf_tail closed shells, inside = 3-ray parity majority (check_p2_deform.verts_inside), a vertex "
                    "counts when inside at the frame and not at the baseline"}, True


# ---------------------------------------------------------------- C12 penetration (T331)
C12_TUBE = ("arm", "fist")
C12_PAIR_BODY = ("tunic", "belt", "pouch", "head")
C12_VOL = {"tunic": ("tunic",), "head": ("head",)}
C12_NOTE = ("T332 blocking: calibrated on the user-approved Sword_Idle and Sword_Attack_01 maxima (d-06 §4), "
            "regression guard. d-06 section 3 (user): sleeve-cap contact with the scarf, tunic or belt is accepted as "
            "cloth bunching (C11 report); only real penetration blocks, all measures over the C11 baseline")
# T332 limits (d-06 section 4): (limit id, measure keys, [(series, max)]) - each checked on every frame
C12_LIMITS = (
    ("fist_x_body_pairs", [f"fist_{x}~{b}" for x in ("l", "r") for b in C12_PAIR_BODY],
     [("series_excess_pairs", 0)]),
    ("fist_inside_volume", [f"fist_{x}~{v}_volume" for x in ("l", "r") for v in C12_VOL],
     [("series_new_inside_verts", 0)]),
    ("arm_x_tunic_belt_pouch", [f"arm_{x}~{b}" for x in ("l", "r") for b in ("tunic", "belt", "pouch")],
     [("series_excess_pairs", 3), ("series_excess_depth_mm", 20.0)]),
    ("arm_x_head_pairs", [f"arm_{x}~head" for x in ("l", "r")], [("series_excess_pairs", 0)]),
    ("arm_inside_tunic_volume", [f"arm_{x}~tunic_volume" for x in ("l", "r")],
     [("series_new_inside_verts", 8), ("series_max_depth_mm", 40.0)]),
    ("arm_inside_head_volume", [f"arm_{x}~head_volume" for x in ("l", "r")],
     [("series_new_inside_verts", 4), ("series_max_depth_mm", 12.0)]),
    ("sleeve_x_head", [f"sleeve_{x}~head" for x in ("l", "r")],
     [("series_excess_pairs", 22), ("series_excess_depth_mm", 13.0)]),
)


def c12_limits(meas, frames):
    """T332: every limit on every frame -> ({limit: {...}}, failed limit ids)."""
    out, failed = {}, []
    for lid, keys, checks in C12_LIMITS:
        rec = {"measures": keys, "limits": {s: mx for s, mx in checks}, "failures": {}}
        for k in keys:
            m = meas.get(k)
            if m is None:
                rec["failures"][k] = "measure missing"
                continue
            for s, mx in checks:
                bad = [[f, v] for f, v in zip(frames, m[s]) if v > mx]
                if bad:
                    rec["failures"].setdefault(k, {})[s] = {"max": max(v for _f, v in bad), "limit": mx,
                                                            "frames": [f for f, _v in bad]}
        rec["ok"] = not rec["failures"]
        if not rec["ok"]:
            failed.append(lid)
        out[lid] = rec
    return out, failed


def c12_metrics(ctx, V, ss, ms):
    """per arm tube / fist part: inside depth of its vertices in the tunic-only volume and in the head volume."""
    c = ms["c11"]
    out = {}
    for vol, parts in C12_VOL.items():
        shells = [sh for sh in ctx.geo.topo.shells if sh["part"] in parts]
        for x in ("l", "r"):
            for a in C12_TUBE:
                ap = f"{a}_{x}"
                _rec, best = p2d.verts_inside(ctx.c7ctx, V, c["arm_verts"][x][ap], shells, ss)
                out[f"{ap}~{vol}_volume"] = best
    return out


def c12_baseline(ctx, S, ms, cfg, own_clip, f0):
    """same baseline rule as C11 (c11_baseline): -> (c11 metrics, c12 metrics, record)."""
    clip = str(cfg.get("clip") or own_clip) if isinstance(cfg, dict) else own_clip
    fr = int(cfg.get("frame", f0)) if isinstance(cfg, dict) else f0
    if clip == own_clip and fr in S.get("c12", {}):
        return S["c11"][fr], S["c12"][fr], {"clip": clip, "frame": fr, "source": "this clip's sampled frame"}
    path = PLAYER_RIG / "anim" / f"pl_a_{clip}.blend"
    if not path.exists():
        raise Blocked(f"C12 baseline: missing input {rel(path)}")
    p1.open_blend(path)
    arm, mo, act = bpy.data.objects.get(P["arm"]), bpy.data.objects.get(P["mesh"]), bpy.data.actions.get(clip)
    if arm is None or mo is None or act is None:
        raise Blocked(f"C12 baseline {rel(path)}: {P['arm']} / {P['mesh']} / action {clip!r} missing")
    p1.assign_action(arm, act)
    arm.data.pose_position = "POSE"
    bpy.context.scene.frame_set(fr)
    V = p2d.eval_verts(mo)
    t = ctx.geo.topo
    if len(V) != t.nv:
        raise Blocked(f"C12 baseline {rel(path)}: {len(V)} verts, work mesh {t.nv}")
    ss = p2d.ShellSet(t, V, t.shells)
    return c11_metrics(ctx, V, ss, ms), c12_metrics(ctx, V, ss, ms), \
        {"clip": clip, "frame": fr, "source": rel(path), "action": act.name}


def c12(ctx, S, frames, ms, cfg, cfg_src, own_clip):
    b11, b12, rec = c12_baseline(ctx, S, ms, cfg, own_clip, frames[0])
    meas, any_f = {}, {}

    def pair_measure(key, group):
        bv, bd = b11["pairs"][key]
        ep = [S["c11"][f]["pairs"][key][0] - bv for f in frames]
        ed = [S["c11"][f]["pairs"][key][1] - bd for f in frames]
        ip, idd = int(np.argmax(ep)), int(np.argmax(ed))
        for f, a, d in zip(frames, ep, ed):
            if a > 0 or d > C11_DEPTH_EPS:
                any_f.setdefault(f, []).append(key)
        meas[key] = {"group": group, "baseline_[visible,depth_mm]": [bv, rnd(bd, 3)],
                     "max_excess_pairs": ep[ip], "max_excess_pairs_frame": frames[ip] if ep[ip] > 0 else None,
                     "max_excess_depth_mm": rnd(ed[idd], 3),
                     "max_excess_depth_frame": frames[idd] if ed[idd] > C11_DEPTH_EPS else None,
                     "series_excess_pairs": ep, "series_excess_depth_mm": [rnd(v, 3) for v in ed]}
    for x in ("l", "r"):
        for a in C12_TUBE:
            for bp in C12_PAIR_BODY:
                pair_measure(f"{a}_{x}~{bp}", "a")
        pair_measure(f"sleeve_{x}~head", "b")
    for key in b12:
        ins0 = b12[key] > 0
        n, dep = [], []
        for f in frames:
            b = S["c12"][f][key]
            new = (b > 0) & ~ins0
            k = int(new.sum())
            n.append(k)
            dep.append(float(b[new].max()) * 1000.0 if k else 0.0)
            if k:
                any_f.setdefault(f, []).append(key)
        i = int(np.argmax(n))
        j = int(np.argmax(dep))
        meas[key] = {"group": "c", "n_verts": int(len(b12[key])), "inside_at_baseline": int(ins0.sum()),
                     "max_new_inside_verts": n[i], "max_new_inside_verts_frame": frames[i] if n[i] else None,
                     "max_new_inside_depth_mm": rnd(dep[j], 3), "max_new_inside_depth_frame": frames[j] if dep[j] else None,
                     "series_new_inside_verts": n, "series_max_depth_mm": [rnd(v, 3) for v in dep]}
    summary = {k: ([v["max_excess_pairs"], v["max_excess_pairs_frame"], v["max_excess_depth_mm"],
                    v["max_excess_depth_frame"]] if v["group"] in ("a", "b") else
                   [v["max_new_inside_verts"], v["max_new_inside_verts_frame"], v["max_new_inside_depth_mm"],
                    v["max_new_inside_depth_frame"]]) for k, v in meas.items()}
    vols = {vol: [{"shell": sh["label"], "closed": sh["closed"], "virtual": bool(sh.get("virtual"))}
                  for sh in ctx.geo.topo.shells if sh["part"] in parts] for vol, parts in C12_VOL.items()}
    lim, failed = c12_limits(meas, frames)   # T332
    return {"baseline": {**rec, "selected_by": cfg_src}, "frames": [frames[0], frames[-1]],
            "failed_limits": failed, "limits": lim,
            "summary_[max_a,frame,max_b,frame] (a/b: excess pairs, depth mm; c: new inside verts, depth mm)": summary,
            "frames_with_any_gt0": sorted(any_f),
            "frames_with_any_gt0_detail": {f: sorted(set(v)) for f, v in sorted(any_f.items())},
            "measures": meas, "volumes": vols,
            "rule": "(a) / (b): the C11 per-frame pair measure (P2.2 visible rule, check_p2_deform.cross_isect with "
                    "depth) minus the same pair at the baseline; (c) check_p2_deform.verts_inside (3-ray parity) of "
                    "the arm tube / fist vertices in the tunic shell (virtually capped) and in the head shell, "
                    f"counted when inside at the frame and not at the baseline; > 0 = pairs > 0, depth > "
                    f"{C11_DEPTH_EPS} mm, verts > 0; series aligned to the frames; T332: a limit fails on any frame "
                    "whose series value is > the limit"}, not failed


# ---------------------------------------------------------------- evidence
IDS = (
    ("C1", "blocking", f"every CTRL_* channel inside its effective sweep (manifest + clip sweep_overrides; channels "
                       f"without a sweep entry = 0, scale = 1, unlisted CTRL_* = identity) and every PROPS value "
                       f"inside its range, every frame, tol {VAL_TOL:g}; resolved JSON sweeps = derived sweeps"),
    ("C2", "blocking", f"branch flips 0 (G6.5b) and sub-frame pops 0 (G6.5 bisection); k ratio distribution report "
                       f"(draft k = {K_DRAFT:g}, {CALIB})"),
    ("C3", "blocking", f"planted ranges: cumulative contact slip <= {PLANT_SLIP * 1000:g} mm and lowest shoe z in "
                       f"[{PLANT_Z[0] * 1000:g}, {PLANT_Z[1] * 1000:g}] mm every frame (G6.7); >= 1 range declared"),
    ("C4", "blocking", f"lowest PL_mesh vertex z >= -{GROUND_PEN * 1000:g} mm on every judged frame (ground_ranges, "
                       "else every frame)"),
    ("C5", "blocking", f"locked / in_place_cycle: DEF Root world vs rest and CTRL_root basis <= {ROOT_TOL:g}; "
                       f"in_place_cycle: loop seam DEF + CTRL <= {SEAM_POS * 1000:g} mm / {SEAM_ROT:g} deg, PROPS <= "
                       f"{SEAM_PROP:g}; root: path reported"),
    ("C6", "report", "skirt poke-out springs off per frame (P2.8b rule), max / worst frames, knee vs thigh split"),
    ("C7", "report", "visible self-intersection per frame: fist vs head / torso, sleeve vs scarf, arm vs tunic (P2.2)"),
    ("C8", "blocking", f"(only when the clip JSON has 'weapon') weapon vs body intersecting triangle pairs = "
                       f"{C8_DRAFT_PAIRS} on every frame, holding fist excluded; blade clearance mm reported"),
    ("C9", "blocking", f"(only when the clip JSON has 'match_pose') every DEF bone world at the clip's first / last "
                       f"frame vs the reference clip frame <= {C9_POS * 1000:g} mm / {C9_ROT:g} deg and PROPS equal "
                       f"(<= {C9_PROP:g}); ok = blocking_candidate_ok (d-05 section 8)"),
    ("C10", "report", f"(only when the clip JSON has 'events') names in {list(C10_NAMES)}, each once, frames inside the "
                      f"range, hit_start < hit_end; blade tip speed m/s inside the window +- {C10_PAD} frames and the "
                      f"peak frame (d-05 SA1.C10)"),
    ("C11", "report", "C11 arm vs body (all pairs, report): sleeve / arm / fist (L, R) x tunic, belt, pouch, scarf, "
                      "scarf_tail, head "
                      "visible pairs and depth (P2.2 rule) and arm vertices inside the torso volume, each as the excess "
                      "over the baseline (approved posed frame, T327); blocking candidate: no visible penetration beyond "
                      "the baseline (d-05 section 5)"),
    ("C12", "blocking", "C12 penetration, over the C11 baseline, every frame (d-06 section 4): fist x tunic / belt / "
                        "pouch / head 0 excess pairs and fist vertices newly inside the tunic / head volume 0; arm tube "
                        "x tunic / belt / pouch <= 3 excess pairs and <= 20 mm excess depth; arm tube x head 0 excess "
                        "pairs; arm tube newly inside the tunic volume <= 8 vertices and <= 40 mm; inside the head "
                        "volume <= 4 vertices and <= 12 mm; sleeve x head <= 22 excess pairs and <= 13 mm"),
)


class Evidence(p2d.Evidence):
    def write(self):
        out = Path(P["out"])
        out.parent.mkdir(parents=True, exist_ok=True)
        order = {cid(i): k for k, (i, _s, _t) in enumerate(IDS)}
        self.criteria.sort(key=lambda c: order.get(c["id"], 99))
        doc = {"gate": P["prefix"] or P["clip"], "checker": CHECKER, "task": TASK, "clip": P["clip"],
               "blender": bpy.app.version_string, **jsonable(self.extra), "inputs": self.inputs,
               "criteria": self.criteria}
        with open(out, "w", encoding="utf-8", newline="\n") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            print(f"[{CHECKER}] {c['id']} ({c['status']}) ok={c['ok']} measured={json.dumps(c['measured'])[:300]}")
        print(f"[{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


def cid(i):
    return f"{P['prefix']}.{i}" if P["prefix"] else i


_STATE = {"missing": []}


def parse_args(argv):
    keys = {"--" + k.replace("_", "-"): k for k in P}
    i = 0
    while i < len(argv):
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: --clip <name> {' '.join(sorted(keys))} <value>")
        k = keys[argv[i]]
        P[k] = Path(argv[i + 1]) if k in PATH_KEYS else argv[i + 1]
        i += 2
    if not P["clip"]:
        raise ValueError("--clip <name> is required")
    c = P["clip"]
    P["action"] = P["action"] or c
    P["work"] = P["work"] or PLAYER_RIG / "anim" / f"pl_a_{c}.blend"
    P["clip_json"] = P["clip_json"] or PLAYER_RIG / "data" / "clips" / f"{c}.json"
    P["resolved"] = P["resolved"] or PLAYER_RIG / "anim" / f"{c}_resolved.json"
    P["out"] = P["out"] or REPO / "work" / "player" / "inspect" / "clips" / c / "check_clip.json"


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    for m in (__file__, p1.__file__, p2d.__file__, p2s.__file__):
        ev.add_input(m)
    ctx = Ctx()
    th = {i: t for i, _s, t in IDS}
    st = {i: s for i, s, _t in IDS}
    ids = [i for i, _s, _t in IDS]
    done = set()

    def put(i, measured, ok, note=""):
        okv = bool(ok) if st[i] == "blocking" else True
        nt = note or (CALIB if st[i] == "report" else "")
        if st[i] == "report" and not ok:
            nt += " (component flag false)"
        ev.criterion(cid(i), measured, th[i], okv, nt)
        ev.criteria[-1]["status"] = st[i]
        done.add(i)

    def block(cids, reason):
        for i in cids:
            if i not in done:
                ev.criterion(cid(i), f"blocked: {reason}", th[i], False, "")
                ev.criteria[-1]["status"] = st[i]
                done.add(i)

    try:
        required = ("work", "clip_json", "manifest", "parts", "loops", "canon")
        miss = [rel(P[k]) for k in required if not Path(P[k]).exists()]
        for k in required + ("resolved",):
            ev.add_input(P[k])
        for q in sorted(HERE.glob("p30*.py")) + sorted(HERE.glob("p31*.py")):
            ev.add_input(q)
        _STATE["missing"] = miss
        ev.extra["names"] = {k: str(P[k]) for k in ("clip", "prefix", "action", "arm", "mesh")}
        if miss:
            block(ids, "missing input " + ", ".join(miss))
            return
        ctx.man = p2d.load_json(P["manifest"])
        ctx.clip = p2d.load_json(P["clip_json"])
        ctx.res = p2d.load_json(P["resolved"]) if Path(P["resolved"]).exists() else None
        canon = p2d.load_json(P["canon"])
        ctx.names = [b["name"] for b in canon["bones"]]
        ctx.parent = {b["name"]: b.get("parent") for b in canon["bones"]}
        slot = open_work(ctx)
        sc = bpy.context.scene
        # clip definition
        fps, fps_src = pick(ctx.res, ctx.clip, "fps", FPS_DEFAULT)
        fr, fr_src = pick(ctx.res, ctx.clip, "frame_range", None)
        if fr is None:
            fs_, _s1 = pick(ctx.res, ctx.clip, "frame_start", None)
            fe_, _s2 = pick(ctx.res, ctx.clip, "frame_end", None)
            fr, fr_src = ([fs_, fe_], _s1) if fs_ is not None and fe_ is not None else (None, "default")
        act_range = [int(round(x)) for x in ctx.act.frame_range]
        if fr is None:
            fr, fr_src = act_range, "action frame_range"
        f0, f1 = int(fr[0]), int(fr[1])
        frames = list(range(f0, f1 + 1))
        loop, loop_src = pick(ctx.res, ctx.clip, "loop", None)
        seam, seam_src = pick(ctx.res, ctx.clip, "loop_seam", "first_eq_last")
        rm, rm_src = pick(ctx.res, ctx.clip, "root_motion", None)
        policy = rm.get("policy") if isinstance(rm, dict) else rm
        ov = (ctx.clip or {}).get("sweep_overrides")
        contacts, ct_src = pick(ctx.res, ctx.clip, "contacts", {})
        contacts = contacts if isinstance(contacts, dict) else {}
        loop_pair = (f0, f1 + 1) if seam == "first_eq_last_plus_1" else (f0, f1)
        scene_fps = p1.fps_of(sc)
        ctx.info = {"blend": rel(P["work"]), "action": ctx.act.name, "slot": slot, "fps": fps, "fps_source": fps_src,
                    "scene_fps": scene_fps, "fps_matches_scene": abs(float(scene_fps) - float(fps)) < 1e-6,
                    "frame_range": [f0, f1], "frame_range_source": fr_src, "action_frame_range": act_range,
                    "scene_frame_start_end": [sc.frame_start, sc.frame_end],
                    "frame_range_matches_scene": [sc.frame_start, sc.frame_end] == [f0, f1],
                    "loop": loop, "loop_source": loop_src, "loop_seam": seam, "loop_seam_source": seam_src,
                    "root_motion": policy, "root_motion_source": rm_src, "contacts_source": ct_src,
                    "resolved_json": rel(P["resolved"]) if ctx.res is not None else "absent",
                    "n_fcurves": len(p1.fcurves_of(ctx.act))}
        ev.extra["clip_info"] = ctx.info
        if seam not in SEAMS:
            ev.extra["clip_info"]["loop_seam_issue"] = f"unknown loop_seam {seam!r}"
        # geometry
        parts = p2s.parts_map(p2d.load_json(P["parts"]))
        gctx = type("GCtx", (), {})()
        gctx.parts, gctx.loops = parts, p2d.load_json(P["loops"])
        me = ctx.mo.data
        co = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", co)
        Mw = np.array(ctx.mo.matrix_world, dtype=np.float64)
        V0 = co.reshape(-1, 3) @ Mw[:3, :3].T + Mw[:3, 3]
        ctx.geo = g = p2s.Geo(gctx, ctx.mo, V0)
        t = g.topo
        ctx.hem_z = float(V0[g.hem, 2].min()) if len(g.hem) else float("inf")
        ctx.leg_sets = {x: {"knee": g.leg_verts[x][V0[g.leg_verts[x], 2] < ctx.hem_z],
                            "thigh": g.leg_verts[x][V0[g.leg_verts[x], 2] >= ctx.hem_z]} for x in ("l", "r")}
        cz = (gctx.loops.get("tunic") or {}).get("ceiling_z")
        c7ctx = type("C7Ctx", (), {})()
        c7ctx.topo = t
        tp = t.polys_of_fams(("tunic", "skirt"))
        c7ctx.ceiling_polys = ({int(q) for q in tp.tolist()
                                if np.all(np.abs(V0[t.poly_verts[int(q)], 2] - float(cz)) <= p2d.CEIL_TOL)}
                               if isinstance(cz, (int, float)) else set())
        ctx.c7ctx = c7ctx
        mesh_sets = {"shoe": {x: t.verts_of_polys(t.polys_of_fams(("shoe",), x)) for x in ("l", "r")},
                     "leg": ctx.leg_sets,
                     "c7": {item: {x: (t.polys_of_fams(a, x), t.polys_of_fams(b)) for x in ("l", "r")}
                            for item, a, b in C7_ITEMS}}
        mesh_sets["c11"] = c11_sets(t)   # T325
        chains = {}
        for k, ch in ((ctx.man or {}).get("ikfk") or {}).items():
            d = ch.get("def") or []
            if len(d) >= 2 and d[0] in ctx.arm.pose.bones and d[1] in ctx.arm.pose.bones:
                chains[k] = (d[0], d[1])
        weapon, wp_src = pick(ctx.res, ctx.clip, "weapon", None)
        ctx.info["weapon"], ctx.info["weapon_source"] = weapon, wp_src
        c8_block = None
        if weapon is None:
            ids.remove("C8")
        else:
            try:
                load_weapons(ctx, weapon)
            except (Blocked, p2d.Blocked) as e:
                ctx.weapons, c8_block = None, str(e)
            except Exception:
                ctx.weapons, c8_block = None, "weapon load error: " + tb_tail(4)
        mp, mp_src = pick(ctx.res, ctx.clip, "match_pose", None)     # T321 C9
        evts, evts_src = pick(ctx.res, ctx.clip, "events", None)    # T321 C10
        ctx.info.update({"match_pose": mp, "match_pose_source": mp_src, "events": evts, "events_source": evts_src})
        if mp is None:
            ids.remove("C9")
        if evts is None:
            ids.remove("C10")
        extra_frames = [f1 + 1]
        t0 = time.time()
        S = sample(ctx, frames, extra_frames, chains, mesh_sets)
        ev.extra["sampling_seconds"] = rnd(time.time() - t0, 1)
        # C1
        sweeps, applied, issues = derive_sweeps(ctx.man, ov)
        rsw = (ctx.res or {}).get("sweeps") if isinstance(ctx.res, dict) else None
        sinfo = {"source": "manifest controls.<CTRL>.sweep + clip JSON sweep_overrides (derived here)",
                 "overrides_reason": (ov or {}).get("_reason") if isinstance(ov, dict) else None,
                 "overrides_applied": applied, "issues": issues,
                 "resolved_vs_derived": compare_sweeps(sweeps, rsw) if rsw is not None else None,
                 "resolved_sweeps_present": rsw is not None}
        hp = ctx.arm.pose.bones.get("PROPS")
        pr = prop_ranges(ctx.man, hp)
        steps = [("C1", lambda: c1(ctx, S, frames, sweeps, sinfo, pr)),
                 ("C2", lambda: c2(ctx, S, frames, chains, loop_pair)),
                 ("C3", lambda: c3(ctx, S, *parse_planted(contacts.get("planted"), f0, f1),
                                   declared_empty=contacts.get("planted") == [])),
                 ("C4", lambda: c4(ctx, S, frames, *parse_ground(contacts.get("ground_ranges"), f0, f1))),
                 ("C5", lambda: c5(ctx, S, frames, policy, seam, loop_pair)),
                 ("C6", lambda: c6(ctx, S, frames)),
                 ("C7", lambda: c7(ctx, S, frames, c7_metrics(ctx, V0, p2d.ShellSet(t, V0, t.shells), mesh_sets)))]
        c11_rest = c11_metrics(ctx, V0, p2d.ShellSet(t, V0, t.shells), mesh_sets)   # T327: info only
        c11_cfg, c11_src = pick(ctx.res, ctx.clip, "c11_baseline", None)
        if c11_cfg is not None:
            c11_src = f"c11_baseline ({c11_src})"
        elif isinstance(mp, dict) and mp.get("clip"):
            c11_cfg, c11_src = mp, "match_pose"
        else:
            c11_src = "own first frame (no c11_baseline / match_pose)"
        if weapon is not None and c8_block is None:
            steps.append(("C8", lambda: c8(ctx, S, frames)))
        elif c8_block is not None:
            block(["C8"], c8_block)
        if evts is not None:
            steps.append(("C10", lambda: c10(ctx, S, frames, evts, fps)))
        if mp is not None:   # last: opens the reference blend (the work rig is no longer valid afterwards)
            if isinstance(mp, dict) and mp.get("clip"):
                steps.append(("C9", lambda: c9(ctx, S, frames, mp)))
            else:
                block(["C9"], f"match_pose {mp!r} needs at least {{clip, frame}}")
        # T327: C11 last (a reference-clip baseline opens that clip's anim blend)
        steps.append(("C11", lambda: c11_row(ctx, S, frames, mesh_sets, c11_rest, c11_cfg, c11_src, P["clip"])))
        steps.append(("C12", lambda: c12(ctx, S, frames, mesh_sets, c11_cfg, c11_src, P["clip"])))   # T331
        notes ={"C1": C1_NOTE, "C8": C8_NOTE, "C9": C9_NOTE, "C11": C11_NOTE, "C12": C12_NOTE}
        for i, fn in steps:
            try:
                put(i, *fn(), note=notes.get(i, ""))
            except Exception:
                block([i], "row error: " + tb_tail(4))
                _STATE.setdefault("row_errors", []).append(i)
    except (Blocked, p1.Blocked, p2d.Blocked, p2s.Blocked) as e:
        block(ids, str(e))
    finally:
        try:
            block(ids, "checker aborted before this criterion: " + tb_tail(2) if sys.exc_info()[0] else
                  "checker aborted before this criterion")
        finally:
            ev.write()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(2)
    if _STATE["missing"] or _STATE.get("row_errors"):
        print("missing input: " + ", ".join(_STATE["missing"]) if _STATE["missing"] else
              f"row errors: {_STATE['row_errors']}")
        sys.stdout.flush()
        sys.exit(2)
