"""p30_clip_anim.py - T300 (P3 tooling, spec work/player/d-03-player-anim-tools.md section 2): clip JSON -> action.

Input : the rig blend opened by Blender (work/player/rig/pl_r04_ctrl.blend; never saved over),
        work/player/rig/data/clips/<clip>.json (authored clip, read only),
        work/player/rig/data/ctrl_manifest.json (sweeps / clamps / spaces / ikfk / props spec, read only)
Output: work/player/rig/anim/pl_a_<clip>.blend   (save copy): action <clip> on PL_rig (slot OBPL_rig, fake user,
                                                  use_frame_range), scene fps / fps_base 1 / frame_start / frame_end
        work/player/rig/anim/<clip>_resolved.json  what was keyed, effective sweeps, contacts, raw per-frame values
                                                  (contract for rig/scripts/check_player_clip.py; no verdicts)
        work/player/inspect/clips/<clip>/          key_f###_<view>.png (front / side / three_quarter at the key frames),
                                                  strip_f###_<view>.png (every Nth frame, front / side),
                                                  keyposes_sheet.png, strip_sheet.png (Workbench MATERIAL, as p23 / p24e)

CLI:  blender -b --factory-startup work/player/rig/pl_r04_ctrl.blend --python p30_clip_anim.py -- --clip <name>
      debug: --no-render (no PNGs), --no-save (no blend / resolved JSON)

Copied / adapted from work/goblin_swing/rig/scripts/s12_clip_anim.py (goblin file unchanged):
- apply_sweep_overrides: s12_clip_anim.py:104-130 (widening only, in memory, _reason required, RuntimeError otherwise)
- action build + interpolation incl. interpolation.props_overrides: s12_clip_anim.py:177-228 (PROP_INTERP s12:72)
- base = reset_rig + ready_pose through the rig's own UI operators: s12_clip_anim.py:331-342 (player.* here)
- saving: s12_clip_anim.py:506-509 (save_as_mainfile copy=True, save_version 0)
and from work/player/rig/scripts/p25_rigtest.py: PROPS ranges (_prop_range p25:111-120), channel / sweep asserts
(_write p25:98-109, range_report p25:428-450), rot_path p25:55-57, keying p25:83-91.
Renders: p23_skin.py:347-445 / p24e_rig_ui.py:191-262 (orthographic Workbench, STUDIO light, MATERIAL colours, film
transparent), composed over white as check_p2_deform.py:1082-1123.
Player difference from s12: keys are authored per control channel (frame -> value), not as key poses + breakdowns.

Clip JSON (authored; unknown top-level keys starting with "_" are ignored, any other unknown key -> RuntimeError):
  clip            str, = file name (optional; must equal --clip when given). Characters [A-Za-z0-9_].
  action          str, default = clip (must equal clip: the FBX AnimStack / Unity clip name).
  fps             int > 0, default 30.
  frame_range     [a, b] ints, a < b, inclusive.
  loop            bool. true: every keyed channel gets a closing key at b = its value at a unless the JSON keys b
                  (then it must equal the value at a, else RuntimeError).
  root_motion     "locked" | "in_place_cycle" | "root". locked / in_place_cycle: CTRL_root keys must equal the base
                  (identity) -> else RuntimeError; in_place_cycle requires loop true. root: CTRL_root may be keyed.
  base            "player.reset_rig + player.ready_pose" (the only supported base, default).
  key_frames      [int] frames of the key-pose sheets / Unity renders (default: every JSON-keyed frame).
  keys            {CTRL_*: {"<frame>": {"loc": [x, y, z] | {"X": v, ..}, "rot": [x, y, z] | {"Y": v, ..}}}}
                  absolute control values (loc m, rot Euler degrees, the control's XYZ mode, bone-local axes as
                  ctrl_manifest _doc.sweep). A list keys all three axes, a dict only the named axes.
  props           {PROPS name: {"<frame>": value}} (float props: within the clamp / props spec range; int props:
                  integer within the space / spec range).
  interpolation   {ctrl: {type: BEZIER|LINEAR|CONSTANT, handles: AUTO_CLAMPED|..}  (default BEZIER / AUTO_CLAMPED),
                   ctrl_overrides: {CTRL_*: {type, handles}} (optional),
                   props: type (default CONSTANT),
                   props_overrides: {prop: BEZIER|LINEAR|CONSTANT} (default {"skirt_follow": "BEZIER"}, d-03 1;
                   int props must stay CONSTANT)}.
  sweep_overrides {"_reason": text, CTRL_*: [{channel "loc"|"rot", axis "X"|"Y"|"Z", min, max}]} (s12 rule).
  contacts        {planted: [{foot "l"|"r", frames [a, b]}], ground_ranges: [[a, b], ..]} (frames inside
                  frame_range; used by the checker: C3 planted slip / height, C4 ground).
  sheet           {every_n: int >= 1} (strip spacing, default 3).
  events          [{name, frame}] (optional, T320): name in EVENT_NAMES (hit_start / hit_end; others refused), frame an
                  integer inside frame_range, no other item keys. Written to the resolved JSON as given (sorted by frame);
                  Unity AnimationEvents come later.
  match_pose      {clip, frame, at} (optional, T320 addendum): the frames `at` ("first" / "last") of this clip are meant to
                  equal <clip> frame <frame> (checker C9). Types validated, passed through to the resolved JSON.
  weapon          {"R" | "L": <name>} (optional, T310): preview only. rig/weapons/<name>.blend holds exactly one WPN_*
                  object; for the sheets it is appended after the blend is saved and attached to WeaponSocket_<side> by a
                  Child Of constraint without inverse (data/weapon_socket_contract.json: pivot = grip centre = socket
                  head, blade +Z, guard Y), then removed. Never in the saved anim blend, the stage or the FBX.
Base state: every CTRL_* location / rotation_euler / scale axis and every numeric PROPS property is keyed at frame a with
the base value (reset_rig + ready_pose) unless the JSON keys that channel at a. Channels the JSON keys get their keys at
the JSON frames, + frame a = base when the JSON does not key a, + the loop closing key. A value on an axis without a
sweep entry must equal the base value.
Keys outside the effective sweep (manifest + sweep_overrides) -> RuntimeError (nothing is written).

Self measurement (resolved JSON, raw values, no verdicts): per frame every CTRL channel against the effective sweep
(interpolated values outside, e.g. BEZIER overshoot), DEF Root world path, loop seam (DEF world frame b vs a), mode
warnings (a JSON key on a control whose ik/fk mode prop is not the control's mode at that frame).
"""
import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import bpy
import numpy as np
from bpy_extras import anim_utils
from mathutils import Matrix, Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
DATA = RIG / "data"
CLIPS = DATA / "clips"
ANIM = RIG / "anim"
INSPECT = RIG.parent / "inspect" / "clips"
WEAPONS = RIG / "weapons"
CTRL_MAN = DATA / "ctrl_manifest.json"
CANON = DATA / "canonical_skeleton.json"
ARM = "PL_rig"
MESH = "PL_mesh"
PROPS = "PROPS"
TEXT_UI = "player_rig_ui.py"
BASE = "player.reset_rig + player.ready_pose"
ROOT_POLICIES = ("locked", "in_place_cycle", "root")
PROP_INTERP = ("BEZIER", "LINEAR", "CONSTANT")        # s12_clip_anim.py:72
DEFAULT_PROPS_OVERRIDES = {"skirt_follow": "BEZIER"}  # d-03 section 1: skirt_follow keyed, BEZIER
AXES = "XYZ"
EPS = 1e-6
KNOWN_KEYS = {"clip", "action", "fps", "frame_range", "loop", "root_motion", "base", "key_frames", "keys", "props",
              "interpolation", "sweep_overrides", "contacts", "sheet", "weapon", "events", "match_pose"}
EVENT_NAMES = ("hit_start", "hit_end")   # T320
VIEWS_KEY = ("front", "side", "three_quarter")
VIEWS_STRIP = ("front", "side")
RES_KEY = 640
RES_STRIP = 400
CELL_GAP = 6


def log(msg):
    print(f"[p30] {msg}")
    sys.stdout.flush()


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def rel(p):
    try:
        return Path(p).resolve().relative_to(REPO).as_posix()
    except ValueError:
        return str(p)


def rot_path(pb):   # p25_rigtest.py:55-57
    return {"QUATERNION": "rotation_quaternion", "AXIS_ANGLE": "rotation_axis_angle"}.get(pb.rotation_mode,
                                                                                         "rotation_euler")


def refresh(rig):   # playbook section 5: drivers need update_tag + view_layer.update
    rig.update_tag()
    bpy.context.view_layer.update()


def mdiff(a, b):
    q = a.to_quaternion().conjugated() @ b.to_quaternion()
    ang = 2.0 * math.atan2(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z), abs(q.w))
    return (a.translation - b.translation).length * 1000.0, math.degrees(ang)


# ---------------------------------------------------------------- clip JSON
def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="p30_clip_anim.py")
    ap.add_argument("--clip", required=True)
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--no-save", action="store_true")
    return ap.parse_args(argv)


def frame_of(k, fa, fb, where):
    try:
        f = int(k)
    except (TypeError, ValueError):
        raise RuntimeError(f"{where}: frame {k!r} is not an integer")
    if str(f) != str(k).strip() and not isinstance(k, int):
        raise RuntimeError(f"{where}: frame {k!r} is not an integer")
    if not fa <= f <= fb:
        raise RuntimeError(f"{where}: frame {f} outside frame_range [{fa}, {fb}]")
    return f


def load_clip(name):
    if not re.fullmatch(r"[A-Za-z0-9_]+", name):
        raise RuntimeError(f"clip name {name!r}: only [A-Za-z0-9_] (Unity Player@<clip>.fbx rule)")
    path = CLIPS / f"{name}.json"
    clip = json.loads(path.read_text(encoding="utf-8"))
    unknown = sorted(k for k in clip if not k.startswith("_") and k not in KNOWN_KEYS)
    if unknown:
        raise RuntimeError(f"{path.name}: unknown keys {unknown}")
    if clip.get("clip", name) != name:
        raise RuntimeError(f"{path.name}: clip {clip.get('clip')!r} != --clip {name!r}")
    if clip.get("action", name) != name:
        raise RuntimeError(f"{path.name}: action {clip.get('action')!r} must equal the clip name {name!r}")
    fps = clip.get("fps", 30)
    if not isinstance(fps, int) or fps <= 0:
        raise RuntimeError(f"fps {fps!r}: positive int required")
    fr = clip.get("frame_range")
    if not (isinstance(fr, list) and len(fr) == 2 and all(isinstance(x, int) for x in fr) and fr[0] < fr[1]):
        raise RuntimeError(f"frame_range {fr!r}: [a, b] ints with a < b required")
    if not isinstance(clip.get("loop"), bool):
        raise RuntimeError(f"loop {clip.get('loop')!r}: bool required")
    rm = clip.get("root_motion")
    if rm not in ROOT_POLICIES:
        raise RuntimeError(f"root_motion {rm!r}: one of {ROOT_POLICIES}")
    if rm == "in_place_cycle" and not clip["loop"]:
        raise RuntimeError("root_motion in_place_cycle requires loop true")
    if clip.get("base", BASE) != BASE:
        raise RuntimeError(f"unsupported base {clip.get('base')!r} (only {BASE!r})")
    wp = clip.get("weapon") or {}
    if not isinstance(wp, dict) or any(s not in ("R", "L") or not isinstance(n, str) for s, n in wp.items()):
        raise RuntimeError(f"weapon {wp!r}: {{\"R\" | \"L\": name}} required")
    for n in wp.values():
        if not (WEAPONS / f"{n}.blend").exists():
            raise RuntimeError(f"weapon {n}: {WEAPONS / (n + '.blend')} missing (p32_weapon_placeholder.py)")
    ev = clip.get("events")
    if ev is not None:   # T320
        if not isinstance(ev, list):
            raise RuntimeError(f"events {ev!r}: list of {{name, frame}} required")
        fa, fb = clip["frame_range"]
        for e in ev:
            if not isinstance(e, dict) or set(e) != {"name", "frame"}:
                raise RuntimeError(f"events item {e!r}: exactly {{name, frame}} required")
            if e["name"] not in EVENT_NAMES:
                raise RuntimeError(f"events: unknown name {e['name']!r} (known {EVENT_NAMES})")
            if not isinstance(e["frame"], int) or not fa <= e["frame"] <= fb:
                raise RuntimeError(f"events: {e['name']} frame {e['frame']!r} not an integer inside [{fa}, {fb}]")
    mp = clip.get("match_pose")
    if mp is not None:   # T320 addendum
        if (not isinstance(mp, dict) or set(mp) != {"clip", "frame", "at"} or not isinstance(mp["clip"], str)
                or not isinstance(mp["frame"], int) or not isinstance(mp["at"], list) or not mp["at"]
                or any(a not in ("first", "last") for a in mp["at"])):
            raise RuntimeError(f"match_pose {mp!r}: {{clip: str, frame: int, at: ['first' | 'last', ..]}} required")
    return path, clip


def attach_weapons(weapon, rig):
    """T310: append each weapon's single WPN_* object and attach it to WeaponSocket_<side> (Child Of, no inverse)."""
    out = []
    for side, n in sorted(weapon.items()):
        path = WEAPONS / f"{n}.blend"
        with bpy.data.libraries.load(str(path), link=False) as (src, dst):
            names = [o for o in src.objects if o.startswith("WPN_")]
            if len(names) != 1:
                raise RuntimeError(f"{path.name}: {len(names)} WPN_* objects (expected 1)")
            dst.objects = names
        ob = dst.objects[0]
        bpy.context.scene.collection.objects.link(ob)
        ob.matrix_basis = Matrix.Identity(4)
        c = ob.constraints.new("CHILD_OF")
        c.target = rig
        c.subtarget = f"WeaponSocket_{side}"
        c.inverse_matrix = Matrix.Identity(4)
        c.set_inverse_pending = False
        out.append((side, n, ob))
    return out


def detach_weapons(ws):
    for _, _, ob in ws:
        me, mats = ob.data, [m for m in ob.data.materials if m is not None]
        bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.meshes.remove(me)
        for m in mats:
            if m.users == 0:
                bpy.data.materials.remove(m)


def apply_sweep_overrides(clip, man):
    """s12_clip_anim.py:104-130: clip["sweep_overrides"] -> patch man["controls"][CTRL]["sweep"] in memory (widening
    only); -> applied list."""
    ov = clip.get("sweep_overrides")
    if not ov:
        return [], None
    reason = ov.get("_reason")
    if not isinstance(reason, str) or not reason.strip():
        raise RuntimeError("sweep_overrides: _reason missing")
    applied = []
    for n, entries in ov.items():
        if n == "_reason":
            continue
        if n not in man["controls"]:
            raise RuntimeError(f"sweep_overrides: unknown control {n}")
        for e in entries:
            ch, ax, lo, hi = e.get("channel"), e.get("axis"), float(e["min"]), float(e["max"])
            cur = [s for s in man["controls"][n]["sweep"] if s["channel"] == ch and s["axis"] == ax]
            if len(cur) != 1:
                raise RuntimeError(f"sweep_overrides: {n} has no sweep entry {ch} {ax}")
            s = cur[0]
            if lo > s["min"] or hi < s["max"]:
                raise RuntimeError(f"sweep_overrides: {n} {ch} {ax} [{lo}, {hi}] narrows [{s['min']}, {s['max']}]")
            old = [s["min"], s["max"]]
            s["min"], s["max"] = lo, hi
            applied.append({"control": n, "channel": ch, "axis": ax, "old": old, "new": [lo, hi]})
            log(f"SWEEP OVERRIDE {n} {ch} {ax}: {old} -> {[lo, hi]} ({reason})")
    return applied, reason


def prop_range(man, k):
    """p25_rigtest.py:111-120 (clamps, spaces, ikfk) + the props spec min / max (skirt_follow and others)."""
    if k in man.get("clamps", {}):
        return tuple(man["clamps"][k])
    for sp in man["spaces"].values():
        if sp["prop"] == k:
            return (0, len(sp["values"]) - 1)
    for e in man["ikfk"].values():
        if e["prop"] == k:
            return (0.0, 1.0)
    for p in man["props"]["spec"]:
        if p["name"] == k and p.get("min") is not None and p.get("max") is not None:
            return (p["min"], p["max"])
    raise RuntimeError(f"PROPS {k}: no range in ctrl_manifest")


# ---------------------------------------------------------------- base state
def snapshot(rig, ctrls, prop_names):
    pbs = rig.pose.bones
    c = {}
    for n in ctrls:
        pb = pbs[n]
        c[n] = {"loc": [float(x) for x in pb.location],
                "rot": [math.degrees(x) for x in pb.rotation_euler],
                "scale": [float(x) for x in pb.scale]}
    p = {k: pbs[PROPS][k] for k in prop_names}
    return c, p


def base_state(rig):
    for op in ("reset_rig", "ready_pose"):
        r = getattr(bpy.ops.player, op)()
        if "FINISHED" not in r:
            raise RuntimeError(f"player.{op} returned {r}")
    refresh(rig)


# ---------------------------------------------------------------- channels
def resolve_channels(clip, rig, man, sweep, base_c, base_p, prop_names):
    """-> ctrl channels {(ctrl, ch, axis): {frame: value}}, prop channels {name: {frame: value}}, loop-closed list."""
    fa, fb = clip["frame_range"]
    pbs = rig.pose.bones
    ch_keys = {}
    for n, frames in (clip.get("keys") or {}).items():
        if n not in man["controls"] or n not in pbs:
            raise RuntimeError(f"keys: unknown control {n}")
        if pbs[n].rotation_mode != "XYZ":
            raise RuntimeError(f"keys: {n} rotation_mode {pbs[n].rotation_mode} (XYZ expected)")
        for fk, v in frames.items():
            f = frame_of(fk, fa, fb, f"keys.{n}")
            unknown = sorted(set(v) - {"loc", "rot"})
            if unknown:
                raise RuntimeError(f"keys.{n}.{fk}: unknown channels {unknown} (loc / rot only)")
            for ch in ("loc", "rot"):
                if ch not in v:
                    continue
                vals = v[ch]
                if isinstance(vals, list):
                    if len(vals) != 3:
                        raise RuntimeError(f"keys.{n}.{fk}.{ch}: 3 values required")
                    items = list(zip(AXES, vals))
                elif isinstance(vals, dict):
                    bad = sorted(set(vals) - set(AXES))
                    if bad:
                        raise RuntimeError(f"keys.{n}.{fk}.{ch}: unknown axes {bad}")
                    items = sorted(vals.items())
                else:
                    raise RuntimeError(f"keys.{n}.{fk}.{ch}: list or dict required")
                for ax, x in items:
                    x = float(x)
                    rng = sweep.get((n, ch, ax))
                    b = base_c[n][ch][AXES.index(ax)]
                    if rng is None:
                        if abs(x - b) > EPS:
                            raise RuntimeError(f"keys.{n}.{fk}.{ch}.{ax} = {x}: axis not in the ctrl_manifest sweep "
                                               f"(only the base value {b} allowed)")
                    elif not rng[0] - EPS <= x <= rng[1] + EPS:
                        raise RuntimeError(f"keys.{n}.{fk}.{ch}.{ax} = {x} outside the sweep {list(rng)}")
                    if n == "CTRL_root" and clip["root_motion"] != "root" and abs(x - b) > EPS:
                        raise RuntimeError(f"keys.CTRL_root.{fk}.{ch}.{ax} = {x}: root_motion "
                                           f"{clip['root_motion']!r} keeps CTRL_root at the base {b}")
                    ch_keys.setdefault((n, ch, ax), {})[f] = x
    pr_keys = {}
    for k, frames in (clip.get("props") or {}).items():
        if k not in prop_names:
            raise RuntimeError(f"props: unknown PROPS property {k}")
        lo, hi = prop_range(man, k)
        is_int = isinstance(base_p[k], int)
        for fk, x in frames.items():
            f = frame_of(fk, fa, fb, f"props.{k}")
            if is_int and (not isinstance(x, (int, float)) or float(x) != int(x)):
                raise RuntimeError(f"props.{k}.{fk} = {x!r}: integer property")
            if not lo - EPS <= float(x) <= hi + EPS:
                raise RuntimeError(f"props.{k}.{fk} = {x} outside {[lo, hi]}")
            pr_keys.setdefault(k, {})[f] = int(x) if is_int else float(x)
    for (n, ch, ax), d in ch_keys.items():   # state at frame a = base unless keyed
        d.setdefault(fa, base_c[n][ch][AXES.index(ax)])
    for k, d in pr_keys.items():
        d.setdefault(fa, base_p[k])
    closed = []
    if clip["loop"]:
        for key, d in list(ch_keys.items()) + [((k,), d) for k, d in pr_keys.items()]:
            v0 = d[fa]
            if fb in d:
                if abs(float(d[fb]) - float(v0)) > EPS:
                    raise RuntimeError(f"loop: {'.'.join(key)} keyed at {fb} = {d[fb]} != frame {fa} value {v0}")
            else:
                d[fb] = v0
                closed.append(".".join(key))
    return ch_keys, pr_keys, closed


def build_action(clip, rig, ctrls, prop_names, base_c, base_p, ch_keys, pr_keys):
    """s12_clip_anim.py:177-228 adapted: sparse per-channel keys instead of full-state key poses."""
    sc = bpy.context.scene
    name = clip.get("action", clip.get("clip"))
    old = bpy.data.actions.get(name)
    if old is not None:
        bpy.data.actions.remove(old)
    ad = rig.animation_data or rig.animation_data_create()
    act = bpy.data.actions.new(name)
    act.use_fake_user = True
    slot = act.slots.new(id_type="OBJECT", name=ARM)
    ad.action = act
    ad.action_slot = slot
    fa, fb = clip["frame_range"]
    sc.render.fps, sc.render.fps_base = int(clip.get("fps", 30)), 1.0
    sc.frame_start, sc.frame_end = fa, fb
    act.use_frame_range = True
    act.frame_start, act.frame_end = fa, fb
    pbs = rig.pose.bones
    n_keys = 0
    for n in ctrls:
        pb = pbs[n]
        if rot_path(pb) != "rotation_euler":
            raise RuntimeError(f"{n}: rotation_mode {pb.rotation_mode} (Euler expected)")
        for path, ch in (("location", "loc"), ("rotation_euler", "rot"), ("scale", "scale")):
            for i in range(3):
                d = ch_keys.get((n, ch, AXES[i])) or {fa: base_c[n][ch][i]}
                for f in sorted(d):
                    getattr(pb, path)[i] = math.radians(d[f]) if ch == "rot" else d[f]
                    pb.keyframe_insert(path, index=i, frame=f, group=n)
                    n_keys += 1
    props = pbs[PROPS]
    for k in prop_names:
        d = pr_keys.get(k) or {fa: base_p[k]}
        for f in sorted(d):
            props[k] = type(base_p[k])(d[f])
            props.keyframe_insert(f'["{k}"]', frame=f, group=PROPS)
            n_keys += 1
    ip = clip.get("interpolation", {})
    ci = ip.get("ctrl", {"type": "BEZIER", "handles": "AUTO_CLAMPED"})
    co = ip.get("ctrl_overrides", {})
    for n, v in co.items():
        if n not in ctrls or v.get("type") not in PROP_INTERP:
            raise RuntimeError(f"interpolation.ctrl_overrides: {n!r}: {v!r}")
    if ci.get("type") not in PROP_INTERP:
        raise RuntimeError(f"interpolation.ctrl.type {ci.get('type')!r} not in {PROP_INTERP}")
    pi = ip.get("props", "CONSTANT")
    if pi not in PROP_INTERP:
        raise RuntimeError(f"interpolation.props {pi!r} not in {PROP_INTERP}")
    po = dict(DEFAULT_PROPS_OVERRIDES)
    po.update(ip.get("props_overrides", {}))
    for k, v in po.items():   # s12_clip_anim.py:205-208
        if k not in prop_names or v not in PROP_INTERP:
            raise RuntimeError(f"interpolation.props_overrides: {k!r}: {v!r} (props {prop_names}, types {PROP_INTERP})")
        if isinstance(base_p[k], int) and v != "CONSTANT":
            raise RuntimeError(f"interpolation.props_overrides: {k} is an int property (CONSTANT only)")
    ad2 = rig.animation_data
    fcs = anim_utils.action_get_channelbag_for_slot(ad2.action, ad2.action_slot).fcurves
    pre = f'pose.bones["{PROPS}"]'
    interp_used = {}
    for fc in fcs:   # s12_clip_anim.py:209-227
        is_prop = fc.data_path.startswith(pre)
        pname = fc.data_path[len(pre):].strip('[]"') if is_prop else None
        m = re.match(r'pose\.bones\["([^"]+)"\]', fc.data_path)
        bone = m.group(1) if m else None
        for kp in fc.keyframe_points:
            if is_prop and isinstance(base_p.get(pname), int):
                kp.interpolation = "CONSTANT"
            elif is_prop and pname in po:
                kp.interpolation = po[pname]
                if po[pname] == "BEZIER":
                    kp.handle_left_type = kp.handle_right_type = "AUTO_CLAMPED"
            elif is_prop:
                kp.interpolation = pi
            else:
                c = co.get(bone, ci)
                kp.interpolation = c["type"]
                if c.get("handles"):
                    kp.handle_left_type = kp.handle_right_type = c["handles"]
            interp_used[kp.interpolation] = interp_used.get(kp.interpolation, 0) + 1
        fc.update()
    return act, slot, len(fcs), n_keys, interp_used, po


# ---------------------------------------------------------------- renders
def view_basis(view):   # p23_skin.py:347-371
    if view == "front":
        d, wup = Vector((0, 1, 0)), Vector((0, 0, 1))
    elif view == "side":
        d, wup = Vector((-1, 0, 0)), Vector((0, 0, 1))
    elif view == "three_quarter":
        az, el = math.radians(45.0), math.radians(15.0)
        pos = Vector((-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), math.sin(el)))
        d, wup = -pos, Vector((0, 0, 1))
    else:
        raise ValueError(view)
    d = d.normalized()
    right = d.cross(wup).normalized()
    up = right.cross(d).normalized()
    return d, right, up


class Renderer:
    """One render scene (PL_mesh + PL_rig linked, ortho camera, black frame label), Workbench as p23 ortho_render."""

    def __init__(self, objs, bbox):
        self.mn, self.mx = Vector(bbox[0]), Vector(bbox[1])
        self.sc = bpy.data.scenes.new("_p30_render")
        for o in objs:
            self.sc.collection.objects.link(o)
        self.cam_data = bpy.data.cameras.new("_p30_cam")
        self.cam_data.type = "ORTHO"
        self.cam_data.sensor_fit = "VERTICAL"
        self.cam = bpy.data.objects.new("_p30_cam", self.cam_data)
        self.sc.collection.objects.link(self.cam)
        self.sc.camera = self.cam
        self.font = bpy.data.curves.new("_p30_label", "FONT")
        self.mat = bpy.data.materials.new("_p30_label")
        self.mat.diffuse_color = (0.0, 0.0, 0.0, 1.0)
        self.font.materials.append(self.mat)
        self.label = bpy.data.objects.new("_p30_label", self.font)
        self.sc.collection.objects.link(self.label)
        r = self.sc.render
        r.engine = "BLENDER_WORKBENCH"
        r.resolution_percentage = 100
        r.film_transparent = True
        r.use_file_extension = False
        r.image_settings.file_format = "PNG"
        r.image_settings.color_mode = "RGBA"
        self.sc.view_settings.view_transform = "Standard"
        sh = self.sc.display.shading
        sh.light = "STUDIO"
        sh.color_type = "MATERIAL"
        sh.show_xray = False

    def shot(self, view, frame, out_png, res_h, text):
        out_png = Path(out_png).resolve()
        out_png.parent.mkdir(parents=True, exist_ok=True)
        d, right, up = view_basis(view)
        mn, mx = self.mn, self.mx
        corners = [Vector((x, y, z)) for x in (mn.x, mx.x) for y in (mn.y, mx.y) for z in (mn.z, mx.z)]
        us = [q.dot(right) for q in corners]
        vs = [q.dot(up) for q in corners]
        ws = [q.dot(d) for q in corners]
        u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
        w, h = max(u1 - u0, 1e-6), max(v1 - v0, 1e-6)
        depth = max(ws) - min(ws)
        center_plane = right * ((u0 + u1) / 2) + up * ((v0 + v1) / 2)
        dist = depth + max(w, h) + 1.0
        rot = Matrix((right, up, -d)).transposed()
        self.cam.matrix_world = Matrix.Translation(center_plane + d * (min(ws) - dist)) @ rot.to_4x4()
        self.cam_data.ortho_scale = h
        self.cam_data.clip_start = 0.001
        self.cam_data.clip_end = dist + depth + 10.0
        self.font.body = text
        self.font.size = 0.045 * h
        pos = right * (u0 + 0.02 * w) + up * (v1 - 0.06 * h) + d * (min(ws) - 0.05)
        self.label.matrix_world = Matrix.Translation(pos) @ rot.to_4x4()
        r = self.sc.render
        r.resolution_x = max(1, int(round(res_h * w / h)))
        r.resolution_y = res_h
        r.filepath = str(out_png)
        # the pose comes from the main scene evaluation (a frame change on the render scene alone left the pose at the
        # main scene frame in the T300 first run), so both scenes are set to the frame
        main_sc = bpy.context.scene
        main_sc.frame_set(frame)
        for o in bpy.context.view_layer.objects:
            if o.type == "ARMATURE":
                o.update_tag()
        bpy.context.view_layer.update()
        self.sc.frame_set(frame)
        bpy.ops.render.render(write_still=True, scene=self.sc.name)
        return out_png

    def close(self):
        bpy.data.objects.remove(self.label, do_unlink=True)
        bpy.data.curves.remove(self.font)
        bpy.data.materials.remove(self.mat)
        bpy.data.objects.remove(self.cam, do_unlink=True)
        bpy.data.cameras.remove(self.cam_data)
        bpy.data.scenes.remove(self.sc)


def load_rgba(png):   # check_p2_deform.py:1082-1093
    img = bpy.data.images.load(str(Path(png).resolve()), check_existing=False)
    try:
        w, h = img.size
        buf = np.empty(w * h * img.channels, dtype=np.float32)
        img.pixels.foreach_get(buf)
        px = buf.reshape(h, w, img.channels)
    finally:
        bpy.data.images.remove(img)
    if px.shape[2] == 3:
        px = np.concatenate([px, np.ones((h, w, 1), np.float32)], axis=2)
    return px[::-1]


def compose_sheet(rows, out_png):
    """rows of png paths -> one sheet, alpha over white, grey gaps (check_p2_deform.py:1096-1123, variable cell size)."""
    imgs = [[load_rgba(p) for p in row] for row in rows]
    ch = max(im.shape[0] for row in imgs for im in row)
    cws = [max(row[j].shape[1] for row in imgs if j < len(row)) for j in range(max(len(r) for r in imgs))]
    H = len(imgs) * ch + (len(imgs) + 1) * CELL_GAP
    W = sum(cws) + (len(cws) + 1) * CELL_GAP
    sheet = np.full((H, W, 4), 1.0, dtype=np.float32)
    sheet[:, :, :3] = 0.55
    for i, row in enumerate(imgs):
        x0 = CELL_GAP
        for j, im in enumerate(row):
            y0 = CELL_GAP + i * (ch + CELL_GAP)
            sheet[y0:y0 + ch, x0:x0 + cws[j], :3] = 1.0
            a = im[:, :, 3:4]
            sheet[y0:y0 + im.shape[0], x0:x0 + im.shape[1], :3] = im[:, :, :3] * a + (1.0 - a)
            x0 += cws[j] + CELL_GAP
    out_png = Path(out_png).resolve()
    img = bpy.data.images.new("_p30_sheet", W, H, alpha=True)
    try:
        img.pixels.foreach_set(np.ascontiguousarray(sheet[::-1]).ravel())
        img.filepath_raw = str(out_png)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)
    return out_png


def mesh_bbox(ob):
    dg = bpy.context.evaluated_depsgraph_get()
    oe = ob.evaluated_get(dg)
    pts = [oe.matrix_world @ Vector(c) for c in oe.bound_box]
    return (min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)), \
           (max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts))


def render_all(clip_name, rig, mesh, key_frames, strip_frames, bbox, weapon=None):
    out_dir = INSPECT / clip_name
    out_dir.mkdir(parents=True, exist_ok=True)
    ws = attach_weapons(weapon or {}, rig)
    if ws:   # T310: widen the fixed framing by the weapons over every frame
        sc = bpy.context.scene
        lo, hi = list(bbox[0]), list(bbox[1])
        for f in range(sc.frame_start, sc.frame_end + 1):
            sc.frame_set(f)
            refresh(rig)
            for _, _, ob in ws:
                a, b = mesh_bbox(ob)
                lo = [min(x, y) for x, y in zip(lo, a)]
                hi = [max(x, y) for x, y in zip(hi, b)]
        bbox = (lo, hi)
        log(f"WEAPON {[(s, n, ob.name, 'WeaponSocket_' + s) for s, n, ob in ws]} attached for the renders (Child Of, no "
            f"inverse); framing bbox {[round(x, 4) for x in lo]} .. {[round(x, 4) for x in hi]}")
    rd = Renderer([mesh, rig] + [ob for _, _, ob in ws], bbox)
    files, key_rows, strip_rows = [], [], []
    try:
        for f in key_frames:
            row = []
            for v in VIEWS_KEY:
                p = rd.shot(v, f, out_dir / f"key_f{f:03d}_{v}.png", RES_KEY, f"{clip_name} f{f} {v}")
                row.append(p)
                files.append(p)
            key_rows.append(row)
        for v in VIEWS_STRIP:
            row = []
            for f in strip_frames:
                p = rd.shot(v, f, out_dir / f"strip_f{f:03d}_{v}.png", RES_STRIP, f"f{f}")
                row.append(p)
                files.append(p)
            strip_rows.append(row)
    finally:
        rd.close()
        detach_weapons(ws)
    ks = compose_sheet(key_rows, out_dir / "keyposes_sheet.png")
    ss = compose_sheet(strip_rows, out_dir / "strip_sheet.png")
    return [rel(p) for p in files], rel(ks), rel(ss)


# ---------------------------------------------------------------- main
def main():
    a = parse_args()
    clip_path, clip = load_clip(a.clip)
    name = a.clip
    fa, fb = clip["frame_range"]
    fps = int(clip.get("fps", 30))
    if not bpy.data.filepath:
        raise RuntimeError("no rig blend open (blender -b --factory-startup work/player/rig/pl_r04_ctrl.blend ...)")
    src_blend = Path(bpy.data.filepath).resolve()
    out_blend = (ANIM / f"pl_a_{name}.blend").resolve()
    if out_blend == src_blend:
        raise RuntimeError("the output blend would overwrite the opened blend")
    man = json.loads(CTRL_MAN.read_text(encoding="utf-8"))
    canon = json.loads(CANON.read_text(encoding="utf-8"))
    def_names = [b["name"] for b in canon["bones"]]
    sweep_ov, reason = apply_sweep_overrides(clip, man)
    sweep = {(c, s["channel"], s["axis"]): (s["min"], s["max"]) for c, e in man["controls"].items() for s in e["sweep"]}
    ov_set = {(x["control"], x["channel"], x["axis"]) for x in sweep_ov}
    log(f"CLIP {rel(clip_path)} sha256 {sha(clip_path)}; SOURCE {rel(src_blend)} sha256 {sha(src_blend)}")
    log(f"CLIP {name}: fps {fps}, frames {fa}..{fb}, loop {clip['loop']}, root_motion {clip['root_motion']}")

    rig = bpy.data.objects[ARM]
    mesh = bpy.data.objects[MESH]
    sc = bpy.context.scene
    if bpy.context.object and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.objects.active = rig
    rig.data.pose_position = "POSE"
    bpy.data.texts[TEXT_UI].as_module().register()
    sc.tool_settings.use_keyframe_insert_auto = False
    if rig.animation_data is not None:
        rig.animation_data.action = None
    pbs = rig.pose.bones
    ctrls = [pb.name for pb in pbs if pb.name.startswith("CTRL_")]
    prop_names = [k for k in pbs[PROPS].keys() if isinstance(pbs[PROPS][k], (int, float))]
    base_state(rig)
    base_c, base_p = snapshot(rig, ctrls, prop_names)
    log(f"BASE {BASE}: {len(ctrls)} CTRL_*, {len(prop_names)} PROPS; non-identity controls "
        f"{sorted(n for n, v in base_c.items() if any(abs(x) > EPS for x in v['loc'] + v['rot']))}")

    ch_keys, pr_keys, closed = resolve_channels(clip, rig, man, sweep, base_c, base_p, prop_names)
    log(f"KEYS {len(ch_keys)} control channels, {len(pr_keys)} PROPS channels from the JSON; loop closing keys at f{fb}: "
        f"{closed}")
    act, slot, nfc, nk, interp_used, po = build_action(clip, rig, ctrls, prop_names, base_c, base_p, ch_keys, pr_keys)
    log(f"ACTION {act.name} slot {slot.identifier!r}: {nfc} fcurves, {nk} keys, interpolation {interp_used}, "
        f"props_overrides {po}; scene {sc.frame_start}..{sc.frame_end} @ {sc.render.fps} fps")

    json_frames = sorted({f for d in ch_keys.values() for f in d} | {f for d in pr_keys.values() for f in d})
    key_frames = clip.get("key_frames")
    if key_frames is None:
        key_frames = json_frames
    for f in key_frames:
        frame_of(f, fa, fb, "key_frames")
    key_frames = sorted(int(f) for f in key_frames)
    every_n = int((clip.get("sheet") or {}).get("every_n", 3))
    if every_n < 1:
        raise RuntimeError("sheet.every_n must be >= 1")
    strip_frames = list(range(fa, fb + 1, every_n))
    if strip_frames[-1] != fb:
        strip_frames.append(fb)

    # per-frame evaluation (raw)
    mode_of = {n: e.get("mode") for n, e in man["controls"].items() if e.get("mode")}
    outside, mode_warn, root_path, defw, bb = [], [], [], {}, None
    key_at = {}
    for (n, ch, ax), d in ch_keys.items():
        for f in d:
            key_at.setdefault(f, set()).add(n)
    for f in range(fa, fb + 1):
        sc.frame_set(f)
        refresh(rig)
        for n in ctrls:
            pb = pbs[n]
            for ch, vec in (("loc", list(pb.location)), ("rot", [math.degrees(x) for x in pb.rotation_euler])):
                for i, ax in enumerate(AXES):
                    x = vec[i]
                    rng = sweep.get((n, ch, ax))
                    b = base_c[n][ch][i]
                    if rng is None:
                        if abs(x - b) > (1e-4 if ch == "loc" else 0.01):
                            outside.append({"f": f, "control": n, "channel": ch, "axis": ax, "value": x,
                                            "sweep": None, "base": b})
                    elif not rng[0] - 1e-4 <= x <= rng[1] + 1e-4:
                        outside.append({"f": f, "control": n, "channel": ch, "axis": ax, "value": x,
                                        "sweep": list(rng)})
        for n in sorted(key_at.get(f, ())):
            for p, want in (mode_of.get(n) or {}).items():
                if abs(float(pbs[PROPS][p]) - float(want)) >= 0.5:
                    mode_warn.append({"f": f, "control": n, "prop": p, "value": float(pbs[PROPS][p]), "mode": want})
        m = rig.matrix_world @ pbs["Root"].matrix
        q = m.to_quaternion()
        root_path.append({"f": f, "pos_m": [float(x) for x in m.translation], "rot_wxyz": [q.w, q.x, q.y, q.z],
                          "yaw_deg": math.degrees(m.to_euler("XYZ").z)})
        if f in (fa, fb):
            defw[f] = {n: (rig.matrix_world @ pbs[n].matrix).copy() for n in def_names}
        lo, hi = mesh_bbox(mesh)
        bb = (lo, hi) if bb is None else (tuple(min(x, y) for x, y in zip(bb[0], lo)),
                                           tuple(max(x, y) for x, y in zip(bb[1], hi)))
    seam_p, seam_bone = max((mdiff(defw[fa][n], defw[fb][n])[0], n) for n in def_names)
    seam_r = max(mdiff(defw[fa][n], defw[fb][n])[1] for n in def_names)
    seam = (seam_p, seam_r, seam_bone)
    root_move = max(Vector(r["pos_m"]).length for r in root_path) * 1000.0
    log(f"FRAMES {fa}..{fb}: control values outside the effective sweep {len(outside)} {outside[:5]}; mode warnings "
        f"{len(mode_warn)} {mode_warn[:5]}")
    log(f"LOOP SEAM f{fb} vs f{fa} DEF world max {seam_p:.6f} mm / {seam_r:.6f} deg (worst {seam[2]}); DEF Root world "
        f"max |pos| {root_move:.6f} mm")
    for w in mode_warn[:20]:
        log(f"WARNING mode: f{w['f']} {w['control']} keyed while {w['prop']} = {w['value']} (control mode {w['mode']})")

    if a.no_save:
        return
    sc.frame_set(fa)
    refresh(rig)
    ANIM.mkdir(parents=True, exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(out_blend), copy=True)
    log(f"SAVED {rel(out_blend)} (copy=True, save_version 0)")

    pad = 0.05 * max(bb[1][i] - bb[0][i] for i in range(3))
    bbox = ([x - pad for x in bb[0]], [x + pad for x in bb[1]])
    files, ks, ss = [], None, None
    if not a.no_render:
        files, ks, ss = render_all(name, rig, mesh, key_frames, strip_frames, bbox, clip.get("weapon"))
        log(f"RENDER {len(files)} cells; {ks} (rows = key frames {key_frames}, columns {list(VIEWS_KEY)}); {ss} "
            f"(rows {list(VIEWS_STRIP)}, columns = frames {strip_frames})")

    eff = {n: [dict(channel=s["channel"], axis=s["axis"], min=s["min"], max=s["max"],
                    overridden=(n, s["channel"], s["axis"]) in ov_set) for s in e["sweep"]]
           for n, e in man["controls"].items()}
    channels = []
    for (n, ch, ax), d in sorted(ch_keys.items()):
        c = (clip.get("interpolation", {}).get("ctrl_overrides", {}).get(n)
             or clip.get("interpolation", {}).get("ctrl", {"type": "BEZIER", "handles": "AUTO_CLAMPED"}))
        channels.append({"control": n, "channel": ch, "axis": ax,
                         "data_path": f'pose.bones["{n}"].{"location" if ch == "loc" else "rotation_euler"}',
                         "index": AXES.index(ax), "unit": "m" if ch == "loc" else "deg",
                         "keys": [[f, d[f]] for f in sorted(d)], "interpolation": c,
                         "base": base_c[n][ch][AXES.index(ax)], "sweep": list(sweep[(n, ch, ax)]) if (n, ch, ax) in sweep
                         else None, "loop_closed": f"{n}.{ch}.{ax}" in closed})
    pchannels = []
    for k, d in sorted(pr_keys.items()):
        if k not in (clip.get("props") or {}):
            continue
        pchannels.append({"prop": k, "data_path": f'pose.bones["{PROPS}"]["{k}"]', "keys": [[f, d[f]] for f in sorted(d)],
                          "interpolation": "CONSTANT" if isinstance(base_p[k], int) else po.get(k, clip.get(
                              "interpolation", {}).get("props", "CONSTANT")), "range": list(prop_range(man, k)),
                          "base": base_p[k], "loop_closed": k in closed})
    resolved = {
        "_doc": ("written by work/player/rig/scripts/p30_clip_anim.py (T300); raw values, no verdicts. channels = the "
                 "JSON-keyed control channels (absolute values, loc m / rot deg, + loop closing keys); prop_channels = "
                 "the JSON-keyed PROPS; every other CTRL_* loc / rot / scale axis and PROPS property is keyed once at "
                 "frame_range[0] with base_state. effective_sweeps = ctrl_manifest sweeps after sweep_overrides. "
                 "per_frame values are evaluated from the saved action (frame_set + update_tag + view_layer.update)."),
        "clip": name, "action": act.name, "slot": slot.identifier, "armature": ARM,
        "fps": fps, "frame_range": [fa, fb], "loop": clip["loop"], "root_motion": clip["root_motion"], "base": BASE,
        "key_frames": key_frames, "json_keyed_frames": json_frames, "strip_frames": strip_frames,
        "inputs": {"clip_json": [rel(clip_path), sha(clip_path)], "source_blend": [rel(src_blend), sha(src_blend)],
                   "ctrl_manifest": [rel(CTRL_MAN), sha(CTRL_MAN)], "script": [rel(__file__), sha(__file__)]},
        "output_blend": rel(out_blend),
        "base_state": {"ctrl": base_c, "props": base_p},
        "channels": channels, "prop_channels": pchannels,
        "interpolation": {"ctrl": clip.get("interpolation", {}).get("ctrl", {"type": "BEZIER", "handles": "AUTO_CLAMPED"}),
                          "ctrl_overrides": clip.get("interpolation", {}).get("ctrl_overrides", {}),
                          "props": clip.get("interpolation", {}).get("props", "CONSTANT"), "props_overrides": po,
                          "int_props": "CONSTANT", "keys_by_type": interp_used},
        "action_stats": {"fcurves": nfc, "keys": nk},
        "sweep_overrides": {"reason": reason, "applied": sweep_ov},
        "effective_sweeps": eff,
        "contacts": clip.get("contacts", {}),
        "mode_warnings": mode_warn,
        "per_frame": {
            "ctrl_outside_sweep": outside,
            "root_path": root_path,
            "root_rule": "DEF Root world (PL_rig matrix_world @ pose matrix): pos m, rotation quaternion wxyz, yaw = "
                         "Euler XYZ z in degrees",
            "loop_seam": {"frames": [fb, fa], "def_world_max_mm": seam_p, "def_world_max_deg": seam_r,
                          "worst_bone": seam[2]},
            "mesh_bbox_union_m": [list(bb[0]), list(bb[1])]},
        "renders": {"cells": files, "keyposes_sheet": ks, "strip_sheet": ss},
    }
    if clip.get("events") is not None:   # T320
        resolved["events"] = sorted(clip["events"], key=lambda e: (e["frame"], e["name"]))
    if clip.get("match_pose") is not None:   # T320 addendum
        resolved["match_pose"] = clip["match_pose"]
    if clip.get("weapon"):  # main 2026-09-27: omit for clips without a weapon so C8 is skipped, not blocked
        resolved["weapon"] = clip["weapon"]
        resolved["weapon_rule"] = ("preview only: appended for the sheets after the save, Child Of WeaponSocket_<side> "
                                   "without inverse, removed afterwards (data/weapon_socket_contract.json)")
    out_json = ANIM / f"{name}_resolved.json"
    with open(out_json, "w", encoding="utf-8", newline="") as fh:
        fh.write(json.dumps(resolved, indent=1) + "\n")
    log(f"OUTPUT {rel(out_json)}")
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
