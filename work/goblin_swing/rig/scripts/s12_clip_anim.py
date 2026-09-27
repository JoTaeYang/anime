"""s12_clip_anim.py - clip-agnostic animation builder (T80, playbook section 4; first clip: idle, spec d-26).

Input  rig/gob_r04_ctrl.blend (opened, never overwritten) + rig/data/clips/<clip>.json (--clip <clip>).
Output rig/gob_r08_<clip>.blend (copy=True, save_version 0): action <clip> (fps, frame_range from the JSON) keyed on
       CTRL_* pose bones (location, rotation_euler, scale) and PROPS only, at the key pose frames and the breakdown
       frames, interpolation from the JSON.
       rig/data/clips/<clip>.json: written back per key pose: ctrl (full CTRL state), props (full PROPS state), def
       (24 DEF bone world 4x4, row-major, evaluated from the action), measure; plus keyed_frames and anim_measure.
       Authored keys (ctrl_overrides, props_overrides, breakdowns, ...) are kept as they are.
       rig/inspect/<clip>/front_####.png, 34_####.png (every frame; cameras = reference.camera JSON front /
       three_quarter), side_by_side.mp4 (24 fps, left = reference frame, right = ours front), ours_34.mp4,
       contact_sheet.png (ref | ours pairs, 5 per row), keyposes_sheet.png (rows = key poses, columns = reference
       (2x) | ours front + 0.1 grid | ours 3/4) + key_f##_<name>_row.png, s12_measure.json.

Clip JSON (authored part):
  action, fps, frame_range [a, b], base ("goblin.reset_rig + goblin.ready_pose"), loop (bool: last frame = first),
  root_motion ("locked" = CTRL_root identity, reported),
  poses: [{frame, name, intent, contact, ctrl_overrides {CTRL: {loc [m], rot [deg XYZ]}}, props_overrides {}}]
         (state = base + overrides; sweep / clamp checked),
  breakdowns: [{frame, source {"copy": f} | {"blend": [fa, fb, t]}, ctrl_overrides, props_overrides, purpose}]
         (blend = linear in CTRL value space, t = 0 -> fa; sources = key poses or earlier breakdowns; PROPS = those
         of the copy source / fa),
  interpolation: {ctrl: {type, handles}, props: type, props_overrides (optional, HR2) {prop: "BEZIER"|"LINEAR"|"CONSTANT"}}
         (props_overrides gives one PROPS key its own interpolation, BEZIER with AUTO_CLAMPED handles; every other
         PROPS key uses `props`, default CONSTANT; an unknown prop or type -> RuntimeError),
  contact_ranges [{foot, frames}], sections [{name, frames}], targets {..}, key_frames [..],
  reference: {frames (dir relative to work/goblin_swing), pattern (python format of the frame number), camera
         (rig/data JSON with front / three_quarter)},
  match_first_frame (optional): {blend (rig-relative), action, frame}: that action is appended in memory, the first
         frame DEF world / CTRL / PROPS are compared with ours (report), then it is removed (not saved).
  sweep_overrides (optional, T101b): {"_reason": text, CTRL: [{channel "loc"|"rot", axis "X"|"Y"|"Z", min, max}, ..]}:
         this clip's sweep widening; the matching ctrl_manifest.json sweep entries are patched in memory only (the
         manifest file is never written). Widening only (min <= manifest min, max >= manifest max); an unknown
         control / channel / axis, a narrowing or a missing _reason -> RuntimeError. Printed as SWEEP OVERRIDE lines
         and recorded in s12_measure.json (sweep_overrides).

Self measurement per frame (stdout + s12_measure.json; raw values, no verdicts):
  amplitudes relative to the first frame (check_anim_clip contract): head = DEF head head z rise; fist = hand_l shell
  vertex centroid (rise z, out +x); club tip = centroid of the GOB_club vertices within 1 mm of the max local Y; also
  the eye-line point rise (FrontView eye attached to DEF head); club clearance / visible self intersection / legs =
  s10a definitions (check_a_swing); contact foot DEF movement per contact range; branch flips (G6.5b); max adjacent
  DEF rotation per section; key pose DEF (action vs directly applied state); loop; CTRL_root motion; DEF scale.

Renders (HR2): Workbench colour type MATERIAL during render_all only (after the blend is saved): GOB_mesh material
  slot 0 (GOB_skin) and a temporary club material get the former object colours (s10a MESH_RGBA / CLUB_RGBA), so the
  body and club look as before; the mouth slots (GOB_mouth_inner / GOB_teeth / GOB_tongue) show their own colours.
  All of it is restored / removed afterwards.

Run (repo root):
  powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s12_clip_anim.py -Blend gob_r04_ctrl.blend -- --clip idle
Debug: --no-render (skip renders / videos), --no-save (no blend / JSON write).
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402
import s10a_swing_keyposes as S  # noqa: E402  (helpers only; main() does not run on import)
import s10b_swing_anim as B  # noqa: E402  (helpers only)

import bpy  # noqa: E402
import numpy as np  # noqa: E402

FFMPEG = B.FFMPEG
BASE = "goblin.reset_rig + goblin.ready_pose"
WORK = goblib.RIG.parent
PROP_INTERP = ("BEZIER", "LINEAR", "CONSTANT")   # HR2: allowed interpolation.props_overrides values
SKIN_MAT = "GOB_skin"                            # HR2: GOB_mesh material slot 0 (s02e)


# ---------------------------------------------------------------- args / clip
def parse_args():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if "--clip" not in args:
        raise RuntimeError("usage: -- --clip <name> [--no-render] [--no-save]")
    return {"clip": args[args.index("--clip") + 1], "render": "--no-render" not in args,
            "save": "--no-save" not in args}


def clip_rel(name):
    return f"clips/{name}.json"


# ---------------------------------------------------------------- states
def state_of(vals):
    return {n: {"loc": list(v["loc"]), "rot": list(v["rot"])} for n, v in vals.items()}


def apply_full(rig, man, base_c, base_p, st, props, name):
    """Apply a full CTRL state + PROPS through s10a apply_state (sweep / clamp checked)."""
    return S.apply_state(rig, man, base_c, base_p, {"name": name, "ctrl": st, "props": props})


def def_world(rig, names):
    mw = rig.matrix_world
    return {n: (mw @ rig.pose.bones[n].matrix).copy() for n in names}


def apply_sweep_overrides(clip, man):
    """clip["sweep_overrides"] -> patch man["controls"][CTRL]["sweep"] in memory (widening only); -> applied list."""
    ov = clip.get("sweep_overrides")
    if not ov:
        return []
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
            print(f"[s12] SWEEP OVERRIDE {n} {ch} {ax}: {old} -> {[lo, hi]} ({reason})")
    return applied


def mat_diff(A, Bm):
    return max(abs(A[i][j] - Bm[i][j]) for i in range(4) for j in range(4))


def build_states(clip, rig, man, base_c, base_p):
    """Key poses (base + overrides) and breakdowns (copy / blend + overrides) -> {frame: (ctrl state, props, why)}."""
    out = {}
    for p in clip["poses"]:
        pose = {"name": p["name"], "ctrl": p.get("ctrl_overrides", {}), "props": p.get("props_overrides", {})}
        vals, props = S.apply_state(rig, man, base_c, base_p, pose)
        out[int(p["frame"])] = (state_of(vals), dict(props), f"key pose {p['name']}")
    pending = {int(b["frame"]): b for b in clip.get("breakdowns", [])}
    while pending:
        ready = []
        for f, b in pending.items():
            src = b["source"]
            need = [src["copy"]] if "copy" in src else list(src["blend"][:2])
            if all(int(x) in out for x in need):
                ready.append(f)
        if not ready:
            raise RuntimeError(f"breakdown sources unresolved: {sorted(pending)}")
        for f in sorted(ready):
            b = pending.pop(f)
            src = b["source"]
            if "copy" in src:
                st0, pr0, _ = out[int(src["copy"])]
                st = state_of(st0)
            else:
                fa, fb, t = int(src["blend"][0]), int(src["blend"][1]), float(src["blend"][2])
                st = B.state_blend(out[fa][0], out[fb][0], t)
                pr0 = out[fa][1]
            for n, v in b.get("ctrl_overrides", {}).items():
                for ch in ("loc", "rot"):
                    if ch in v:
                        st[n][ch] = [float(x) for x in v[ch]]
            pr = dict(pr0)
            pr.update(b.get("props_overrides", {}))
            if f in out:
                raise RuntimeError(f"frame {f} defined twice (pose / breakdown)")
            out[f] = (st, pr, b.get("purpose", ""))
    return out


# ---------------------------------------------------------------- action
def build_action(clip, rig, states):
    sc = bpy.context.scene
    name = clip["action"]
    old = bpy.data.actions.get(name)
    if old is not None:
        bpy.data.actions.remove(old)
    ad = rig.animation_data or rig.animation_data_create()
    act = bpy.data.actions.new(name)
    act.use_fake_user = True
    slot = act.slots.new(id_type="OBJECT", name=S.ARM)
    ad.action = act
    ad.action_slot = slot
    fa, fb = clip["frame_range"]
    sc.render.fps, sc.render.fps_base = int(clip["fps"]), 1.0
    sc.frame_start, sc.frame_end = fa, fb
    act.use_frame_range = True
    act.frame_start, act.frame_end = fa, fb
    pbs = rig.pose.bones
    for f in sorted(states):
        st, pr, _ = states[f]
        B.apply_ctrl(rig, st)
        for k, v in pr.items():
            pbs[S.PROPS][k] = type(pbs[S.PROPS][k])(v)
        S.key_state(rig, f)
    ip = clip.get("interpolation", {})
    ci = ip.get("ctrl", {"type": "BEZIER", "handles": "AUTO_CLAMPED"})
    pi = ip.get("props", "CONSTANT")
    po = ip.get("props_overrides", {})   # HR2
    for k, v in po.items():
        if k not in S.prop_names(rig) or v not in PROP_INTERP:
            raise RuntimeError(f"interpolation.props_overrides: {k!r}: {v!r} (props {S.prop_names(rig)}, types "
                               f"{PROP_INTERP})")
    fcs = S.channelbag(rig).fcurves
    nk = 0
    pre = f'pose.bones["{S.PROPS}"]'
    for fc in fcs:
        is_prop = fc.data_path.startswith(pre)
        pname = fc.data_path[len(pre):].strip('[]"') if is_prop else None
        for k in fc.keyframe_points:
            if is_prop and pname in po:
                k.interpolation = po[pname]
                if po[pname] == "BEZIER":
                    k.handle_left_type = k.handle_right_type = "AUTO_CLAMPED"
            elif is_prop:
                k.interpolation = pi
            else:
                k.interpolation = ci["type"]
                if ci.get("handles"):
                    k.handle_left_type = k.handle_right_type = ci["handles"]
            nk += 1
        fc.update()
    return act, slot, len(fcs), nk


# ---------------------------------------------------------------- measurement
class Amp:
    """Amplitude probes (check_anim_clip contract) + eye-line point."""

    def __init__(self, topo, parts, fv):
        self.hand_l = np.unique(topo.tri_v[np.nonzero(topo.part[topo.tri_poly] == parts["hand_l"])[0]].ravel())
        s = topo.club_local[:, 1]
        self.tip = np.nonzero(s >= s.max() - 0.001)[0]
        self.fv = fv

    def probe(self, rig):
        V, _ = S.eval_world(bpy.data.objects[S.MESH])
        C, _ = S.eval_world(bpy.data.objects[S.CLUB])
        hm = rig.matrix_world @ rig.pose.bones["head"].matrix
        return {"head": np.array(hm.translation), "eye": np.array(hm @ self.fv.eye_local),
                "fist": V[self.hand_l].mean(0), "tip": C[self.tip].mean(0), "club": C}


def amp_rel(a, a0):
    mm = 1000.0
    return {"head_rise_mm": round(float(a["head"][2] - a0["head"][2]) * mm, 3),
            "eye_rise_mm": round(float(a["eye"][2] - a0["eye"][2]) * mm, 3),
            "fist_rise_mm": round(float(a["fist"][2] - a0["fist"][2]) * mm, 3),
            "fist_out_mm": round(float(a["fist"][0] - a0["fist"][0]) * mm, 3),
            "fist_fwd_mm": round(float(a0["fist"][1] - a["fist"][1]) * mm, 3),
            "club_tip_move_mm": round(float(np.linalg.norm(a["tip"] - a0["tip"])) * mm, 3),
            "club_max_vertex_move_mm": round(float(np.linalg.norm(a["club"] - a0["club"], axis=1).max()) * mm, 3)}


def contact_at(ranges, f):
    return [s for s in ("l", "r") if any(r["foot"] == s and r["frames"][0] <= f <= r["frames"][1] for r in ranges)]


def section_of(sections, f):
    for s in sections:
        if s["frames"][0] <= f <= s["frames"][1]:
            return s["name"]
    return "-"


def def_scale_dev(defw):
    return max(abs(defw[n].to_3x3().col[i].length - 1.0) for n in defw for i in range(3))


def match_first(rig, cfg, def_names, ours_def, ours_c, ours_p):
    """Append cfg.action from rig/<cfg.blend> in memory, evaluate cfg.frame on the same rig, compare, remove."""
    path = goblib.RIG / cfg["blend"]
    before = set(bpy.data.actions.keys())
    with bpy.data.libraries.load(str(path), link=False) as (src, dst):
        if cfg["action"] not in src.actions:
            raise RuntimeError(f"{path.name} has no action {cfg['action']}")
        dst.actions = [cfg["action"]]
    act = dst.actions[0]
    ad = rig.animation_data
    cur_act, cur_slot = ad.action, ad.action_slot
    sc = bpy.context.scene
    try:
        ad.action = act
        ad.action_slot = act.slots[0]
        sc.frame_set(int(cfg["frame"]))
        S.refresh(rig)
        ref_def = def_world(rig, def_names)
        ref_c, ref_p = S.snapshot(rig)
    finally:
        ad.action = cur_act
        ad.action_slot = cur_slot
        bpy.data.actions.remove(act)
        for n in set(bpy.data.actions.keys()) - before:
            bpy.data.actions.remove(bpy.data.actions[n])
        sc.frame_set(sc.frame_start)
        S.refresh(rig)
    dmax, dbone = max((mat_diff(ours_def[n], ref_def[n]), n) for n in def_names)
    cmax, cname = max((max(abs(a - b) for a, b in zip(ours_c[n]["loc"] + ours_c[n]["rot"],
                                                        ref_c[n]["loc"] + ref_c[n]["rot"])), n) for n in ours_c)
    pd = {k: [ours_p.get(k), ref_p.get(k)] for k in set(ours_p) | set(ref_p) if ours_p.get(k) != ref_p.get(k)}
    return {"blend": cfg["blend"], "action": cfg["action"], "frame": int(cfg["frame"]),
            "def_max_abs": dmax, "def_worst_bone": dbone, "ctrl_max_abs": cmax, "ctrl_worst": cname,
            "props_differ": pd}


# ---------------------------------------------------------------- main
def main():
    a = parse_args()
    clip = goblib.load_json(clip_rel(a["clip"]))
    if clip.get("base", BASE) != BASE:
        raise RuntimeError(f"unsupported base {clip.get('base')!r} (only {BASE!r})")
    name = clip["action"]
    fa, fb = (int(x) for x in clip["frame_range"])
    insp = goblib.INSPECT / a["clip"]
    out_blend = goblib.RIG / f"gob_r08_{a['clip']}.blend"
    rig = bpy.data.objects[S.ARM]
    man = goblib.load_json("ctrl_manifest.json")
    sweep_ov = apply_sweep_overrides(clip, man)
    parts = goblib.load_json("parts.json")
    def_names = [b["name"] for b in goblib.load_json("canonical_skeleton.json")["bones"]]
    sc = bpy.context.scene
    if bpy.context.object and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    bpy.context.view_layer.objects.active = rig
    rig.data.pose_position = "POSE"
    mod = bpy.data.texts[S.TEXT_UI].as_module()
    mod.register()
    ts = sc.tool_settings
    ts.use_keyframe_insert_auto = False
    if rig.animation_data is not None:
        rig.animation_data.action = None
    for op in ("reset_rig", "ready_pose"):
        r = getattr(bpy.ops.goblin, op)()
        if "FINISHED" not in r:
            raise RuntimeError(f"goblin.{op} returned {r}")
    S.refresh(rig)
    base_c, base_p = S.snapshot(rig)

    # states (validated) and the directly applied DEF of every key pose
    states = build_states(clip, rig, man, base_c, base_p)
    direct_def = {}
    for f, (st, pr, why) in sorted(states.items()):
        apply_full(rig, man, base_c, base_p, st, pr, f"f{f}")
        if any(int(p["frame"]) == f for p in clip["poses"]):
            direct_def[f] = def_world(rig, def_names)
    key_frames = [int(p["frame"]) for p in clip["poses"]]
    act, slot, nfc, nk = build_action(clip, rig, states)
    print(f"[s12] CLIP {a['clip']}: ACTION {act.name} slot {slot.identifier!r} {nfc} fcurves {nk} keys, keyed frames "
          f"{sorted(states)}, {sc.frame_start}-{sc.frame_end} @ {sc.render.fps} fps, interpolation "
          f"{json.dumps(clip.get('interpolation'))}")
    for f in sorted(states):
        print(f"[s12] KEY f{f:02d}: {states[f][2]}")

    # measure every frame
    topo = S.Topo(bpy.data.objects[S.MESH], bpy.data.objects[S.CLUB], parts)
    cams = goblib.load_json(clip["reference"]["camera"])
    sc.frame_set(fa)
    S.refresh(rig)
    fv = S.FrontView(cams["front"], rig)
    amp = Amp(topo, parts, fv)
    ranges = clip["contact_ranges"]
    sections = clip["sections"]
    pbs = rig.pose.bones
    per, defw, feet, thetas, probes, ctrl_at, root = {}, {}, {}, {}, {}, {}, {}
    for f in range(fa, fb + 1):
        sc.frame_set(f)
        S.refresh(rig)
        vals, props = S.snapshot(rig)
        ctrl_at[f] = (vals, props)
        pose = {"frame": f, "name": f"f{f}", "contact": contact_at(ranges, f)}
        m = S.measure(rig, topo, man, vals, props, pose, fv)
        per[f] = m
        defw[f] = def_world(rig, def_names)
        feet[f] = {s: ((rig.matrix_world @ pbs[f"foot_{s}"].head).copy(), (rig.matrix_world @ pbs[f"foot_{s}"].tail).copy())
                   for s in ("l", "r")}
        thetas[f] = B.bend_theta(rig, man)
        probes[f] = amp.probe(rig)
        root[f] = max(abs(x) for x in vals["CTRL_root"]["loc"] + vals["CTRL_root"]["rot"])
    amps = {f: amp_rel(probes[f], probes[fa]) for f in probes}
    move = {}
    for f in range(fa, fb + 1):
        move[f] = {}
        for r in ranges:
            s, (r0, r1) = r["foot"], r["frames"]
            if r0 <= f <= r1:
                h0, t0 = feet[r0][s]
                move[f][s] = round(max((feet[f][s][0] - h0).length, (feet[f][s][1] - t0).length) * 1000.0, 4)
        m, am, g = per[f], amps[f], per[f]["legs"]
        print(f"[s12] F{f:02d} {section_of(sections, f):7s} head {am['head_rise_mm']:+6.2f} (eye {am['eye_rise_mm']:+6.2f}) "
              f"fist rise {am['fist_rise_mm']:+6.2f} out {am['fist_out_mm']:+5.2f} | club tip {am['club_tip_move_mm']:5.2f} "
              f"(max vtx {am['club_max_vertex_move_mm']:5.2f}) | clear {m['club']['min_mm']:6.1f} mm @ {m['club']['where']} | "
              f"vis {m['self_isect']['visible_pairs']}p cover {m['self_isect']['visible_cover']} (total "
              f"{m['self_isect']['pairs']}p) {m['self_isect']['visible_groups']} | contact {''.join(contact_at(ranges, f))} "
              f"move {move[f]} reach {g['l']['reach_mm']}/{g['r']['reach_mm']} shoe z {g['l']['shoe_min_z_mm']}/"
              f"{g['r']['shoe_min_z_mm']} ratio {g['l']['ik_ratio']}/{g['r']['ik_ratio']} knee {g['l']['knee']}/"
              f"{g['r']['knee']} | wrDEF r {m['arms']['r']['wrist_bend_def']}/{m['arms']['r']['wrist_twist_def']} | "
              f"outside {m['outside_safe_range']}")

    # summaries
    keym = {f: max(mat_diff(defw[f][n], direct_def[f][n]) for n in def_names) for f in key_frames}
    ctrl_key = {}
    for f in key_frames:
        st, pr, _ = states[f]
        vals, props = ctrl_at[f]
        ctrl_key[f] = max(max(abs(x - y) for x, y in zip(vals[n]["loc"] + vals[n]["rot"], st[n]["loc"] + st[n]["rot"]))
                          for n in st)
    loop = max(mat_diff(defw[fb][n], defw[fa][n]) for n in def_names)
    flips = []
    for k in man["ikfk"]:
        for f in range(fa, fb):
            x, y = thetas[f][k], thetas[f + 1][k]
            if (x > 0) != (y > 0) and abs(x) > 2.0 and abs(y) > 2.0:
                flips.append({"chain": k, "frames": [f, f + 1], "theta": [round(x, 2), round(y, 2)]})
    adj = {}
    for f in range(fa + 1, fb + 1):
        sn = section_of(sections, f)
        mx, bone = max((B.rot_diff_deg(defw[f - 1][n], defw[f][n]), n) for n in def_names)
        if mx >= adj.get(sn, {}).get("max_deg", -1.0):
            adj[sn] = {"max_deg": round(mx, 3), "bone": bone, "frames": [f - 1, f]}
    contact = []
    for r in ranges:
        s, (r0, r1) = r["foot"], r["frames"]
        contact.append({"foot": s, "frames": [r0, r1], "max_move_mm": max(move[f][s] for f in range(r0, r1 + 1)),
                        "max_reach_mm": max(per[f]["legs"][s]["reach_mm"] for f in range(r0, r1 + 1)),
                        "shoe_min_z_mm": [min(per[f]["legs"][s]["shoe_min_z_mm"] for f in range(r0, r1 + 1)),
                                          max(per[f]["legs"][s]["shoe_min_z_mm"] for f in range(r0, r1 + 1))]})
    secm = {}
    for s in sections:
        fr = range(s["frames"][0], s["frames"][1] + 1)
        vc = [per[f]["self_isect"]["visible_cover"] for f in fr]
        cl = min((per[f]["club"]["min_mm"], f) for f in fr)

        def ext(key):
            v = [amps[f][key] for f in fr]
            return [round(min(v), 3), round(max(v), 3)]
        secm[s["name"]] = {
            "frames": s["frames"], "head_rise_mm": ext("head_rise_mm"), "eye_rise_mm": ext("eye_rise_mm"),
            "fist_rise_mm": ext("fist_rise_mm"), "fist_out_mm": ext("fist_out_mm"),
            "club_tip_move_mm_max": max(amps[f]["club_tip_move_mm"] for f in fr),
            "club_max_vertex_move_mm_max": max(amps[f]["club_max_vertex_move_mm"] for f in fr),
            "min_clearance_mm": cl[0], "min_clearance_frame": cl[1],
            "max_visible_cover": ">=2" if ">=2" in vc else max(vc),
            "max_visible_pairs": max(per[f]["self_isect"]["visible_pairs"] for f in fr),
            "contact_max_move_mm": {ft: max(move[f].get(ft, 0.0) for f in fr) for ft in ("l", "r")},
            "outside_safe_range": sorted({x for f in fr for x in per[f]["outside_safe_range"]})}
    theta_rng = {k: [round(min(thetas[f][k] for f in thetas), 2), round(max(thetas[f][k] for f in thetas), 2)]
                 for k in man["ikfk"]}
    scale_dev = max(def_scale_dev(defw[f]) for f in defw)
    peak = {k: max(((amps[f][k], f) for f in amps), key=lambda t: abs(t[0])) for k in amps[fa]}
    print(f"[s12] KEYPOSE DEF (action vs directly applied state) max abs {json.dumps({f: f'{v:.2e}' for f, v in keym.items()})}; "
          f"CTRL readback max {json.dumps({f: f'{v:.2e}' for f, v in ctrl_key.items()})}")
    print(f"[s12] LOOP f{fb} vs f{fa} DEF max abs {loop:.2e} | CTRL_root max abs {max(root.values()):.2e} | DEF scale "
          f"max |len-1| {scale_dev:.2e}")
    print(f"[s12] FLIPS {len(flips)} {flips[:10]} | THETA range {theta_rng}")
    print(f"[s12] ADJ ROT per section {json.dumps(adj)}")
    print(f"[s12] CONTACT {json.dumps(contact)}")
    print(f"[s12] AMPLITUDE peak (value, frame) {json.dumps({k: [v[0], v[1]] for k, v in peak.items()})} | targets "
          f"{json.dumps(clip.get('targets'))}")
    for s, v in secm.items():
        print(f"[s12] SECTION {s} {json.dumps(v)}")

    mf = None
    if clip.get("match_first_frame"):
        mf = match_first(rig, clip["match_first_frame"], def_names, defw[fa], *ctrl_at[fa])
        print(f"[s12] MATCH_FIRST_FRAME {json.dumps(goblib._jsonable(mf))}")
        # the action is restored; re-check the first frame after the round trip
        sc.frame_set(fa)
        S.refresh(rig)
        rt = max(mat_diff(def_world(rig, def_names)[n], defw[fa][n]) for n in def_names)
        print(f"[s12] action {rig.animation_data.action.name!r} restored, f{fa} DEF round trip max abs {rt:.2e}")

    if not a["save"]:
        return
    # JSON write-back (authored keys kept)
    for p in clip["poses"]:
        f = int(p["frame"])
        vals, props = ctrl_at[f]
        p["ctrl"] = {n: {"loc": [round(x, 6) for x in v["loc"]], "rot": [round(x, 4) for x in v["rot"]],
                         "scale": [1.0, 1.0, 1.0]} for n, v in vals.items()}
        p["props"] = props
        p["def"] = {n: [[round(float(x), 7) for x in row] for row in defw[f][n]] for n in def_names}
        p["measure"] = dict(per[f], amplitude=amps[f])
    clip["keyed_frames"] = {str(f): states[f][2] for f in sorted(states)}
    summary = {"keypose_def_action_vs_direct_max_abs": keym, "loop_def_max_abs": loop,
               "ctrl_root_max_abs": max(root.values()), "def_scale_max_dev": scale_dev, "flips": flips,
               "theta_range_deg": theta_rng, "max_adjacent_rot_per_section": adj, "contact": contact,
               "sections": secm, "amplitude_peak": {k: {"value": v[0], "frame": v[1]} for k, v in peak.items()},
               "match_first_frame": mf}
    clip["anim_measure"] = dict(summary, _doc="written by s12_clip_anim.py (self measurement, raw values)")
    goblib.save_json(clip_rel(a["clip"]), clip)
    insp.mkdir(parents=True, exist_ok=True)
    extra = {"sweep_overrides": sweep_ov} if sweep_ov else {}
    with open(insp / "s12_measure.json", "w", encoding="utf-8") as fh:
        json.dump(goblib._jsonable(dict(summary, **extra, per_frame={
            f: {"amplitude": amps[f], "club": per[f]["club"], "self_isect": per[f]["self_isect"],
                "legs": per[f]["legs"], "contact_move_mm": move[f], "front": per[f]["front"],
                "wrist_r": [per[f]["arms"]["r"]["wrist_bend_def"], per[f]["arms"]["r"]["wrist_twist_def"]],
                "outside_safe_range": per[f]["outside_safe_range"]} for f in per})), fh, indent=1)
    print(f"[s12] JSON {goblib._rel(goblib.DATA / clip_rel(a['clip']))} (poses ctrl / props / def / measure, keyed_frames, "
          f"anim_measure), {goblib._rel(insp / 's12_measure.json')}")
    sc.frame_set(fa)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(out_blend), copy=True)
    print(f"[s12] SAVED {out_blend.name} (copy=True, save_version 0)")
    if a["render"]:
        render_all(clip, rig, cams, insp, fa, fb)


def ref_frame(clip, f):
    r = clip["reference"]
    return S.load_rgba(WORK / r["frames"] / r["pattern"].format(f))[:, :, :3]


def render_all(clip, rig, cams, insp, fa, fb):
    sc = bpy.context.scene
    colors0 = {n: tuple(bpy.data.objects[n].color) for n in (S.MESH, S.CLUB)}
    S.setup_render()
    mat_restore = material_colours(sc)   # HR2: mouth materials visible, body / club colours unchanged
    tmp = tempfile.mkdtemp(prefix="s12_")
    keyp = {int(p["frame"]): p["name"] for p in clip["poses"]}
    try:
        cells, rows = [], []
        for f in range(fa, fb + 1):
            sc.frame_set(f)
            S.refresh(rig)
            fr = S.over_white(S.render(cams["front"], insp / f"front_{f:04d}.png"))
            S.save_rgb(fr, insp / f"front_{f:04d}.png")
            tq = S.over_white(S.render(cams["three_quarter"], insp / f"34_{f:04d}.png"))
            S.save_rgb(tq, insp / f"34_{f:04d}.png")
            ref = ref_frame(clip, f)
            pair = np.concatenate([ref, S.half(fr)], axis=1)
            S.save_rgb(pair, os.path.join(tmp, f"sbs_{f:04d}.png"))
            cells.append(S.half(pair))
            if f in keyp:
                ref2 = np.repeat(np.repeat(ref, 2, axis=0), 2, axis=1)
                row = np.concatenate([ref2, S.grid(fr), tq], axis=1)
                S.save_rgb(row, insp / f"key_f{f:02d}_{keyp[f]}_row.png")
                rows.append(S.half(row))
        cols, sep = 5, 4
        h, w = cells[0].shape[:2]
        nr = (len(cells) + cols - 1) // cols
        sheet = np.ones((nr * (h + sep), cols * (w + sep), 3), np.float32)
        for i, c in enumerate(cells):
            r, k = divmod(i, cols)
            sheet[r * (h + sep):r * (h + sep) + h, k * (w + sep):k * (w + sep) + w] = c
        S.save_rgb(sheet, insp / "contact_sheet.png")
        sp = np.full((6, rows[0].shape[1], 3), 1.0, np.float32)
        ks = np.concatenate(sum(([r, sp] for r in rows), [])[:-1], axis=0)
        S.save_rgb(ks, insp / "keyposes_sheet.png")

        def enc(pattern, out):
            cmd = [FFMPEG, "-y", "-loglevel", "error", "-framerate", str(clip["fps"]), "-start_number", str(fa),
                   "-i", pattern, "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                   "-r", str(clip["fps"]), str(out)]
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.returncode != 0:
                raise RuntimeError(f"ffmpeg failed ({r.returncode}): {r.stderr[-400:]}")
        enc(os.path.join(tmp, "sbs_%04d.png"), insp / "side_by_side.mp4")
        enc(str(insp / "34_%04d.png"), insp / "ours_34.mp4")
        print(f"[s12] RENDER {goblib._rel(insp)}: front_{fa:04d}..{fb:04d}.png, 34_####.png, contact_sheet.png "
              f"({sheet.shape[1]}x{sheet.shape[0]}), keyposes_sheet.png ({ks.shape[1]}x{ks.shape[0]}, rows "
              f"{sorted(keyp)}: reference 2x | front + grid | 3/4), side_by_side.mp4 ({clip['fps']} fps, ref | ours), "
              f"ours_34.mp4")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        for n, c in colors0.items():
            bpy.data.objects[n].color = c
        mat_restore()


def material_colours(sc):
    """HR2: Workbench colour type MATERIAL; GOB_skin (slot 0) = MESH_RGBA, a temporary club material = CLUB_RGBA, the
    mouth materials keep their own diffuse colours. Returns a function that restores the previous state."""
    sh = sc.display.shading
    ct0 = sh.color_type
    me = bpy.data.objects[S.MESH].data
    club = bpy.data.objects[S.CLUB].data
    skin = next((m for m in me.materials if m is not None and (m.name == SKIN_MAT or m.name.startswith(SKIN_MAT + "."))),
                None)
    skin0 = tuple(skin.diffuse_color) if skin is not None else None
    if skin is not None:
        skin.diffuse_color = S.MESH_RGBA
    tmp_mat = None
    if len(club.materials) == 0:
        tmp_mat = bpy.data.materials.new("_s12_club")
        tmp_mat.diffuse_color = S.CLUB_RGBA
        club.materials.append(tmp_mat)
    sh.color_type = "MATERIAL"
    print(f"[s12] RENDER colours: shading MATERIAL; {S.MESH} materials "
          f"{[(m.name, [round(x, 3) for x in m.diffuse_color]) for m in me.materials if m is not None]}; {S.CLUB} "
          f"material {[m.name for m in club.materials if m is not None]}")

    def restore():
        sh.color_type = ct0
        if skin is not None:
            skin.diffuse_color = skin0
        if tmp_mat is not None:
            club.materials.pop(index=len(club.materials) - 1)
            bpy.data.materials.remove(tmp_mat)
    return restore


if __name__ == "__main__":
    goblib.run_main(main)
