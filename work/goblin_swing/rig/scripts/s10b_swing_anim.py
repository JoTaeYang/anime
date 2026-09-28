"""s10b_swing_anim.py - goblin club swing animation, frames 1-39 (T52, spec d-25 A2).

Input  rig/gob_r06_swing.blend (s10a output: action attack_swing with the 7 approved key poses, CONSTANT) - opened,
       never overwritten.
Output rig/gob_r07_swing_anim.blend (copy=True, save_version 0): action attack_swing rebuilt with Bezier keys on
       CTRL_* (location, rotation_euler, scale) and PROPS only:
         - the 7 approved key poses (values read back from the s10a action, unchanged),
         - holds / loop copies (f5 = f1, f35 = f39 = f1),
         - breakdowns (BREAKDOWNS below: blend of the neighbouring key states + overrides).
       rig/data/swing_keyposes.json: contact_ranges updated to the actual step timing (other content untouched).
       rig/inspect/A2/front_####.png (39 front frames, swing_cam front camera), 34_####.png (3/4),
       side_by_side.mp4 (24 fps, left = ref/reference.mp4 frame, right = ours), contact_sheet.png (39 ref|ours
       pairs), ours_34.mp4 (24 fps).
Self measurement per frame (stdout + rig/inspect/A2/s10b_measure.json): club clearance, visible / total self
intersection (s10a = check_a_swing definitions), leg / arm bend theta and branch flips (G6.5b), max adjacent DEF
rotation per section, contact foot DEF movement, f39 vs f1 DEF difference.

Run (repo root):
  powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s10b_swing_anim.py -Blend gob_r06_swing.blend
Debug: -- --no-render (skip renders / videos), --no-save.
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
sys.path.insert(0, str(goblib.RIG.parents[2] / "tools" / "pipeline"))
import platform_tools  # noqa: E402  (ffmpeg discovery, T340)
import s10a_swing_keyposes as S  # noqa: E402  (helpers only; its main() does not run on import)

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Vector  # noqa: E402

OUT_BLEND = goblib.RIG / "gob_r07_swing_anim.blend"
INSP = goblib.INSPECT / "A2"
REF_FRAMES = goblib.RIG.parent / "ref" / "frames"
FFMPEG = platform_tools.ffmpeg(required=False)   # env FFMPEG or PATH (T340)
KEY_FRAMES = (1, 9, 12, 13, 15, 24, 29)
SECTIONS = (("idle", 1, 5), ("windup", 6, 12), ("swing", 13, 15), ("hold", 16, 23), ("recovery", 24, 34),
            ("settle", 35, 39))
WIDE_R = [0.065, 0.01, 0.0]
IDLE_R = [0.0, 0.0, 0.0]


def arm(side, sh, ua, la, hand):
    """FK arm override (degrees): CTRL_shoulder (X, 0, Z), upperarm XYZ, lowerarm X, hand XYZ."""
    return {f"CTRL_shoulder_{side}": {"rot": list(sh)}, f"CTRL_upperarm_fk_{side}": {"rot": list(ua)},
            f"CTRL_lowerarm_fk_{side}": {"rot": [la, 0.0, 0.0]}, f"CTRL_hand_fk_{side}": {"rot": list(hand)}}


def merge(*ds):
    out = {}
    for d in ds:
        out.update(d)
    return out


# ---------------------------------------------------------------- breakdowns
# frame: (source, overrides, purpose). source = ("copy", f) or ("blend", fa, fb, t) in CTRL value space
# (location m, Euler deg; t = 0 -> fa). overrides: {ctrl: {"loc"/"rot": [x, y, z]}} absolute values.
BREAKDOWNS = {
    5: (("copy", 1), {}, "idle hold end (= f1): wind-up starts after f5"),
    7: (("blend", 5, 9, 0.45), merge(arm("r", (4.0, 0, -10.0), (17.0, 14.0, 40.0), 50.5, (38.8, 36.8, 6.8)),
                                     arm("l", (0, 0, 0), (-23.7, 0.0, -33.3), 44.4, (0, 0, 0))),
        "wind-up in-between: arms re-solved so the lowered arms do not pass the armpit crease"),
    8: (("blend", 7, 9, 0.5), {}, "wind-up: club head led outward past the head corner (head clearance)"),
    17: (("blend", 15, 19, 0.5), arm("r", (27.0, 0, 0.0), (65.6, -13.5, -1.4), 0.0, (-36.7, 6.7, 7.2)),
         "impact hold: club kept above the ground (no Euler dip into the floor), shoulder held (no shrug)"),
    22: (("blend", 19, 24, 0.6), arm("r", (23.5, 0, 0.0), (69.0, -15.0, -6.5), 0.0, (-25.0, 20.4, 21.7)),
         "impact hold: club above the ground, wrist bend kept <= 40, shoulder held (no shrug)"),
    14: (("blend", 13, 15, 0.5), arm("r", (11.6, 0, 7.7), (88.0, 36.8, 33.5), 3.3, (-24.0, 33.6, 16.3)), "swing breakdown (2-frame swing): club mid-arc in front of the body"),
    19: (("blend", 15, 24, 0.5), arm("r", (25.5, 0, 0.0), (69.0, -15.0, -5.0), 0.0, (-29.9, 18.1, 16.0)),
         "impact hold settle: club kept just above the ground, shoulder held (no shrug)"),
    26: (("blend", 24, 29, 0.3), {"CTRL_foot_ik_r": {"loc": WIDE_R, "rot": [0, 0, 0]}},
         "recovery: right foot still planted wide (last contact frame), body starts to rise"),
    32: (("blend", 29, 35, 0.5), {"CTRL_foot_ik_r": {"loc": IDLE_R, "rot": [0, 0, 0]}},
         "right foot lands back on the idle spot (step, not slide)"),
    35: (("copy", 1), {}, "settle: back on the idle pose (reference f35 = idle)"),
    39: (("copy", 1), {}, "loop: f39 = f1"),
}

CONTACT_RANGES = {"l": [[1, 39]], "r": [[1, 9], [13, 26], [32, 39]]}


# ---------------------------------------------------------------- helpers
def state_copy(st):
    return {n: {"loc": list(v["loc"]), "rot": list(v["rot"])} for n, v in st.items()}


def state_blend(a, b, t):
    return {n: {"loc": [x + (y - x) * t for x, y in zip(a[n]["loc"], b[n]["loc"])],
                "rot": [x + (y - x) * t for x, y in zip(a[n]["rot"], b[n]["rot"])]} for n in a}


def apply_ctrl(rig, st):
    pbs = rig.pose.bones
    for n, v in st.items():
        pb = pbs[n]
        pb.location = v["loc"]
        pb.rotation_euler = [math.radians(x) for x in v["rot"]]
        pb.scale = (1.0, 1.0, 1.0)


def bend_theta(rig, man):
    """G6.5b / check_a_swing: theta = atan2(dot(cross(u, l), X_upper), dot(u, l)) per ikfk chain (DEF upper, lower)."""
    pbs = rig.pose.bones
    mw = rig.matrix_world
    th = {}
    for key, ch in man["ikfk"].items():
        pu, pl = pbs[ch["def"][0]], pbs[ch["def"][1]]
        u = ((mw @ pu.tail) - (mw @ pu.head)).normalized()
        ll = ((mw @ pl.tail) - (mw @ pl.head)).normalized()
        x = (mw.to_3x3() @ pu.matrix.to_3x3()).col[0].normalized()
        th[key] = math.degrees(math.atan2(u.cross(ll).dot(x), u.dot(ll)))
    return th


def rot_diff_deg(a, b):
    q = a.to_quaternion().conjugated() @ b.to_quaternion()
    return math.degrees(2.0 * math.atan2(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z), abs(q.w)))


def contact_at(f):
    return [s for s in ("l", "r") if any(a <= f <= b for a, b in CONTACT_RANGES[s])]


# ---------------------------------------------------------------- main
def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    do_render = "--no-render" not in args
    do_save = "--no-save" not in args
    rig = bpy.data.objects[S.ARM]
    man = goblib.load_json("ctrl_manifest.json")
    parts = goblib.load_json("parts.json")
    kp = goblib.load_json(S.KEYPOSE_JSON)
    sc = bpy.context.scene
    bpy.context.view_layer.objects.active = rig
    mod = bpy.data.texts[S.TEXT_UI].as_module()
    mod.register()
    sc.tool_settings.use_keyframe_insert_auto = False

    # key states read back from the approved s10a action
    old = rig.animation_data.action
    if old is None or old.name != S.ACTION:
        raise RuntimeError(f"{S.ARM} has no action {S.ACTION}")
    keys, props0 = {}, None
    for f in KEY_FRAMES:
        sc.frame_set(f)
        S.refresh(rig)
        c, p = S.snapshot(rig)
        keys[f] = state_copy(c)
        if props0 is None:
            props0 = p
        elif p != props0:
            raise RuntimeError(f"PROPS differ at f{f}: {p} vs {props0} (s10b keys PROPS constant)")
    def_snap = {p["frame"]: p["def"] for p in kp["poses"]}

    # all keyed states
    states = dict(keys)
    purposes = {f: f"approved key pose ({next(p['name'] for p in kp['poses'] if p['frame'] == f)})" for f in keys}
    pending = sorted(BREAKDOWNS)
    order = []
    while pending:  # resolve breakdowns whose sources are already defined (keys or earlier breakdowns)
        ready = [f for f in pending if all(x in keys or x in order for x in BREAKDOWNS[f][0][1:3]
                                           if isinstance(x, int) and x != f)]
        if not ready:
            raise RuntimeError(f"breakdown sources unresolved: {pending}")
        order += ready
        pending = [f for f in pending if f not in ready]
    for f in order:
        src, ov, why = BREAKDOWNS[f]
        if src[0] == "copy":
            st = state_copy(keys[src[1]] if src[1] in keys else states[src[1]])
        else:
            _, fa, fb, t = src
            st = state_blend(states[fa], states[fb], t)
        for n, v in ov.items():
            for ch in ("loc", "rot"):
                if ch in v:
                    st[n][ch] = [float(x) for x in v[ch]]
        states[f] = st
        purposes[f] = why

    if "--dump" in args:
        with open(args[args.index("--dump") + 1], "w", encoding="utf-8") as fh:
            json.dump({str(f): st for f, st in states.items()}, fh)

    # rebuild the action
    ad = rig.animation_data
    act_name = old.name
    old.name = act_name + "_s10a"
    act = bpy.data.actions.new(act_name)
    act.use_fake_user = True
    slot = act.slots.new(id_type="OBJECT", name=S.ARM)
    ad.action = act
    ad.action_slot = slot
    bpy.data.actions.remove(old)
    sc.render.fps, sc.render.fps_base = S.FPS, 1.0
    sc.frame_start, sc.frame_end = S.FRAME_START, S.FRAME_END
    act.use_frame_range = True
    act.frame_start, act.frame_end = S.FRAME_START, S.FRAME_END
    pbs = rig.pose.bones
    for k, v in props0.items():
        pbs[S.PROPS][k] = type(pbs[S.PROPS][k])(v)
    for f in sorted(states):
        apply_ctrl(rig, states[f])
        S.key_state(rig, f)
    fcs = S.channelbag(rig).fcurves
    nk = 0
    for fc in fcs:
        is_prop = fc.data_path.startswith(f'pose.bones["{S.PROPS}"]')
        for kpt in fc.keyframe_points:
            if is_prop:
                kpt.interpolation = "CONSTANT"
            else:
                kpt.interpolation = "BEZIER"
                kpt.handle_left_type = kpt.handle_right_type = "AUTO_CLAMPED"
            nk += 1
        fc.update()
    print(f"[s10b] ACTION {act.name} slot {slot.identifier!r}: {len(fcs)} fcurves, {nk} keys, keyed frames "
          f"{sorted(states)} (Bezier AUTO_CLAMPED on CTRL, CONSTANT on PROPS)")
    for f in sorted(states):
        print(f"[s10b] KEY f{f:02d}: {purposes[f]}")

    # ---------------------------------------------------------------- measure every frame
    topo = S.Topo(bpy.data.objects[S.MESH], bpy.data.objects[S.CLUB], parts)
    cams = goblib.load_json(S.CAM_JSON)
    sc.frame_set(1)
    S.refresh(rig)
    fv = S.FrontView(cams["front"], rig)
    def_names = [b["name"] for b in goblib.load_json("canonical_skeleton.json")["bones"]]
    per, defw, feet, thetas = {}, {}, {}, {}
    for f in range(S.FRAME_START, S.FRAME_END + 1):
        sc.frame_set(f)
        S.refresh(rig)
        vals, props = S.snapshot(rig)
        pose = {"frame": f, "name": f"f{f}", "contact": contact_at(f)}
        m = S.measure(rig, topo, man, vals, props, pose, fv)
        per[f] = m
        defw[f] = {n: (rig.matrix_world @ pbs[n].matrix).copy() for n in def_names}
        feet[f] = {s: ((rig.matrix_world @ pbs[f"foot_{s}"].head).copy(), (rig.matrix_world @ pbs[f"foot_{s}"].tail).copy())
                   for s in ("l", "r")}
        thetas[f] = bend_theta(rig, man)
        a = m["arms"]
        print(f"[s10b] F{f:02d} club {m['club']['min_mm']:6.1f} mm @ {m['club']['where']} z {m['club_min_z_mm']:6.1f} | vis "
              f"{m['self_isect']['visible_pairs']}p cover {m['self_isect']['visible_cover']} (total "
              f"{m['self_isect']['pairs']}) {m['self_isect']['visible_groups']} | wrDEF r {a['r']['wrist_bend_def']}/"
              f"{a['r']['wrist_twist_def']} | knee l {m['legs']['l']['knee']} r {m['legs']['r']['knee']} ratio "
              f"{m['legs']['l']['ik_ratio']}/{m['legs']['r']['ik_ratio']} reach {m['legs']['l']['reach_mm']}/"
              f"{m['legs']['r']['reach_mm']} shoe z {m['legs']['l']['shoe_min_z_mm']}/{m['legs']['r']['shoe_min_z_mm']} "
              f"| lean {m['front']['lean_hip_eye_deg']} | outside {m['outside_safe_range']}")

    # key pose DEF match (A2.1)
    km = {}
    for f in KEY_FRAMES:
        worst = 0.0
        for n, rows in def_snap[f].items():
            A = defw[f][n]
            worst = max(worst, max(abs(A[i][j] - rows[i][j]) for i in range(4) for j in range(4)))
        km[f] = worst
    # loop (A2.2)
    loop = max(max(abs(defw[39][n][i][j] - defw[1][n][i][j]) for i in range(4) for j in range(4)) for n in def_names)
    # flips / adjacent rotation (A2.4)
    flips = []
    for k in man["ikfk"]:
        for f in range(S.FRAME_START, S.FRAME_END):
            a, b = thetas[f][k], thetas[f + 1][k]
            if (a > 0) != (b > 0) and abs(a) > 2.0 and abs(b) > 2.0:
                flips.append({"chain": k, "frames": [f, f + 1], "theta": [round(a, 2), round(b, 2)]})
    sec = {}
    for f in range(S.FRAME_START + 1, S.FRAME_END + 1):
        name = next(s for s, a, b in SECTIONS if a <= f <= b)
        mx, bone = max((rot_diff_deg(defw[f - 1][n], defw[f][n]), n) for n in def_names)
        s = sec.setdefault(name, {"max_deg": 0.0})
        if mx >= s["max_deg"]:
            sec[name] = {"max_deg": round(mx, 2), "bone": bone, "frames": [f - 1, f]}
    # contact (A2.5)
    cr = []
    for s in ("l", "r"):
        for a, b in CONTACT_RANGES[s]:
            h0, t0 = feet[a][s]
            mv = max(max((feet[f][s][0] - h0).length, (feet[f][s][1] - t0).length) for f in range(a, b + 1))
            cr.append({"foot": s, "frames": [a, b], "max_move_mm": round(mv * 1000.0, 4)})
    # sections: clearance / cover
    secm = {}
    for name, a, b in SECTIONS:
        fr = range(a, b + 1)
        cl = min((per[f]["club"]["min_mm"], f) for f in fr)
        vc = [per[f]["self_isect"]["visible_cover"] for f in fr]
        vcm = ">=2" if ">=2" in vc else max(vc)
        secm[name] = {"frames": [a, b], "min_clearance_mm": cl[0], "at_frame": cl[1], "max_visible_cover": vcm,
                      "max_visible_pairs": max(per[f]["self_isect"]["visible_pairs"] for f in fr)}
    print(f"[s10b] KEYPOSE DEF match max abs {json.dumps({f: f'{v:.2e}' for f, v in km.items()})}")
    print(f"[s10b] LOOP f39 vs f1 DEF max abs {loop:.2e}")
    print(f"[s10b] FLIPS {len(flips)} {flips[:10]}")
    print(f"[s10b] ADJ ROT per section {json.dumps(sec)}")
    print(f"[s10b] CONTACT {json.dumps(cr)}")
    print(f"[s10b] SECTIONS {json.dumps(secm)}")
    theta_rng = {k: [round(min(thetas[f][k] for f in thetas), 2), round(max(thetas[f][k] for f in thetas), 2)]
                 for k in man["ikfk"]}
    print(f"[s10b] THETA range {theta_rng}")

    if not do_save:
        return
    # contact_ranges -> swing_keyposes.json (only this key changes)
    kp["contact_ranges"] = [{"foot": s, "frames": r} for s in ("l", "r") for r in CONTACT_RANGES[s]]
    kp["contact_notes"] = {
        "l": "left foot planted for the whole clip",
        "r": "right foot: planted on the idle spot f1-9, lifts f10, in the air f10-12 (heel first), lands in the "
             "wide stance f13, planted f13-26, lifts f27, steps back in the air f27-31 (a step, not the reference "
             "slide), lands on the idle spot f32, planted f32-39 (s10b animation timing)"}
    goblib.save_json(S.KEYPOSE_JSON, kp)
    INSP.mkdir(parents=True, exist_ok=True)
    with open(INSP / "s10b_measure.json", "w", encoding="utf-8") as fh:
        json.dump(goblib._jsonable({"keypose_def_match_max_abs": km, "loop_f39_f1_def_max_abs": loop, "flips": flips,
                                    "theta_range_deg": theta_rng, "max_adjacent_rot_per_section": sec,
                                    "contact": cr, "sections": secm,
                                    "keyed_frames": {f: purposes[f] for f in sorted(states)},
                                    "per_frame": {f: {"club": per[f]["club"], "club_min_z_mm": per[f]["club_min_z_mm"],
                                                      "self_isect": per[f]["self_isect"], "front": per[f]["front"],
                                                      "legs": per[f]["legs"],
                                                      "wrist_r": [per[f]["arms"]["r"]["wrist_bend_def"],
                                                                  per[f]["arms"]["r"]["wrist_twist_def"]],
                                                      "outside_safe_range": per[f]["outside_safe_range"]}
                                                  for f in per}}), fh, indent=1)
    sc.frame_set(S.FRAME_START)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s10b] SAVED {OUT_BLEND.name} (copy=True, save_version 0)")
    if do_render:
        render_all(rig, cams)


def render_all(rig, cams):
    sc = bpy.context.scene
    colors0 = {n: tuple(bpy.data.objects[n].color) for n in (S.MESH, S.CLUB)}
    S.setup_render()
    tmp = tempfile.mkdtemp(prefix="s10b_")
    try:
        sheet_cells = []
        for f in range(S.FRAME_START, S.FRAME_END + 1):
            sc.frame_set(f)
            S.refresh(rig)
            fr = S.over_white(S.render(cams["front"], INSP / f"front_{f:04d}.png"))
            S.save_rgb(fr, INSP / f"front_{f:04d}.png")
            tq = S.over_white(S.render(cams["three_quarter"], INSP / f"34_{f:04d}.png"))
            S.save_rgb(tq, INSP / f"34_{f:04d}.png")
            ref = S.load_rgba(REF_FRAMES / f"f_{f:04d}.png")[:, :, :3]          # 448x576
            ours = S.half(fr)                                                   # 448x576
            pair = np.concatenate([ref, ours], axis=1)
            S.save_rgb(pair, os.path.join(tmp, f"sbs_{f:04d}.png"))
            sheet_cells.append(S.half(pair))                                    # 448x288
        # contact sheet: 5 pairs per row
        cols, sep = 5, 4
        h, w = sheet_cells[0].shape[:2]
        rows = (len(sheet_cells) + cols - 1) // cols
        sheet = np.ones((rows * (h + sep), cols * (w + sep), 3), np.float32)
        for i, c in enumerate(sheet_cells):
            r, k = divmod(i, cols)
            sheet[r * (h + sep):r * (h + sep) + h, k * (w + sep):k * (w + sep) + w] = c
        S.save_rgb(sheet, INSP / "contact_sheet.png")

        def enc(pattern, out):
            cmd = [FFMPEG, "-y", "-loglevel", "error", "-framerate", "24", "-start_number", "1", "-i", pattern,
                   "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-r", "24",
                   str(out)]
            r = subprocess.run(cmd, capture_output=True, text=True)
            if r.returncode != 0:
                raise RuntimeError(f"ffmpeg failed ({r.returncode}): {r.stderr[-400:]}")
        enc(os.path.join(tmp, "sbs_%04d.png"), INSP / "side_by_side.mp4")
        enc(str(INSP / "34_%04d.png"), INSP / "ours_34.mp4")
        print(f"[s10b] RENDER {goblib._rel(INSP)}: front_0001..0039.png, 34_0001..0039.png, contact_sheet.png "
              f"({sheet.shape[1]}x{sheet.shape[0]}, ref|ours pairs, 5 per row), side_by_side.mp4 (24 fps, 896x576), "
              f"ours_34.mp4 (24 fps)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        for n, c in colors0.items():
            bpy.data.objects[n].color = c


if __name__ == "__main__":
    goblib.run_main(main)
