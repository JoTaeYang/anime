"""p33_ref_sheet.py - T312 (SI2): reference-matched key-pose sheet and review videos for one player clip.

Input : the clip's anim blend opened by Blender (work/player/rig/anim/pl_a_<clip>.blend; never saved),
        work/player/rig/anim/<clip>_resolved.json (p30: fps, frame_range, loop, key_frames, weapon),
        the reference frames (--ref-dir, --ref-pattern, --ref-fps; read only),
        work/player/rig/weapons/<name>.blend via p30.attach_weapons (preview only, removed again)
Output: work/player/inspect/clips/<clip>/
          <prefix>_keypose_sheet.png   rows = key_frames; columns = reference frame at the matching time | ours ref_front |
                                       ours three_quarter | ours side (the SI1 layout); cells <prefix>_f###_<view>.png
          <prefix>_side_by_side.mp4    reference (resampled to the clip time) | ours ref_front | ours three_quarter, with
                                       the weapon, --loops loops (default 3), frame counter in the Blender cells
          <prefix>_side_by_side.gif    one loop, 15 fps, 960 px wide
          <prefix>_upper_2x.mp4        ours ref_front cropped to the upper body at 2x zoom (half the full framing height,
                                       top-anchored, full width so the weapon fist stays in view) over fixed horizontal guide lines every 16 px, --loops loops
          <prefix>_ref_sheet.json      mapping, files and the ffmpeg commands (raw)

CLI:  blender -b --factory-startup work/player/rig/anim/pl_a_<clip>.blend --python p33_ref_sheet.py --
          --clip <clip> --ref-dir work/player/refs/<ref> [--ref-fps 24] [--ref-pattern f{:03d}.png] [--prefix SI2]
          [--loops 3] [--no-video]
      (paths relative to the repo root; ffmpeg must be on PATH)

Time mapping: clip frame f -> t = (f - frame_range[0]) / fps; reference frame = 1 + round(t * ref_fps), clamped to the
available reference frames (e.g. Sword_Idle f43 -> ref f35, f72 -> ref f58). Loop clips play frame_range[0] ..
frame_range[1] - 1 (the last frame equals the first).
Views: ref_front = front with a 10 deg high angle (reference framing); three_quarter / side = p30 (p23 / p24e) views;
top (T320) = straight down, character front (-Y) at the bottom of the image (slash arcs). --views picks the key-pose
sheet columns after the reference (default ref_front,three_quarter,side); --video-views the video columns after the
reference (default ref_front,three_quarter).
T323: key-pose cells are flattened onto white (opaque PNGs); video frames inside the resolved-JSON events window
(hit_start .. hit_end) get "HIT" in the first Blender cell label and a red band on top; --pad N holds the last frame N
times between plays (non-loop clips); --slow k also writes <prefix>_slow.mp4 (the same sequence at k x speed).
Renders reuse p30_clip_anim.Renderer (Workbench MATERIAL, fixed framing = the union of PL_mesh and the weapon over every
clip frame + 5 %); p30.view_basis is extended in memory with "ref_front" (p30 itself is not changed).
Replaces the T310 scratch work/player/inspect/clips/Sword_Idle/si1_sheet.py (sheet part).
"""
import argparse
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import p30_clip_anim as p30  # noqa: E402

RIG = HERE.parent
REPO = RIG.parents[2]
ANIM = RIG / "anim"
INSPECT = RIG.parent / "inspect" / "clips"
REF_EL_DEG = 10.0
RES = 512
GIF_FPS, GIF_W = 15, 960
GUIDE_PX = 16
_view_basis = p30.view_basis


def log(msg):
    print(f"[p33] {msg}")
    sys.stdout.flush()


def view_basis(view):
    """p30.view_basis + ref_front (front, REF_EL_DEG high angle) + top (T320)."""
    if view == "top":
        d = Vector((0.0, 0.0, -1.0))
        right = Vector((1.0, 0.0, 0.0))
        return d, right, right.cross(d).normalized()   # up = +Y (back): the front is at the bottom
    if view == "ref_front":
        el = math.radians(REF_EL_DEG)
        d = -Vector((0.0, -math.cos(el), math.sin(el)))
        d.normalize()
        right = d.cross(Vector((0, 0, 1))).normalized()
        return d, right, right.cross(d).normalized()
    return _view_basis(view)


def ffmpeg(args, log_cmds):
    cmd = ["ffmpeg", "-y", "-loglevel", "error"] + [str(a) for a in args]
    log_cmds.append(" ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"ffmpeg failed ({r.returncode}): {r.stderr[-600:]}")


def save_png(arr, path):
    h, w = arr.shape[:2]
    img = bpy.data.images.new("_p33_frame", w, h, alpha=True)
    try:
        rgba = np.concatenate([arr[:, :, :3], np.ones((h, w, 1), np.float32)], axis=2)
        img.pixels.foreach_set(np.ascontiguousarray(rgba[::-1]).ravel())
        img.filepath_raw = str(path)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)


def over_white(px, guides=False):
    h, w = px.shape[:2]
    bg = np.ones((h, w, 3), np.float32)
    if guides:
        bg[::GUIDE_PX, :, :] = 0.8
    a = px[:, :, 3:4]
    return px[:, :, :3] * a + bg * (1.0 - a)


def hstack(cells, gap=6):
    h = max(c.shape[0] for c in cells)
    w = sum(c.shape[1] for c in cells) + gap * (len(cells) + 1)
    out = np.full((h + 2 * gap, w, 3), 0.55, np.float32)
    x = gap
    for c in cells:
        out[gap:gap + c.shape[0], x:x + c.shape[1]] = c
        x += c.shape[1] + gap
    return out


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="p33_ref_sheet.py")
    ap.add_argument("--clip", required=True)
    ap.add_argument("--ref-dir", required=True)
    ap.add_argument("--ref-fps", type=float, default=24.0)
    ap.add_argument("--ref-pattern", default="f{:03d}.png")
    ap.add_argument("--prefix", default=None)
    ap.add_argument("--loops", type=int, default=3)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--views", default="ref_front,three_quarter,side")   # T320: key-pose sheet columns
    ap.add_argument("--video-views", default="ref_front,three_quarter")   # T323: video columns
    ap.add_argument("--pad", type=int, default=0)   # T323: idle pad frames between plays
    ap.add_argument("--slow", type=float, default=0.0)   # T323: extra slow-motion copy (speed factor)
    a = ap.parse_args(argv)
    clip = a.clip
    prefix = a.prefix or clip
    res = json.loads((ANIM / f"{clip}_resolved.json").read_text(encoding="utf-8"))
    fps, (fa, fb), loop = float(res["fps"]), res["frame_range"], bool(res["loop"])
    key_frames = res["key_frames"]
    ref_dir = (REPO / a.ref_dir).resolve()
    n_ref = 0
    while (ref_dir / a.ref_pattern.format(n_ref + 1)).exists():
        n_ref += 1
    if n_ref == 0:
        raise RuntimeError(f"no reference frames {ref_dir / a.ref_pattern}")

    def ref_of(f):
        return max(1, min(n_ref, 1 + int(round((f - fa) / fps * a.ref_fps))))

    out = INSPECT / clip
    out.mkdir(parents=True, exist_ok=True)
    p30.view_basis = view_basis
    rig = bpy.data.objects[p30.ARM]
    mesh = bpy.data.objects[p30.MESH]
    sc = bpy.context.scene
    ws = p30.attach_weapons(res.get("weapon") or {}, rig)
    lo, hi = [1e9] * 3, [-1e9] * 3
    for f in range(fa, fb + 1):
        sc.frame_set(f)
        p30.refresh(rig)
        for ob in [mesh] + [w[2] for w in ws]:
            x, y = p30.mesh_bbox(ob)
            lo = [min(u, v) for u, v in zip(lo, x)]
            hi = [max(u, v) for u, v in zip(hi, y)]
    pad = 0.05 * max(h - l for l, h in zip(lo, hi))
    full = ([x - pad for x in lo], [x + pad for x in hi])
    hh = (full[1][2] - full[0][2]) / 2.0
    upper = ([full[0][0], full[0][1], full[1][2] - hh], [full[1][0], full[1][1], full[1][2]])
    log(f"CLIP {clip}: {fps} fps, frames {fa}..{fb}, loop {loop}, key frames {key_frames}; reference {rel(ref_dir)} "
        f"{n_ref} frames @ {a.ref_fps} fps; weapon {[(s, n) for s, n, _ in ws]}; framing {[round(x, 4) for x in full[0]]} .. "
        f"{[round(x, 4) for x in full[1]]}; upper 2x {[round(x, 4) for x in upper[0]]} .. {[round(x, 4) for x in upper[1]]}")
    cmds, files = [], []
    rd = p30.Renderer([mesh, rig] + [w[2] for w in ws], full)
    rd_up = None
    tmp = Path(tempfile.mkdtemp(prefix="p33_"))
    try:
        # key-pose sheet (SI1 layout)
        rows = []
        for f in key_frames:
            rf = ref_of(f)
            row = [ref_dir / a.ref_pattern.format(rf)]
            for v in a.views.split(","):
                p = rd.shot(v, f, out / f"{prefix}_f{f:03d}_{v}.png", RES, f"f{f} {v}  (ref f{rf} @{a.ref_fps:g})")
                save_png(over_white(p30.load_rgba(p)), p)   # T323: opaque cell (flattened onto white)
                row.append(p)
                files.append(p)
            rows.append(row)
        sheet = p30.compose_sheet(rows, out / f"{prefix}_keypose_sheet.png")
        log(f"SHEET {rel(sheet)}: rows {key_frames} -> ref {[ref_of(f) for f in key_frames]}")
        vids = {}
        if not a.no_video:
            frames = list(range(fa, fb if loop else fb + 1))
            ev = {e["name"]: e["frame"] for e in res.get("events") or []}
            hit = (ev["hit_start"], ev["hit_end"]) if "hit_start" in ev and "hit_end" in ev else None
            vviews = a.video_views.split(",")
            rd_up = p30.Renderer([mesh, rig] + [w[2] for w in ws], upper)
            for i, f in enumerate(frames, start=1):
                rf = ref_of(f)
                t = (f - fa) / fps
                is_hit = hit is not None and hit[0] <= f <= hit[1]
                cells = []
                for j, v in enumerate(vviews):
                    lab = (f"f{f:02d}  t {t:.2f} s  ref f{rf}" + ("   HIT" if is_hit else "")) if j == 0 else f"f{f:02d} {v}"
                    cells.append(over_white(p30.load_rgba(rd.shot(v, f, tmp / f"v{j}.png", RES, lab))))
                ref = p30.load_rgba(ref_dir / a.ref_pattern.format(rf))
                if ref.shape[0] != RES:
                    k = RES / ref.shape[0]
                    yy = (np.arange(RES) / k).astype(int).clip(0, ref.shape[0] - 1)
                    xx = (np.arange(int(ref.shape[1] * k)) / k).astype(int).clip(0, ref.shape[1] - 1)
                    ref = ref[yy][:, xx]
                img = hstack([over_white(ref)] + cells)
                if is_hit:   # T323: red band on the event frames
                    img[:6, :, :] = np.array([0.85, 0.1, 0.1], np.float32)
                save_png(img, tmp / f"sbs_{i:04d}.png")
                up = p30.load_rgba(rd_up.shot("ref_front", f, tmp / "up.png", RES, f"f{f:02d}  2x"))
                save_png(hstack([over_white(up, guides=True)]), tmp / f"up_{i:04d}.png")
            n = len(frames)
            pad_n = 0 if loop else max(0, a.pad)
            one = list(range(1, n + 1)) + [n] * pad_n
            seq = []
            for k in range(a.loops):
                seq += list(range(1, n + 1)) + ([n] * pad_n if k < a.loops - 1 else [])
            for name in ("sbs", "up"):   # T323: play sequence with idle pads (copies of the rendered frames)
                for s, src in enumerate(seq, start=1):
                    shutil.copyfile(tmp / f"{name}_{src:04d}.png", tmp / f"seq_{name}_{s:04d}.png")
                for s, src in enumerate(one, start=1):
                    shutil.copyfile(tmp / f"{name}_{src:04d}.png", tmp / f"one_{name}_{s:04d}.png")
            pad_vf = "pad=ceil(iw/2)*2:ceil(ih/2)*2"
            outs = [("side_by_side", "seq_sbs_%04d.png", fps), ("upper_2x", "seq_up_%04d.png", fps)]
            if a.slow > 0:
                outs.append(("slow", "seq_sbs_%04d.png", fps * a.slow))
            for name, pat, rate in outs:
                dst = out / f"{prefix}_{name}.mp4"
                ffmpeg(["-framerate", rate, "-start_number", 1, "-i", tmp / pat, "-vf", pad_vf, "-c:v", "libx264",
                        "-pix_fmt", "yuv420p", "-r", fps, dst], cmds)
                vids[name] = rel(dst)
            gif = out / f"{prefix}_side_by_side.gif"
            ffmpeg(["-framerate", fps, "-start_number", 1, "-i", tmp / "one_sbs_%04d.png", "-vf",
                    f"fps={GIF_FPS},scale={GIF_W}:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse",
                    "-loop", 0, gif], cmds)
            vids["gif"] = rel(gif)
            log(f"VIDEO {vids}: {n} frames per play ({frames[0]}..{frames[-1]}), {a.loops} plays, pad {pad_n} between, "
                f"{len(seq)} frames @ {fps:g} fps; columns reference | {' | '.join(vviews)}; HIT frames {hit}; slow "
                f"{a.slow or '-'}; gif 1 play + pad @ {GIF_FPS} fps")
    finally:
        rd.close()
        if rd_up is not None:
            rd_up.close()
        p30.detach_weapons(ws)
        shutil.rmtree(tmp, ignore_errors=True)
    man = {"_doc": "written by work/player/rig/scripts/p33_ref_sheet.py (T312); raw", "clip": clip, "fps": fps,
           "frame_range": [fa, fb], "loop": loop, "reference": {"dir": rel(ref_dir), "pattern": a.ref_pattern,
                                                                "fps": a.ref_fps, "frames": n_ref},
           "time_mapping": "ref = 1 + round((f - frame_range[0]) / fps * ref_fps), clamped",
           "key_rows": [{"f": f, "ref": ref_of(f)} for f in key_frames], "views": {"ref_front_elevation_deg": REF_EL_DEG},
           "framing_m": full, "upper_2x_framing_m": upper, "sheet": rel(out / f"{prefix}_keypose_sheet.png"),
           "cells": [rel(p) for p in files], "videos": vids, "loops": a.loops, "pad": a.pad, "slow": a.slow,
           "video_views": a.video_views, "events": res.get("events"), "ffmpeg": cmds}
    with open(out / f"{prefix}_ref_sheet.json", "w", encoding="utf-8", newline="") as fh:
        fh.write(json.dumps(man, indent=1) + "\n")
    log(f"OUTPUT {rel(out / (prefix + '_ref_sheet.json'))}")
    log("DONE")


def rel(p):
    return p30.rel(p)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
