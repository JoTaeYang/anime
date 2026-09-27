"""p31_export_clip.py - T300 (P3 tooling, spec work/player/d-03-player-anim-tools.md section 2): export one player clip.

Input : work/player/rig/anim/pl_a_<clip>.blend (p30; opened by p26.build_export, never saved),
        work/player/rig/data/clips/<clip>.json (fps / frame_range, read only),
        work/player/rig/data/export_preset.json (player preset, read only; written only by p27),
        work/player/rig/data/canonical_skeleton.json (via p26, read only)
Output: work/player/rig/export/stage_<clip>.blend   p26.build_export (DEF-only `Player`, every frame baked from the
                                                     PL_rig DEF world matrices, so the skirt follow is baked in)
        work/player/rig/export/Player@<clip>.fbx     armature `Player` + Player_mesh + action <clip> (AnimStack <clip>),
                                                     scene fps = the clip fps
        unity/AvatarCheck/Assets/Player/Player@<clip>.fbx  (byte copy). Player.fbx (rest) is not touched (its sha256
                                                     before / after is logged).

CLI:  blender -b --factory-startup --python p31_export_clip.py -- --clip <name>

Copied / adapted from work/goblin_swing/rig/scripts/s11_export_clip.py (goblin file unchanged): stage build, scene
renamed to the clip in memory (s11:157-171, the active-action export names its single AnimStack after the scene),
export with the preset + bake_anim True (s11:48-65), byte copy to Unity (s11:185-189), self-check reimport (s11:68-130).
Player helpers reused unchanged: p26_export_stage.build_export, p27_export_fbx.export_fbx / fbx_summary / stage_world /
reimport (every reimported bone unconnected first; reimport frame = stage frame + 0 / 1).
The clip name `rigtest` is refused (its stage / FBX belong to p27).
"""
import argparse
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

import bpy

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import p26_export_stage as p26  # noqa: E402
import p27_export_fbx as p27  # noqa: E402

RIG = HERE.parent
REPO = RIG.parents[2]
ANIM = RIG / "anim"
EXPORT = RIG / "export"
CLIPS = RIG / "data" / "clips"
PRESET = RIG / "data" / "export_preset.json"
UNITY_DIR = REPO / "unity" / "AvatarCheck" / "Assets" / "Player"
ARM, MESH = "Player", "Player_mesh"


def log(msg):
    print(f"[p31] {msg}")
    sys.stdout.flush()


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="p31_export_clip.py")
    ap.add_argument("--clip", required=True)
    a = ap.parse_args(argv)
    clip = a.clip
    if not re.fullmatch(r"[A-Za-z0-9_]+", clip) or clip == "rigtest":
        raise RuntimeError(f"clip name {clip!r}: [A-Za-z0-9_] only, and not 'rigtest' (p27 owns it)")
    cj = json.loads((CLIPS / f"{clip}.json").read_text(encoding="utf-8"))
    fps = int(cj.get("fps", 30))
    fa, fb = cj["frame_range"]
    src = ANIM / f"pl_a_{clip}.blend"
    stage = EXPORT / f"stage_{clip}.blend"
    fbx = EXPORT / f"Player@{clip}.fbx"
    if not src.exists():
        raise RuntimeError(f"{src} missing (run p30_clip_anim.py --clip {clip} first)")
    preset = json.loads(PRESET.read_text(encoding="utf-8"))
    kw_exp = preset["export_scene_fbx"]
    kw_imp = preset["blender_reimport"]["import_scene_fbx"]
    rest_unity = UNITY_DIR / "Player.fbx"
    rest_sha0 = sha(rest_unity) if rest_unity.exists() else None
    log(f"SOURCE {src} sha256 {sha(src)}; preset {PRESET.name} sha256 {sha(PRESET)} (read only); clip fps {fps}, "
        f"frames {fa}..{fb}")

    # 1. stage (p26: the anim blend is opened, never saved)
    EXPORT.mkdir(parents=True, exist_ok=True)
    p26.build_export(src, clip, stage)

    # 2. FBX from the stage
    bpy.ops.wm.open_mainfile(filepath=str(stage), load_ui=False)
    sc = bpy.context.scene
    arm = bpy.data.objects[ARM]
    act = arm.animation_data.action if arm.animation_data else None
    if act is None or act.name != clip:
        raise RuntimeError(f"stage: {ARM} action {act.name if act else None!r}, expected {clip!r}")
    if (sc.render.fps, sc.render.fps_base) != (fps, 1.0):
        log(f"STAGE scene fps {sc.render.fps}/{sc.render.fps_base} -> {fps}/1.0 (in memory)")
        sc.render.fps, sc.render.fps_base = fps, 1.0
    if (sc.frame_start, sc.frame_end) != (fa, fb) or tuple(int(round(x)) for x in act.frame_range) != (fa, fb):
        raise RuntimeError(f"stage frames scene {sc.frame_start}..{sc.frame_end}, action {tuple(act.frame_range)}; "
                           f"clip {fa}..{fb}")
    old = sc.name
    sc.name = clip
    log(f"STAGE {stage.name}: scene {old!r} -> {sc.name!r} (in memory, AnimStack name), action {act.name!r} frames "
        f"{tuple(act.frame_range)}, fps {sc.render.fps}")
    p27.export_fbx(fbx, [bpy.data.objects[ARM], bpy.data.objects[MESH]], True, kw_exp)
    s = p27.fbx_summary(fbx)
    log(f"FILE {fbx.name}: FBX {s['fbx_version']}; header {s['header']}; nodes {s['nodes']}; limb nodes "
        f"{s['limb_nodes']}, _end {s['limb_end_nodes']}; skin clusters {s['skin_clusters']}; materials "
        f"{s['materials']}; anim stacks {s['anim_stacks']}")

    # 3. self-check reimport (stdout only)
    names, parent, stage_w, rng = p27.stage_world(stage)
    p27.reimport(fbx, kw_imp, names, parent, stage_w, rng)
    log(f"REIMPORT scene fps after import {bpy.context.scene.render.fps}")

    # 4. Unity copy (Player.fbx untouched)
    UNITY_DIR.mkdir(parents=True, exist_ok=True)
    dst = UNITY_DIR / fbx.name
    shutil.copy2(fbx, dst)
    rest_sha1 = sha(rest_unity) if rest_unity.exists() else None
    log(f"COPY {fbx.name} -> {dst} sha256 equal {sha(dst) == sha(fbx)}; Assets/Player/Player.fbx sha256 unchanged "
        f"{rest_sha0 == rest_sha1} ({rest_sha1})")
    log(f"UNITY Assets/Player now: {sorted(x.name for x in UNITY_DIR.iterdir())}")
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
