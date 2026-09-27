"""p34_weapon_preview.py - T313 (Sword_Idle SI3, spec work/player/d-04-player-sword-idle.md section 5): weapon preview FBX
for Unity, and the Unity preview media of a clip.

Mode --export (default):
  Input : work/player/rig/weapons/<weapon>.blend (p32; one WPN_* object, mesh authored in socket space per
          data/weapon_socket_contract.json: pivot = grip centre, blade +Z, guard Y), data/export_preset.json (read only)
  Output: work/player/rig/export/preview/<object>.fbx and unity/AvatarCheck/Assets/Player/Preview/<object>.fbx (byte copy).
          Never part of Player.fbx or a Player@ clip FBX (separate file, separate folder, not read by p26 / p27 / p31).
  Export = the player preset (axis_forward -Z, axis_up Y, use_space_transform, FBX_SCALE_ALL, global_scale 1, no leaf
  bones) with object_types {MESH}, bake_anim False and bake_space_transform True (the axis conversion is baked into the
  vertices, so the Unity node carries no conversion rotation and an identity local pose under the socket is exact).
  Compensation (T313 addendum, derived, not assumed): for each socket, K = Qu^-1 . C . Rb from the measured rest frames,
  Qu = the Unity rest world rotation (unity/AvatarCheck/player_report.json "rest", PlayerRigCheck, fresh Player.fbx
  instance), Rb = the Blender rest_matrix rotation (data/canonical_skeleton.json), C = export_preset axis_mapping
  blender_to_unity. K maps Blender bone-local to Unity bone-local. A baked export maps a Blender object point v to Unity
  mesh space C . v, so the mesh is pre-transformed (in memory) by P = C^-1 . K: an identity child of WeaponSocket_<side>
  then puts socket-space (a, b, c) exactly where Blender has it. P must be a proper rotation and equal for L and R (else
  RuntimeError); it is written to data/weapon_socket_contract.json "unity" (the approved Blender fields are not touched).
  PlayerClipCheck instantiates the preview under WeaponSocket_<side> with an identity local pose; U5 (checker) compares.
  Self-check (stdout): reimport with the preset reimport options + bake_space_transform True, bbox / vertex count.
  CLI: blender -b --factory-startup --python p34_weapon_preview.py -- --weapon placeholder_sword

Mode --unity-media:
  Input : work/player/inspect/clips/<clip>/unity_<cam>_<f>.png (PlayerClipCheck renders; PLAYER_CLIP_RENDER_FRAMES=all)
  Output: <prefix>_unity_sheet.png (rows = --frames, columns = front | three_quarter, frame label) and <prefix>_unity.gif
          (--gif-cam, every --gif-step frame of one loop, 30 / step fps, 512 px), <prefix>_unity_media.json (raw)
  CLI: blender -b --factory-startup --python p34_weapon_preview.py -- --unity-media --clip Sword_Idle --prefix SI3
          --frames 1,22,43,57,72,86 [--gif-cam three_quarter] [--gif-step 2]
  Uses ffmpeg (PATH) with drawtext (C:/Windows/Fonts/arial.ttf) for labels, tile for the sheet, palettegen for the gif.
"""
import argparse
import hashlib
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
PRESET = RIG / "data" / "export_preset.json"
WEAPONS = RIG / "weapons"
EXPORT_DIR = RIG / "export" / "preview"
UNITY_DIR = REPO / "unity" / "AvatarCheck" / "Assets" / "Player" / "Preview"
INSPECT = RIG.parent / "inspect" / "clips"
CONTRACT = RIG / "data" / "weapon_socket_contract.json"
CANON = RIG / "data" / "canonical_skeleton.json"
UNITY_REPORT = REPO / "unity" / "AvatarCheck" / "player_report.json"
FONT = "C\\:/Windows/Fonts/arial.ttf"


def log(msg):
    print(f"[p34] {msg}")
    sys.stdout.flush()


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def rel(p):
    try:
        return Path(p).resolve().relative_to(REPO).as_posix()
    except ValueError:
        return str(p)


def qmat(x, y, z, w):
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def tr(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def derive_compensation(preset):
    """P = C^-1 . Qu^-1 . C . Rb per socket from the measured rest frames (see the docstring)."""
    C = preset["axis_mapping"]["blender_to_unity"]
    rep = json.loads(UNITY_REPORT.read_text(encoding="utf-8"))
    canon = {b["name"]: b for b in json.loads(CANON.read_text(encoding="utf-8"))["bones"]}
    out = {}
    for side in ("R", "L"):
        n = f"WeaponSocket_{side}"
        r = rep["rest"][n]
        Qu = qmat(*r[3:7])
        Rb = [row[:3] for row in canon[n]["rest_matrix"][:3]]
        K = mm(mm(tr(Qu), C), Rb)
        P = mm(tr(C), K)   # C is orthonormal: C^-1 = C^T
        pos_err = math.dist([sum(C[i][k] * canon[n]["rest_matrix"][k][3] for k in range(3)) for i in range(3)], r[:3])
        out[side] = {"K": K, "P": P, "rest_pos_err_mm": pos_err * 1000.0}
    dLR = max(abs(out["R"]["P"][i][j] - out["L"]["P"][i][j]) for i in range(3) for j in range(3))
    P = out["R"]["P"]
    det = (P[0][0] * (P[1][1] * P[2][2] - P[1][2] * P[2][1]) - P[0][1] * (P[1][0] * P[2][2] - P[1][2] * P[2][0])
           + P[0][2] * (P[1][0] * P[2][1] - P[1][1] * P[2][0]))
    ortho = max(abs(sum(P[k][i] * P[k][j] for k in range(3)) - (1.0 if i == j else 0.0)) for i in range(3) for j in range(3))
    if dLR > 1e-3 or abs(det - 1.0) > 1e-3 or ortho > 1e-3:
        raise RuntimeError(f"compensation not a common proper rotation: |P_R - P_L| {dLR:.2e}, det {det:.6f}, ortho {ortho:.2e}")
    Pr = [[float(round(x)) if abs(x - round(x)) < 1e-3 else x for x in row] for row in P]   # snap measurement noise
    resid = max(abs(Pr[i][j] - P[i][j]) for i in range(3) for j in range(3))
    return Pr, {"per_side": out, "P_R_minus_P_L_max_abs": dLR, "det": det, "orthonormality_residual": ortho,
                "snap_residual": resid}


def export(weapon):
    import bpy
    from mathutils import Matrix
    src = WEAPONS / f"{weapon}.blend"
    bpy.ops.wm.open_mainfile(filepath=str(src), load_ui=False)
    obs = [o for o in bpy.data.objects if o.name.startswith("WPN_") and o.type == "MESH"]
    if len(obs) != 1:
        raise RuntimeError(f"{src.name}: {len(obs)} WPN_* mesh objects (expected 1)")
    ob = obs[0]
    if ob.matrix_world != Matrix.Identity(4):
        raise RuntimeError(f"{ob.name}: object transform is not identity")
    me = ob.data
    zs = [v.co.z for v in me.vertices]
    log(f"SOURCE {rel(src)} sha256 {sha(src)}: {ob.name} {len(me.vertices)} verts, socket-space z {min(zs):.4f}..{max(zs):.4f}")
    preset = json.loads(PRESET.read_text(encoding="utf-8"))
    P, der = derive_compensation(preset)
    Pm = Matrix(P)
    q, e = Pm.to_quaternion(), Pm.to_euler("XYZ")
    log(f"COMPENSATION P (Blender, pre-transform of the socket-space mesh) = {P}; quaternion wxyz "
        f"{[round(x, 6) for x in q]}, Euler XYZ deg {[round(math.degrees(x), 4) for x in e]}; K_R {der['per_side']['R']['K']}, "
        f"K_L {der['per_side']['L']['K']}; |P_R - P_L| {der['P_R_minus_P_L_max_abs']:.2e}, det {der['det']:.6f}, snap "
        f"residual {der['snap_residual']:.2e}; rest pos err R/L {der['per_side']['R']['rest_pos_err_mm']:.4f} / "
        f"{der['per_side']['L']['rest_pos_err_mm']:.4f} mm")
    me.transform(Pm.to_4x4())   # in memory only; the blend is never saved
    me.update()
    kw = dict(preset["export_scene_fbx"])
    kw.update(object_types={"MESH"}, bake_anim=False, bake_space_transform=True, use_selection=True)
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    fbx = EXPORT_DIR / f"{ob.name}.fbx"
    res = bpy.ops.export_scene.fbx(filepath=str(fbx), **kw)
    if res != {"FINISHED"}:
        raise RuntimeError(f"export_scene.fbx returned {res}")
    diff = {k: (preset["export_scene_fbx"].get(k), kw[k]) for k in kw if preset["export_scene_fbx"].get(k) != kw[k]
            and k != "object_types"}
    log(f"EXPORT {rel(fbx)}: {fbx.stat().st_size} bytes sha256 {sha(fbx)}; preset {PRESET.name} with {diff}, object_types "
        f"MESH; mesh pre-transformed by P")
    # self-check: reimport (preset reimport options + baked space transform)
    imp = dict(preset["blender_reimport"]["import_scene_fbx"])
    imp["bake_space_transform"] = True
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(fbx), **imp)
    for o in bpy.data.objects:
        vs = [o.matrix_world @ v.co for v in o.data.vertices] if o.type == "MESH" else []
        if vs:
            lo = [round(min(v[i] for v in vs), 4) for i in range(3)]
            hi = [round(max(v[i] for v in vs), 4) for i in range(3)]
            log(f"REIMPORT node {o.name!r} ({o.type}) parent {o.parent.name if o.parent else None}: world matrix identity "
                f"{o.matrix_world == Matrix.Identity(4)}, {len(vs)} verts, bbox {lo} .. {hi} (Blender axes after the "
                f"reimport conversion = the P-transformed socket-space mesh)")
    UNITY_DIR.mkdir(parents=True, exist_ok=True)
    dst = UNITY_DIR / fbx.name
    shutil.copy2(fbx, dst)
    log(f"COPY -> {rel(dst)} sha256 equal {sha(dst) == sha(fbx)}")
    unity = {
        "attach": "identity local transform under WeaponSocket_<side> (Unity); the compensation is baked into the weapon FBX",
        "export_compensation": {
            "matrix_blender": P, "quaternion_wxyz": [round(x, 6) for x in q],
            "euler_xyz_deg": [round(math.degrees(x), 4) for x in e],
            "applied_to": "the socket-space weapon mesh before the FBX export (p34_weapon_preview.py), export with the player "
                          "preset + bake_space_transform True, object_types MESH",
            "derivation": "K = Qu^-1 . C . Rb (Unity rest world rotation of WeaponSocket_<side> from unity/AvatarCheck/"
                          "player_report.json rest; Blender rest_matrix from data/canonical_skeleton.json; C = "
                          "export_preset axis_mapping blender_to_unity); a baked export maps v -> C . v, so P = C^-1 . K. "
                          f"Measured K_R = {der['per_side']['R']['K']}, K_L = {der['per_side']['L']['K']} (Unity bone-local = "
                          f"Blender bone-local with x negated); |P_R - P_L| = {der['P_R_minus_P_L_max_abs']:.2e} (one "
                          f"rotation for both sides); det {der['det']:.6f}; snapped to the nearest signed permutation "
                          f"(residual {der['snap_residual']:.2e})",
            "note": "a bone frame mapped as a rotation (C . R . C^T) differs from the Unity bone rotation by Rx(90) on every "
                    "bone; the axis mapping of the frame (C . R) differs only by the x flip K, which is what an identity "
                    "child inherits",
            "sources": {"player_report.json": sha(UNITY_REPORT), "canonical_skeleton.json": sha(CANON),
                        "export_preset.json": sha(PRESET)}},
        "preview_assets": {weapon: rel(dst)}}
    txt = CONTRACT.read_text(encoding="utf-8")
    con = json.loads(txt)
    cut = txt.find(',\n "unity": ')
    if cut >= 0:   # an earlier p34 entry (always the last key): drop it textually
        txt = txt[:cut] + "\n}\n"
    elif "unity" in con:
        raise RuntimeError(f"{CONTRACT.name}: a 'unity' entry not written by p34 (not at the end)")
    body = txt.rstrip()   # append textually so the approved fields keep their bytes
    if not body.endswith("}"):
        raise RuntimeError(f"{CONTRACT.name}: unexpected ending")
    txt = body[:-1].rstrip() + ',\n "unity": ' + json.dumps(unity, indent=1).replace("\n", "\n ") + "\n}\n"
    json.loads(txt)
    with open(CONTRACT, "w", encoding="utf-8", newline="") as fh:
        fh.write(txt)
    log(f"CONTRACT {rel(CONTRACT)}: unity entry written (attach identity, export_compensation)")


def ffmpeg(args, cmds):
    cmd = ["ffmpeg", "-y", "-loglevel", "error"] + [str(a) for a in args]
    cmds.append(" ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"ffmpeg failed ({r.returncode}): {r.stderr[-600:]}")


def label(text):
    return (f"drawtext=fontfile='{FONT}':text='{text}':x=16:y=12:fontsize=36:fontcolor=black:"
            f"box=1:boxcolor=white@0.7:boxborderw=6")


def media(a):
    d = INSPECT / a.clip
    frames = [int(x) for x in a.frames.split(",")]
    res = json.loads((RIG / "anim" / f"{a.clip}_resolved.json").read_text(encoding="utf-8"))
    fa, fb = res["frame_range"]
    loop_frames = list(range(fa, fb if res["loop"] else fb + 1))
    cams = ("front", "three_quarter")
    need = [d / f"unity_{c}_{f}.png" for f in frames for c in cams] + \
           [d / f"unity_{a.gif_cam}_{f}.png" for f in loop_frames[::a.gif_step]]
    miss = [rel(p) for p in need if not p.exists()]
    if miss:
        raise RuntimeError(f"missing Unity renders ({len(miss)}), e.g. {miss[:3]} (run PlayerClipCheck with "
                           f"PLAYER_CLIP_RENDER_FRAMES=all)")
    cmds = []
    tmp = Path(tempfile.mkdtemp(prefix="p34_"))
    try:
        i = 0
        for f in frames:
            for c in cams:
                ffmpeg(["-i", d / f"unity_{c}_{f}.png", "-vf", label(f"Unity f{f} {c}"), tmp / f"cell_{i:02d}.png"], cmds)
                i += 1
        sheet = d / f"{a.prefix}_unity_sheet.png"
        ffmpeg(["-framerate", 1, "-i", tmp / "cell_%02d.png", "-vf",
                f"tile={len(cams)}x{len(frames)}:padding=6:margin=6:color=gray", "-frames:v", 1, sheet], cmds)
        for j, f in enumerate(loop_frames[::a.gif_step]):
            ffmpeg(["-i", d / f"unity_{a.gif_cam}_{f}.png", "-vf", f"{label(f'Unity f{f}')},scale=512:-1:flags=lanczos",
                    tmp / f"g_{j:03d}.png"], cmds)
        gif = d / f"{a.prefix}_unity.gif"
        fps = res["fps"] / a.gif_step
        ffmpeg(["-framerate", fps, "-i", tmp / "g_%03d.png", "-vf",
                "split[x][y];[x]palettegen[p];[y][p]paletteuse", "-loop", 0, gif], cmds)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    man = {"_doc": "written by work/player/rig/scripts/p34_weapon_preview.py --unity-media (T313); raw", "clip": a.clip,
           "sheet": rel(sheet), "sheet_rows_frames": frames, "sheet_columns": list(cams), "gif": rel(gif),
           "gif_cam": a.gif_cam, "gif_frames": loop_frames[::a.gif_step], "gif_fps": fps, "sources": [rel(p) for p in need],
           "ffmpeg": cmds}
    with open(d / f"{a.prefix}_unity_media.json", "w", encoding="utf-8", newline="") as fh:
        fh.write(json.dumps(man, indent=1) + "\n")
    log(f"MEDIA {rel(sheet)} (rows {frames}, columns {list(cams)}); {rel(gif)} ({len(man['gif_frames'])} frames "
        f"{man['gif_frames'][0]}..{man['gif_frames'][-1]} @ {fps:g} fps)")


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="p34_weapon_preview.py")
    ap.add_argument("--unity-media", action="store_true")
    ap.add_argument("--weapon", default="placeholder_sword")
    ap.add_argument("--clip")
    ap.add_argument("--prefix")
    ap.add_argument("--frames", default="")
    ap.add_argument("--gif-cam", default="three_quarter")
    ap.add_argument("--gif-step", type=int, default=2)
    a = ap.parse_args(argv)
    if a.unity_media:
        if not (a.clip and a.prefix and a.frames):
            raise RuntimeError("--unity-media needs --clip, --prefix and --frames")
        media(a)
    else:
        export(a.weapon)
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
