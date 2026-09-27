"""
STEP 04 - reference / camera setup for the goblin club-swing shot.

Idempotent.  Run it on ANY version of the goblin blend (v03, v04, ...):

    "C:\\Program Files\\Blender Foundation\\Blender 5.1\\blender.exe" -b <file.blend> \
        --python scripts\\step04_reference_setup.py -- [options]

It (re)creates, without ever duplicating anything:

  * scene settings        fps 24, frames 1..39, 448x576 (x --scale), 1:1 video<->blender frames
  * collection  REF
  * camera      REF_CAM   (in REF, made the scene camera) solved against reference frame 1
  * background  the 39 reference frames as an image sequence on REF_CAM, alpha 0.5,
                video frame N == blender frame N
  * REF_ground  a 4x4 m plane at z=0 in REF, hide_render=True (viewport contact reference only)

The render engine / shading style is NOT touched here (render_compare.py sets its own).

Options (after the `--`):
  --scale N        render at N x 448x576 (resolution_percentage = 100*N).   default 1
  --ref-dir DIR    reference directory holding frames/ and metrics.json.
                   default <blend dir>/ref, else <blend dir>/../ref
  --save PATH      save the result to PATH
  --no-save        do not save at all (just report)
  (default, if neither is given: save over the opened .blend)
  --no-ground      do not create REF_ground
  --solve          re-run the full camera solve against ref frame 1 instead of using the
                   baked-in solution, and print the new numbers (see SOLVER below)
  --solve-report   solve, print, but keep the baked camera (for auditing drift)

SOLVED CAMERA MODEL (baked in, see CAM_SOLVED below)
----------------------------------------------------
Perspective, sensor_fit HORIZONTAL, sensor_width 36 mm, lens 159.91 mm
(horizontal FOV 12.87 deg), located 10.06 m in front of the character
(-Y is the direction the character faces), 1.319 m high, pitched 1.85 deg down.
Fit against reference frame 1 with the character in its rest pose:
    silhouette IoU 0.9727,  rigid-landmark RMS 1.67 px  (see the step-04 report).

SOLVER
------
cost = 2.0 * landmark_RMS_px + 150.0 * (1 - silhouette_IoU)
  landmarks (rigid parts only, pose-independent):
     both eye centres, both foot ground-contact centres (x+y),
     head-top y (x unweighted: the reference silhouette bbox centre includes the club)
  silhouette: reference = per-row background hue model on the uniform grey cyclorama
              (threshold 0.035, morphological close + hole fill);
              ours = real Workbench alpha render at 448x576.
  optimiser: coarse landmark-only Nelder-Mead from a grid of seeds -> joint
             7-parameter Nelder-Mead on the full cost with real renders, 3 restarts.

  CAVEAT - the focal length is genuinely under-determined.  A dedicated sweep (lens fixed,
  everything else optimised, real renders) gave IoU 0.939 @70mm, 0.967 @100, 0.9749 @115,
  0.9743 @145, 0.9732 @160, 0.9633 @210, 0.949 @320, 0.941 @450, and 0.941 for a true
  ORTHOGRAPHIC camera.  So wide angle and orthographic are both clearly rejected, but
  anything in ~115-180 mm fits within 0.002 IoU of the best.  --solve therefore lands
  anywhere in that valley (a rerun produced 114 mm / IoU 0.9746 / RMS 2.50 px).  The baked
  value is the joint-cost minimum.  Do NOT treat 159.91 mm as a measured focal length.

  --solve needs the character in its REST pose (true for v03/v04 at frame 1 before any
  animation is keyed).  Run it on an animated file and the solve is meaningless.
"""

import bpy
import sys
import os
import math
import json

import numpy as np
from mathutils import Euler, Vector

# --------------------------------------------------------------------------------------
# constants
# --------------------------------------------------------------------------------------
REF_W, REF_H = 448, 576
FPS = 24
FRAME_START, FRAME_END = 1, 39
SENSOR_WIDTH = 36.0

COLL_NAME = "REF"
CAM_NAME = "REF_CAM"
GROUND_NAME = "REF_ground"

MESH_NAME = "GOB_body"
RIG_NAME = "GOB_rig"

# solved 2026-09-20 against ref/frames/f_0001.png with goblin_v03_controls.blend rest pose
CAM_SOLVED = {
    "type": "PERSP",
    "location": (-0.434320, -10.059320, 1.318990),
    "rotation_euler_deg": (88.148960, -0.092050, -2.406310),
    "lens": 159.9115,
    "sensor_fit": "HORIZONTAL",
    "sensor_width": SENSOR_WIDTH,
    "fit": {"iou": 0.9727, "landmark_rms_px": 1.666},
}

SEG_THRESHOLD = 0.035   # hue-distance threshold for the reference background segmentation


# --------------------------------------------------------------------------------------
# args
# --------------------------------------------------------------------------------------
def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    a = {"scale": 1.0, "ref_dir": None, "save": None, "no_save": False,
         "ground": True, "solve": False, "solve_report": False}
    i = 0
    while i < len(argv):
        t = argv[i]
        if t == "--scale":
            i += 1; a["scale"] = float(argv[i])
        elif t == "--ref-dir":
            i += 1; a["ref_dir"] = argv[i]
        elif t == "--save":
            i += 1; a["save"] = argv[i]
        elif t == "--no-save":
            a["no_save"] = True
        elif t == "--no-ground":
            a["ground"] = False
        elif t == "--solve":
            a["solve"] = True
        elif t == "--solve-report":
            a["solve_report"] = True
        else:
            print("step04: WARNING unknown option %r" % t)
        i += 1
    return a


def find_ref_dir(explicit):
    if explicit:
        return os.path.abspath(explicit)
    base = os.path.dirname(os.path.abspath(bpy.data.filepath or "."))
    for cand in (os.path.join(base, "ref"), os.path.join(base, os.pardir, "ref")):
        if os.path.isdir(os.path.join(cand, "frames")):
            return os.path.abspath(cand)
    return os.path.abspath(os.path.join(base, "ref"))


# --------------------------------------------------------------------------------------
# scene
# --------------------------------------------------------------------------------------
def setup_scene(scene, scale):
    scene.render.fps = FPS
    scene.render.fps_base = 1.0
    scene.frame_start = FRAME_START
    scene.frame_end = FRAME_END
    scene.render.resolution_x = REF_W
    scene.render.resolution_y = REF_H
    scene.render.resolution_percentage = int(round(100 * scale))
    scene.render.pixel_aspect_x = 1.0
    scene.render.pixel_aspect_y = 1.0


def get_collection(scene):
    coll = bpy.data.collections.get(COLL_NAME)
    if coll is None:
        coll = bpy.data.collections.new(COLL_NAME)
    if coll.name not in {c.name for c in scene.collection.children}:
        # only link if it is not already a child somewhere in this scene
        linked = any(coll.name in {c.name for c in p.children} for p in bpy.data.collections)
        if not linked:
            scene.collection.children.link(coll)
    return coll


def link_only_to(obj, coll):
    for c in list(obj.users_collection):
        if c is not coll:
            c.objects.unlink(obj)
    if obj.name not in coll.objects:
        coll.objects.link(obj)


def setup_camera(scene, coll, params):
    cam_data = bpy.data.cameras.get(CAM_NAME)
    if cam_data is None:
        cam_data = bpy.data.cameras.new(CAM_NAME)
        cam_data.name = CAM_NAME
    obj = bpy.data.objects.get(CAM_NAME)
    if obj is None or obj.type != "CAMERA":
        obj = bpy.data.objects.new(CAM_NAME, cam_data)
    obj.data = cam_data
    link_only_to(obj, coll)

    cam_data.type = params["type"]
    cam_data.sensor_fit = params["sensor_fit"]
    cam_data.sensor_width = params["sensor_width"]
    if params["type"] == "ORTHO":
        cam_data.ortho_scale = params["ortho_scale"]
    else:
        cam_data.lens = params["lens"]
    cam_data.clip_start = 0.05
    cam_data.clip_end = 200.0
    cam_data.show_name = True

    obj.location = params["location"]
    obj.rotation_mode = "XYZ"
    obj.rotation_euler = Euler([math.radians(v) for v in params["rotation_euler_deg"]], "XYZ")
    obj.scale = (1.0, 1.0, 1.0)

    scene.camera = obj
    return obj


def setup_background(cam_obj, ref_dir):
    """Reference frames as an image sequence on the camera, video frame N == blender frame N."""
    cam = cam_obj.data
    first = os.path.join(ref_dir, "frames", "f_0001.png")
    if not os.path.isfile(first):
        print("step04: WARNING no reference frames at %s - background skipped" % first)
        return None

    # reuse / (re)load the image datablock so repeated runs do not pile up copies
    img = bpy.data.images.get("REF_SEQ")
    if img is not None and os.path.abspath(bpy.path.abspath(img.filepath)) != os.path.abspath(first):
        bpy.data.images.remove(img)
        img = None
    if img is None:
        img = bpy.data.images.load(first, check_existing=False)
        img.name = "REF_SEQ"
    img.source = "SEQUENCE"
    img.filepath = first               # absolute; Blender keeps it absolute unless remapped

    cam.show_background_images = True
    cam.background_images.clear()      # idempotency: never stack duplicates
    bg = cam.background_images.new()
    bg.source = "IMAGE"
    bg.image = img
    bg.alpha = 0.5
    bg.display_depth = "FRONT"         # draw over the model so the overlay is readable
    bg.frame_method = "FIT"            # aspect is identical (448x576), so FIT is exact
    bg.show_background_image = True
    iu = bg.image_user
    iu.frame_duration = FRAME_END - FRAME_START + 1
    iu.frame_start = FRAME_START       # sequence starts at blender frame 1
    iu.frame_offset = 0                # ... showing f_0001 -> 1:1 mapping
    iu.use_cyclic = False
    iu.use_auto_refresh = True
    return bg


def setup_ground(coll):
    obj = bpy.data.objects.get(GROUND_NAME)
    if obj is None or obj.type != "MESH":
        mesh = bpy.data.meshes.get(GROUND_NAME) or bpy.data.meshes.new(GROUND_NAME)
        mesh.clear_geometry()
        s = 2.0
        mesh.from_pydata([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)], [], [(0, 1, 2, 3)])
        mesh.update()
        obj = bpy.data.objects.new(GROUND_NAME, mesh)
    link_only_to(obj, coll)
    obj.location = (0, 0, 0)
    obj.hide_render = True             # never contaminates a render
    obj.hide_select = True
    obj.display_type = "WIRE"
    return obj


# --------------------------------------------------------------------------------------
# reference silhouette + mesh landmarks  (shared by the solver and the report)
# --------------------------------------------------------------------------------------
def _load_rgba(path):
    img = bpy.data.images.load(path, check_existing=False)
    px = np.empty(len(img.pixels), dtype=np.float32)
    img.pixels.foreach_get(px)
    a = px.reshape(img.size[1], img.size[0], -1)[::-1].copy()   # -> origin top-left
    bpy.data.images.remove(img)
    return a


def _dil(m):
    o = m.copy(); o[1:] |= m[:-1]; o[:-1] |= m[1:]; o[:, 1:] |= m[:, :-1]; o[:, :-1] |= m[:, 1:]; return o


def _ero(m):
    o = m.copy(); o[1:] &= m[:-1]; o[:-1] &= m[1:]; o[:, 1:] &= m[:, :-1]; o[:, :-1] &= m[:, 1:]; return o


def _fill(m):
    out = np.zeros_like(m)
    out[0] = ~m[0]; out[-1] = ~m[-1]; out[:, 0] = ~m[:, 0]; out[:, -1] = ~m[:, -1]
    free = ~m
    for _ in range(4000):
        nxt = _dil(out) & free
        if nxt.sum() == out.sum():
            break
        out = nxt
    return m | ~out


def reference_mask(ref_dir, frame, threshold=SEG_THRESHOLD):
    """Segment the character off the uniform grey cyclorama with a per-row hue model."""
    a = _load_rgba(os.path.join(ref_dir, "frames", "f_%04d.png" % frame))[:, :, :3]
    side = np.concatenate([a[:, 0:36, :], a[:, 410:448, :]], axis=1)   # clean background columns
    bg = np.median(side, axis=1)
    hue = a / (a.sum(2, keepdims=True) + 1e-9)
    bghue = (bg / (bg.sum(1, keepdims=True) + 1e-9))[:, None, :]
    return _fill(_ero(_dil(np.abs(hue - bghue).sum(2) > threshold)))


def rest_vertices(mesh_obj):
    mw = np.array(mesh_obj.matrix_world)
    co = np.empty(len(mesh_obj.data.vertices) * 3, dtype=np.float64)
    mesh_obj.data.vertices.foreach_get("co", co)
    return (co.reshape(-1, 3) @ mw[:3, :3].T) + mw[:3, 3]


def mesh_landmarks(mesh_obj, rig_obj):
    """Rigid rest-pose landmarks in world space.  Character faces -Y, up +Z, feet on z=0."""
    V = rest_vertices(mesh_obj)
    out = {}
    hb = rig_obj.data.bones.get("HEAD")
    z0 = hb.head_local.z if hb else 1.32
    hl = hb.length if hb else 0.53
    cx = hb.head_local.x if hb else 0.0
    head = V[(V[:, 2] > z0 - 0.05 * hl) & (np.abs(V[:, 0] - cx) < 0.9 * hl)]
    front = head[head[:, 1] < head[:, 1].min() + 0.015]       # the two protruding pupils
    left = front[front[:, 0] > cx + 0.05 * hl]
    right = front[front[:, 0] < cx - 0.05 * hl]
    if len(left) and len(right):
        out["eye_L"] = left.mean(0)     # character's LEFT eye  -> screen RIGHT
        out["eye_R"] = right.mean(0)    # character's RIGHT eye -> screen LEFT
        out["eye_mid"] = (out["eye_L"] + out["eye_R"]) / 2.0
    zmin = V[:, 2].min()
    ground = V[V[:, 2] < zmin + 0.005]
    for tag, sel in (("foot_L", ground[:, 0] > 0), ("foot_R", ground[:, 0] < 0)):
        P = ground[sel]
        if len(P):
            out[tag] = np.array([(P[:, 0].min() + P[:, 0].max()) / 2,
                                 (P[:, 1].min() + P[:, 1].max()) / 2, zmin])
    out["head_top"] = V[V[:, 2] > V[:, 2].max() - 0.004].mean(0)
    return out


def project_np(P, p, ortho=False, W=REF_W, H=REF_H):
    """Same projection Blender uses (validated against world_to_camera_view to 0.01 px)."""
    C = np.array(p[:3])
    R = np.array(Euler((p[3], p[4], p[5]), "XYZ").to_matrix())
    d = (np.atleast_2d(P) - C) @ R
    if ortho:
        k = W / p[6]
        return np.stack([W / 2 + d[:, 0] * k, H / 2 - d[:, 1] * k], 1)
    z = np.maximum(-d[:, 2], 1e-6)
    s = p[6] / (SENSOR_WIDTH / 2) * W / 2
    return np.stack([W / 2 + d[:, 0] / z * s, H / 2 - d[:, 1] / z * s], 1)


# --------------------------------------------------------------------------------------
# solver (only used with --solve / --solve-report)
# --------------------------------------------------------------------------------------
def _nelder_mead(f, x0, step, iters=400, tol=1e-5):
    n = len(x0)
    pts = [np.array(x0, float)]
    for i in range(n):
        q = np.array(x0, float); q[i] += step[i]; pts.append(q)
    fv = [f(p) for p in pts]
    for _ in range(iters):
        o = np.argsort(fv); pts = [pts[i] for i in o]; fv = [fv[i] for i in o]
        if abs(fv[-1] - fv[0]) < tol:
            break
        cen = np.mean(pts[:-1], 0)
        xr = cen + (cen - pts[-1]); fr = f(xr)
        if fr < fv[0]:
            xe = cen + 2 * (cen - pts[-1]); fe = f(xe)
            pts[-1], fv[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < fv[-2]:
            pts[-1], fv[-1] = xr, fr
        else:
            xc = cen + 0.5 * (pts[-1] - cen); fc = f(xc)
            if fc < fv[-1]:
                pts[-1], fv[-1] = xc, fc
            else:
                for i in range(1, n + 1):
                    pts[i] = pts[0] + 0.5 * (pts[i] - pts[0]); fv[i] = f(pts[i])
    o = int(np.argmin(fv))
    return list(pts[o]), fv[o]


def solve_camera(scene, ref_dir, mesh_obj, rig_obj, verbose=True):
    """Return (params_dict, diagnostics).  Needs the mesh in its rest pose (v03-style)."""
    mask = reference_mask(ref_dir, 1)
    lm = mesh_landmarks(mesh_obj, rig_obj)
    names = ["eye_L", "eye_R", "foot_L", "foot_R", "head_top"]
    LM3D = np.array([lm[n] for n in names])

    mj = json.load(open(os.path.join(ref_dir, "metrics.json")))
    f1 = mj["frames"][0]
    e_sl, e_sr = f1["eyes"]          # [0] screen-left = char RIGHT, [1] screen-right = char LEFT
    fb = f1["feet_bboxes"]           # [0] char right (screen-left), [1] char left (screen-right)
    LM2D = np.array([
        [e_sr[0], e_sr[1]],
        [e_sl[0], e_sl[1]],
        [(fb[1][0] + fb[1][2]) / 2.0, fb[1][3]],
        [(fb[0][0] + fb[0][2]) / 2.0, fb[0][3]],
        [(f1["silhouette_bbox"][0] + f1["silhouette_bbox"][2]) / 2.0, f1["head_top_y"]],
    ])
    WX = np.array([1., 1., 1., 1., 0.])
    WY = np.array([1., 1., 1., 1., 0.7])

    # scratch camera for the real renders
    cam_data = bpy.data.cameras.new("_SOLVE_CAM")
    cam_obj = bpy.data.objects.new("_SOLVE_CAM", cam_data)
    scene.collection.objects.link(cam_obj)
    prev_cam, prev_engine = scene.camera, scene.render.engine
    prev_film, prev_out = scene.render.film_transparent, scene.render.filepath
    prev_pct = scene.render.resolution_percentage
    prev_x, prev_y = scene.render.resolution_x, scene.render.resolution_y
    scene.camera = cam_obj
    cam_data.sensor_fit = "HORIZONTAL"; cam_data.sensor_width = SENSOR_WIDTH
    scene.render.resolution_x, scene.render.resolution_y = REF_W, REF_H
    scene.render.resolution_percentage = 100
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.film_transparent = True
    scene.display.shading.light = "FLAT"
    scene.display.shading.color_type = "SINGLE"
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    tmp = os.path.join(bpy.app.tempdir, "step04_solve.png")
    scene.render.filepath = tmp
    scene.frame_set(1)

    hidden = []
    for o in scene.objects:
        if o.type == "MESH" and (o.name.startswith("WGT_") or o.name == GROUND_NAME) and not o.hide_render:
            o.hide_render = True; hidden.append(o)

    counter = [0]

    def render_mask(p, ortho):
        cam_obj.location = p[:3]
        cam_obj.rotation_euler = Euler((p[3], p[4], p[5]), "XYZ")
        if ortho:
            cam_data.type = "ORTHO"; cam_data.ortho_scale = p[6]
        else:
            cam_data.type = "PERSP"; cam_data.lens = p[6]
        bpy.ops.render.render(write_still=True)
        counter[0] += 1
        return _load_rgba(tmp)[:, :, 3] > 0.5

    def iou(p, ortho=False):
        rm = render_mask(p, ortho)
        return float((rm & mask).sum()) / max(float((rm | mask).sum()), 1.0)

    def lm_rms(p, ortho=False):
        d = project_np(LM3D, p, ortho) - LM2D
        return math.sqrt(float(((d[:, 0] * WX) ** 2 + (d[:, 1] * WY) ** 2).sum()) / (WX.sum() + WY.sum()))

    def cost(p, ortho=False):
        return 2.0 * lm_rms(p, ortho) + 150.0 * (1.0 - iou(p, ortho))

    # 1) coarse, landmark-only, over a lens grid (cheap: no renders)
    best = None
    for lens in (85., 115., 145., 180., 250.):
        for dm in (3.5, 5.0, 7.0):
            x0 = [0., -lens / 50.0 * dm, 1.1, math.radians(90), 0., 0.]
            p, c = _nelder_mead(lambda q: lm_rms(list(q) + [lens]), x0,
                                [0.05, 0.4, 0.1, 0.03, 0.02, 0.02], 500)
            if best is None or c < best[1]:
                best = (list(p) + [lens], c)
    # 2) joint 7-parameter refine on the full cost, with real renders
    p = best[0]
    for k in range(3):
        p, _ = _nelder_mead(lambda q: cost(list(q)), p,
                            [0.01, 0.15, 0.02, 0.005, 0.004, 0.004, 12.0 / (k + 1)], 400)
    fit_iou = iou(p)
    fit_rms = lm_rms(p)
    err = (project_np(LM3D, p) - LM2D)

    for o in hidden:
        o.hide_render = False
    scene.camera, scene.render.engine = prev_cam, prev_engine
    scene.render.film_transparent, scene.render.filepath = prev_film, prev_out
    scene.render.resolution_percentage = prev_pct
    scene.render.resolution_x, scene.render.resolution_y = prev_x, prev_y
    bpy.data.objects.remove(cam_obj)
    bpy.data.cameras.remove(cam_data)

    params = {"type": "PERSP",
              "location": tuple(float(v) for v in p[:3]),
              "rotation_euler_deg": tuple(math.degrees(float(v)) for v in p[3:6]),
              "lens": float(p[6]), "sensor_fit": "HORIZONTAL", "sensor_width": SENSOR_WIDTH,
              "fit": {"iou": fit_iou, "landmark_rms_px": fit_rms}}
    diag = {"renders": counter[0], "landmark_err_px": {names[i]: [float(err[i][0]), float(err[i][1])]
                                                       for i in range(len(names))}}
    if verbose:
        print("step04 solve: IoU %.4f  landmark RMS %.3f px  (%d renders)" % (fit_iou, fit_rms, counter[0]))
        print("step04 solve: location %s" % (params["location"],))
        print("step04 solve: rotation_euler_deg %s" % (params["rotation_euler_deg"],))
        print("step04 solve: lens %.4f mm on a %.1f mm horizontal sensor" % (params["lens"], SENSOR_WIDTH))
        for n in names:
            print("step04 solve:   %-8s dx %+6.2f  dy %+6.2f px" % (n, *diag["landmark_err_px"][n]))
    return params, diag


# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------
def main():
    a = parse_args()
    scene = bpy.context.scene
    ref_dir = find_ref_dir(a["ref_dir"])
    print("step04: blend   %s" % bpy.data.filepath)
    print("step04: ref dir %s" % ref_dir)

    setup_scene(scene, a["scale"])

    params = dict(CAM_SOLVED)
    if a["solve"] or a["solve_report"]:
        mesh_obj = bpy.data.objects.get(MESH_NAME)
        rig_obj = bpy.data.objects.get(RIG_NAME)
        if mesh_obj is None or rig_obj is None:
            print("step04: cannot solve, %s / %s missing" % (MESH_NAME, RIG_NAME))
        else:
            solved, _ = solve_camera(scene, ref_dir, mesh_obj, rig_obj)
            if a["solve"]:
                params = solved
            else:
                print("step04: --solve-report, keeping the baked camera")

    coll = get_collection(scene)
    cam_obj = setup_camera(scene, coll, params)
    bg = setup_background(cam_obj, ref_dir)
    ground = setup_ground(coll) if a["ground"] else None

    # ---- verification -----------------------------------------------------------------
    ok = True
    cams = [o for o in bpy.data.objects if o.type == "CAMERA"]
    print("step04: cameras in file: %s" % [o.name for o in cams])
    print("step04: REF collection objects: %s" % sorted(o.name for o in coll.objects))
    print("step04: scene.camera = %s" % (scene.camera.name if scene.camera else None))
    print("step04: fps %d  frames %d..%d  res %dx%d @%d%%" % (
        scene.render.fps, scene.frame_start, scene.frame_end,
        scene.render.resolution_x, scene.render.resolution_y, scene.render.resolution_percentage))
    print("step04: camera loc %s rot_deg %s %s %.4f" % (
        tuple(round(v, 5) for v in cam_obj.location),
        tuple(round(math.degrees(v), 5) for v in cam_obj.rotation_euler),
        "ortho_scale" if cam_obj.data.type == "ORTHO" else "lens",
        cam_obj.data.ortho_scale if cam_obj.data.type == "ORTHO" else cam_obj.data.lens))
    n_bg = len(cam_obj.data.background_images)
    print("step04: background_images=%d show_background_images=%s" % (n_bg, cam_obj.data.show_background_images))
    if n_bg != 1 or not cam_obj.data.show_background_images:
        ok = False
        print("step04: FAIL background not registered on the camera data")
    else:
        b = cam_obj.data.background_images[0]
        print("step04: bg image %r source=%s alpha=%.2f depth=%s frame_start=%d duration=%d offset=%d" % (
            b.image.filepath, b.image.source, b.alpha, b.display_depth,
            b.image_user.frame_start, b.image_user.frame_duration, b.image_user.frame_offset))
        if not os.path.isabs(bpy.path.abspath(b.image.filepath)):
            ok = False; print("step04: FAIL background path is not absolute")
    if ground is not None:
        print("step04: %s hide_render=%s" % (ground.name, ground.hide_render))

    if not a["no_save"]:
        path = a["save"] or bpy.data.filepath
        if not path:
            print("step04: no filepath to save to (use --save)")
        else:
            bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(path), compress=False)
            print("step04: saved %s" % os.path.abspath(path))
    else:
        print("step04: --no-save, nothing written")
    print("step04: %s" % ("OK" if ok else "PROBLEMS - see FAIL lines above"))


main()
