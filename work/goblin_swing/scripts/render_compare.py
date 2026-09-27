"""
render_compare.py - render the goblin through REF_CAM and compare with the reference frames.

Dual mode.  Normally you run it under Blender; it then calls itself with the SYSTEM python
(which has PIL) to build the composites.

    "C:\\Program Files\\Blender Foundation\\Blender 5.1\\blender.exe" -b <file.blend> \
        --python scripts\\render_compare.py -- --frames 1,9,15 --out inspect\\step04

Options (after the `--`):
  --frames LIST      frames to render: "1,9,15" or "1-39" or "1-39:3" (start-end:step).  default 1
  --out DIR          output directory (created).                    default inspect/step04
  --ref-dir DIR      reference dir with frames/ and metrics.json.   default <blend dir>/ref or ../ref
  --json PATH        screen-space metrics json.      default <out>/screen_metrics.json
  --scale N          render at N x 448x576 (composites are made at the render size).  default 1
  --shading MODE     workbench light mode: FLAT | STUDIO | MATCAP.  default STUDIO
  --bg MODE          render background: transparent | grey.         default grey
  --club-head-offset M   metres below the WEAPON tail for `club_head_center`.  default 0.12
  --no-composite     skip the side-by-side / overlay images (still writes the json)
  --python-exe PATH  system python to use for the composites (needs PIL).  default: `python` on PATH

Outputs in --out:
  render_f0001.png      the raw render through REF_CAM
  side_f0001.png        reference | render, frame number burned in
  over_f0001.png        50% overlay of render on reference, frame number burned in
  screen_metrics.json   see below

=======================================================================================
SCREEN-SPACE CONVENTIONS  (identical to ref/metrics.json, so Gate 04/05/10 can diff)
=======================================================================================
  * image is 448 x 576 (the reference resolution).  With --scale N the render is larger
    but every coordinate below is normalised by the RENDER size, so it stays comparable.
  * origin is TOP-LEFT.   x = px / width,  y = py / height.   Both in 0..1.
    A point outside the frame keeps going past 0..1; `visible` says whether it is inside.
  * screen-LEFT is the character's RIGHT (the character faces the camera), exactly as in
    REFERENCE_ANALYSIS.md.  Every key below is named after the CHARACTER's side:
    `foot_L` is the character's left foot and appears on screen-RIGHT.
  * `chest_tilt_deg` = angle of the hips->eye-midpoint vector away from screen vertical,
    positive = top tilted toward screen-RIGHT (= the character's left).
    Same sign convention and same definition as `spine_lean_deg_screen` in metrics.json.
  * all points are evaluated through the depsgraph at each frame, so rig constraints,
    IK and drivers are included.

  point key            how it is defined
  -------------------- ----------------------------------------------------------------
  weapon_tip           WEAPON bone TAIL in world space (the far end of the club head)
  club_head_center     WEAPON tail - 0.12 m along the bone axis (toward the grip)
  club_head_center_blobfit
                       WEAPON tail - 0.21 m along the bone.  THIS is the one that matches
                       metrics.json `club.head_center` (the max-inscribed-circle centre of
                       the segmented brown blob): calibrated on frame 1 to 0.87 px.
                       `club_head_center` at the spec'd 0.12 m sits 18.6 px higher.
  hand_R               HAND_R bone HEAD (the grip)
  hand_L               HAND_L bone centre ((head+tail)/2)
  eye_L / eye_R        the two eye discs, located once on the REST mesh (the pupils are
                       the front-most head vertices) and then carried by the HEAD bone
  head                 eye-line midpoint = (eye_L + eye_R)/2.  NOTE: this is the eye line,
                       not the head's volumetric centre - same as metrics.json `eye_mid`.
  hips                 HIPS bone HEAD.  NOTE: metrics.json `hips_belt_centroid` is the
                       centroid of the brown belt BLOB, which sits HIGHER.  Measured on the
                       rest pose at frame 1: HIPS bone head = belt centroid + (-1.3, +20.2) px.
                       Subtract that offset before diffing absolute hip positions.
  weapon_tip vs metrics.json `club.head_outer_tip`: the reference point is the far end of
                       the segmented blob's medial axis, 18.4 px BELOW our WEAPON tail at f1.
  foot_L / foot_R      ground-contact point = bottom-centre of each foot, located once on
                       the REST mesh (vertices at the lowest z, bbox centre) and then
                       carried by the FOOT_L / FOOT_R bone
  head_top             highest rest vertex, carried by the HEAD bone (for scale checks)

  json layout:
    {"convention": {...}, "resolution": [w,h], "camera": {...},
     "frames": [{"frame": 1,
                 "points": {"weapon_tip": {"px":[x,y], "norm":[x,y], "world":[x,y,z],
                                           "visible": true}, ...},
                 "chest_tilt_deg": 0.7}, ...]}
"""

import sys
import os
import json
import math

# ======================================================================================
# composite mode (system python, needs PIL)
# ======================================================================================
def composite_main(argv):
    from PIL import Image, ImageDraw, ImageFont
    out = ref = None
    frames = []
    i = 0
    while i < len(argv):
        if argv[i] == "--out":
            i += 1; out = argv[i]
        elif argv[i] == "--ref-dir":
            i += 1; ref = argv[i]
        elif argv[i] == "--frames":
            i += 1; frames = [int(x) for x in argv[i].split(",") if x]
        i += 1

    def font(sz):
        for p in (r"C:\Windows\Fonts\arialbd.ttf", r"C:\Windows\Fonts\arial.ttf"):
            if os.path.isfile(p):
                return ImageFont.truetype(p, sz)
        try:
            return ImageFont.load_default(size=sz)
        except TypeError:
            return ImageFont.load_default()

    def label(img, text, sz=22):
        d = ImageDraw.Draw(img)
        for dx in (-2, 0, 2):
            for dy in (-2, 0, 2):
                d.text((8 + dx, 6 + dy), text, font=font(sz), fill=(0, 0, 0))
        d.text((8, 6), text, font=font(sz), fill=(255, 255, 0))
        return img

    made = 0
    for f in frames:
        rp = os.path.join(out, "render_f%04d.png" % f)
        fp = os.path.join(ref, "frames", "f_%04d.png" % f)
        if not (os.path.isfile(rp) and os.path.isfile(fp)):
            print("composite: missing %s or %s" % (rp, fp)); continue
        R = Image.open(rp).convert("RGB")
        F = Image.open(fp).convert("RGB").resize(R.size, Image.LANCZOS)
        w, h = R.size
        side = Image.new("RGB", (w * 2 + 6, h), (24, 24, 24))
        side.paste(F, (0, 0)); side.paste(R, (w + 6, 0))
        d = ImageDraw.Draw(side)
        d.text((10, h - 30), "REF", font=font(20), fill=(255, 120, 120))
        d.text((w + 16, h - 30), "BLENDER", font=font(20), fill=(120, 255, 120))
        label(side, "f%03d" % f)
        side.save(os.path.join(out, "side_f%04d.png" % f))
        over = Image.blend(F, R, 0.5)
        label(over, "f%03d  50%% overlay" % f)
        over.save(os.path.join(out, "over_f%04d.png" % f))
        made += 1
    print("composite: wrote %d side/overlay pairs to %s" % (made, out))
    return 0


# ======================================================================================
# blender mode
# ======================================================================================
def blender_main():
    import bpy
    import numpy as np
    from mathutils import Vector
    from bpy_extras.object_utils import world_to_camera_view

    REF_W, REF_H = 448, 576
    CAM_NAME, GROUND_NAME = "REF_CAM", "REF_ground"
    MESH_NAME, RIG_NAME = "GOB_body", "GOB_rig"

    def pick_mesh():
        """Whichever GOB body mesh is actually RENDER-visible.

        v05 carries both a high-poly (GOB_body) and a 12k low-poly proxy
        (GOB_body_lo); only one of them is render-visible at a time and all the
        landmarks below are read off the rest mesh of that object.
        """
        cands = [o for o in bpy.data.objects
                 if o.type == "MESH" and o.name.startswith("GOB_body")]
        def renderable(o):
            if o.hide_render:
                return False
            return not any(c.hide_render for c in o.users_collection)
        vis = [o for o in cands if renderable(o)]
        if len(vis) == 1:
            return vis[0]
        if vis:
            print("render_compare: WARNING %d GOB meshes render-visible, using %s"
                  % (len(vis), vis[0].name))
            return vis[0]
        return bpy.data.objects.get(MESH_NAME)
    CLUB_HEAD_BACK_OFF = 0.12      # m below the WEAPON tail, along the bone (spec)
    CLUB_HEAD_BLOBFIT = 0.21       # m: reproduces metrics.json club.head_center (0.87 px at f1)

    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    a = {"frames": "1", "out": None, "ref_dir": None, "json": None, "scale": 1.0,
         "shading": "STUDIO", "bg": "grey", "composite": True, "python_exe": None}
    i = 0
    while i < len(argv):
        t = argv[i]
        if t in ("--frames", "--out", "--ref-dir", "--json", "--shading", "--bg", "--python-exe"):
            key = t[2:].replace("-", "_")
            i += 1; a[key] = argv[i]
        elif t == "--scale":
            i += 1; a["scale"] = float(argv[i])
        elif t == "--club-head-offset":
            i += 1; CLUB_HEAD_BACK_OFF = float(argv[i])
        elif t == "--no-composite":
            a["composite"] = False
        else:
            print("render_compare: WARNING unknown option %r" % t)
        i += 1

    def parse_frames(spec):
        out = []
        for part in spec.split(","):
            part = part.strip()
            if not part:
                continue
            if "-" in part:
                rng, _, st = part.partition(":")
                s, _, e = rng.partition("-")
                out += list(range(int(s), int(e) + 1, int(st) if st else 1))
            else:
                out.append(int(part))
        return sorted(set(out))

    frames = parse_frames(a["frames"])
    base = os.path.dirname(os.path.abspath(bpy.data.filepath or "."))
    ref_dir = os.path.abspath(a["ref_dir"]) if a["ref_dir"] else None
    if ref_dir is None:
        for c in (os.path.join(base, "ref"), os.path.join(base, os.pardir, "ref")):
            if os.path.isdir(os.path.join(c, "frames")):
                ref_dir = os.path.abspath(c); break
        ref_dir = ref_dir or os.path.abspath(os.path.join(base, "ref"))
    out_dir = os.path.abspath(a["out"] or os.path.join(base, "inspect", "step04"))
    os.makedirs(out_dir, exist_ok=True)
    json_path = os.path.abspath(a["json"] or os.path.join(out_dir, "screen_metrics.json"))

    scene = bpy.context.scene
    cam = bpy.data.objects.get(CAM_NAME)
    if cam is None or cam.type != "CAMERA":
        print("render_compare: FATAL no %s in this blend - run step04_reference_setup.py first" % CAM_NAME)
        return 2
    scene.camera = cam
    rig = bpy.data.objects.get(RIG_NAME)
    mesh = pick_mesh()
    if mesh is not None:
        MESH_NAME = mesh.name
        print("render_compare: using mesh %s" % MESH_NAME)
    if rig is None or mesh is None:
        print("render_compare: FATAL missing %s / %s" % (RIG_NAME, MESH_NAME))
        return 2

    # ---- render settings -------------------------------------------------------------
    scene.render.resolution_x, scene.render.resolution_y = REF_W, REF_H
    scene.render.resolution_percentage = int(round(100 * a["scale"]))
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.film_transparent = (a["bg"] == "transparent")
    sh = scene.display.shading
    sh.light = a["shading"].upper()
    sh.color_type = "SINGLE"
    sh.single_color = (0.62, 0.62, 0.60)
    sh.show_shadows = False
    sh.show_cavity = False
    scene.display.render_aa = "8"
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA" if scene.render.film_transparent else "RGB"
    if not scene.render.film_transparent:
        w = scene.world or bpy.data.worlds.new("REF_W")
        scene.world = w
        try:
            w.use_nodes = False
            w.color = (0.34, 0.31, 0.30)
        except Exception:
            pass
        sh.background_type = "VIEWPORT"
        sh.background_color = (0.34, 0.31, 0.30)
    # never render helper geometry
    hidden = []
    for o in scene.objects:
        if o.type == "MESH" and (o.name.startswith("WGT_") or o.name == GROUND_NAME) and not o.hide_render:
            o.hide_render = True; hidden.append(o)
    # the camera background image must not bleed into a render (it never does, but be explicit)
    prev_show_bg = cam.data.show_background_images

    # ---- rest-pose landmarks, expressed in bone space --------------------------------
    mwm = np.array(mesh.matrix_world)
    co = np.empty(len(mesh.data.vertices) * 3, dtype=np.float64)
    mesh.data.vertices.foreach_get("co", co)
    V = (co.reshape(-1, 3) @ mwm[:3, :3].T) + mwm[:3, 3]

    warn = []
    rest_pts = {}
    hb = rig.data.bones.get("HEAD")
    if hb is not None:
        z0, hl, cx = hb.head_local.z, hb.length, hb.head_local.x
        head = V[(V[:, 2] > z0 - 0.05 * hl) & (np.abs(V[:, 0] - cx) < 0.9 * hl)]
        front = head[head[:, 1] < head[:, 1].min() + 0.015]
        L = front[front[:, 0] > cx + 0.05 * hl]
        R = front[front[:, 0] < cx - 0.05 * hl]
        if len(L) > 5 and len(R) > 5:
            rest_pts["eye_L"] = ("HEAD", L.mean(0))
            rest_pts["eye_R"] = ("HEAD", R.mean(0))
        else:
            warn.append("eye discs not found on the mesh (L=%d R=%d verts); "
                        "head point falls back to the HEAD bone midpoint" % (len(L), len(R)))
        rest_pts["head_top"] = ("HEAD", V[V[:, 2] > V[:, 2].max() - 0.004].mean(0))
    zmin = V[:, 2].min()
    ground = V[V[:, 2] < zmin + 0.005]
    for tag, bone, sel in (("foot_L", "FOOT_L", ground[:, 0] > 0), ("foot_R", "FOOT_R", ground[:, 0] < 0)):
        P = ground[sel]
        if len(P) and rig.data.bones.get(bone):
            rest_pts[tag] = (bone, np.array([(P[:, 0].min() + P[:, 0].max()) / 2,
                                             (P[:, 1].min() + P[:, 1].max()) / 2, zmin]))
        else:
            warn.append("%s ground contact not found" % tag)
    for w in warn:
        print("render_compare: WARNING %s" % w)

    # rest point -> bone-local offset (armature space)
    aw_inv = rig.matrix_world.inverted()
    local = {}
    for k, (bname, p) in rest_pts.items():
        b = rig.data.bones[bname]
        local[k] = (bname, b.matrix_local.inverted() @ (aw_inv @ Vector(p)))

    def bone_point(dg_rig, key):
        bname, off = local[key]
        pb = dg_rig.pose.bones[bname]
        return dg_rig.matrix_world @ (pb.matrix @ off)

    # ---- per-frame evaluation --------------------------------------------------------
    results = []
    for f in frames:
        scene.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        drig = rig.evaluated_get(dg)
        pb = drig.pose.bones
        W = drig.matrix_world
        pts = {}
        if "WEAPON" in pb:
            wb = pb["WEAPON"]
            tail = W @ wb.tail
            axis = (W @ wb.tail - W @ wb.head)
            axis = axis.normalized() if axis.length > 1e-9 else Vector((0, 0, 1))
            pts["weapon_tip"] = tail
            pts["club_head_center"] = tail - axis * CLUB_HEAD_BACK_OFF
            pts["club_head_center_blobfit"] = tail - axis * CLUB_HEAD_BLOBFIT
        if "HAND_R" in pb:
            pts["hand_R"] = W @ pb["HAND_R"].head
        if "HAND_L" in pb:
            pts["hand_L"] = W @ ((pb["HAND_L"].head + pb["HAND_L"].tail) / 2.0)
        if "HIPS" in pb:
            pts["hips"] = W @ pb["HIPS"].head
        for k in ("eye_L", "eye_R", "foot_L", "foot_R", "head_top"):
            if k in local:
                pts[k] = bone_point(drig, k)
        if "eye_L" in pts and "eye_R" in pts:
            pts["head"] = (pts["eye_L"] + pts["eye_R"]) / 2.0
        elif "HEAD" in pb:
            pts["head"] = W @ ((pb["HEAD"].head + pb["HEAD"].tail) / 2.0)

        rw = int(REF_W * a["scale"]); rh = int(REF_H * a["scale"])
        entry = {"frame": f, "points": {}}
        for k, v in pts.items():
            c = world_to_camera_view(scene, cam, v)
            nx, ny = c.x, 1.0 - c.y
            entry["points"][k] = {"px": [nx * rw, ny * rh], "norm": [nx, ny],
                                  "world": [v.x, v.y, v.z],
                                  "visible": bool(0.0 <= nx <= 1.0 and 0.0 <= ny <= 1.0 and c.z > 0)}
        if "hips" in entry["points"] and "head" in entry["points"]:
            hx, hy = entry["points"]["hips"]["norm"]
            ex, ey = entry["points"]["head"]["norm"]
            dx = (ex - hx) * rw
            dy = (hy - ey) * rh        # up is positive
            entry["chest_tilt_deg"] = math.degrees(math.atan2(dx, dy))
        results.append(entry)

        scene.render.filepath = os.path.join(out_dir, "render_f%04d.png" % f)
        bpy.ops.render.render(write_still=True)

    cam.data.show_background_images = prev_show_bg
    for o in hidden:
        o.hide_render = False

    cd = cam.data
    doc = {
        "convention": {
            "origin": "top-left",
            "norm": "x = px/width, y = px/height, both 0..1 (same as ref/metrics.json)",
            "sides": "screen-left = the character's RIGHT; keys are named for the CHARACTER's side",
            "chest_tilt_deg": "hips->eye-midpoint angle from screen vertical, + = top toward screen-right",
            "head": "eye-line midpoint, not the head's volumetric centre",
            "hips": "HIPS bone head (metrics.json hips_belt_centroid is the belt blob centroid, lower)",
            "club_head_center": "WEAPON tail minus %.3f m along the bone" % CLUB_HEAD_BACK_OFF,
            "club_head_center_blobfit":
                "WEAPON tail minus %.3f m along the bone; this one reproduces metrics.json "
                "club.head_center (0.87 px at frame 1)" % CLUB_HEAD_BLOBFIT,
            "hips_offset_vs_metrics_px":
                "HIPS bone head = metrics.json hips_belt_centroid + (-1.3, +20.2) px at frame 1",
            "weapon_tip_offset_vs_metrics_px":
                "our WEAPON tail is 18.4 px ABOVE metrics.json club.head_outer_tip at frame 1",
        },
        "blend": bpy.data.filepath,
        "resolution": [int(REF_W * a["scale"]), int(REF_H * a["scale"])],
        "reference_resolution": [REF_W, REF_H],
        "camera": {"name": cam.name, "type": cd.type,
                   "location": list(cam.location),
                   "rotation_euler_deg": [math.degrees(v) for v in cam.rotation_euler],
                   "lens": cd.lens, "ortho_scale": cd.ortho_scale,
                   "sensor_fit": cd.sensor_fit, "sensor_width": cd.sensor_width},
        "warnings": warn,
        "frames": results,
    }
    with open(json_path, "w") as fh:
        json.dump(doc, fh, indent=1)
    print("render_compare: rendered %d frames -> %s" % (len(frames), out_dir))
    print("render_compare: screen metrics -> %s" % json_path)

    if a["composite"]:
        import shutil, subprocess
        exe = a["python_exe"] or shutil.which("python") or shutil.which("python3")
        if not exe:
            print("render_compare: WARNING no system python found, composites skipped")
        else:
            cmd = [exe, os.path.abspath(__file__), "--composite",
                   "--out", out_dir, "--ref-dir", ref_dir,
                   "--frames", ",".join(str(f) for f in frames)]
            r = subprocess.run(cmd, capture_output=True, text=True)
            print((r.stdout or "").strip())
            if r.returncode != 0:
                print("render_compare: WARNING composite step failed:\n%s" % (r.stderr or "").strip())
    return 0


if __name__ == "__main__":
    if "--composite" in sys.argv:
        sys.exit(composite_main(sys.argv[1:]))
    else:
        try:
            import bpy  # noqa
        except ImportError:
            print("render_compare: run me under Blender, or with --composite")
            sys.exit(2)
        sys.exit(blender_main() or 0)
