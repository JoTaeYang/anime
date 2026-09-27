"""s09_blender_ref - Blender reference data for the G9 Unity comparison (plan T39, data contract data/g9_contract.md).

Run:    bl.ps1 -Script s09_blender_ref.py -Blend export/stage_rigtest.blend
Input:  export/stage_rigtest.blend (GOB_export 24 bones baked action 'rigtest', goblin_mesh, goblin_club bone-parented
        to weapon_socket_r), data/rigtest_manifest.json (segments), data/export_preset.json (axis_mapping),
        data/canonical_skeleton.json (canonical_bind)
Output:
  data/compare_cams.json            cameras 'front' / 'three_quarter' (ortho, 1024 x 1024), Blender world matrix and the
                                    Unity camera (position / rotation / orthographicSize, axis_mapping M applied),
                                    render frames (3 of the G9.4 sample set)
  inspect/G9/blender_dump.json      Blender world coordinates (Z-up, m), quaternions [x, y, z, w] + rotm 3x3 (rows):
                                    rest, club_rest, samples (integer frames), subsamples (f + 0.25 / 0.5 / 0.75,
                                    evaluated with scene.frame_set(int(f), subframe=frac)), canonical_bind
  inspect/G9/blender_<cam>_<frame>.png   Workbench, 1024 px, RGBA with transparent background (film_transparent, alpha =
                                    silhouette; world color white), single flat-ish color
  inspect/G9/blender_lit_<cam>_<frame>.png  EEVEE, one sun per camera (Unity extra.render_lit to_light, else fixed; compare_cams lit_light) + ambient 0.3,
                                    gray 0.8 / roughness 0.8, custom normals, composited over white (G9.7 [U])
  blender_dump.json next_frames     integer frame f + 1 for every subsampled f (G9.8 linear / step references)
Nothing is saved to the blend.  Exit 2 on error (goblib.run_main).
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import goblib  # noqa: E402,E702

import json  # noqa: E402
import math  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

ARM = "GOB_export"
MESH = "goblin_mesh"
CLUB = "goblin_club"
ACTION = "rigtest"
FPS = 24
FRAME_FIRST = 1
FRAME_LAST = 583
SUB_FRACS = (0.25, 0.5, 0.75)
CAMS = ("front", "three_quarter")
RES = 1024
PAD = 0.05                                   # framing margin per side (fraction of the larger extent)
CAM_BACK = 2.0                               # camera distance in front of the nearest framed point (m)
SINGLE_COLOR = (0.50, 0.55, 0.45)            # Workbench single color (kept well below the white background)
RENDER_FRAMES = (58, 184, 431)
RENDER_NOTES = {
    58: "cog mid: COG down / tilted, knees bent (thigh 64 / 42 deg)",
    184: "arm_fk mid: both arms raised forward, club in front of the body (clearest pose)",
    431: "weapon_world mid: club moved in the world, right arm moved away",
}
OUT_DIR = goblib.INSPECT / "G9"
DUMP = OUT_DIR / "blender_dump.json"
STAGE = goblib.EXPORT / "stage_rigtest.blend"


def axis_M():
    pre = goblib.load_json("export_preset.json")
    return Matrix(pre["axis_mapping"]["blender_to_unity"])


def sample_frames(manifest):
    out = set()
    for s in manifest["segments"]:
        a, b = int(s["start"]), int(s["end"])
        out.update((a, (a + b) // 2, b))
    return sorted(out)


def sub_frames(samples, last):
    return [(f, fr) for f in samples if f + 1 <= last for fr in SUB_FRACS]


def xyzw(q):
    return [q.x, q.y, q.z, q.w]


def xf(W):
    loc, rot, sca = W.decompose()
    rot.normalize()
    return {"pos": list(loc), "rot": xyzw(rot), "rotm": [list(r) for r in rot.to_matrix()], "scale": list(sca)}


def state(arm, club, names):
    return {"bones": {n: xf(goblib.bone_world(arm, n)) for n in names}, "club": xf(club.matrix_world.copy())}


def eval_points(objs):
    """World vertex positions of the evaluated (deformed) meshes."""
    dg = bpy.context.evaluated_depsgraph_get()
    pts = []
    for o in objs:
        oe = o.evaluated_get(dg)
        me = oe.to_mesh()
        try:
            co = np.empty(len(me.vertices) * 3, dtype=np.float64)
            me.vertices.foreach_get("co", co)
            co = co.reshape(-1, 3)
        finally:
            oe.to_mesh_clear()
        mw = np.array(oe.matrix_world, dtype=np.float64)
        pts.append(co @ mw[:3, :3].T + mw[:3, 3])
    return np.concatenate(pts, axis=0)


def look_rotation_xyzw(fwd, up):
    """Unity Quaternion.LookRotation(fwd, up) as [x, y, z, w] (columns x = up x fwd, y = fwd x x, z = fwd)."""
    z = fwd.normalized()
    x = up.cross(z).normalized()
    y = z.cross(x).normalized()
    R = Matrix((x, y, z)).transposed()
    return xyzw(R.to_quaternion()), R.determinant()


def make_cameras(sc, pts_by_frame, M):
    pts = np.concatenate(pts_by_frame, axis=0)
    cams = {}
    for view in CAMS:
        d, right, up = goblib._view_basis(view)
        dn, rn, un = (np.array(v, dtype=np.float64) for v in (d, right, up))
        u, v, w = pts @ rn, pts @ un, pts @ dn
        size = max(u.max() - u.min(), v.max() - v.min()) * (1.0 + 2.0 * PAD)
        uc, vc = 0.5 * (u.max() + u.min()), 0.5 * (v.max() + v.min())
        loc = right * uc + up * vc + d * (w.min() - CAM_BACK)
        depth = float(w.max() - w.min())
        clip_start, clip_end = 0.01, CAM_BACK + depth + 10.0
        rot = Matrix((right, up, -d)).transposed()
        mw = Matrix.Translation(loc) @ rot.to_4x4()

        cd = bpy.data.cameras.new(f"_g9cam_{view}")
        cd.type = "ORTHO"
        cd.sensor_fit = "VERTICAL"
        cd.ortho_scale = size
        cd.clip_start, cd.clip_end = clip_start, clip_end
        co = bpy.data.objects.new(f"_g9cam_{view}", cd)
        co.matrix_world = mw
        sc.collection.objects.link(co)

        fwd_u, up_u, right_u = M @ d, M @ up, M @ right
        q_u, det_u = look_rotation_xyzw(fwd_u, up_u)
        cams[view] = {
            "obj": co,
            "json": {
                "type": "orthographic",
                "blender": {"matrix_world": [list(r) for r in mw], "location": list(loc), "view_dir": list(d),
                            "up": list(up), "right": list(right), "ortho_scale": size, "sensor_fit": "VERTICAL",
                            "clip_start": clip_start, "clip_end": clip_end},
                "unity": {"position": list(M @ loc), "rotation": q_u, "forward": list(fwd_u), "up": list(up_u),
                          "right": list(right_u), "orthographic": True, "orthographicSize": size / 2.0,
                          "aspect": 1.0, "nearClipPlane": clip_start, "farClipPlane": clip_end,
                          "rotation_det": det_u},
            },
        }
    return cams


def setup_render(sc):
    r = sc.render
    r.engine = "BLENDER_WORKBENCH"
    r.resolution_x = r.resolution_y = RES
    r.resolution_percentage = 100
    r.pixel_aspect_x = r.pixel_aspect_y = 1.0
    r.film_transparent = True
    r.use_file_extension = False
    r.image_settings.file_format = "PNG"
    r.image_settings.color_mode = "RGBA"
    r.image_settings.color_depth = "8"
    sc.view_settings.view_transform = "Standard"
    sc.view_settings.look = "None"
    sc.view_settings.exposure = 0.0
    sc.view_settings.gamma = 1.0
    if sc.world is None:
        sc.world = bpy.data.worlds.new("_g9_world")
    sc.world.color = (1.0, 1.0, 1.0)
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "SINGLE"
    sh.single_color = SINGLE_COLOR
    sh.show_specular_highlight = False
    sh.show_cavity = False
    sh.show_object_outline = False
    sh.show_shadows = False
    sh.show_xray = False


# ---------------------------------------------------------------- lit renders (G9.7 [U] shading / custom normals)
LIT_ENGINE = "BLENDER_EEVEE"
LIT_ALBEDO = 0.8                              # Unity Standard gray 0.8
LIT_ROUGHNESS = 0.8                           # Unity smoothness 0.2
LIT_AMBIENT = 0.3                             # uniform ambient (world color), Unity ambient 0.3
LIT_TO_LIGHT_B = Vector((-0.5, -1.0, 1.0)).normalized()   # Blender: toward the light = front (-Y), up (+Z), image left (-X)
LIT_SUN_STRENGTH = math.pi                    # Blender sun W/m2: pi -> albedo * NdotL (Unity intensity 1, no 1/pi)
LIT_SAMPLES = 16
LIT_VIEW = "Raw"   # Unity project m_ActiveColorSpace 0 (Gamma): shading math on the raw 0..1 values, written as is


UNITY_REPORT = goblib.RIG.parent.parent.parent / "unity" / "AvatarCheck" / "goblin_report.json"


def lit_dirs(M):
    """Blender to_light per camera: Unity report extra.render_lit.per_camera[cam].to_light (Unity world) mapped
    with M^T, else the fixed LIT_TO_LIGHT_B."""
    out = {v: (LIT_TO_LIGHT_B.copy(), "fixed by s09 (front, up, front-camera image left)") for v in CAMS}
    try:
        with open(UNITY_REPORT, "r", encoding="utf-8") as fh:
            pc = ((json.load(fh).get("extra") or {}).get("render_lit") or {}).get("per_camera") or {}
    except (OSError, ValueError):
        return out
    for v in CAMS:
        tl = (pc.get(v) or {}).get("to_light")
        if tl and len(tl) == 3:
            out[v] = ((M.transposed() @ Vector(tl)).normalized(),
                      "goblin_report extra.render_lit.per_camera.%s.to_light (Unity world) -> M^T" % v)
    return out


def lit_light_json(M, dirs):
    per = {}
    for v, (tb, src) in dirs.items():
        fwd_u = M @ -tb
        q_u, _det = look_rotation_xyzw(fwd_u, Vector((0.0, 1.0, 0.0)))
        per[v] = {"source": src, "blender_to_light": list(tb), "blender_ray_dir": list(-tb),
                  "unity_forward": list(fwd_u), "unity_to_light": list(M @ tb), "unity_rotation": q_u}
    return {
        "type": "directional",
        "per_camera": per,
        "blender": {"engine": LIT_ENGINE, "sun_strength": LIT_SUN_STRENGTH, "shadow": False, "world_ambient_rgb": [LIT_AMBIENT] * 3,
                    "material": {"base_color": [LIT_ALBEDO] * 3, "roughness": LIT_ROUGHNESS},
                    "normals": "mesh custom normals (goblin_mesh), evaluated deformed mesh",
                    "background": "film_transparent, composited over white", "samples": LIT_SAMPLES,
                    "view_transform": LIT_VIEW,
                    "color_space_note": "Unity project Gamma color space -> Blender Raw view transform (no sRGB encode)"},
        "unity": {"intensity": 1.0,
                  "color": [1.0, 1.0, 1.0], "shadows": "None", "ambient_rgb": [LIT_AMBIENT] * 3,
                  "material": {"shader": "Standard", "albedo": [LIT_ALBEDO] * 3, "smoothness": 1.0 - LIT_ROUGHNESS},
                  "background_rgb": [1.0, 1.0, 1.0]},
    }


def setup_lit(sc, objs):
    r = sc.render
    r.engine = LIT_ENGINE
    if hasattr(sc, "eevee") and hasattr(sc.eevee, "taa_render_samples"):
        sc.eevee.taa_render_samples = LIT_SAMPLES
    r.film_transparent = True
    r.image_settings.color_mode = "RGBA"
    sc.view_settings.view_transform = LIT_VIEW
    mat = bpy.data.materials.new("_g9_lit")
    try:
        mat.use_nodes = True
    except (AttributeError, TypeError):
        pass
    bsdf = next((n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is None:
        raise RuntimeError("lit material has no Principled BSDF node")
    bsdf.inputs["Base Color"].default_value = (LIT_ALBEDO, LIT_ALBEDO, LIT_ALBEDO, 1.0)
    bsdf.inputs["Roughness"].default_value = LIT_ROUGHNESS
    for o in objs:
        o.data.materials.clear()
        o.data.materials.append(mat)
    w = bpy.data.worlds.new("_g9_lit_world")
    try:
        w.use_nodes = True
    except (AttributeError, TypeError):
        pass
    bg = next((n for n in w.node_tree.nodes if n.type == "BACKGROUND"), None) if w.node_tree else None
    if bg is None:
        raise RuntimeError("lit world has no Background node")
    bg.inputs["Color"].default_value = (LIT_AMBIENT, LIT_AMBIENT, LIT_AMBIENT, 1.0)
    bg.inputs["Strength"].default_value = 1.0
    w.color = (LIT_AMBIENT,) * 3
    sc.world = w
    ld = bpy.data.lights.new("_g9_sun", type="SUN")
    ld.energy = LIT_SUN_STRENGTH
    ld.use_shadow = False
    lo = bpy.data.objects.new("_g9_sun", ld)
    lo.rotation_mode = "QUATERNION"
    lo.rotation_quaternion = (-LIT_TO_LIGHT_B).to_track_quat("-Z", "Y")
    sc.collection.objects.link(lo)
    return lo


def composite_over_white(png):
    img = bpy.data.images.load(str(png), check_existing=False)
    try:
        w, h = img.size
        ch = img.channels
        buf = np.empty(w * h * ch, dtype=np.float32)
        img.pixels.foreach_get(buf)
    finally:
        bpy.data.images.remove(img)
    px = buf.reshape(h, w, ch)
    if ch == 4:
        a = px[:, :, 3:4]
        px = np.concatenate([px[:, :, :3] * a + (1.0 - a), np.ones((h, w, 1), dtype=np.float32)], axis=2)
    out = bpy.data.images.new("_g9_lit_out", w, h, alpha=False)
    try:
        out.pixels.foreach_set(np.ascontiguousarray(px).ravel())
        out.filepath_raw = str(png)
        out.file_format = "PNG"
        out.save()
    finally:
        bpy.data.images.remove(out)


def main():
    sc = bpy.context.scene
    arm = bpy.data.objects.get(ARM)
    mesh = bpy.data.objects.get(MESH)
    club = bpy.data.objects.get(CLUB)
    if arm is None or mesh is None or club is None:
        raise RuntimeError(f"stage objects missing: {ARM}={arm} {MESH}={mesh} {CLUB}={club}")
    act = arm.animation_data.action if arm.animation_data else None
    if act is None or act.name != ACTION:
        raise RuntimeError(f"{ARM} action is {act.name if act else None!r}, expected {ACTION!r}")
    canon = goblib.load_json("canonical_skeleton.json")
    names = [b["name"] for b in canon["bones"]]
    parent = {b["name"]: b["parent"] for b in canon["bones"]}
    if sorted(names) != sorted(pb.name for pb in arm.pose.bones):
        raise RuntimeError("GOB_export bones differ from canonical_skeleton.json")
    manifest = goblib.load_json("rigtest_manifest.json")
    M = axis_M()
    samples = sample_frames(manifest)
    subs = sub_frames(samples, FRAME_LAST)
    for f in RENDER_FRAMES:
        if f not in samples:
            raise RuntimeError(f"render frame {f} not in the G9.4 sample set {samples}")
    vl = bpy.context.view_layer

    # rest (pose_position REST)
    arm.data.pose_position = "REST"
    vl.update()
    rest_w = {n: goblib.bone_world(arm, n).copy() for n in names}
    rest = {}
    for n in names:
        e = xf(rest_w[n])
        loc = rest_w[parent[n]].inverted() @ rest_w[n] if parent[n] else rest_w[n]
        ll, lr, _ls = loc.decompose()
        e["localPos"], e["localRot"] = list(ll), xyzw(lr.normalized())
        rest[n] = e
    club_rest = xf(club.matrix_world.copy())
    arm.data.pose_position = "POSE"
    vl.update()

    out_samples = []
    for f in samples:
        sc.frame_set(int(f), subframe=0.0)
        st = state(arm, club, names)
        out_samples.append({"frame": float(f), "time_s": (f - FRAME_FIRST) / FPS, **st})
    out_next = []
    for f in sorted({f for f, _fr in subs}):
        sc.frame_set(int(f) + 1, subframe=0.0)
        st = state(arm, club, names)
        out_next.append({"frame": float(f + 1), "time_s": (f + 1 - FRAME_FIRST) / FPS, **st})
    out_subs = []
    for f, fr in subs:
        sc.frame_set(int(f), subframe=fr)
        st = state(arm, club, names)
        out_subs.append({"frame": f + fr, "time_s": (f + fr - FRAME_FIRST) / FPS,
                         "frame_current_final": sc.frame_current_final, **st})

    # cameras (fixed per view, framing the union of the 3 render frames) + renders
    pts_by_frame = []
    for f in RENDER_FRAMES:
        sc.frame_set(int(f), subframe=0.0)
        pts_by_frame.append(eval_points([mesh, club]))
    cams = make_cameras(sc, pts_by_frame, M)
    setup_render(sc)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    renders = []
    for view in CAMS:
        sc.camera = cams[view]["obj"]
        for f in RENDER_FRAMES:
            sc.frame_set(int(f), subframe=0.0)
            out = OUT_DIR / f"blender_{view}_{f}.png"
            sc.render.filepath = str(out)
            bpy.ops.render.render(write_still=True)
            if not out.exists():
                raise RuntimeError(f"render not written: {out}")
            renders.append(goblib._rel(out))
            print(f"[s09] render {goblib._rel(out)}")
    sun = setup_lit(sc, [mesh, club])
    dirs = lit_dirs(M)
    for view in CAMS:
        sc.camera = cams[view]["obj"]
        sun.rotation_quaternion = (-dirs[view][0]).to_track_quat("-Z", "Y")
        for f in RENDER_FRAMES:
            sc.frame_set(int(f), subframe=0.0)
            out = OUT_DIR / f"blender_lit_{view}_{f}.png"
            sc.render.filepath = str(out)
            bpy.ops.render.render(write_still=True)
            if not out.exists():
                raise RuntimeError(f"render not written: {out}")
            composite_over_white(out)
            renders.append(goblib._rel(out))
            print(f"[s09] render {goblib._rel(out)}")

    cams_json = {
        "_doc": ("G9 compare cameras (data/g9_contract.md). One fixed orthographic camera per view framing the union of "
                 "goblin_mesh + goblin_club over the render frames. Blender: camera looks along its local -Z, up = local "
                 "+Y. Unity: position = M @ blender location, forward = M @ view_dir, up = M @ up, rotation = "
                 "Quaternion.LookRotation(forward, up) as [x, y, z, w], orthographicSize = ortho_scale / 2, square "
                 "aspect. M = export_preset axis_mapping.blender_to_unity. Unity time t = (frame - 1) / 24 s."),
        "generated_by": "scripts/s09_blender_ref.py",
        "source_blend": goblib._rel(STAGE),
        "axis_mapping_blender_to_unity": [list(r) for r in M],
        "resolution": [RES, RES],
        "res_h": RES,
        "aspect": 1.0,
        "background_rgb": [1.0, 1.0, 1.0],
        "background_alpha": 0.0,
        "frames": list(RENDER_FRAMES),
        "frame_notes": {str(f): RENDER_NOTES[f] for f in RENDER_FRAMES},
        "time_s": {str(f): (f - FRAME_FIRST) / FPS for f in RENDER_FRAMES},
        "png": {"blender": "rig/inspect/G9/blender_<cam>_<frame>.png",
                "unity": "rig/inspect/G9/unity_<cam>_<frame>.png",
                "blender_lit": "rig/inspect/G9/blender_lit_<cam>_<frame>.png",
                "unity_lit": "rig/inspect/G9/unity_lit_<cam>_<frame>.png",
                "frame_format": "integer, no padding (e.g. unity_front_184.png)"},
        "cameras": {v: cams[v]["json"] for v in CAMS},
        "lit_light": lit_light_json(M, dirs),
    }
    p_cams = goblib.save_json("compare_cams.json", cams_json)
    print(f"[s09] wrote {goblib._rel(p_cams)}")

    dump = {
        "meta": {
            "generated_by": "scripts/s09_blender_ref.py",
            "source_blend": goblib._rel(STAGE),
            "source_blend_sha256": goblib.sha256(bpy.data.filepath),
            "blender_version": bpy.app.version_string,
            "armature": ARM, "club": CLUB, "action": ACTION,
            "action_frame_range": list(act.frame_range),
            "fps": FPS,
            "frame_to_time": "t = (frame - 1) / 24 s",
            "coordinates": "Blender world (Z-up, right-handed, m); rot = [x, y, z, w]; rotm = 3x3 rows; scale from "
                           "matrix decompose",
            "bone_world": "GOB_export.matrix_world @ pose_bone.matrix (head position = pos)",
            "club_world": "goblin_club.matrix_world (bone parent weapon_socket_r)",
            "rest": "pose_position REST; localPos / localRot = parent rest world^-1 @ rest world",
            "subframes": "scene.frame_set(int(f), subframe=frac), frac in (0.25, 0.5, 0.75), f + 1 <= 583",
            "sample_frames": samples,
            "subsample_frames": [f + fr for f, fr in subs],
            "render_frames": list(RENDER_FRAMES),
        },
        "bones": names,
        "rest": rest,
        "club_rest": club_rest,
        "samples": out_samples,
        "subsamples": out_subs,
        "next_frames": out_next,
        "canonical_bind": {b["name"]: b["rest_matrix"] for b in canon["bones"]},
        "renders": renders,
    }
    with open(DUMP, "w", encoding="utf-8") as fh:
        json.dump(goblib._jsonable(dump), fh, indent=1)
    print(f"[s09] wrote {goblib._rel(DUMP)}: {len(names)} bones, samples {len(out_samples)}, "
          f"subsamples {len(out_subs)}, renders {len(renders)}")
    sys.stdout.flush()


if __name__ == "__main__":
    goblib.run_main(main)
