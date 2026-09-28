"""goblib - shared helpers for the goblin rig Blender scripts (rig/scripts/*.py).

Scripts are run headless via bl.ps1 (``--factory-startup --background``), so the
script directory may not be on sys.path.  goblib stays a plain module; each
caller must do, before ``import goblib``::

    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import goblib

Evidence JSON schema (design doc d-23 section 7):
    {gate, checker, inputs:[{path, sha256}], criteria:[{id, measured, threshold, ok, note}],
     renders:[path]}
Paths are relative to the rig root with '/' separators.

Self test:  bl.ps1 -Script goblib.py -- --selftest
"""
import hashlib
import json
import math
import os
import re
import sys
import traceback
from pathlib import Path

import numpy as np

try:
    import bpy
    import mathutils
    from mathutils import Matrix, Vector, Quaternion
except ImportError:  # allows importing the pure helpers outside Blender
    bpy = None
    mathutils = None

# ---------------------------------------------------------------- paths
RIG = Path(__file__).resolve().parent.parent
DATA = RIG / "data"
INSPECT = RIG / "inspect"
WORK = RIG / "work"
EXPORT = RIG / "export"
SRC_FBX = RIG.parents[2] / "assets" / "source" / "goblin" / "Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx"


# ---------------------------------------------------------------- files / json
def sha256(path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_json(name: str) -> dict:
    with open(DATA / name, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(name: str, obj: dict) -> Path:
    DATA.mkdir(parents=True, exist_ok=True)
    p = DATA / name
    with open(p, "w", encoding="utf-8") as f:
        json.dump(_jsonable(obj), f, indent=1, ensure_ascii=False)
    return p


def _rel(path) -> str:
    """Path relative to RIG with '/' separators (absolute '/' path if on another drive)."""
    p = Path(path).resolve()
    try:
        r = os.path.relpath(p, RIG)
    except ValueError:
        r = str(p)
    return r.replace("\\", "/")


def _jsonable(v):
    """Convert numpy / mathutils / Path values into plain JSON-serializable python values."""
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else str(f)
    if v is None or isinstance(v, str):
        return v
    if isinstance(v, Path):
        return str(v).replace("\\", "/")
    if isinstance(v, np.ndarray):
        return _jsonable(v.tolist())
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if mathutils is not None and isinstance(v, mathutils.Matrix):
        return [_jsonable(list(row)) for row in v]
    if isinstance(v, (list, tuple, set)) or (
            mathutils is not None and isinstance(v, (mathutils.Vector, mathutils.Quaternion,
                                                     mathutils.Euler, mathutils.Color))):
        return [_jsonable(x) for x in v]
    return str(v)


# ---------------------------------------------------------------- evidence
class Evidence:
    """Collects gate evidence and writes INSPECT/<gate>/<checker>.json."""

    def __init__(self, gate: str, checker: str):
        self.gate = gate
        self.checker = checker
        self.inputs = []
        self.criteria = []
        self.renders = []

    def add_input(self, path) -> None:
        rel = _rel(path)
        if any(i["path"] == rel for i in self.inputs):
            return
        self.inputs.append({"path": rel, "sha256": sha256(path)})

    def add_stage_inputs(self, upto: str) -> None:
        """Add RIG/scripts/sNN*.py with NN <= upto's NN, goblib.py and DATA/*.json."""
        m = re.search(r"(\d+)", str(upto))
        if not m:
            raise ValueError(f"upto has no stage number: {upto!r}")
        limit = int(m.group(1))
        scripts = RIG / "scripts"
        for p in sorted(scripts.glob("s*.py")):
            mm = re.match(r"s(\d\d)", p.name[:3])
            if mm and int(mm.group(1)) <= limit:
                self.add_input(p)
        self.add_input(scripts / "goblib.py")
        for p in sorted(DATA.glob("*.json")):
            self.add_input(p)

    def criterion(self, id: str, measured, threshold, ok: bool, note: str = "") -> None:
        self.criteria.append({"id": id, "measured": _jsonable(measured),
                              "threshold": _jsonable(threshold), "ok": bool(ok),
                              "note": note})

    def render(self, path) -> None:
        rel = _rel(path)
        if rel not in self.renders:
            self.renders.append(rel)

    def write(self) -> Path:
        d = INSPECT / self.gate
        d.mkdir(parents=True, exist_ok=True)
        p = d / f"{self.checker}.json"
        doc = {"gate": self.gate, "checker": self.checker, "inputs": self.inputs,
               "criteria": self.criteria, "renders": self.renders}
        with open(p, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            print(f"[{self.gate}/{self.checker}] {c['id']} ok={c['ok']} "
                  f"measured={json.dumps(c['measured'], ensure_ascii=False)} "
                  f"threshold={json.dumps(c['threshold'], ensure_ascii=False)}"
                  + (f" note={c['note']}" if c["note"] else ""))
        print(f"[{self.gate}/{self.checker}] wrote {_rel(p)}")
        sys.stdout.flush()
        return p


# ---------------------------------------------------------------- math
def quat_angle_deg(q1, q2) -> float:
    """degrees(2*acos(min(1,|q1.q2|))); q = mathutils.Quaternion or (w,x,y,z)."""
    a = [float(x) for x in q1]
    b = [float(x) for x in q2]
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def bone_world(arm_obj, name) -> "Matrix":
    """World matrix of pose bone `name`: arm_obj.matrix_world @ pose_bone.matrix.

    Assumes the depsgraph is current; calling view_layer.update() (after pose/frame
    changes) is the caller's responsibility.
    """
    return arm_obj.matrix_world @ arm_obj.pose.bones[name].matrix


# ---------------------------------------------------------------- rendering
_VIEWS = ("front", "back", "side", "top", "three_quarter")


def _view_basis(view):
    """Return (d, right, up): view direction (camera looks along d) and image axes."""
    if view == "front":
        d, wup = Vector((0, 1, 0)), Vector((0, 0, 1))
    elif view == "back":
        d, wup = Vector((0, -1, 0)), Vector((0, 0, 1))
    elif view == "side":
        d, wup = Vector((-1, 0, 0)), Vector((0, 0, 1))
    elif view == "top":
        d, wup = Vector((0, 0, -1)), Vector((0, 1, 0))
    elif view == "three_quarter":
        az, el = math.radians(45.0), math.radians(15.0)
        # camera position direction: front (-Y) turned 45 deg toward -X, raised 15 deg
        pos = Vector((-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), math.sin(el)))
        d, wup = -pos, Vector((0, 0, 1))
    else:
        raise ValueError(f"unknown view {view!r}; expected one of {_VIEWS}")
    d = d.normalized()
    right = d.cross(wup).normalized()
    up = right.cross(d).normalized()
    return d, right, up


def _resolve_objs(objs):
    if isinstance(objs, (str, bytes)) or not hasattr(objs, "__iter__"):
        objs = [objs]
    out = []
    for o in objs:
        out.append(bpy.data.objects[o] if isinstance(o, str) else o)
    return out


def _world_bbox(objs):
    dg = bpy.context.evaluated_depsgraph_get()
    pts = []
    for o in objs:
        oe = o.evaluated_get(dg)
        mw = oe.matrix_world
        pts.extend(mw @ Vector(c) for c in oe.bound_box)
    if not pts:
        raise ValueError("ortho_render: no objects to frame")
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return mn, mx


def ortho_render(objs, view: str, out_png, res_h=1024, frame_bbox=None, wire=False,
                 colors=None) -> Path:
    """Workbench ortho render of `objs` (objects or names) into a transparent PNG.

    view: front (from -Y looking +Y), back (+Y -> -Y), side (+X -> -X), top (+Z down,
    -Y at image bottom), three_quarter (front turned 45 deg toward the character's
    right (-X) and 15 deg above).
    frame_bbox: world AABB ((minx,miny,minz),(maxx,maxy,maxz)) framed as given; None
    frames the evaluated world bbox of objs, each image side padded by 5% of the
    larger projected extent.  Height = res_h px, width follows the framing aspect.
    colors: {obj_name: (r,g,b,a)} -> color type OBJECT (obj.color, restored after);
    None -> color type MATERIAL.  wire: black wireframe overlay (Workbench final
    renders ignore show_wire, so temporary Wireframe-modifier copies of the evaluated
    meshes, ~1.5 px thick, are drawn instead).
    Renders in a temporary scene (same frame as the current scene) that is deleted
    afterwards; the caller's scene, camera and settings are untouched.
    """
    objs = _resolve_objs(objs)
    out_png = Path(out_png).resolve()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    d, right, up = _view_basis(view)

    if frame_bbox is None:
        mn, mx = _world_bbox(objs)
        pad = True
    else:
        mn, mx = Vector(frame_bbox[0]), Vector(frame_bbox[1])
        pad = False
    corners = [Vector((x, y, z)) for x in (mn.x, mx.x) for y in (mn.y, mx.y) for z in (mn.z, mx.z)]
    us = [c.dot(right) for c in corners]
    vs = [c.dot(up) for c in corners]
    ws = [c.dot(d) for c in corners]
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    if pad:
        m = 0.05 * max(u1 - u0, v1 - v0)
        u0, u1, v0, v1 = u0 - m, u1 + m, v0 - m, v1 + m
    w = max(u1 - u0, 1e-6)
    h = max(v1 - v0, 1e-6)
    depth = max(ws) - min(ws)
    res_h = int(res_h)
    res_w = max(1, int(round(res_h * w / h)))
    center_plane = right * ((u0 + u1) / 2) + up * ((v0 + v1) / 2)
    dist = depth + max(w, h) + 1.0
    cam_loc = center_plane + d * (min(ws) - dist)  # in front of the nearest corner

    src_scene = bpy.context.scene
    sc = bpy.data.scenes.new("_goblib_render")
    cam_data = cam_obj = wire_mat = None
    temp_objs, temp_meshes = [], []
    saved_color = {}
    try:
        sc.frame_current = src_scene.frame_current
        for o in objs:
            sc.collection.objects.link(o)

        cam_data = bpy.data.cameras.new("_goblib_cam")
        cam_data.type = "ORTHO"
        cam_data.sensor_fit = "VERTICAL"
        cam_data.ortho_scale = h
        cam_data.clip_start = 0.001
        cam_data.clip_end = dist + depth + 10.0
        cam_obj = bpy.data.objects.new("_goblib_cam", cam_data)
        rot = Matrix((right, up, -d)).transposed()  # columns: local X, Y, Z
        cam_obj.matrix_world = Matrix.Translation(cam_loc) @ rot.to_4x4()
        sc.collection.objects.link(cam_obj)
        sc.camera = cam_obj

        r = sc.render
        r.engine = "BLENDER_WORKBENCH"
        r.resolution_x = res_w
        r.resolution_y = res_h
        r.resolution_percentage = 100
        r.pixel_aspect_x = r.pixel_aspect_y = 1.0
        r.film_transparent = True
        r.use_file_extension = False
        r.image_settings.file_format = "PNG"
        r.image_settings.color_mode = "RGBA"
        r.image_settings.color_depth = "8"
        r.filepath = str(out_png)
        sc.view_settings.view_transform = "Standard"
        sc.view_settings.look = "None"
        sh = sc.display.shading
        sh.light = "STUDIO"
        sh.color_type = "OBJECT" if colors else "MATERIAL"
        sh.show_xray = False

        if colors:
            for name, col in colors.items():
                o = bpy.data.objects[name]
                saved_color[name] = tuple(o.color)
                o.color = tuple(col)

        if wire:
            dg = bpy.context.evaluated_depsgraph_get()
            px = h / res_h
            wire_mat = bpy.data.materials.new("_goblib_wire")
            wire_mat.diffuse_color = (0.0, 0.0, 0.0, 1.0)
            for o in objs:
                if o.type != "MESH":
                    continue
                oe = o.evaluated_get(dg)
                me = bpy.data.meshes.new_from_object(oe, preserve_all_data_layers=False,
                                                     depsgraph=dg)
                me.materials.clear()
                me.materials.append(wire_mat)
                temp_meshes.append(me)
                wo = bpy.data.objects.new("_goblib_wire_" + o.name, me)
                wo.matrix_world = oe.matrix_world.copy()
                wo.color = (0.0, 0.0, 0.0, 1.0)
                sc.collection.objects.link(wo)
                temp_objs.append(wo)
                sx = max(wo.matrix_world.to_scale()) or 1.0
                mod = wo.modifiers.new("wire", "WIREFRAME")
                mod.thickness = 1.5 * px / sx
                mod.offset = 0.0
                mod.use_boundary = True
                mod.use_replace = True
                mod.use_even_offset = True

        bpy.ops.render.render(write_still=True, scene=sc.name)
    finally:
        for name, col in saved_color.items():
            bpy.data.objects[name].color = col
        for wo in temp_objs:
            bpy.data.objects.remove(wo, do_unlink=True)
        for me in temp_meshes:
            bpy.data.meshes.remove(me)
        if wire_mat is not None:
            bpy.data.materials.remove(wire_mat)
        if cam_obj is not None:
            bpy.data.objects.remove(cam_obj, do_unlink=True)
        if cam_data is not None:
            bpy.data.cameras.remove(cam_data)
        bpy.data.scenes.remove(sc)
    if not out_png.exists():
        raise RuntimeError(f"ortho_render: output not written: {out_png}")
    return out_png


# ---------------------------------------------------------------- masks
def silhouette_mask(png) -> "np.ndarray[bool]":
    """PNG alpha > 0.5 -> True.  Row 0 is the image top."""
    img = bpy.data.images.load(str(Path(png).resolve()), check_existing=False)
    try:
        w, h = img.size
        buf = np.empty(w * h * img.channels, dtype=np.float32)
        img.pixels.foreach_get(buf)
        px = buf.reshape(h, w, img.channels)
        alpha = px[:, :, 3] if img.channels == 4 else np.ones((h, w), dtype=np.float32)
    finally:
        bpy.data.images.remove(img)
    return np.ascontiguousarray((alpha > 0.5)[::-1])


def _contour(mask):
    """Mask pixels with at least one 4-neighbour outside the mask (image border = outside)."""
    m = np.asarray(mask, dtype=bool)
    p = np.pad(m, 1, constant_values=False)
    inner = p[:-2, 1:-1] & p[2:, 1:-1] & p[1:-1, :-2] & p[1:-1, 2:]
    return np.argwhere(m & ~inner).astype(np.float64)


def _directed_max_min(a, b, max_elems=4_000_000):
    step = max(1, max_elems // max(1, len(b)))
    worst = 0.0
    for i in range(0, len(a), step):
        ca = a[i:i + step]
        d2 = ((ca[:, None, :] - b[None, :, :]) ** 2).sum(axis=2)
        worst = max(worst, float(d2.min(axis=1).max()))
    return math.sqrt(worst)


def contour_max_dev_px(mask_a, mask_b) -> float:
    """Symmetric Hausdorff distance (px) between the contour pixels of two masks.

    Masks must have the same shape (ValueError otherwise).  Both empty -> 0.0,
    exactly one empty -> inf.  numpy only, processed in chunks.
    """
    a = np.asarray(mask_a, dtype=bool)
    b = np.asarray(mask_b, dtype=bool)
    if a.shape != b.shape:
        raise ValueError(f"mask shapes differ: {a.shape} vs {b.shape}")
    ca, cb = _contour(a), _contour(b)
    if len(ca) == 0 and len(cb) == 0:
        return 0.0
    if len(ca) == 0 or len(cb) == 0:
        return float("inf")
    return max(_directed_max_min(ca, cb), _directed_max_min(cb, ca))


# ---------------------------------------------------------------- entry
def run_main(fn) -> None:
    """Run fn(); on exception print the traceback and sys.exit(2)."""
    try:
        fn()
    except Exception:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        sys.exit(2)
    sys.stdout.flush()


def _selftest():
    cube = bpy.data.objects.get("Cube")
    if cube is None:
        me = bpy.data.meshes.new("Cube")
        import bmesh
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=2.0)
        bm.to_mesh(me)
        bm.free()
        cube = bpy.data.objects.new("Cube", me)
        bpy.context.scene.collection.objects.link(cube)
    png = ortho_render([cube], "front", INSPECT / "_selftest" / "cube_front.png")
    print(f"selftest render: {_rel(png)}")
    m = silhouette_mask(png)
    print(f"selftest mask shape={m.shape} filled={int(m.sum())}")
    d0 = contour_max_dev_px(m, m)
    print(f"selftest contour_max_dev_px(m, m) = {d0}")
    sh = np.zeros_like(m)
    sh[:, 2:] = m[:, :-2]
    d2 = contour_max_dev_px(m, sh)
    print(f"selftest contour_max_dev_px(m, shift2px) = {d2}")
    qi = Quaternion()
    qz = Quaternion((0.0, 0.0, 1.0), math.radians(90.0))
    a0 = quat_angle_deg(qi, qi)
    a90 = quat_angle_deg(qi, (qz.w, qz.x, qz.y, qz.z))
    print(f"selftest quat_angle_deg(I, I) = {a0}")
    print(f"selftest quat_angle_deg(I, Z90) = {a90}")

    ev = Evidence("_selftest", "goblib")
    ev.add_input(RIG / "scripts" / "goblib.py")
    ev.criterion("mask_nonempty", int(m.sum()), "> 0", m.sum() > 0)
    ev.criterion("contour_self", d0, 0.0, abs(d0 - 0.0) < 1e-9)
    ev.criterion("contour_shift2px", d2, 2.0, abs(d2 - 2.0) < 1e-9)
    ev.criterion("quat_identity", a0, 0.0, abs(a0) < 1e-4)
    ev.criterion("quat_z90", a90, 90.0, abs(a90 - 90.0) < 1e-4)
    ev.render(png)
    ev.write()


if __name__ == "__main__":
    _argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if "--selftest" in _argv:
        run_main(_selftest)
