"""p23_skin.py - T230 (P2.4): final skin of the player (spec d-02 §2 P2.4).

Input : work/player/rig/pl_r02_skeleton.blend (PL_mesh, PL_rig; opened as the main file, not saved over)
        work/player/rig/data/parts.json, work/player/rig/data/retopo_loops.json (p20)
Output: work/player/rig/pl_r03_skinned.blend (save copy): PL_mesh with one vertex group per weighted deform bone and
            one Armature modifier (PL_rig, vertex groups only, no preserve volume); bones unchanged
        work/player/inspect/P2b/*.png: bone overlay front / side, weight maps (arm, leg, tunic / skirt, shoulder)

CLI:  blender --background --factory-startup work/player/rig/pl_r02_skeleton.blend --python p23_skin.py

Weights (adapted from work/goblin_swing/rig/scripts/s05_skin.py: per-joint axial profiles along the tube axis, the
hinge elbow / knee also by the angle to the bend-inner side, wrist / ankle tube ends 100 % on the rigid bone, limit
MAX_INF, normalise, rest-deformation self check; and from p21_temprig.py / p10_crude_rig.py for the tunic):
- rigid parts 100 % on one bone (RIGID): head, eyes -> Head; scarf, scarf_tail -> Spine_02; belt, pouch -> Pelvis;
  sleeves -> Arm_x; fists -> Hand_x; cuffs, shoes -> Foot_x.
- arm_x tube, axis a = unit(Hand head - Arm head) (straight tube), s from the Forearm / Hand heads:
  Hand = 1 for s_wrist >= -RIGID_EPS (wrist_x_1 .. end + cap, the fist), else WRIST(s_wrist);
  Forearm share = ELBOW(s_elbow, ci) (smoothstep ramp whose centre / half width depend on ci = cos of the angle
  to the bend-inner side -Y; values from a scratch sweep with the P2.2 tube-ratio method, see ELBOW / KNEE);
  Hand = hnd, Forearm = (1 - hnd) * e, Arm = (1 - hnd) * (1 - e).  No twist bones (the P2.2 wrist twist 60 deg
  ratio was 0.836 with auto weights; the tree stays the P1 v0 tree).
- leg_x tube, axis a = unit(Foot head - Thigh head): Foot = 1 for s_ankle >= -RIGID_EPS (inside cuff / shoe), else
  ANKLE(s_ankle); Calf share = KNEE(s_knee, ci) (bend-inner side +Y); Thigh = the rest (the hip end is hidden
  inside the tunic, so no Pelvis share on the tube).
- tunic (outer wall; every inner-wall vertex copies its paired outer vertex; ceiling ring + interior Pelvis 100 %):
  z >= belt top: p10 rule (Spine_01 / Spine_02 smoothstep about the Spine_02 head, Pelvis fading out over
  PELVIS_FADE above the belt top), then Neck share = smooth01((z - NECK_Z[0]) / (NECK_Z[1] - NECK_Z[0])) (tunic
  neck band up to the hidden neck rim), then Clavicle_x share = CLAV_MAX * smooth01(1 - |p - Arm_x head| /
  CLAV_R) on the tunic shoulder area next to the sleeve roots; belt bottom .. belt top: Pelvis 100 %;
  below the belt bottom (skirt rings): height table SKIRT_H over z (belt_bot Pelvis, skirt_3 Skirt_*_01, skirt_2
  01/02 half, skirt_1 and hem Skirt_*_02), azimuth blend between the two neighbouring skirt chains (linear in the
  vertex azimuth about the tunic axis; the 8 chain azimuths are every 4th column); T232: vertices of the hem /
  skirt_1..3 rings (and their inner partners) get Pelvis 0, renormalised onto their Skirt_* weights.
- every vertex <= MAX_INF influences (largest kept), weights < EPS_W dropped, normalised.
"""
import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
IN_BLEND = RIG / "pl_r02_skeleton.blend"
OUT_BLEND = RIG / "pl_r03_skinned.blend"
PARTS_JSON = RIG / "data" / "parts.json"
LOOPS_JSON = RIG / "data" / "retopo_loops.json"
INSPECT = REPO / "work" / "player" / "inspect" / "P2b"
MESH = "PL_mesh"
ARM = "PL_rig"
SIDES = ("L", "R")
MAX_INF = 4
EPS_W = 1e-4
SKIRT_AZ = [("F", 0), ("FL", 45), ("L", 90), ("BL", 135), ("B", 180), ("BR", 225), ("R", 270), ("FR", 315)]
RIGID = {"head": "Head", "eye_l": "Head", "eye_r": "Head", "fist_l": "Hand_L", "fist_r": "Hand_R",
         "shoe_l": "Foot_L", "shoe_r": "Foot_R", "cuff_l": "Foot_L", "cuff_r": "Foot_R",
         "belt": "Pelvis", "pouch": "Pelvis", "sleeve_l": "Arm_L", "sleeve_r": "Arm_R",
         "scarf": "Spine_02", "scarf_tail": "Spine_02"}

# ---------------------------------------------------------------- weight shaping constants (m)
RIGID_EPS = 0.001
# hinge profiles (s05 "c"/"h" form): distal share = smoothstep ramp(s; c, h), c / h = [inner, mid (90 deg), outer],
# linear in ci between; tuned (T230 scratch sweep, P2.2 tube-ratio method) for the 60 deg section ratio
ELBOW = {"c": [0.005, 0.015, 0.0], "h": [0.035, 0.010, 0.005]}
KNEE = {"c": [0.0, 0.010, 0.0], "h": [0.035, 0.025, 0.005]}
WRIST = {"s": [-0.050, -0.001], "w": [0.0, 1.0]}
ANKLE = {"s": [-0.023, -0.001], "w": [0.0, 1.0]}
INNER_DIR = {"arm": (0.0, -1.0, 0.0), "leg": (0.0, 1.0, 0.0)}
PELVIS_FADE = 0.030
NECK_Z = (0.690, 0.745)
CLAV_MAX = 0.6
CLAV_R = 0.090
SKIRT_H = {"z": [0.290, 0.318, 0.348, 0.378, 0.408], "w01": [0.0, 0.0, 0.5, 1.0, 0.0],
           "w02": [1.0, 1.0, 0.5, 0.0, 0.0], "wpel": [0.0, 0.0, 0.0, 0.0, 1.0]}

BONE_COLORS = {"Pelvis": (0.95, 0.85, 0.20), "Spine_01": (0.30, 0.70, 0.95), "Spine_02": (0.20, 0.40, 0.90),
               "Neck": (0.90, 0.30, 0.90), "Head": (0.85, 0.85, 0.85), "Clavicle": (1.00, 0.55, 0.10),
               "Arm": (0.95, 0.25, 0.25), "Forearm": (0.25, 0.85, 0.35), "Hand": (0.55, 0.30, 0.10),
               "Thigh": (0.95, 0.25, 0.25), "Calf": (0.25, 0.85, 0.35), "Foot": (0.55, 0.30, 0.10),
               "Skirt_01": (0.10, 0.80, 0.80), "Skirt_02": (0.60, 0.20, 0.85)}
SKIRT_TINT = {"F": 1.0, "FL": 0.75, "L": 1.0, "BL": 0.75, "B": 1.0, "BR": 0.75, "R": 1.0, "FR": 0.75}


def log(msg):
    print(f"[p23] {msg}")
    sys.stdout.flush()


def default_params():
    return json.loads(json.dumps({"ELBOW": ELBOW, "KNEE": KNEE, "WRIST": WRIST, "ANKLE": ANKLE,
                                  "RIGID_EPS": RIGID_EPS, "PELVIS_FADE": PELVIS_FADE, "NECK_Z": NECK_Z,
                                  "CLAV_MAX": CLAV_MAX, "CLAV_R": CLAV_R, "SKIRT_H": SKIRT_H}))


# ---------------------------------------------------------------- helpers (s05)
def ramp(x, c, h):
    x = np.asarray(x, dtype=np.float64)
    hh = np.maximum(np.asarray(h, dtype=np.float64), 1e-9)
    t = np.clip((x - (np.asarray(c, dtype=np.float64) - hh)) / (2.0 * hh), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def smooth01(x):
    x = np.clip(np.asarray(x, dtype=np.float64), 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def profile(s, prof, cos_in=None):
    s = np.asarray(s, dtype=np.float64)
    if "w" in prof:
        return np.interp(s, prof["s"], prof["w"])
    if "grid" in prof:
        ci = np.asarray(cos_in, dtype=np.float64)
        rows = np.stack([np.interp(s, prof["s"], r) for r in prof["grid"]], axis=1)
        cik = np.asarray(prof["ci"], dtype=np.float64)
        j = np.clip(np.searchsorted(cik, ci) - 1, 0, len(cik) - 2)
        t = np.clip((ci - cik[j]) / (cik[j + 1] - cik[j]), 0.0, 1.0)
        n = np.arange(len(s))
        return (1.0 - t) * rows[n, j] + t * rows[n, j + 1]
    if "c" in prof:
        ci = np.asarray(cos_in, dtype=np.float64)
        (c_in, c_mid, c_out), (h_in, h_mid, h_out) = prof["c"], prof["h"]
        cc = np.where(ci >= 0, ci * c_in + (1.0 - ci) * c_mid, -ci * c_out + (1.0 + ci) * c_mid)
        hh = np.where(ci >= 0, ci * h_in + (1.0 - ci) * h_mid, -ci * h_out + (1.0 + ci) * h_mid)
        return ramp(s, cc, hh)
    raise ValueError(f"unknown profile keys {sorted(prof)}")


def unit(v):
    v = np.asarray(v, dtype=np.float64)
    return v / max(float(np.linalg.norm(v)), 1e-12)


# ---------------------------------------------------------------- context
class Ctx:
    pass


def load_ctx():
    c = Ctx()
    c.arm = bpy.data.objects[ARM]
    c.gm = bpy.data.objects[MESH]
    c.parts = json.loads(PARTS_JSON.read_text(encoding="utf-8"))["part_id"]
    c.loops = json.loads(LOOPS_JSON.read_text(encoding="utf-8"))
    me = c.gm.data
    c.nv = len(me.vertices)
    co = np.empty(c.nv * 3)
    me.vertices.foreach_get("co", co)
    c.co = co.reshape(-1, 3)
    if not np.allclose(np.array(c.gm.matrix_world), np.eye(4)) or not np.allclose(np.array(c.arm.matrix_world), np.eye(4)):
        raise RuntimeError("PL_mesh / PL_rig transform is not identity")
    pid = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid)
    lt = np.empty(len(me.polygons), dtype=np.int32)
    me.polygons.foreach_get("loop_total", lt)
    lv = np.empty(len(me.loops), dtype=np.int32)
    me.loops.foreach_get("vertex_index", lv)
    vpart = np.full(c.nv, -1, dtype=np.int32)
    vpart[lv] = np.repeat(pid, lt)
    if (vpart < 0).any():
        raise RuntimeError("loose vertices")
    c.vpart = vpart
    c.bones = c.arm.data.bones
    c.deform = [b.name for b in c.bones if b.use_deform]
    c.bi = {n: i for i, n in enumerate(c.deform)}
    return c


def head(c, n):
    return np.array(c.bones[n].head_local)


def cos_inner(P, center, ax, inner):
    d = P - center
    r = d - np.outer(d @ ax, ax)
    rn = np.linalg.norm(r, axis=1)
    iv = unit(np.array(inner) - np.dot(inner, ax) * ax)
    return np.where(rn > 1e-9, (r @ iv) / np.maximum(rn, 1e-9), 0.0)


# ---------------------------------------------------------------- weights
def tunic_row_weights(c, P, p, skirt_rows=None):
    """Weights (n, n_deform) for tunic outer-wall positions P (rules in the module docstring).
    skirt_rows (bool mask, T232): positions on the hem / skirt_1..3 rings get Pelvis 0, renormalised onto Skirt_*."""
    W = np.zeros((len(P), len(c.deform)))
    bi = c.bi
    t = c.loops["tunic"]
    zr = {r["name"]: r["z"] for r in t["outer_rows"]}
    z_bb, z_bt = zr["belt_bot"], zr["belt_top"]
    cx, cy = t["axis_xy"]
    z = P[:, 2]
    # torso
    up = z >= z_bt - 1e-6
    z2 = head(c, "Spine_02")[2]
    s2 = smooth01((z - (z2 - 0.04)) / 0.08)
    pel = np.clip(1.0 - (z - z_bt) / p["PELVIS_FADE"], 0.0, 1.0)
    neck = smooth01((z - p["NECK_Z"][0]) / (p["NECK_Z"][1] - p["NECK_Z"][0]))
    W[up, bi["Spine_01"]] = ((1.0 - s2) * (1.0 - pel))[up]
    W[up, bi["Spine_02"]] = (s2 * (1.0 - pel))[up]
    W[up, bi["Pelvis"]] = pel[up]
    W[up] *= (1.0 - neck[up])[:, None]
    W[up, bi["Neck"]] = neck[up]
    for s in SIDES:
        d = np.linalg.norm(P - head(c, f"Arm_{s}"), axis=1)
        wc = p["CLAV_MAX"] * smooth01(1.0 - d / p["CLAV_R"])
        wc = np.where(up, wc, 0.0)
        W[up] *= (1.0 - wc[up])[:, None]
        W[up, bi[f"Clavicle_{s}"]] += wc[up]
    # belt band
    band = (z >= z_bb - 1e-6) & ~up
    W[band, bi["Pelvis"]] = 1.0
    # skirt
    sk = z < z_bb - 1e-6
    h = p["SKIRT_H"]
    w01 = np.interp(z, h["z"], h["w01"])
    w02 = np.interp(z, h["z"], h["w02"])
    wpl = np.interp(z, h["z"], h["wpel"])
    az = np.degrees(np.arctan2(P[:, 0] - cx, -(P[:, 1] - cy))) % 360.0
    i0 = (az // 45.0).astype(int) % 8
    f = (az - i0 * 45.0) / 45.0
    for k in np.nonzero(sk)[0]:
        W[k, bi["Pelvis"]] = wpl[k]
        for ti, wa in ((i0[k], 1.0 - f[k]), ((i0[k] + 1) % 8, f[k])):
            tag = SKIRT_AZ[ti][0]
            W[k, bi[f"Skirt_{tag}_01"]] += wa * w01[k]
            W[k, bi[f"Skirt_{tag}_02"]] += wa * w02[k]
    if skirt_rows is not None:
        W[skirt_rows, bi["Pelvis"]] = 0.0
        W[skirt_rows] /= W[skirt_rows].sum(axis=1)[:, None]
    return W


def build_weights(c, p):
    P = c.co
    bi = c.bi
    W = np.zeros((c.nv, len(c.deform)))
    eps = p["RIGID_EPS"]
    for s in SIDES:
        x = s.lower()
        # arm tube
        m = c.vpart == c.parts[f"arm_{x}"]
        sh, el, wr = head(c, f"Arm_{s}"), head(c, f"Forearm_{s}"), head(c, f"Hand_{s}")
        a = unit(wr - sh)
        s_el, s_wr = (P[m] - el) @ a, (P[m] - wr) @ a
        e = profile(s_el, p["ELBOW"], cos_inner(P[m], el, a, INNER_DIR["arm"]))
        hnd = np.where(s_wr >= -eps, 1.0, profile(s_wr, p["WRIST"]))
        Wm = np.zeros((int(m.sum()), len(c.deform)))
        Wm[:, bi[f"Hand_{s}"]] = hnd
        Wm[:, bi[f"Forearm_{s}"]] = (1.0 - hnd) * e
        Wm[:, bi[f"Arm_{s}"]] = (1.0 - hnd) * (1.0 - e)
        W[m] = Wm
        # leg tube
        m = c.vpart == c.parts[f"leg_{x}"]
        hp, kn, an = head(c, f"Thigh_{s}"), head(c, f"Calf_{s}"), head(c, f"Foot_{s}")
        a = unit(an - hp)
        s_kn, s_an = (P[m] - kn) @ a, (P[m] - an) @ a
        k = profile(s_kn, p["KNEE"], cos_inner(P[m], kn, a, INNER_DIR["leg"]))
        fo = np.where(s_an >= -eps, 1.0, profile(s_an, p["ANKLE"]))
        Wm = np.zeros((int(m.sum()), len(c.deform)))
        Wm[:, bi[f"Foot_{s}"]] = fo
        Wm[:, bi[f"Calf_{s}"]] = (1.0 - fo) * k
        Wm[:, bi[f"Thigh_{s}"]] = (1.0 - fo) * (1.0 - k)
        W[m] = Wm
    # tunic
    t = c.loops["tunic"]
    tm = c.vpart == c.parts["tunic"]
    pair = {}
    for r in t["inner_rows"]:
        if r["paired_outer_row"] is not None:
            for vi, vo in zip(r["verts"], t["outer_rows"][r["paired_outer_row"]]["verts"]):
                pair[vi] = vo
    ceil = set(t["ceiling_interior"]) | set([r for r in t["inner_rows"] if r["paired_outer_row"] is None][0]["verts"])
    tidx = np.nonzero(tm)[0]
    src = np.array([pair.get(int(i), int(i)) for i in tidx])
    skirt_outer = {v for r in t["outer_rows"] if r["name"] in ("hem_outer", "skirt_1", "skirt_2", "skirt_3")
                   for v in r["verts"]}
    Wt = tunic_row_weights(c, P[src], p, np.isin(src, list(skirt_outer)))
    for j, i in enumerate(tidx.tolist()):
        if i in ceil:
            Wt[j] = 0.0
            Wt[j, bi["Pelvis"]] = 1.0
    W[tidx] = Wt
    # rigid
    for pname, bname in RIGID.items():
        mm_ = c.vpart == c.parts[pname]
        W[mm_] = 0.0
        W[mm_, bi[bname]] = 1.0
    # limit + normalise
    W = np.where(W > 0, W, 0.0)
    before = int(((W > EPS_W).sum(axis=1) > MAX_INF).sum())
    order = np.argsort(-W, axis=1)
    keep = np.zeros_like(W, dtype=bool)
    np.put_along_axis(keep, order[:, :MAX_INF], True, axis=1)
    W = np.where(keep & (W > EPS_W), W, 0.0)
    ssum = W.sum(axis=1)
    if (ssum <= 0).any():
        raise RuntimeError(f"{int((ssum <= 0).sum())} vertices without weight")
    W = W / ssum[:, None]
    return W, {"limited_verts": before, "pairs": len(pair), "ceiling_pelvis": len(ceil)}


def assign(c, W):
    gm = c.gm
    for vg in list(gm.vertex_groups):
        gm.vertex_groups.remove(vg)
    for n in c.deform:
        col = W[:, c.bi[n]]
        idx = np.nonzero(col > 0)[0]
        if not len(idx):
            continue
        vg = gm.vertex_groups.new(name=n)
        for i in idx.tolist():
            vg.add([i], float(col[i]), "REPLACE")
    for m in list(gm.modifiers):
        gm.modifiers.remove(m)
    mod = gm.modifiers.new("Armature", "ARMATURE")
    mod.object = c.arm
    mod.use_vertex_groups = True
    mod.use_bone_envelopes = False
    mod.use_deform_preserve_volume = False


def reset_pose(arm):
    for pb in arm.pose.bones:
        pb.rotation_mode = "QUATERNION"
        pb.rotation_quaternion = Quaternion()
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)


def eval_co(gm):
    dg = bpy.context.evaluated_depsgraph_get()
    e = gm.evaluated_get(dg)
    em = e.to_mesh()
    a = np.empty(len(em.vertices) * 3)
    em.vertices.foreach_get("co", a)
    e.to_mesh_clear()
    return a.reshape(-1, 3)


# ---------------------------------------------------------------- renders (p20 ortho_render + colour modes)
def view_basis(view):
    if view == "front":
        d, wup = Vector((0, 1, 0)), Vector((0, 0, 1))
    elif view == "back":
        d, wup = Vector((0, -1, 0)), Vector((0, 0, 1))
    elif view == "side":
        d, wup = Vector((-1, 0, 0)), Vector((0, 0, 1))
    elif view == "three_quarter":
        az, el = math.radians(45.0), math.radians(15.0)
        pos = Vector((-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), math.sin(el)))
        d, wup = -pos, Vector((0, 0, 1))
    elif view == "three_quarter_l":
        az, el = math.radians(-45.0), math.radians(15.0)
        pos = Vector((-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), math.sin(el)))
        d, wup = -pos, Vector((0, 0, 1))
    elif view == "below_front":
        el = math.radians(-35.0)
        pos = Vector((0.0, -math.cos(el), math.sin(el)))
        d, wup = -pos, Vector((0, 0, 1))
    else:
        raise ValueError(view)
    d = d.normalized()
    right = d.cross(wup).normalized()
    up = right.cross(d).normalized()
    return d, right, up


def ortho_render(objs, view, out_png, res_h=1024, frame_bbox=None, color_type="MATERIAL", xray=0.0):
    out_png = Path(out_png).resolve()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    d, right, up = view_basis(view)
    if frame_bbox is None:
        dg = bpy.context.evaluated_depsgraph_get()
        pts = []
        for o in objs:
            oe = o.evaluated_get(dg)
            pts.extend(oe.matrix_world @ Vector(cc) for cc in oe.bound_box)
        mn = Vector((min(q.x for q in pts), min(q.y for q in pts), min(q.z for q in pts)))
        mx = Vector((max(q.x for q in pts), max(q.y for q in pts), max(q.z for q in pts)))
        pad = True
    else:
        mn, mx = Vector(frame_bbox[0]), Vector(frame_bbox[1])
        pad = False
    corners = [Vector((x, y, z)) for x in (mn.x, mx.x) for y in (mn.y, mx.y) for z in (mn.z, mx.z)]
    us = [q.dot(right) for q in corners]
    vs = [q.dot(up) for q in corners]
    ws = [q.dot(d) for q in corners]
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    if pad:
        m = 0.05 * max(u1 - u0, v1 - v0)
        u0, u1, v0, v1 = u0 - m, u1 + m, v0 - m, v1 + m
    w = max(u1 - u0, 1e-6)
    h = max(v1 - v0, 1e-6)
    depth = max(ws) - min(ws)
    res_w = max(1, int(round(res_h * w / h)))
    center_plane = right * ((u0 + u1) / 2) + up * ((v0 + v1) / 2)
    dist = depth + max(w, h) + 1.0
    cam_loc = center_plane + d * (min(ws) - dist)
    sc = bpy.data.scenes.new("_p23_render")
    cam_data = cam_obj = None
    try:
        for o in objs:
            sc.collection.objects.link(o)
        cam_data = bpy.data.cameras.new("_p23_cam")
        cam_data.type = "ORTHO"
        cam_data.sensor_fit = "VERTICAL"
        cam_data.ortho_scale = h
        cam_data.clip_start = 0.001
        cam_data.clip_end = dist + depth + 10.0
        cam_obj = bpy.data.objects.new("_p23_cam", cam_data)
        rot = Matrix((right, up, -d)).transposed()
        cam_obj.matrix_world = Matrix.Translation(cam_loc) @ rot.to_4x4()
        sc.collection.objects.link(cam_obj)
        sc.camera = cam_obj
        r = sc.render
        r.engine = "BLENDER_WORKBENCH"
        r.resolution_x = res_w
        r.resolution_y = res_h
        r.resolution_percentage = 100
        r.film_transparent = True
        r.use_file_extension = False
        r.image_settings.file_format = "PNG"
        r.image_settings.color_mode = "RGBA"
        r.filepath = str(out_png)
        sc.view_settings.view_transform = "Standard"
        sh = sc.display.shading
        sh.light = "STUDIO" if color_type != "VERTEX" else "FLAT"
        sh.color_type = color_type
        sh.show_xray = xray > 0
        if xray > 0:
            sh.xray_alpha = xray
        bpy.ops.render.render(write_still=True, scene=sc.name)
    finally:
        if cam_obj is not None:
            bpy.data.objects.remove(cam_obj, do_unlink=True)
        if cam_data is not None:
            bpy.data.cameras.remove(cam_data)
        bpy.data.scenes.remove(sc)
    return out_png


def bone_mesh_object(arm):
    """Octahedral bone shapes (width 0.1 x length, in the bone's rest frame) as one mesh with two materials."""
    V, F, mi = [], [], []
    for b in arm.data.bones:
        M = b.matrix_local
        L = b.length
        w = 0.1 * L
        loc = [(0, 0, 0), (w, 0.1 * L, 0), (0, 0.1 * L, w), (-w, 0.1 * L, 0), (0, 0.1 * L, -w), (0, L, 0)]
        base = len(V)
        V += [tuple(M @ Vector(q)) for q in loc]
        for a_, b_ in ((1, 2), (2, 3), (3, 4), (4, 1)):
            F.append((base, base + a_, base + b_))
            F.append((base + 5, base + b_, base + a_))
            mi += [0 if b.use_deform else 1] * 2
    me = bpy.data.meshes.new("_p23_bones")
    me.from_pydata(V, [], F)
    for name, col in (("_p23_def", (0.95, 0.15, 0.10, 1.0)), ("_p23_nondef", (0.10, 0.45, 1.0, 1.0))):
        mat = bpy.data.materials.new(name)
        mat.diffuse_color = col
        me.materials.append(mat)
    me.polygons.foreach_set("material_index", np.array(mi, dtype=np.int32))
    me.update()
    return bpy.data.objects.new("_p23_bones", me)


def weight_object(c, W, mode, bone=None):
    """Copy of PL_mesh with a POINT colour attribute: mode 'heat' (weight of `bone`, blue -> red) or 'blend'
    (sum of weight x bone colour)."""
    me = c.gm.data.copy()
    ob = bpy.data.objects.new("_p23_w", me)
    cols = np.zeros((c.nv, 4))
    cols[:, 3] = 1.0
    if mode == "heat":
        w = W[:, c.bi[bone]]
        cols[:, 0] = np.clip(2.0 * w - 1.0, 0, 1)
        cols[:, 1] = np.where(w < 0.5, 2.0 * w, 2.0 - 2.0 * w)
        cols[:, 2] = np.clip(1.0 - 2.0 * w, 0, 1)
    else:
        for n, i in c.bi.items():
            if n.startswith("Skirt_"):
                tag, lv = n.split("_")[1], n.split("_")[2]
                col = np.array(BONE_COLORS[f"Skirt_{lv}"]) * SKIRT_TINT[tag]
            else:
                col = np.array(BONE_COLORS[n.rsplit("_", 1)[0] if n.endswith(("_L", "_R")) else n])
                if n.endswith("_R"):
                    col = col * 0.8
            cols[:, :3] += W[:, i][:, None] * col[None, :]
    ca = me.color_attributes.new("weights", "FLOAT_COLOR", "POINT")
    ca.data.foreach_set("color", cols.astype(np.float32).ravel())
    me.color_attributes.active_color = ca
    return ob


def renders(c, W):
    out = []
    bo = bone_mesh_object(c.arm)
    for view in ("front", "side"):
        out.append(ortho_render([c.gm, bo], view, INSPECT / f"bones_{view}.png", xray=0.35))
    bm_ = bo.data
    bpy.data.objects.remove(bo, do_unlink=True)
    bpy.data.meshes.remove(bm_)
    el, sh = head(c, "Forearm_L"), head(c, "Arm_L")
    kn = head(c, "Calf_L")
    jobs = [("heat", "Forearm_L", "front", "weights_arm_l_forearm.png",
             ((sh[0] - 0.02, -0.3, 0.33), (0.47, 0.3, sh[2] + 0.05))),
            ("blend", None, "front", "weights_arm_l.png", ((sh[0] - 0.02, -0.3, 0.33), (0.47, 0.3, sh[2] + 0.05))),
            ("heat", "Calf_L", "front", "weights_leg_l_calf.png", ((kn[0] - 0.09, -0.3, 0.0), (kn[0] + 0.12, 0.3, 0.46))),
            ("blend", None, "side", "weights_leg_l.png", ((0.0, -0.20, 0.0), (0.3, 0.12, 0.46))),
            ("blend", None, "front", "weights_tunic_front.png", None),
            ("blend", None, "back", "weights_tunic_back.png", None),
            ("blend", None, "three_quarter", "weights_skirt.png", ((-0.25, -0.25, 0.26), (0.25, 0.25, 0.50))),
            ("blend", None, "below_front", "weights_skirt_below.png", ((-0.25, -0.25, 0.26), (0.25, 0.25, 0.48))),
            ("heat", "Clavicle_L", "three_quarter_l", "weights_shoulder_l_clavicle.png",
             ((0.0, -0.2, 0.56), (0.30, 0.2, 0.80))),
            ("blend", None, "three_quarter_l", "weights_shoulder_l.png", ((0.0, -0.2, 0.56), (0.30, 0.2, 0.80))),
            ("heat", "Neck", "front", "weights_neck.png", ((-0.2, -0.3, 0.60), (0.2, 0.3, 0.80)))]
    for mode, bone, view, fname, box in jobs:
        ob = weight_object(c, W, mode, bone)
        out.append(ortho_render([ob], view, INSPECT / fname, frame_bbox=box, color_type="VERTEX"))
        m_ = ob.data
        bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.meshes.remove(m_)
    return out


# ---------------------------------------------------------------- main
def main():
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    c = load_ctx()
    if len(c.gm.vertex_groups) or len(c.gm.modifiers):
        raise RuntimeError("PL_mesh already has vertex groups / modifiers")
    p = default_params()
    W, info = build_weights(c, p)
    assign(c, W)
    log(f"WEIGHTS built: deform bones {len(c.deform)}, verts limited to {MAX_INF} (had more) {info['limited_verts']}, "
        f"inner-wall verts paired {info['pairs']}, ceiling ring + interior (Pelvis) {info['ceiling_pelvis']}")
    inv = {v: k for k, v in c.parts.items()}
    for pv in sorted(inv):
        m = c.vpart == pv
        cnt = (W[m] > 0).sum(axis=1)
        dom = np.bincount(W[m].argmax(axis=1), minlength=len(c.deform))
        nz = (W[m] > 0).sum(axis=0)
        log(f"PART {inv[pv]:10s} ({int(m.sum())} v): influences {({k: int((cnt == k).sum()) for k in range(1, MAX_INF + 1)})}, "
            f"|sum-1|>1e-6 {int((np.abs(W[m].sum(axis=1) - 1) > 1e-6).sum())}; bones (verts w>0): "
            + ", ".join(f"{c.deform[i]}:{int(nz[i])}" for i in np.argsort(-nz) if nz[i] > 0)
            + "; dominant: " + ", ".join(f"{c.deform[i]}:{int(dom[i])}" for i in np.argsort(-dom) if dom[i] > 0))
    no_w = [n for n in c.deform if not (W[:, c.bi[n]] > 0).any()]
    log(f"STATS max influences {int((W > 0).sum(axis=1).max())}, unweighted verts {int((W.sum(axis=1) <= 0).sum())}, "
        f"|sum-1| max {float(np.abs(W.sum(axis=1) - 1).max()):.2e}, deform bones without weights {no_w}, "
        f"non-deform groups {[g.name for g in c.gm.vertex_groups if not c.bones[g.name].use_deform]}")
    log(f"PARAMS {json.dumps(p)}")
    reset_pose(c.arm)
    c.arm.data.pose_position = "POSE"
    bpy.context.view_layer.update()
    rdev = np.linalg.norm(eval_co(c.gm) - c.co, axis=1)
    log(f"REST deformation max {rdev.max() * 1000:.6f} mm (vertex {int(rdev.argmax())})")
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    bpy.context.view_layer.objects.active = c.arm
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    log(f"OUTPUT {OUT_BLEND} objects {sorted((o.name, o.type) for o in bpy.data.objects)}; modifiers "
        f"{[(m.type, m.object.name) for m in c.gm.modifiers]}, vertex groups {len(c.gm.vertex_groups)}")
    out = renders(c, W)
    log("RENDERS " + ", ".join(str(Path(q).relative_to(REPO)).replace("\\", "/") for q in out))
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
