"""s02e_mouth.py - T111 / T111b (spec d-29 section 3 R1): mouth opening, cavity, tongue, fangs and one open shape
key per variant (mouth_open_v1, _v2, _v3; the chosen one is renamed mouth_open later).

Input : rig/work/r01d_merged.blend (s02d output, opened, never saved over; HR2 contract, rig/data/hr2_contract.md).
        Poses / renders only (--pose): afterwards rig/gob_r04_ctrl.blend is opened in memory (never saved), its
        GOB_mesh gets the new mesh data with the weights of the original vertices by index (new / region vertices:
        weights of the nearest original head vertex).
Output: rig/gob_r01_retopo.blend (the canonical retopo final; copy=True, save_version 0; GOB_mesh with shape keys
        Basis / mouth_open and the material slots GOB_skin (0) / GOB_mouth_inner (1) / GOB_teeth (2) /
        GOB_tongue (3)), rig/data/mouth.json, rig/inspect/R1/*.png

Run:  bl.ps1 -Script s02e_mouth.py -Blend work/r01d_merged.blend [-- --no-render] [--pose]
      Chain (run_all_gates.ps1): no arguments -> modeling + rest wire renders (inspect/R1/wire_rest_*); the pose phase
      (ready / roar poses on rig/gob_r04_ctrl.blend, a downstream product of this chain, with its measures and the
      inspect/R1/hr1_* renders / sheet, mouth.json "poses" / "renders") runs only with --pose.
mouth.json "region" (HR2 contract, read by the G2 checkers): the definition / removed region faces and vertices of the
  input, plus boundary_loop_co and removed_region_verts_co (world = GOB_mesh space, identity) and output_faces /
  output_verts (indices in GOB_mesh of rig/gob_r01_retopo.blend: every new face inside the boundary loop, by group:
  lip patch (material 0), cavity (1), teeth (2), tongue (3)).

Coordinates: front -Y, up +Z, the front view projects on (x, z).  cx = mouth centre x = mean of the two fang
centres; Z_M = closed mouth line = mean fang base z (measured on the fang plates).
Face surface S(x, z): first hit of a +Y ray on the original head faces with the fang and eye-protrusion faces
removed and their holes fan-filled (fallback: polynomial fit of the front face, FIT terms, also used for the fang /
eye detection).

Mouth region (design change like the ball fists; every vertex outside it keeps its exact position in Basis):
  head faces (part_id head) whose vertices all lie in the front window (y < REGION_Y_MAX, |x - cx| <= REGION_X,
  REGION_Z_MIN <= z <= REGION_Z_MAX), face normal y < REGION_NY and no vertex on an eye protrusion (eye disc
  EYE_DISC and fit residual < -EYE_RES) and none in the eye undercut (z > REGION_UNDERCUT z_min with vertex
  normal z < nz_max); plus every face touching a fang plate vertex (FANG_ZONE, residual < -FANG_RES); connected
  component holding the mouth centre, enclosed holes filled.  Region vertices = vertices used only by region faces;
  they are re-used (same indices) for the new geometry, further new vertices are appended after the last original
  vertex.  Boundary loop B = original vertices, unchanged.
New geometry (all faces part_id head; Basis and topology shared by every variant, only the open positions differ):
  rim loop (M = 2 * N_SEG verts: 2 corners shared by the upper / lower lip, N_SEG - 1 per lip)
    closed (Basis): x = cx + s * CLOSED_HALF_W (s = u/2 + sin(pi u/2)/2, u uniform in -1..1), lower lip z = Z_M,
      upper lip z = Z_M + GAP * sqrt(1 - |s|^8) (thin mouth line like reference f7, the fangs stand on it), y = S(x, z);
    open (variant): superellipse |dx / half_w|^n + |dz / h|^n = 1 between z_bot and z_top (a rounded rectangle, taller
      than wide ~0.6); the corners sit on its sides at z = Z_M (no outward pull); upper lip vertices spaced by arc
      length over the top, lower lip over the sides and bottom; y = S(x, z + w D) - w F, w = 0 upper / 1 lower /
      0.5 corners.
  lip annulus: rings at RING_T between B (t = 0) and the rim (t = 1); a ring vertex blends the rim vertex and the
    point of B at the same normalised angle phi (upper lip phi = acos(s), lower -acos(s)) in (x, z), y = S (B's own
    offset to S is not blended: B runs partly in the undercut below the eyes); open: (x, z) and jaw weight blended the
    same way from B's open position and the open rim; zipper triangles between B and the outer ring (merged by phi).
    Material 0.
  cavity (material 1, GOB_mouth_inner, flat near-black): a closed sack on the rim loop: per column, the profile
    rim_u -> 3 roof loops -> back line -> 3 floor loops -> rim_l (offsets POCKET_CLOSED / POCKET_OPEN (depth +y,
    height z), depth taper 1 - 0.5 |s|^4, height taper 1 - |s|^6; the back line merges upper and lower loops).
    Closed it is a thin lens inside the head behind the mouth line.
  tongue (material 3, GOB_tongue): a dome (TONGUE_NS segments, flat base) on the cavity floor TONGUE_BACK behind the
    open lower lip centre, half width tongue_w * half_w; closed (variant independent): shrunk (TONGUE_CLOSED) inside the chin below the
    cavity floor.
  fangs (closed triangular prisms, material 2, GOB_teeth): lower fangs = the original fang plates rebuilt (front
    triangle, measured front plane, depth FANG_DEPTH) = the closed look; open: scaled lo_scale, centre at
    cx +- fang_x * half_w standing on the lower lip (base FANG_EMBED below it, front FANG_PROUD ahead of the lip,
    depth FANG_OPEN_DEPTH).  Upper fangs (new): open uf_w x uf_h hanging from the upper lip at the same x; closed
    squashed to UF_CLOSED_W x UF_CLOSED_H and hidden above the cavity roof UF_CLOSED_BACK behind the face.
Shape key mouth_open_<variant> (relative to Basis): new vertices -> their open positions; original head vertices
  outside the region -> V + w J with J = (0, -jaw_f, -jaw_d) and the jaw field w = angle term (1 below / 0 above the
  mouth line inside |x - cx| <= CLOSED_HALF_W, else smoothstep((a + JAW_ANG0) / (pi / 2 + JAW_ANG0)) with the angle
  a = atan2(Z_M - z, |x - cx| - CLOSED_HALF_W) below the corner) *
  smoothstep lateral (JAW_X0 .. JAW_X1) * smoothstep depth (JAW_Y0 .. JAW_Y1); other parts 0.
Normals: untouched faces keep their original corner normals (re-set as custom normals); new faces smooth with
  sharp edges = rim edges + new edges > SHARP_ANGLE, auto corner normals, except skin corners on B which take the
  original normal of that vertex (mean of its original corners in the removed non-fang region faces) and the skin
  corners of lip / rim vertices, which take the original shading normal at their Basis point (barycentric blend of
  the hit original triangle's corner normals on the fang-free face surface; hole-fill hits or a result with normal
  y >= LIPN_MIN_FRONT keep the auto normal), so the closed face shades like the original.  T111d: original
  vertices hidden behind the closed fang plates (footprint + FP_MARGIN) get the fitted face-surface normal on their
  skin corners (their original normals were shaded next to the fused fang and showed as a notch once it moved).
UVs: new faces get local islands (lip annulus: front (x, z) of Basis; cavity: closed profile grid; fang / tongue
  faces: one planar island per face, upper fangs / tongue on the first variant's open shape), all at the head-front
  texel density of the original UVs (x0.85 per retry until they fit), placed by bounding box into UV space that no
  original face covers (UV_RES raster, UV_MARGIN px).  T122 (G2.9): before saving, the whole GOB_mesh UV layout is
  redone with s02d_finish.make_uvs (the same method as s02d; head-front islands = head faces with normal . -Y > 0.7,
  x HEAD_FRONT_SCALE), replacing these local islands; geometry, shape keys, materials and normals are unchanged.
Poses: ready = goblin.reset_rig + goblin.ready_pose + data/clips/idle.json pose frame 1 (with the club);
  roar = ready + CTRL_head rot X ROAR_HEAD_X (head tilted up, like reference f15).
Measures (raw, no verdicts): counts, region deviation, non-manifold / winding / zero-area, per variant max
  displacement, front-camera (swing_cam.json front) mouth / head extents at rest, ready and roar, body penetration
  per pose, cavity / tongue / hidden fang vertices outside the head skin at 0, rendered mouth pixels.
Renders (inspect/R1): wire_rest_<closed|v*>_{front,tq}.png (rest, unrigged); <ready|roar>_<closed|v*>_
  {front_close,tq_close (3/4 from the character's right = swing_cam), tq_left_close (mirrored, character's left),
  side_close (profile from the character's left), full_front}.png (Workbench, material colours, club);
  mouth_variants_sheet.png (rows closed (ready) / v1 / v2 / v3 (roar); columns reference f7 / f15 close-up | front
  close-up | 3/4 close-up | full front); mouth_v1_v4_sheet.png (rows closed / v1 / v4; columns reference | front |
  left 3/4 | right 3/4 | side profile | full front); mouth_v4_v5_sheet.png (rows closed / v4 / v5, same columns).
  T124: culled_{closed,open}_{front,tq}.png (rest, Workbench with back-face culling on magenta, like Unity).
Variant v5 (T111d): v4 method, bigger opening; slide.lift: global vertical remap of the whole head (z < zlo fixed,
  [zlo, ze] stretched, the eye band [ze, eye_top] = eye rings + eyeballs + lift (rigid), [eye_top, z_fix]
  compressed), blended in the mouth column with the v4 lip slide (upper lip -> ze + lift - (ze - z) cu); the open
  corners sit at the remapped closed-line height; stronger squash-stretch.
Variant v4 (T111c): no jaw (jaw_d = jaw_f = 0); original head vertices near the mouth slide in the face plane (Design.
  slide_disp: upper lip toward the eyes, lower face toward the front-face bottom edge, y following S below the line);
  then the whole head part is squash-stretched in the same key (squash: x narrower, fading to full width at the head
  bottom; z taller, anchored at the head bottom); shallower cavity / tongue (pocket_open, tongue_* overrides).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bmesh  # noqa: E402
import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

IN_BLEND = goblib.WORK / "r01d_merged.blend"      # HR2: s02d output
CTRL_BLEND = goblib.RIG / "gob_r04_ctrl.blend"
OUT_BLEND = goblib.RIG / "gob_r01_retopo.blend"   # HR2: canonical retopo final
INSP = goblib.INSPECT / "R1"
REF_CLOSEUP = goblib.RIG.parent / "ref_roar" / "closeup_f07_f15_f27.png"
REF_F15 = goblib.RIG.parent / "ref_roar" / "frames" / "f_0015.png"
MESH = "GOB_mesh"
# ---- detection
FIT_WIN = dict(y_max=-0.12, ny_max=-0.5, z=(0.97, 1.34), x=0.25)
EYE_DISC = 0.085            # eye protrusion search radius about the eye centres (from the fit residual)
EYE_RES = 0.0015
FANG_ZONE = dict(ax=(0.04, 0.16), z=(1.02, 1.108), y_plate=(-0.2475, -0.2405))   # flat front plate band
FANG_RES = 0.008
# ---- region
REGION_Y_MAX = -0.12
REGION_X = 0.23
REGION_Z_MIN = 0.955         # T111e: enlarged rebuilt region (was 0.962 / 1.14 / -0.30)
REGION_Z_MAX = 1.12
REGION_NY = -0.20
REGION_CENTROID = dict(x=0.200, z=(0.965, 1.105), y_max_all=-0.10)  # T114: faces by centroid only
REGION_EYE_MARGIN = 0.004    # T114: faces touching a vertex within eye radius + this of an eye centre stay out
RING_T_HR = (0.25, 0.50, 0.70, 0.86)   # T114: concentric quad loops between the boundary loop and the rim
FLAT_FOLLOW = dict(y_max=-0.15, x=0.24, z=(0.95, 1.30), max_off=0.004)   # T114: originals following the fit
REGION_FILL_PASSES = 5        # T114: notch filling passes on the region boundary
PHONG_ALPHA = 0.75            # T114: Phong tessellation weight (Face.S3, kept for reference; the patch uses S)
RING_UNWRAP_R = 0.25          # T114: radius of the (u, z) unwrap for the ring grid
RING_RELAX = 200             # T114: Jacobi iterations of the ring grid Laplacian (x, z)
REGION_UNDERCUT = dict(z_min=1.08, nz_max=-0.45)   # eye undercut verts (normal facing down) stay out
# ---- mouth shape
GAP = 0.008                 # closed mouth line height
CLOSED_MARGIN = 0.006       # closed half width = outer fang base |x - cx| + this
N_SEG = 16
RING_T = (0.20, 0.40, 0.58, 0.74, 0.88)   # T111e: 5 concentric loops (was 0.40, 0.72)
PATCH_FIT = dict(z=(0.93, 1.30), x=0.27, y_max=-0.12, reject=0.002, n_samp=12, deg=6, deg_x=4, deg_z=5)
SKIN_MAT, INNER_MAT, TEETH_MAT, TONGUE_MAT = "GOB_skin", "GOB_mouth_inner", "GOB_teeth", "GOB_tongue"
MAT_ORDER = (SKIN_MAT, INNER_MAT, TEETH_MAT, TONGUE_MAT)       # material slot index = position
MAT_RGBA = {SKIN_MAT: (0.60, 0.66, 0.50, 1.0),                # = s10a MESH_RGBA (render colour)
            INNER_MAT: (0.02, 0.02, 0.02, 1.0),               # flat near-black cavity
            TEETH_MAT: (0.93, 0.88, 0.75, 1.0),               # cream fangs
            TONGUE_MAT: (0.30, 0.08, 0.06, 1.0)}              # dark red-brown tongue
KEY_PREFIX = "mouth_open_"   # (T111b-e variant keys; T114: one key, see key_name)


def key_name(nm_):
    return "mouth_open" if len(VARIANTS) == 1 else key_name(nm_)
# ---- open-mouth variants (one shape key each).  Opening = superellipse |dx/half_w|^n + |dz/h|^n = 1 between z_bot
# and z_top (rest coordinates, after the jaw drop); the rim corners sit on its sides at the closed line z Z_M.
# jaw_d / jaw_f: jaw translation (down / forward) of the head below the mouth line.  fang_x: open fang centre
# |x - cx| / half_w; lo_scale: lower fang size / original fang; uf_w, uf_h: upper fang; tongue_w: tongue half
# width / half_w.
VARIANTS = (
    dict(name="open", note="T114 (HR1b): v6 method with v5 proportions on the even-quad head: opening cut into the "
                           "flat front (no jaw, box profile), patch skin on the fitted face surface in both states, "
                           "eyes lifted as a rigid band, squash-stretch anchored at the head bottom",
         half_w=0.150, z_top=1.152, z_bot=0.976, n=3.5, jaw_d=0.0, jaw_f=0.0, fang_x=0.55, lo_scale=0.55, uf_w=0.046,
         uf_h=0.042, tongue_w=0.45, flat=True, smooth_disp=dict(iters=0, k=0.5, z_fix_below=0.995,
                                                                  zone=dict(y_max=-0.12, z_max=1.10, x=0.28)),
         slide=dict(ze=1.100, zb=0.952, cu=0.50, cl=1.0, hc=0.150, ang0=math.radians(20.0), x0=0.18, x1=0.26,
                    y0=-0.18, y1=-0.14, lift=0.072, eye_top=1.262, z_fix=1.3905, zlo=0.99),
         squash=dict(sx=0.87, sz=1.15, start=0.02, fade=0.12),
         pocket_open={"u": ((0.012, 0.000), (0.035, 0.004), (0.060, 0.010)),
                      "l": ((0.010, 0.000), (0.020, -0.002), (0.030, -0.003)), "back": 0.060},
         tongue_back=0.016, tongue_base=0.002, tongue_h=0.020),
)
JAW_X0, JAW_X1 = 0.17, 0.28
JAW_ANG0 = math.radians(30.0)   # beside the mouth corners the jaw weight is 0 above -30 deg (no pull under the eyes)
JAW_Y0, JAW_Y1 = -0.19, -0.10
POCKET_CLOSED = {"u": ((0.006, 0.002), (0.016, 0.006), (0.030, 0.008)),
                 "l": ((0.006, -0.002), (0.016, -0.005), (0.028, -0.006)), "back": 0.040}
POCKET_OPEN = {"u": ((0.012, 0.000), (0.035, 0.004), (0.065, 0.012)),
               "l": ((0.012, 0.000), (0.030, -0.003), (0.050, -0.006)), "back": 0.080}
FANG_DEPTH = 0.025          # closed lower fang (rebuilt original plate)
FANG_OPEN_DEPTH = 0.014
FANG_EMBED = 0.004
FANG_PROUD = 0.003
UF_CLOSED_H = 0.003
UF_CLOSED_W = 0.040
UF_CLOSED_Z = 0.011         # closed upper fang apex this far above the upper lip (above the cavity roof)
UF_CLOSED_BACK = 0.017      # closed upper fang front this far behind the face
UF_CLOSED_DEPTH = 0.010
TONGUE_NS = 10              # segments around
TONGUE_DEPTH_R, TONGUE_H = 0.030, 0.030     # open: half depth (y), dome height above its base
TONGUE_BACK = 0.040         # open: centre this far behind the lower lip centre
TONGUE_BASE = -0.008        # open: base plane this far below the lower lip centre (under the cavity floor)
TONGUE_CLOSED = dict(scale=0.25, half_w=0.055, back=0.030, dz=-0.022)    # closed: shrunk, inside the chin below the cavity floor
ROAR_HEAD_X = -18.0         # roar-like pose: idle f1 + CTRL_head rot X (X- = head tilted up)
SHARP_ANGLE = math.radians(60.0)
FP_MARGIN = 0.002           # fang footprint margin (x, z) for the hidden-vertex normal reset
LIPN_MIN_FRONT = -0.5        # interpolated original normal used only if its y < this (front facing)
UV_RES = 1024
UV_MARGIN = 3
ZERO_AREA = 1e-9            # m^2
RAY_DIRS = [Vector(d).normalized() for d in ((0.5773, 0.5271, 0.6237), (-0.6428, 0.2819, 0.7124),
                                              (0.2113, -0.8356, -0.5071))]


def mm(x):
    return round(float(x) * 1000.0, 2)


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def fit_terms(x, z):
    x, z = np.asarray(x, float), np.asarray(z, float)
    return np.stack([np.ones_like(x), x, z, x * x, x * z, z * z, x ** 3, x * x * z, x * z * z, z ** 3,
                     x ** 4, x * x * z * z, z ** 4], -1)


def mesh_arrays(me):
    V = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    pid = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid)
    faces = [tuple(p.vertices) for p in me.polygons]
    return V.reshape(-1, 3), faces, pid


def edge_face_counts(faces, sel=None):
    cnt = {}
    for fi, f in enumerate(faces):
        if sel is not None and not sel[fi]:
            continue
        n = len(f)
        for k in range(n):
            a, b = f[k], f[(k + 1) % n]
            key = (a, b) if a < b else (b, a)
            cnt[key] = cnt.get(key, 0) + 1
    return cnt


def topo_report(V, faces, pid, head_id):
    """Non-manifold edges (face count != 2), whole mesh and head faces only; winding: edges whose two faces
    traverse them in the same direction; zero-area faces."""
    allc = edge_face_counts(faces)
    headc = edge_face_counts(faces, pid == head_id)
    dirs = {}
    for f in faces:
        n = len(f)
        for k in range(n):
            dirs[(f[k], f[(k + 1) % n])] = dirs.get((f[k], f[(k + 1) % n]), 0) + 1
    same_dir = sum(1 for c in dirs.values() if c > 1)
    areas = face_areas(V, faces)
    return {"nonmanifold_edges_all": sum(1 for c in allc.values() if c != 2),
            "boundary_edges_all": sum(1 for c in allc.values() if c == 1),
            "nonmanifold_edges_head": sum(1 for c in headc.values() if c != 2),
            "same_direction_edges": same_dir,
            "zero_area_faces": int((areas < ZERO_AREA).sum()), "min_face_area_mm2": float(areas.min() * 1e6)}


def face_areas(V, faces):
    out = np.empty(len(faces))
    for i, f in enumerate(faces):
        p = V[list(f)]
        a = np.zeros(3)
        for k in range(1, len(f) - 1):
            a += np.cross(p[k] - p[0], p[k + 1] - p[0])
        out[i] = 0.5 * np.linalg.norm(a)
    return out


def inside(tree, p):
    votes = 0
    for d in RAY_DIRS:
        o, n = Vector(p), 0
        for _ in range(512):
            hit = tree.ray_cast(o, d)
            if hit[0] is None:
                break
            n += 1
            o = hit[0] + d * 1e-6
        votes += n % 2
    return votes >= 2


def point_tri_dist2d(q, T):
    """Distance from 2D point q to triangle T (3 x 2); 0 inside."""
    def cross(a, b):
        return a[0] * b[1] - a[1] * b[0]
    s_ = [cross(T[(k + 1) % 3] - T[k], q - T[k]) for k in range(3)]
    if all(x >= 0 for x in s_) or all(x <= 0 for x in s_):
        return 0.0
    best = 1e9
    for k in range(3):
        a, b = T[k], T[(k + 1) % 3]
        t = np.clip(np.dot(q - a, b - a) / max(np.dot(b - a, b - a), 1e-20), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(q - (a + t * (b - a)))))
    return best


def cam_project(cam, P):
    M = np.array(cam["matrix_world"])
    R, t = M[:3, :3], M[:3, 3]
    s = cam["resolution"][1] / cam["ortho_scale"]
    L = (np.asarray(P) - t) @ R
    return np.c_[cam["resolution"][0] / 2 + L[:, 0] * s, cam["resolution"][1] / 2 - L[:, 1] * s], s


# ================================================================ detection
class Face:
    """Front face surface of the original head (fang faces removed, holes filled) + polynomial fallback."""

    def __init__(self, V, faces, pid, head_id):
        self.head_faces = [i for i in range(len(faces)) if pid[i] == head_id]
        hv = sorted({v for i in self.head_faces for v in faces[i]})
        self.head_verts = np.array(hv)
        H = V[self.head_verts]
        me_n = np.empty(len(bpy.data.objects[MESH].data.vertices) * 3)
        bpy.data.objects[MESH].data.vertices.foreach_get("normal", me_n)
        Nv = me_n.reshape(-1, 3)[self.head_verts]
        w = FIT_WIN
        base = (H[:, 1] < w["y_max"]) & (Nv[:, 1] < w["ny_max"]) & (H[:, 2] > w["z"][0]) & (H[:, 2] < w["z"][1]) \
            & (np.abs(H[:, 0]) < w["x"])
        # first pass: everything in the window; eyes / fangs removed by residual (2 passes)
        m = base.copy()
        for _ in range(3):
            c, *_r = np.linalg.lstsq(fit_terms(H[m, 0], H[m, 2]), H[m, 1], rcond=None)
            res = H[:, 1] - fit_terms(H[:, 0], H[:, 2]) @ c
            m = base & (res > -0.0025)
        self.c = c
        self.fit_n = int(m.sum())
        self.fit_rms = float(np.sqrt((res[m] ** 2).mean()))
        self.res = np.zeros(len(V))
        self.res[self.head_verts] = res
        self.front = np.zeros(len(V), bool)
        self.front[self.head_verts] = base | ((H[:, 1] < w["y_max"]) & (H[:, 2] > 1.0) & (H[:, 2] < 1.3))

    def fit_patch(self, V, faces, drop_faces, pid, head_id):
        """T111e smooth front-face surface y = pf(x, z): least squares of x^i z^j (i <= deg_x, j <= deg_z,
        i + j <= deg) to points sampled on the original head faces (fang / eye faces dropped) in the PATCH_FIT window,
        iteratively rejecting residuals > reject."""
        w = PATCH_FIT
        rng = np.random.default_rng(0)
        pts = []
        for i in self.head_faces:
            if i in drop_faces:
                continue
            P = V[list(faces[i])]
            if (P[:, 1] > w["y_max"]).any() or (np.abs(P[:, 0]) > w["x"]).any() or (P[:, 2] < w["z"][0]).any()                     or (P[:, 2] > w["z"][1]).any():
                continue
            n = np.cross(P[1] - P[0], P[2] - P[0])
            if n[1] > -0.2 * np.linalg.norm(n):
                continue
            for _ in range(w["n_samp"]):
                a, b = rng.random(2)
                if a + b > 1:
                    a, b = 1 - a, 1 - b
                pts.append(P[0] + a * (P[1] - P[0]) + b * (P[2] - P[0]))
        pts = np.array(pts)
        self.pexp = [(i, j) for i in range(w["deg_x"] + 1) for j in range(w["deg_z"] + 1) if i + j <= w["deg"]]
        m = np.ones(len(pts), bool)
        for _ in range(4):
            A = self._pterms(pts[m, 0], pts[m, 2])
            c, *_r = np.linalg.lstsq(A, pts[m, 1], rcond=None)
            res = pts[:, 1] - self._pterms(pts[:, 0], pts[:, 2]) @ c
            m = np.abs(res) < w["reject"]
        self.pc = c
        self.pfit = {"samples": int(len(pts)), "used": int(m.sum()), "rms_m": float(np.sqrt((res[m] ** 2).mean())),
                     "terms": len(self.pexp), "exponents_xz": self.pexp, "coef": c.tolist(),
                     "normalise": "x' = x / 0.25, z' = (z - 1.12) / 0.2, y = sum coef * x'^i z'^j"}

    def _pterms(self, x, z):
        x = (np.asarray(x, float) - 0.0) / 0.25
        z = (np.asarray(z, float) - 1.12) / 0.2
        return np.stack([x ** i * z ** j for i, j in self.pexp], -1)

    def pf(self, x, z):
        return float((self._pterms([x], [z]) @ self.pc)[0])

    def pn(self, x, z):
        """Outward (front) unit normal of the surface y = pf(x, z)."""
        e = 1e-4
        fx = (self.pf(x + e, z) - self.pf(x - e, z)) / (2 * e)
        fz = (self.pf(x, z + e) - self.pf(x, z - e)) / (2 * e)
        n = np.array([fx, -1.0, fz])
        return n / np.linalg.norm(n)

    def poly(self, x, z):
        return float((fit_terms([x], [z]) @ self.c)[0])

    def build_surface(self, V, faces, drop_faces):
        """Triangles of the head without drop_faces; each hole (boundary loop) closed by a fan from its first
        vertex (pinch vertices: the loop is traced greedily)."""
        tris, tri_face = [], []
        for i in self.head_faces:
            if i in drop_faces:
                continue
            f = faces[i]
            for k in range(1, len(f) - 1):
                tris.append((f[0], f[k], f[k + 1]))
                tri_face.append(i)
        d = {}
        for a, b, c in tris:
            for e in ((a, b), (b, c), (c, a)):
                d[e] = d.get(e, 0) + 1
        nxt = {}
        for (a, b) in d:
            if (b, a) not in d:
                nxt.setdefault(a, []).append(b)
        self.n_holes_filled = 0
        while nxt:
            s0 = next(iter(nxt))
            lp, v = [s0], s0
            while True:
                w = nxt[v].pop()
                if not nxt[v]:
                    del nxt[v]
                if w == s0:
                    break
                lp.append(w)
                v = w
                if v not in nxt:
                    break
            for k in range(1, len(lp) - 1):     # hole on the right of the boundary direction -> reversed fan
                tris.append((lp[0], lp[k + 1], lp[k]))
                tri_face.append(-1)
            self.n_holes_filled += 1
        self.tris = np.array(tris)
        self.tri_face = tri_face
        self.rec = None
        self.tree = BVHTree.FromPolygons([tuple(p) for p in V], [tuple(t) for t in tris])
        self.Vs = V
        self.vn = None
        self.n_fallback = 0
        self.fallback_pts = []

    def S3(self, x, z):
        """T114: point on the head surface under (x, z) with Phong tessellation (alpha PHONG_ALPHA) of the hit
        triangle using the head's own corner normals: a smooth surface through the head vertices (no facets)."""
        hit = self.tree.ray_cast(Vector((x, -2.0, z)), Vector((0.0, 1.0, 0.0)))
        if hit[0] is None:
            return np.array([x, self.poly(x, z), z])
        fi = self.tri_face[hit[2]]
        q = np.array(hit[0])
        if fi < 0 or self.rec is None:
            return q
        t = self.tris[hit[2]]
        P = self.Vs[t]
        a = np.cross(P[1] - P[0], P[2] - P[0])
        aa = a @ a
        w1 = np.cross(q - P[0], P[2] - P[0]) @ a / aa
        w2 = np.cross(P[1] - P[0], q - P[0]) @ a / aa
        w = (1 - w1 - w2, w1, w2)
        ps = np.zeros(3)
        for k in range(3):
            n = self.rec[fi][int(t[k])]
            n = n / max(np.linalg.norm(n), 1e-20)
            ps += w[k] * (q - ((q - P[k]) @ n) * n)
        return (1 - PHONG_ALPHA) * q + PHONG_ALPHA * ps

    def normal(self, x, z):
        """Original shading normal at the front surface point (x, z): barycentric blend of self.vn."""
        hit = self.tree.ray_cast(Vector((x, -2.0, z)), Vector((0.0, 1.0, 0.0)))
        if hit[0] is None:
            return None
        t = self.tris[hit[2]]
        P = self.Vs[t]
        q = np.array(hit[0])
        a = np.cross(P[1] - P[0], P[2] - P[0])
        aa = a @ a
        w1 = np.cross(q - P[0], P[2] - P[0]) @ a / aa
        w2 = np.cross(P[1] - P[0], q - P[0]) @ a / aa
        fi = self.tri_face[hit[2]]
        if fi < 0:
            return None
        nn = [self.rec[fi][int(v)] for v in t]
        n = (1 - w1 - w2) * nn[0] + w1 * nn[1] + w2 * nn[2]
        return n / max(np.linalg.norm(n), 1e-20)

    def S(self, x, z):
        hit = self.tree.ray_cast(Vector((x, -2.0, z)), Vector((0.0, 1.0, 0.0)))
        if hit[0] is None or hit[0].y > -0.1:
            self.n_fallback += 1
            self.fallback_pts.append((round(x, 4), round(z, 4)))
            return self.poly(x, z)
        return float(hit[0].y)


def detect(V, faces, pid, face, head_id):
    """Eye / fang vertices, fang triangles, mouth centre and line."""
    res = face.res
    hv = face.head_verts
    H = V[hv]
    fr = np.zeros(len(V), bool)
    fr[hv] = H[:, 1] < REGION_Y_MAX
    # fangs
    fz = FANG_ZONE
    ax0 = np.abs(V[:, 0])
    fang_v = fr & (ax0 > fz["ax"][0]) & (ax0 < fz["ax"][1]) & (V[:, 2] > fz["z"][0]) & (V[:, 2] < fz["z"][1]) \
        & (res < -FANG_RES) & (V[:, 1] > fz["y_plate"][0]) & (V[:, 1] < fz["y_plate"][1])
    fangs = {}
    for side, sel in (("l", V[:, 0] < 0), ("r", V[:, 0] > 0)):
        ids = np.nonzero(fang_v & sel)[0]
        if len(ids) < 3:
            raise RuntimeError(f"fang {side}: only {len(ids)} plate verts")
        P = V[ids]
        zb = P[:, 2].min()
        low = P[:, 2] < zb + 0.004
        bl = P[low][np.argmin(P[low, 0])]
        br = P[low][np.argmax(P[low, 0])]
        ap = P[np.argmax(P[:, 2])]
        fangs[side] = {"plate_verts": ids.tolist(), "base_l": [bl[0], zb], "base_r": [br[0], zb],
                       "apex": [ap[0], ap[2]], "front_y": float(np.median(P[:, 1])),
                       "centre_x": float((bl[0] + br[0]) / 2)}
    cx = (fangs["l"]["centre_x"] + fangs["r"]["centre_x"]) / 2.0
    z_m = (fangs["l"]["base_l"][1] + fangs["r"]["base_l"][1]) / 2.0
    half_c = max(abs(fangs["l"]["base_l"][0] - cx), abs(fangs["r"]["base_r"][0] - cx)) + CLOSED_MARGIN
    # eyes: protruding verts above the fangs
    eye_cand = fr & (V[:, 2] > 1.10) & (res < -EYE_RES) & ~fang_v
    eyes = {}
    for side, sel in (("l", V[:, 0] < 0), ("r", V[:, 0] > 0)):
        ids = np.nonzero(eye_cand & sel & (np.abs(V[:, 0]) > 0.05) & (np.abs(V[:, 0]) < 0.24) & (V[:, 2] < 1.30))[0]
        P = V[ids]
        c = [(P[:, 0].min() + P[:, 0].max()) / 2, (P[:, 2].min() + P[:, 2].max()) / 2]
        r = max(P[:, 0].max() - P[:, 0].min(), P[:, 2].max() - P[:, 2].min()) / 2
        eyes[side] = {"centre_xz": c, "radius": float(r), "z_min": float(P[:, 2].min())}
    eye_v = np.zeros(len(V), bool)
    for e in eyes.values():
        d = np.hypot(V[:, 0] - e["centre_xz"][0], V[:, 2] - e["centre_xz"][1])
        eye_v |= fr & (d < EYE_DISC) & (res < -EYE_RES)
    return {"fang_v": fang_v, "eye_v": eye_v, "fangs": fangs, "eyes": eyes, "cx": float(cx), "z_m": float(z_m),
            "half_c": float(half_c)}


def select_region(V, faces, pid, head_id, det):
    cx = det["cx"]
    n = len(faces)
    fnorm = np.zeros((n, 3))
    for i, f in enumerate(faces):
        p = V[list(f)]
        a = np.zeros(3)
        for k in range(1, len(f) - 1):
            a += np.cross(p[k] - p[0], p[k + 1] - p[0])
        fnorm[i] = a / max(np.linalg.norm(a), 1e-20)
    vn = np.empty(len(V) * 3)
    bpy.data.objects[MESH].data.vertices.foreach_get("normal", vn)
    vn = vn.reshape(-1, 3)
    under = (V[:, 2] > REGION_UNDERCUT["z_min"]) & (vn[:, 2] < REGION_UNDERCUT["nz_max"])
    okv = (V[:, 1] < REGION_Y_MAX) & (np.abs(V[:, 0] - cx) <= REGION_X) & (V[:, 2] >= REGION_Z_MIN) \
        & (V[:, 2] <= REGION_Z_MAX) & ~det["eye_v"] & ~under
    # T114: eye discs from the head's designed hard edges (eye junction / wall loops) above the fangs
    me_ = bpy.data.objects[MESH].data
    shp = np.zeros(len(me_.edges), bool)
    if me_.attributes.get("sharp_edge") is not None:
        me_.attributes["sharp_edge"].data.foreach_get("value", shp)
    Ee = np.empty(len(me_.edges) * 2, dtype=np.int64)
    me_.edges.foreach_get("vertices", Ee)
    Ee = Ee.reshape(-1, 2)
    headv_ = {v for i_, f_ in enumerate(faces) if pid[i_] == head_id for v in f_}
    sv = np.array(sorted({int(v) for e_ in np.nonzero(shp)[0] for v in Ee[e_] if int(v) in headv_}))
    sv = sv[(V[sv, 1] < REGION_Y_MAX) & (V[sv, 2] > det["z_m"] + 0.08)]
    near_eye = np.zeros(len(V), bool)
    eye_discs = []
    for sel in (V[sv, 0] < cx, V[sv, 0] > cx):
        Q = V[sv[sel]]
        if not len(Q):
            continue
        c_ = 0.5 * (Q.min(0) + Q.max(0))
        r_ = 0.5 * max(np.ptp(Q[:, 0]), np.ptp(Q[:, 2]))
        eye_discs.append((float(c_[0]), float(c_[2]), float(r_)))
        near_eye |= np.hypot(V[:, 0] - c_[0], V[:, 2] - c_[2]) < r_ + REGION_EYE_MARGIN
    reg = np.zeros(n, bool)
    fang_faces = set()
    for i, f in enumerate(faces):
        if pid[i] != head_id:
            continue
        vs = list(f)
        if det["fang_v"][vs].any():
            reg[i] = True
            fang_faces.add(i)
        elif near_eye[vs].any():
            continue
        elif REGION_CENTROID and not (det["eye_v"][vs].any() or under[vs].any()) and fnorm[i, 1] < REGION_NY                 and (V[vs, 1] < REGION_CENTROID["y_max_all"]).all():
            c_ = V[vs].mean(0)     # T111e: larger region, centroid rule (keeps the big original triangles whole)
            if abs(c_[0] - cx) <= REGION_CENTROID["x"] and REGION_CENTROID["z"][0] <= c_[2] <= REGION_CENTROID["z"][1]:
                reg[i] = True
    # face adjacency (head faces)
    e2f = {}
    for i, f in enumerate(faces):
        if pid[i] != head_id:
            continue
        for k in range(len(f)):
            a, b = f[k], f[(k + 1) % len(f)]
            e2f.setdefault((min(a, b), max(a, b)), []).append(i)
    adj = {i: set() for i in range(n) if pid[i] == head_id}
    for fs in e2f.values():
        for a in fs:
            for b in fs:
                if a != b:
                    adj[a].add(b)
    # T114: fill boundary notches: non-region front faces (not near the eyes) sharing >= 2 edges with the region, or
    # 1 edge and >= 3 vertices
    vf_ = {}
    for i in adj:
        for v_ in faces[i]:
            vf_.setdefault(v_, []).append(i)
    for _ in range(REGION_FILL_PASSES):
        add = []
        for i in adj:
            if reg[i] or near_eye[list(faces[i])].any() or fnorm[i, 1] > REGION_NY                     or abs(V[list(faces[i])].mean(0)[0] - cx) > REGION_CENTROID["x"] + 0.04:
                continue
            n_sh = 0
            f_ = faces[i]
            for k in range(len(f_)):
                a_, b_ = f_[k], f_[(k + 1) % len(f_)]
                if any(reg[o] for o in e2f[(min(a_, b_), max(a_, b_))] if o != i):
                    n_sh += 1
            n_rv = sum(1 for v_ in f_ if any(reg[o] for o in vf_[v_]))
            if n_sh >= 2 or (n_sh >= 1 and n_rv >= 3):
                add.append(i)
        if not add:
            break
        reg[add] = True
    # component holding the mouth centre
    cen = np.array([V[list(f)].mean(0) for f in faces])
    cand = np.nonzero(reg)[0]
    seed = cand[np.argmin(np.hypot(cen[cand, 0] - cx, cen[cand, 2] - det["z_m"]))]
    comp, stack = {int(seed)}, [int(seed)]
    while stack:
        a = stack.pop()
        for b in adj[a]:
            if reg[b] and b not in comp:
                comp.add(b)
                stack.append(b)
    # fill holes: head faces not reachable from the head top without crossing the component
    top = max(adj, key=lambda i: cen[i, 2])
    seen, stack = {top}, [top]
    while stack:
        a = stack.pop()
        for b in adj[a]:
            if b not in comp and b not in seen:
                seen.add(b)
                stack.append(b)
    filled = [i for i in adj if i not in comp and i not in seen]
    comp |= set(filled)
    # boundary loop
    bedges = []
    for (a, b), fs in e2f.items():
        inn = [f for f in fs if f in comp]
        if len(inn) == 1 and len(fs) == 2:
            f = faces[inn[0]]
            k = f.index(a)
            if f[(k + 1) % len(f)] == b:
                bedges.append((a, b))
            else:
                bedges.append((b, a))
        elif len(inn) == 1:
            raise RuntimeError(f"region edge {(a, b)} has {len(fs)} faces")
    nxt = {}
    for a, b in bedges:
        if a in nxt:
            raise RuntimeError(f"region boundary vertex {a} with two outgoing edges (not a simple loop)")
        nxt[a] = b
    loops, seen_v = [], set()
    for s in nxt:
        if s in seen_v:
            continue
        lp, v = [], s
        while v not in seen_v:
            seen_v.add(v)
            lp.append(v)
            v = nxt[v]
        loops.append(lp)
    if len(loops) != 1:
        raise RuntimeError(f"region boundary has {len(loops)} loops (expected 1)")
    B = loops[0]
    xz = V[B][:, [0, 2]]
    area = 0.5 * np.sum(xz[:, 0] * np.roll(xz[:, 1], -1) - np.roll(xz[:, 0], -1) * xz[:, 1])
    if area < 0:
        B = B[::-1]
    reg_faces = sorted(comp)
    used = {v for i in reg_faces for v in faces[i]}
    bset = set(B)
    reg_verts = sorted(used - bset)
    return {"faces": reg_faces, "verts": reg_verts, "B": B, "fang_faces": sorted(fang_faces), "eye_discs_xzr": eye_discs,
            "filled_faces": len(filled), "winding_face_order_ccw_area": float(abs(area))}


# ================================================================ design
class Design:
    """Closed geometry (variant independent) + the open outline / jaw of one variant."""

    def __init__(self, V, face, det, region, var):
        self.V, self.face, self.det, self.reg, self.var = V, face, det, region, var
        self.cx, self.zm, self.hc = det["cx"], det["z_m"], det["half_c"]
        B = list(region["B"])
        nb = len(B)
        if nb % 2:
            raise RuntimeError(f"boundary loop has an odd vertex count {nb} (1:1 quad loops need it even)")
        # corners: the pair (a, a + nb / 2) nearest the closed line on the left / right sides
        Pb = V[B]
        best = None
        for a in range(nb):
            b = (a + nb // 2) % nb
            if Pb[a, 0] < det["cx"] < Pb[b, 0]:
                cst = abs(Pb[a, 2] - det["z_m"]) + abs(Pb[b, 2] - det["z_m"]) - 0.5 * (Pb[b, 0] - Pb[a, 0])
                if best is None or cst < best[0]:
                    best = (cst, a, b)
        _c, a, b = best
        B = B[b:] + B[:b]          # start at the right corner (CCW: over the top to the left corner, back below)
        self.B = B
        self.N = nb // 2
        self.u = np.linspace(-1.0, 1.0, self.N + 1)
        self.s = 0.5 * self.u + 0.5 * np.sin(0.5 * math.pi * self.u)   # closed: denser at the corners
        self.Bxz = V[B][:, [0, 2]]
        d = self.Bxz - np.array([self.cx, self.zm])
        self.ax = float(np.abs(d[:, 0]).max())
        self.bz_u = float(d[:, 1].max())
        self.bz_l = float(-d[:, 1].min())
        self.Bphi = np.mod(np.arctan2(self.nz(d[:, 1]), d[:, 0] / self.ax), 2 * math.pi)
        self.D, self.F = var["jaw_d"], var["jaw_f"]
        self.J = np.array([0.0, -self.F, -self.D])
        self.Bw = self.jaw_w(V[B])
        self.Bres = np.array([V[b, 1] - face.S(V[b, 0], V[b, 2]) for b in B])
        self._outline()

    def nz(self, dz):
        dz = np.asarray(dz, float)
        return np.where(dz >= 0, dz / self.bz_u, dz / self.bz_l)

    def jaw_w(self, P):
        P = np.atleast_2d(P)
        x = np.abs(P[:, 0] - self.cx)
        z = P[:, 2]
        th = np.arctan2(self.zm - z, x - self.hc)
        wa = np.where(x > self.hc, smoothstep((th + JAW_ANG0) / (0.5 * math.pi + JAW_ANG0)), (z < self.zm).astype(float))
        lx = smoothstep((JAW_X1 - x) / (JAW_X1 - JAW_X0))
        ly = smoothstep((JAW_Y1 - P[:, 1]) / (JAW_Y1 - JAW_Y0))
        return wa * lx * ly

    def slide_disp(self, P):
        """v4 lip slide of original head vertices (rest): above the mouth line z -> ze - (ze - z) cu, below it
        z -> zb + (z - zb) cl; weights: 1 inside |x - cx| <= hc, beside the corner smoothstep of the angle from the
        corner (0 at +-ang0 around horizontal), * lateral smoothstep x0..x1 * depth smoothstep (y0..y1); below the
        line y follows the face surface S (clamped to |dz|)."""
        sl = self.var.get("slide")
        out = np.zeros_like(np.atleast_2d(P), dtype=float)
        if not sl:
            return out
        P = np.atleast_2d(P)
        x = np.abs(P[:, 0] - self.cx)
        z = P[:, 2]
        th = np.arctan2(z - self.zm, x - sl["hc"])
        a0 = sl["ang0"]
        wu = np.where(x > sl["hc"], smoothstep((th - a0) / (0.5 * math.pi - a0)), 1.0)
        wl = np.where(x > sl["hc"], smoothstep((-th - a0) / (0.5 * math.pi - a0)), 1.0)
        lw = smoothstep((sl["x1"] - x) / (sl["x1"] - sl["x0"])) * smoothstep((sl["y1"] - P[:, 1]) / (sl["y1"] - sl["y0"]))
        up = (z > self.zm) & (z < sl["ze"])
        lo = (z < self.zm) & (z > sl["zb"])
        dz = np.zeros(len(P))
        if "lift" in sl:
            # v5: global vertical remap of the whole head (no lateral / depth weights, so no shear creases): bottom
            # below zlo fixed, [zlo, ze] stretched, eye band [ze, eye_top] (eye rings + eyeballs) + lift (rigid),
            # [eye_top, z_fix] compressed; in the mouth column the lips follow the v4 slide instead (upper lip ->
            # ze + lift - (ze - z) cu, lower face -> zb + (z - zb) cl), blended by the corner-angle / lateral / depth
            # weights
            L, ze, zt, zf, zlo = sl["lift"], sl["ze"], sl["eye_top"], sl["z_fix"], sl["zlo"]
            zg = z.copy()
            m1 = (z > zlo) & (z < ze)
            zg[m1] = zlo + (z[m1] - zlo) * (ze + L - zlo) / (ze - zlo)
            m2 = (z >= ze) & (z <= zt)
            zg[m2] = z[m2] + L
            m3 = (z > zt) & (z < zf)
            zg[m3] = z[m3] + L * (zf - z[m3]) / (zf - zt)
            blend = smoothstep((z - self.zm) / (ze - self.zm))
            wfu = (wu + (1.0 - wu) * blend) * lw
            zn = zg.copy()
            zn[up] = zg[up] + wfu[up] * ((ze + L - (ze - z[up]) * sl["cu"]) - zg[up])
            zn[lo] = zg[lo] + (wl * lw)[lo] * ((sl["zb"] + (z[lo] - sl["zb"]) * sl["cl"]) - zg[lo])
            dz = zn - z
            out[:, 2] = dz
            for i in np.nonzero(lo & (np.abs(dz) > 1e-9))[0]:
                dy = self.face.S(P[i, 0], z[i] + dz[i]) - self.face.S(P[i, 0], z[i])
                out[i, 1] = float(np.clip(dy, -abs(dz[i]), abs(dz[i])))
            return out
        else:
            dz[up] = ((sl["ze"] - (sl["ze"] - z[up]) * sl["cu"]) - z[up]) * wu[up] * lw[up]
        dz[lo] = ((sl["zb"] + (z[lo] - sl["zb"]) * sl["cl"]) - z[lo]) * wl[lo] * lw[lo]
        out[:, 2] = dz
        for i in np.nonzero(lo & (np.abs(dz) > 1e-9))[0]:
            dy = self.face.S(P[i, 0], z[i] + dz[i]) - self.face.S(P[i, 0], z[i])
            out[i, 1] = float(np.clip(dy, -abs(dz[i]), abs(dz[i])))
        return out

    def zsrc(self, z):
        """Inverse of the v5/v6 global vertical remap (pre-squash z -> original z); identity without lift."""
        sl = self.var.get("slide") or {}
        if "lift" not in sl:
            return z
        L, ze, zt, zf, zlo = sl["lift"], sl["ze"], sl["eye_top"], sl["z_fix"], sl["zlo"]
        if z <= zlo:
            return z
        if z <= ze + L:
            return zlo + (z - zlo) * (ze - zlo) / (ze + L - zlo)
        if z <= zt + L:
            return z - L
        if z < zf:
            k = L / (zf - zt)
            return (z - k * zf) / (1.0 - k)
        return z

    # -- open outline (superellipse), sampled by arc length
    def _outline(self):
        v = self.var
        a, n = v["half_w"], v["n"]
        zc, b = 0.5 * (v["z_top"] + v["z_bot"]), 0.5 * (v["z_top"] - v["z_bot"])
        sl = v.get("slide") or {}
        zq = self.zm
        if "lift" in sl:     # v5: corners at the global remap height of the closed line
            zq = sl["zlo"] + (self.zm - sl["zlo"]) * (sl["ze"] + sl["lift"] - sl["zlo"]) / (sl["ze"] - sl["zlo"])
        self.z_corner = zq
        q = (zq - zc) / b
        if not -1.0 < q < 1.0:
            raise RuntimeError(f"{v['name']}: closed line z outside the open outline")
        tc = math.asin(math.copysign(abs(q) ** (n / 2.0), q))

        def pts(t0, t1):
            t = np.linspace(t0, t1, 4001)
            c, s = np.cos(t), np.sin(t)
            x = self.cx + a * np.sign(c) * np.abs(c) ** (2.0 / n)
            z = zc + b * np.sign(s) * np.abs(s) ** (2.0 / n)
            P = np.c_[x, z]
            L = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
            return P, L / L[-1]
        self.arc_u = pts(tc, math.pi - tc)                  # right corner -> over the top -> left corner
        self.arc_l = pts(math.pi - tc, 2 * math.pi + tc)    # left corner -> under the bottom -> right corner

    def arc_at(self, arc, f):
        P, L = arc
        return np.array([np.interp(f, L, P[:, 0]), np.interp(f, L, P[:, 1])])

    def arc_z_at_x(self, side, x):
        P, _L = self.arc_u if side == "u" else self.arc_l
        zc = 0.5 * (self.var["z_top"] + self.var["z_bot"])
        m = P[:, 1] > zc if side == "u" else P[:, 1] < zc
        Q = P[m]
        return float(Q[np.argmin(np.abs(Q[:, 0] - x)), 1])

    # -- rim
    def rim_loop(self):
        N = self.N
        lp = [(N, "c")] + [(c, "u") for c in range(N - 1, 0, -1)] + [(0, "c")] + [(c, "l") for c in range(1, N)]
        return lp

    def phi(self, c, side):
        s = self.s[c]
        if side == "c":
            return 0.0 if s > 0 else math.pi
        a = math.acos(max(-1.0, min(1.0, s)))
        return a if side == "u" else 2 * math.pi - a

    def rim_closed(self, c, side):
        s = self.s[c]
        x = self.cx + s * self.hc
        z = self.zm + (GAP * math.sqrt(max(0.0, 1.0 - abs(s) ** 8)) if side == "u" else 0.0)
        return np.array([x, self.face.S(x, z), z])

    def rim_w(self, side):
        return {"u": 0.0, "l": 1.0, "c": 0.5}[side]

    def rim_open(self, c, side):
        u = self.u[c]
        if side == "u":
            x, z = self.arc_at(self.arc_u, (1.0 - u) / 2.0)
        elif side == "l":
            x, z = self.arc_at(self.arc_l, (u + 1.0) / 2.0)
        else:
            x, z = self.arc_at(self.arc_u, 0.0 if u > 0 else 1.0)
        w = self.rim_w(side)
        if self.var.get("flat"):
            return np.array([x, self.face.S(x, self.zsrc(z)), z])
        return np.array([x, self.face.S(x, z + w * self.D) - w * self.F, z])

    # -- B sampling
    def B_at(self, phi):
        """Point of B hit by the ray from the mouth centre at normalised angle phi: (index a, index b, t)."""
        d = np.array([math.cos(phi) * self.ax, math.sin(phi) * (self.bz_u if math.sin(phi) >= 0 else self.bz_l)])
        o = np.array([self.cx, self.zm])
        best = None
        n = len(self.B)
        for i in range(n):
            p, q = self.Bxz[i], self.Bxz[(i + 1) % n]
            e = q - p
            den = d[0] * (-e[1]) - d[1] * (-e[0])
            if abs(den) < 1e-15:
                continue
            r = p - o
            t = (r[0] * (-e[1]) - r[1] * (-e[0])) / den
            u = (d[0] * r[1] - d[1] * r[0]) / den
            if t > 1e-9 and -1e-9 <= u <= 1 + 1e-9 and (best is None or t < best[0]):
                best = (t, i, (i + 1) % n, min(max(u, 0.0), 1.0))
        if best is None:
            raise RuntimeError(f"no B hit at phi {phi}")
        return best[1:]

    def B_point(self, phi, V1):
        i, j, u = self.B_at(phi)
        b0 = (1 - u) * self.V[self.B[i]] + u * self.V[self.B[j]]
        b1 = (1 - u) * V1[self.B[i]] + u * V1[self.B[j]]
        w = (1 - u) * self.Bw[i] + u * self.Bw[j]
        res = (1 - u) * self.Bres[i] + u * self.Bres[j]
        return b0, b1, w, res


def build_geometry(des, V1_orig):
    """New vertices (basis P0, open P1 of des.var) and faces with material, island, per-corner local UV."""
    face, det = des.face, des.det
    var = des.var
    D, F = des.D, des.F
    P0, P1, tags = [], [], []
    faces = []    # (vertex refs, material, island, uv list, group)

    def nv(p0, p1, tag):
        P0.append(np.asarray(p0, float))
        P1.append(np.asarray(p1, float))
        tags.append(tag)
        return ("n", len(P0) - 1)

    loop = des.rim_loop()
    M = len(loop)
    rim, rim0, rim1 = {}, {}, {}
    for j, (c, side) in enumerate(loop):
        a0, a1 = des.rim_closed(c, side), des.rim_open(c, side)
        rim[(c, side)] = nv(a0, a1, "rim_" + ("upper" if side == "u" else "lower" if side == "l" else "corner"))
        rim0[(c, side)], rim1[(c, side)] = a0, a1
    # lip rings: loop k vertex j between boundary vertex B[j] and rim vertex j (1:1, quad bands); start = linear
    # interpolation at RING_T_HR, then RING_RELAX Jacobi iterations of the (x, z) grid Laplacian with the boundary
    # loop and the rim fixed (closed and open state separately) so the loops follow the region shape without folds
    Bref = [("o", b) for b in des.B]
    K = len(RING_T_HR)
    # grid coordinates (u, z): u = arc length around the vertical axis through (cx, 0) (unwraps the rounded head
    # corners, so the boundary loop stays monotone where the lattice bends around the sides)

    def to_u(x, y):
        return RING_UNWRAP_R * math.atan2(x - des.cx, -y)

    def x_from_u(u, z, open_):
        lo, hi = des.cx - 0.40, des.cx + 0.40
        zz = des.zsrc(z) if open_ else z
        for _ in range(48):
            mid = 0.5 * (lo + hi)
            if to_u(mid, face.pf(mid, zz)) < u:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)
    G0 = np.zeros((K + 2, M, 2))
    G1 = np.zeros((K + 2, M, 2))
    for j, (c, side) in enumerate(loop):
        bi = des.B[j]
        b0, b1 = des.V[bi], V1_orig[bi]
        r0, r1 = rim0[(c, side)], rim1[(c, side)]
        G0[0, j] = (to_u(b0[0], b0[1]), b0[2])
        G1[0, j] = (to_u(b1[0], b1[1]), b1[2])
        G0[K + 1, j] = (to_u(r0[0], r0[1]), r0[2])
        G1[K + 1, j] = (to_u(r1[0], r1[1]), r1[2])
    for k, t in enumerate(RING_T_HR):
        G0[k + 1] = (1 - t) * G0[0] + t * G0[K + 1]
        G1[k + 1] = (1 - t) * G1[0] + t * G1[K + 1]
    L1 = G1.copy()
    L0 = G0.copy()
    for G in (G0, G1):
        for _ in range(RING_RELAX):
            G[1:K + 1] = 0.25 * (G[0:K] + G[2:K + 2] + np.roll(G[1:K + 1], 1, axis=1) + np.roll(G[1:K + 1], -1, axis=1))
    # both states: inner loops mostly the straight interpolation (the relaxed grid crosses the rim where the
    # boundary crowds it, e.g. at the slit corners), outer loops mostly relaxed
    for k, t in enumerate(RING_T_HR):
        G1[k + 1] = (1.0 - t) * G1[k + 1] + t * L1[k + 1]
        G0[k + 1] = (1.0 - t) * G0[k + 1] + t * L0[k + 1]
    rings = []
    for k in range(K):
        ring = []
        for j, (c, side) in enumerate(loop):
            u0, z0 = G0[k + 1, j]
            u1, z1 = G1[k + 1, j]
            x0 = x_from_u(u0, z0, False)
            x1 = x_from_u(u1, z1, True)
            y0 = face.S(x0, z0)
            y1 = face.S(x1, des.zsrc(z1))
            ring.append(nv((x0, y0, z0), (x1, y1, z1), f"lip_ring_{k}"))
        rings.append(ring)
    rim_ring = [rim[k] for k in loop]
    des.B_nonmonotone = 0
    lip_faces = []
    seq = [Bref] + rings + [rim_ring]
    for k in range(len(seq) - 1):
        a, b = seq[k], seq[k + 1]
        for q in range(M):
            lip_faces.append((a[q], a[(q + 1) % M], b[(q + 1) % M], b[q]))
    for f in lip_faces:
        faces.append((f, 0, "lip", None, "lip"))
    # cavity: grid (c, p), p = 0 rim_u, 1..3 roof, 4 back, 5..7 floor, 8 rim_l
    N = des.N
    grid = {}
    gp0, gp1 = {}, {}
    pocket_open = var.get("pocket_open", POCKET_OPEN)

    def taper(s):
        return 1.0 - 0.5 * abs(s) ** 4, 1.0 - abs(s) ** 6

    for c in range(N + 1):
        s = des.s[c]
        tau, eps = taper(s)
        corner = c in (0, N)
        ku = (c, "c") if corner else (c, "u")
        kl = (c, "c") if corner else (c, "l")
        grid[(c, 0)], grid[(c, 8)] = rim[ku], rim[kl]
        gp0[(c, 0)], gp1[(c, 0)] = rim0[ku], rim1[ku]
        gp0[(c, 8)], gp1[(c, 8)] = rim0[kl], rim1[kl]
        for k in range(3):
            pu = []
            pl = []
            for prof, base_u, base_l in ((POCKET_CLOSED, rim0[ku], rim0[kl]), (pocket_open, rim1[ku], rim1[kl])):
                du, hu = prof["u"][k]
                dl, hl = prof["l"][k]
                if corner:
                    d = 0.5 * (du + dl) * tau
                    pu.append(base_u + np.array([0.0, d, 0.0]))
                    pl.append(pu[-1])
                else:
                    pu.append(base_u + np.array([0.0, du * tau, hu * eps]))
                    pl.append(base_l + np.array([0.0, dl * tau, hl * eps]))
            if corner:
                r = nv(pu[0], pu[1], "cavity")
                grid[(c, 1 + k)] = grid[(c, 7 - k)] = r
                gp0[(c, 1 + k)] = gp0[(c, 7 - k)] = pu[0]
                gp1[(c, 1 + k)] = gp1[(c, 7 - k)] = pu[1]
            else:
                grid[(c, 1 + k)] = nv(pu[0], pu[1], "cavity")
                grid[(c, 7 - k)] = nv(pl[0], pl[1], "cavity")
                gp0[(c, 1 + k)], gp1[(c, 1 + k)] = pu
                gp0[(c, 7 - k)], gp1[(c, 7 - k)] = pl
        backs = []
        for prof, st_ in ((POCKET_CLOSED, 0), (pocket_open, 1)):
            gu = (gp0 if st_ == 0 else gp1)[(c, 3)]
            gl = (gp0 if st_ == 0 else gp1)[(c, 5)]
            ru = (rim0 if st_ == 0 else rim1)[ku]
            rl = (rim0 if st_ == 0 else rim1)[kl]
            backs.append(np.array([0.5 * (ru[0] + rl[0]), 0.5 * (ru[1] + rl[1]) + prof["back"] * tau,
                                   0.5 * (gu[2] + gl[2])]))
        grid[(c, 4)] = nv(backs[0], backs[1], "cavity")
        gp0[(c, 4)], gp1[(c, 4)] = backs
    # cavity UV: u = closed x of the column, v = arc length along the closed + open profile (variant independent
    # would need the open profile; the closed profile is used so every variant shares the UVs)
    vcoord = {}
    for c in range(N + 1):
        acc = 0.0
        vcoord[(c, 0)] = 0.0
        for p in range(1, 9):
            acc += float(np.linalg.norm(gp0[(c, p)] - gp0[(c, p - 1)])) + 0.01
            vcoord[(c, p)] = acc
    for c in range(N):
        for p in range(8):
            cells = [(c + 1, p), (c, p), (c, p + 1), (c + 1, p + 1)]
            refs = [grid[k] for k in cells]
            uvs = [(gp0[k][0], -vcoord[k]) for k in cells]
            faces.append((tuple(refs), 1, "cavity", uvs, "cavity"))
    # fangs (material 2)
    fang_groups = {}

    def prism(tri0, tri1, y0f, y0b, y1f, y1b, name):
        """tri: 3 (x, z) CCW in the front view; front y / back y per state."""
        f0 = [nv((x0, y0f, z0), (x1, y1f, z1), name) for (x0, z0), (x1, z1) in zip(tri0, tri1)]
        b0 = [nv((x0, y0b, z0), (x1, y1b, z1), name) for (x0, z0), (x1, z1) in zip(tri0, tri1)]
        fs = [tuple(f0), (b0[2], b0[1], b0[0])]
        for k in range(3):
            a, b = k, (k + 1) % 3
            fs.append((f0[b], f0[a], b0[a], b0[b]))
        for k, f in enumerate(fs):
            faces.append((f, 2, f"{name}_{k}", None, name))
        fang_groups[name] = f0 + b0

    for side in ("l", "r"):
        fd = det["fangs"][side]
        tri0 = [tuple(fd["base_l"]), tuple(fd["base_r"]), tuple(fd["apex"])]
        xc0 = fd["centre_x"]
        zb = fd["base_l"][1]
        xf = des.cx + math.copysign(var["fang_x"] * var["half_w"], xc0 - des.cx)
        zl = des.arc_z_at_x("l", xf) - FANG_EMBED
        sc_ = var["lo_scale"]
        tri1 = [(xf + (x - xc0) * sc_, zl + (z - zb) * sc_) for x, z in tri0]
        y1f = face.S(xf, zl + FANG_EMBED + D) - F - FANG_PROUD
        prism(tri0, tri1, fd["front_y"], fd["front_y"] + FANG_DEPTH, y1f, y1f + FANG_OPEN_DEPTH, f"fang_lower_{side}")
        # upper fang: closed squashed + hidden above the cavity roof at the closed fang x; open hanging from the lip
        zt1 = des.arc_z_at_x("u", xf) + FANG_EMBED
        tri1u = [(xf - var["uf_w"] / 2, zt1), (xf, zt1 - var["uf_h"]), (xf + var["uf_w"] / 2, zt1)]
        zt0 = des.zm + GAP + UF_CLOSED_Z + UF_CLOSED_H
        tri0u = [(xc0 - UF_CLOSED_W / 2, zt0), (xc0, zt0 - UF_CLOSED_H), (xc0 + UF_CLOSED_W / 2, zt0)]
        y0f = face.S(xc0, des.zm) + UF_CLOSED_BACK
        y1f = face.S(xf, zt1 - FANG_EMBED) - FANG_PROUD
        prism(tri0u, tri1u, y0f, y0f + UF_CLOSED_DEPTH, y1f, y1f + FANG_OPEN_DEPTH, f"fang_upper_{side}")
    # tongue (material 3): dome (base ring, 2 rings, pole) + flat base (centre fan)
    ns = TONGUE_NS
    cl = rim1[(N // 2, "l")]
    rw1 = var["tongue_w"] * var["half_w"]
    t_h = var.get("tongue_h", TONGUE_H)
    base1 = np.array([des.cx, cl[1] + var.get("tongue_back", TONGUE_BACK), cl[2] + var.get("tongue_base", TONGUE_BASE)])
    tcz = TONGUE_CLOSED
    base0 = np.array([des.cx, face.S(des.cx, des.zm) + tcz["back"], des.zm + tcz["dz"]])
    lat = (0.0, 0.55, 0.9)
    tv = []
    for li, la in enumerate(lat):
        ring = []
        for k in range(ns):
            a = 2 * math.pi * k / ns
            r = math.cos(0.5 * math.pi * la)
            h = math.sin(0.5 * math.pi * la)
            o1 = np.array([rw1 * r * math.cos(a), TONGUE_DEPTH_R * r * math.sin(a), t_h * h])
            o0 = np.array([tcz["half_w"] * r * math.cos(a), TONGUE_DEPTH_R * r * math.sin(a), TONGUE_H * h])
            ring.append(nv(base0 + o0 * tcz["scale"], base1 + o1, "tongue"))
        tv.append(ring)
    pole = nv(base0 + np.array([0, 0, TONGUE_H * tcz["scale"]]), base1 + np.array([0, 0, t_h]), "tongue")
    bctr = nv(base0, base1, "tongue")
    tongue_faces = []
    for li in range(len(lat) - 1):
        for k in range(ns):
            k2 = (k + 1) % ns
            tongue_faces.append(((tv[li][k], tv[li][k2], tv[li + 1][k2], tv[li + 1][k]), (li, k)))
    for k in range(ns):
        k2 = (k + 1) % ns
        tongue_faces.append(((tv[-1][k], tv[-1][k2], pole), (len(lat) - 1, k)))
        tongue_faces.append(((bctr, tv[0][k2], tv[0][k]), (-1, k)))
    for f, (li, k) in tongue_faces:
        faces.append((f, 3, f"tongue_{li}_{k}", None, "tongue"))
    return {"P0": np.array(P0), "P1": np.array(P1), "tags": tags, "faces": faces, "rim": rim, "loop": loop,
            "rim_ring": rim_ring, "grid": grid, "fang_groups": fang_groups,
            "lip_zipper_tris": 0}


# ================================================================ UV
def raster_tris(T, res, owner=None):
    occ = np.zeros((res, res), bool) if owner is None else owner
    for p in T:
        p = p * res
        a2 = (p[1, 0] - p[0, 0]) * (p[2, 1] - p[0, 1]) - (p[1, 1] - p[0, 1]) * (p[2, 0] - p[0, 0])
        if abs(a2) < 1e-12:
            continue
        x0 = max(int(math.floor(p[:, 0].min() - 0.5)), 0)
        x1 = min(int(math.ceil(p[:, 0].max() - 0.5)), res - 1)
        y0 = max(int(math.floor(p[:, 1].min() - 0.5)), 0)
        y1 = min(int(math.ceil(p[:, 1].max() - 0.5)), res - 1)
        if x1 < x0 or y1 < y0:
            continue
        X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        sg = 1.0 if a2 > 0 else -1.0
        ins = np.ones(X.shape, bool)
        for i in range(3):
            q0, q1 = p[i], p[(i + 1) % 3]
            ins &= sg * ((q1[0] - q0[0]) * (Y - q0[1]) - (q1[1] - q0[1]) * (X - q0[0])) >= 0
        occ[y0:y1 + 1, x0:x1 + 1] |= ins
    return occ


def dilate(m, r):
    out = m.copy()
    for _ in range(r):
        p = np.pad(out, 1)
        out = p[1:-1, 1:-1] | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]
    return out


def pack_islands(islands, occ0, density):
    """islands: {name: array (n, 2) local metres}; returns ({name: (scale, offset)}, scale used, tries)."""
    names = sorted(islands, key=lambda n: -np.ptp(islands[n][:, 0]) * np.ptp(islands[n][:, 1]))
    k = density
    for tries in range(30):
        occ = dilate(occ0, UV_MARGIN)
        place = {}
        ok = True
        for n in names:
            P = islands[n]
            lo = P.min(0)
            w = int(math.ceil(np.ptp(P[:, 0]) * k * UV_RES)) + 2 * UV_MARGIN + 1
            h = int(math.ceil(np.ptp(P[:, 1]) * k * UV_RES)) + 2 * UV_MARGIN + 1
            if w >= UV_RES or h >= UV_RES:
                ok = False
                break
            S = np.zeros((UV_RES + 1, UV_RES + 1), np.int64)
            S[1:, 1:] = occ.astype(np.int64).cumsum(0).cumsum(1)
            rs = S[h:, w:] - S[:-h, w:] - S[h:, :-w] + S[:-h, :-w]
            free = np.argwhere(rs == 0)
            if len(free) == 0:
                ok = False
                break
            y, x = free[0]
            occ[y:y + h, x:x + w] = True
            off = np.array([(x + UV_MARGIN + 0.5) / UV_RES, (y + UV_MARGIN + 0.5) / UV_RES]) - lo * k
            place[n] = (k, off)
        if ok:
            return place, k, tries
        k *= 0.85
    raise RuntimeError("UV packing failed")


# ================================================================ main (modeling)
def model():
    if os.path.normcase(os.path.abspath(bpy.data.filepath)) != os.path.normcase(str(IN_BLEND)):
        raise RuntimeError(f"expected input {IN_BLEND}, got {bpy.data.filepath}")
    parts = goblib.load_json("parts.json")
    head_id = parts["head"]
    ob = bpy.data.objects[MESH]
    me = ob.data
    if me.shape_keys is not None or len(me.materials):
        raise RuntimeError("GOB_mesh already has shape keys / materials")
    V, faces, pid = mesh_arrays(me)
    nv0, nf0 = len(V), len(faces)
    me.calc_loop_triangles()
    ntri0 = len(me.loop_triangles)
    tp0 = np.empty(ntri0, dtype=np.int32)
    me.loop_triangles.foreach_get("polygon_index", tp0)
    topo0 = topo_report(V, faces, pid, head_id)
    face = Face(V, faces, pid, head_id)
    det = detect(V, faces, pid, face, head_id)
    region = select_region(V, faces, pid, head_id, det)
    eye_faces = {i for i in face.head_faces if det["eye_v"][list(faces[i])].any()}
    face.build_surface(V, faces, set(region["fang_faces"]))     # T114: eyes kept (the eye-fill plane dented the cheeks)
    _cn = np.empty(len(me.loops) * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", _cn)
    _cn = _cn.reshape(-1, 3)
    face.rec = {p_.index: {int(me.loops[l_].vertex_index): _cn[l_].copy() for l_ in p_.loop_indices}
                for p_ in me.polygons if p_.index in set(face.head_faces)}
    face.fit_patch(V, faces, set(region["fang_faces"]) | eye_faces, pid, head_id)
    print(f"[s02e] PATCH FIT { {k: v for k, v in face.pfit.items() if k not in ('coef', 'exponents_xz')} }")
    rset = set(region["faces"])
    print(f"[s02e] INPUT {goblib._rel(IN_BLEND)} sha256={goblib.sha256(IN_BLEND)}")
    print(f"[s02e] face fit: n={face.fit_n} rms={mm(face.fit_rms)} mm; surface holes filled={face.n_holes_filled}")
    for s_, fd in det["fangs"].items():
        print(f"[s02e] fang {s_}: plate verts={len(fd['plate_verts'])} base x [{mm(fd['base_l'][0])}, "
              f"{mm(fd['base_r'][0])}] z {mm(fd['base_l'][1])} apex ({mm(fd['apex'][0])}, {mm(fd['apex'][1])}) "
              f"front y {mm(fd['front_y'])}")
    for s_, e in det["eyes"].items():
        print(f"[s02e] eye {s_}: centre xz {[mm(v) for v in e['centre_xz']]} r {mm(e['radius'])} z_min {mm(e['z_min'])}")
    print(f"[s02e] mouth centre x {mm(det['cx'])} line z {mm(det['z_m'])} closed half width {mm(det['half_c'])}")
    print(f"[s02e] REGION faces={len(region['faces'])} (fang faces {len(region['fang_faces'])}, hole-filled "
          f"{region['filled_faces']}) region verts={len(region['verts'])} boundary B={len(region['B'])} verts; "
          f"tris removed={int(np.isin(tp0, region['faces']).sum())}")

    # original open positions (jaw field) for head vertices outside the region, per variant
    head_v = np.zeros(nv0, bool)
    head_v[face.head_verts] = True
    reg_v = np.zeros(nv0, bool)
    reg_v[region["verts"]] = True
    mv = head_v & ~reg_v
    geos, V1s, dess = {}, {}, {}
    for var in VARIANTS:
        des = Design(V, face, det, region, var)
        W = np.zeros(nv0)
        W[mv] = des.jaw_w(V[mv])
        V1 = V + W[:, None] * des.J[None, :]
        V1[mv] += des.slide_disp(V[mv])
        sm = var.get("smooth_disp")
        if sm:      # T111e v6: Laplacian smoothing of the displacement field on the original front-face vertices
            #           outside the region below the eyes (zone; eye verts and the bottom zone z < z_fix_below
            #           held), to remove pinches around the mouth
            Dp = V1 - V
            zn_ = sm["zone"]
            fixed = det["eye_v"] | (V[:, 2] < sm["z_fix_below"]) | ~mv | (V[:, 1] > zn_["y_max"])                 | (V[:, 2] > zn_["z_max"]) | (np.abs(V[:, 0] - det["cx"]) > zn_["x"])
            nbr = [[] for _ in range(nv0)]
            for fi_, f_ in enumerate(faces):
                if pid[fi_] != head_id or fi_ in rset:
                    continue
                for k_ in range(len(f_)):
                    a_, b_ = f_[k_], f_[(k_ + 1) % len(f_)]
                    nbr[a_].append(b_)
                    nbr[b_].append(a_)
            idx_s = [v_ for v_ in range(nv0) if mv[v_] and not fixed[v_] and nbr[v_]]
            for _ in range(sm["iters"]):
                Dn = Dp.copy()
                for v_ in idx_s:
                    Dn[v_] = (1 - sm["k"]) * Dp[v_] + sm["k"] * Dp[nbr[v_]].mean(0)
                Dp = Dn
            V1 = V + Dp
        if var.get("flat"):   # T114: original front-face verts (not eyes) keep their closed offset to the fitted surface
            des_ = des
            ff_ = mv & ~det["eye_v"] & (V[:, 1] < FLAT_FOLLOW["y_max"]) & (np.abs(V[:, 0] - det["cx"]) < FLAT_FOLLOW["x"])                 & (V[:, 2] > FLAT_FOLLOW["z"][0]) & (V[:, 2] < FLAT_FOLLOW["z"][1])
            for v_ in np.nonzero(ff_)[0]:
                off = V[v_, 1] - face.S(V[v_, 0], V[v_, 2])
                if abs(off) < FLAT_FOLLOW["max_off"]:
                    V1[v_, 1] = face.S(V1[v_, 0], des_.zsrc(V1[v_, 2])) + off
        V1s[var["name"]] = V1
        geos[var["name"]] = build_geometry(des, V1s[var["name"]])
        dess[var["name"]] = des
    names = [v["name"] for v in VARIANTS]
    geo = geos[names[0]]
    des = dess[names[0]]
    for nm_ in names[1:]:
        g2 = geos[nm_]
        if len(g2["P0"]) != len(geo["P0"]) or float(np.abs(g2["P0"] - geo["P0"]).max()) > 1e-12 \
                or [f[0] for f in g2["faces"]] != [f[0] for f in geo["faces"]]:
            dd = np.abs(g2["P0"] - geo["P0"]).max(1) if len(g2["P0"]) == len(geo["P0"]) else None
            bad = [] if dd is None else [(int(k), geo["tags"][k], float(dd[k])) for k in np.nonzero(dd > 1e-12)[0][:10]]
            raise RuntimeError(f"variant {nm_}: closed geometry / topology differs from {names[0]}: n "
                               f"{len(g2['P0'])}/{len(geo['P0'])} P0 diffs {bad}")
    n_new = len(geo["P0"])
    reuse = region["verts"]
    n_app = max(0, n_new - len(reuse))
    print(f"[s02e] B residual to S (mm): min {mm(des.Bres.min())} max {mm(des.Bres.max())}")
    print(f"[s02e] B phi non-monotone steps={des.B_nonmonotone}; surface fallback points={face.fallback_pts[:20]}")
    print(f"[s02e] NEW verts={n_new} (re-used region slots {min(n_new, len(reuse))}, appended {n_app}); "
          f"faces={len(geo['faces'])}; zipper tris={geo['lip_zipper_tris']}; variants {names} share Basis + topology")
    if n_new < len(reuse):
        raise RuntimeError(f"fewer new verts ({n_new}) than region slots ({len(reuse)}): unused loose verts")

    # ---- recorded original corner normals
    nl = len(me.loops)
    cn = np.empty(nl * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", cn)
    cn = cn.reshape(-1, 3)
    lv = np.empty(nl, dtype=np.int32)
    me.loops.foreach_get("vertex_index", lv)
    ls = np.empty(nf0, dtype=np.int32)
    me.polygons.foreach_get("loop_start", ls)
    lt = np.empty(nf0, dtype=np.int32)
    me.polygons.foreach_get("loop_total", lt)
    rec = {}
    for fi in range(nf0):
        rec[fi] = {int(lv[l]): cn[l].copy() for l in range(ls[fi], ls[fi] + lt[fi])}
    bset = set(region["B"])
    bnorm, bnorm_f = {}, {}
    rfang = set(region["fang_faces"])
    for fi in range(nf0):
        if fi not in rset:
            continue
        for v, n in rec[fi].items():
            if v in bset:
                (bnorm_f if fi in rfang else bnorm).setdefault(v, []).append(n)
    for v, ns in bnorm_f.items():
        bnorm.setdefault(v, ns)
    bnorm = {v: (np.mean(ns, 0) / np.linalg.norm(np.mean(ns, 0))) for v, ns in bnorm.items()}
    fangf = set(region["fang_faces"]) | eye_faces
    vacc = np.zeros((nv0, 3))
    for fi in face.head_faces:
        if fi in fangf:
            continue
        for v, n in rec[fi].items():
            vacc[v] += n
    face.vn = vacc / np.maximum(np.linalg.norm(vacc, axis=1), 1e-20)[:, None]
    face.rec = rec
    # original vertices hidden behind the closed fang plates (footprint + FP_MARGIN, behind the plate): their corner
    # normals were shaded next to the fused fang; they show once the fangs move -> fitted face-surface normal
    fpn = {}
    rverts_ = set(region["verts"])      # T114 fix: region vertices are re-used for new geometry (was: face set)
    for sd_, fd in det["fangs"].items():
        tri = np.array([fd["base_l"], fd["base_r"], fd["apex"]])
        for v in face.head_verts:
            if v in rverts_ or det["eye_v"][v] or V[v, 1] < fd["front_y"] + 0.005 or V[v, 1] > REGION_Y_MAX:
                continue
            q = V[v, [0, 2]]
            if point_tri_dist2d(q, tri) <= FP_MARGIN:
                fpn[int(v)] = face.pn(V[v, 0], V[v, 2])
    lipn = {}
    for k, t in enumerate(geo["tags"]):
        if t.startswith("rim_") or t.startswith("lip_ring"):
            n_ = face.normal(geo["P0"][k][0], geo["P0"][k][2])     # T114: the head's own smooth normal there
            if n_ is not None:
                lipn[k] = n_

    # ---- UV islands
    uv0 = np.empty(nl * 2, dtype=np.float32)
    me.uv_layers.active.uv.foreach_get("vector", uv0)
    uv0 = uv0.reshape(-1, 2).astype(np.float64)
    tl = np.empty(ntri0 * 3, dtype=np.int32)
    me.loop_triangles.foreach_get("loops", tl)
    tl = tl.reshape(-1, 3)
    keep_t = ~np.isin(tp0, region["faces"])
    occ0 = raster_tris(uv0[tl[keep_t]], UV_RES)
    tv = np.empty(ntri0 * 3, dtype=np.int32)
    me.loop_triangles.foreach_get("vertices", tv)
    tv = tv.reshape(-1, 3)
    P3 = V[tv]
    a3 = 0.5 * np.linalg.norm(np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0]), axis=1)
    U = uv0[tl]
    auv = 0.5 * np.abs((U[:, 1, 0] - U[:, 0, 0]) * (U[:, 2, 1] - U[:, 0, 1]) - (U[:, 1, 1] - U[:, 0, 1]) * (U[:, 2, 0] - U[:, 0, 0]))
    hf = np.isin(tp0, face.head_faces) & keep_t
    fn_y = np.array([np.cross(p[1] - p[0], p[2] - p[0])[1] / max(np.linalg.norm(np.cross(p[1] - p[0], p[2] - p[0])), 1e-20) for p in P3])
    hf &= fn_y < -0.7
    density = float(math.sqrt(auv[hf].sum() / a3[hf].sum()))

    def vpos0(ref):
        return geo["P0"][ref[1]] if ref[0] == "n" else V[ref[1]]

    def vpos_uv(ref, grp):
        # fang / tongue faces are unwrapped on their open shape (closed upper fangs / tongue are shrunk)
        if ref[0] == "n" and (grp.startswith("fang_upper") or grp == "tongue"):
            return geo["P1"][ref[1]]
        return vpos0(ref)

    isl_pts = {}
    face_local = []
    for fi, (refs, mat, isl, uvs, grp) in enumerate(geo["faces"]):
        if isl == "lip":
            loc = [(vpos0(r)[0], vpos0(r)[2]) for r in refs]
        elif isl == "cavity":
            loc = list(uvs)
        else:
            P = np.array([vpos_uv(r, grp) for r in refs])
            n = np.cross(P[1] - P[0], P[2] - P[0])
            n /= max(np.linalg.norm(n), 1e-20)
            e1 = (P[1] - P[0]) / max(np.linalg.norm(P[1] - P[0]), 1e-20)
            e2 = np.cross(n, e1)
            loc = [((p - P[0]) @ e1, (p - P[0]) @ e2) for p in P]
        face_local.append(loc)
        isl_pts.setdefault(isl, []).extend(loc)
    isl_pts = {k: np.array(v) for k, v in isl_pts.items()}
    place, k_used, tries = pack_islands(isl_pts, occ0, density)

    # ---- bmesh edit
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    lay_orig = bm.faces.layers.int.new("s02e_orig")
    lay_part = bm.faces.layers.int.get("part_id")
    uvl = bm.loops.layers.uv.active
    for f in bm.faces:
        f[lay_orig] = f.index
    rf = [bm.faces[i] for i in region["faces"]]
    inner_edges = {e for f in rf for e in f.edges if all(lf.index in rset for lf in e.link_faces)}
    bmesh.ops.delete(bm, geom=list(inner_edges) + rf, context="EDGES_FACES")
    bm.verts.ensure_lookup_table()
    if any(len(bm.verts[i].link_edges) for i in reuse):
        raise RuntimeError("region vertex still has edges after the delete")
    vref = {}
    for k in range(n_new):
        if k < len(reuse):
            bv = bm.verts[reuse[k]]
            bv.co = geo["P0"][k]
        else:
            bv = bm.verts.new(geo["P0"][k])
        vref[k] = bv
    bm.verts.ensure_lookup_table()
    bm.verts.index_update()
    idx_new = np.array([vref[k].index for k in range(n_new)])
    new_faces = []
    for fi, (refs, mat, isl, uvs, grp) in enumerate(geo["faces"]):
        vs = [vref[r[1]] if r[0] == "n" else bm.verts[r[1]] for r in refs]
        f = bm.faces.new(vs)
        f[lay_orig] = -1
        f[lay_part] = head_id
        f.material_index = mat
        f.smooth = True
        kk, off = place[isl]
        for l, loc in zip(f.loops, face_local[fi]):
            l[uvl].uv = (loc[0] * kk + off[0], loc[1] * kk + off[1])
        new_faces.append(f)
    bm.normal_update()
    rim_keys = set()
    rr = geo["rim_ring"]
    for q in range(len(rr)):
        rim_keys.add(frozenset((int(idx_new[rr[q][1]]), int(idx_new[rr[(q + 1) % len(rr)][1]]))))
    new_edges = {e for f in new_faces for e in f.edges}
    n_sharp_new = 0
    lip_set = {f for f, g in zip(new_faces, geo["faces"]) if g[4] == "lip"}
    for e in new_edges:
        if len(e.link_faces) == 2 and all(lf[lay_orig] < 0 for lf in e.link_faces):
            key = frozenset((e.verts[0].index, e.verts[1].index))
            both_lip = all(lf in lip_set for lf in e.link_faces)     # T114: skin patch edges stay smooth
            sharp = key in rim_keys or (not both_lip and
                                        e.link_faces[0].normal.angle(e.link_faces[1].normal, 0.0) > SHARP_ANGLE)
            e.smooth = not sharp
            n_sharp_new += int(sharp)
    bm.to_mesh(me)
    bm.free()
    me.update()

    # ---- materials
    for name in MAT_ORDER:
        m = bpy.data.materials.new(name)
        m.diffuse_color = MAT_RGBA[name]
        me.materials.append(m)

    # ---- normals
    nf1, nl1 = len(me.polygons), len(me.loops)
    orig = np.empty(nf1, dtype=np.int32)
    me.attributes["s02e_orig"].data.foreach_get("value", orig)
    me.attributes.remove(me.attributes["s02e_orig"])
    me.normals_split_custom_set([(0.0, 0.0, 0.0)] * nl1)
    auto = np.empty(nl1 * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", auto)
    auto = auto.reshape(-1, 3)
    lv1 = np.empty(nl1, dtype=np.int32)
    me.loops.foreach_get("vertex_index", lv1)
    ls1 = np.empty(nf1, dtype=np.int32)
    me.polygons.foreach_get("loop_start", ls1)
    lt1 = np.empty(nf1, dtype=np.int32)
    me.polygons.foreach_get("loop_total", lt1)
    target = auto.copy()
    n_b_corners = n_lip_corners = n_fp_corners = n_seam_corners = 0
    bseam = {}      # T114: boundary vertices keep the head's own smooth normal (mean of its original corners)
    mat1 = np.empty(nf1, dtype=np.int32)
    me.polygons.foreach_get("material_index", mat1)
    lipn_idx = {int(idx_new[k]): n for k, n in lipn.items()}
    for fi in range(nf1):
        o = orig[fi]
        for l in range(ls1[fi], ls1[fi] + lt1[fi]):
            v = int(lv1[l])
            if mat1[fi] == 0 and v in fpn:
                target[l] = fpn[v]
                n_fp_corners += 1
            elif mat1[fi] == 0 and v in bseam:
                target[l] = bseam[v]
                n_seam_corners += 1
            elif o >= 0:
                target[l] = rec[o][v]
            elif mat1[fi] == 0 and v in bnorm:
                target[l] = bnorm[v]
                n_b_corners += 1
            elif mat1[fi] == 0 and v in lipn_idx:
                target[l] = lipn_idx[v]
                n_lip_corners += 1
    me.normals_split_custom_set(target.tolist())
    got = np.empty(nl1 * 3, dtype=np.float32)
    me.corner_normals.foreach_get("vector", got)
    got = got.reshape(-1, 3)
    old_l = np.repeat(orig >= 0, lt1) & ~np.isin(lv1, list(fpn) + list(bseam))
    ang_err = np.degrees(np.arccos(np.clip(np.sum(got[old_l] * target[old_l], 1), -1, 1)))

    # ---- shape keys (one per variant)
    V2, faces2, pid2 = mesh_arrays(me)
    nv2 = len(V2)
    B0 = V2.copy()
    head_all2 = np.array(sorted({v for i, f in enumerate(faces2) if pid2[i] == head_id for v in f}))
    hz0 = float(B0[head_all2, 2].min())
    hcx = float(0.5 * (B0[head_all2, 0].min() + B0[head_all2, 0].max()))

    def squash(P, sq):
        """Head squash-stretch: z -> z0 + (z - z0) sz (anchored at the head bottom z0), x -> cx + (x - cx) sx_eff with
        sx_eff = 1 - (1 - sx) * smoothstep((z - z0 - start) / fade) (full width kept at the bottom)."""
        P = P.copy()
        f = smoothstep((P[:, 2] - hz0 - sq["start"]) / sq["fade"])
        P[:, 0] = hcx + (P[:, 0] - hcx) * (1.0 - (1.0 - sq["sx"]) * f)
        P[:, 2] = hz0 + (P[:, 2] - hz0) * sq["sz"]
        return P
    Os, Opre = {}, {}
    ob.shape_key_add(name="Basis", from_mix=False)
    for nm_ in names:
        O = np.vstack([V1s[nm_], np.zeros((nv2 - nv0, 3))])
        O[idx_new] = geos[nm_]["P1"]
        sq = next(v for v in VARIANTS if v["name"] == nm_).get("squash")
        Opre[nm_] = O.copy()
        if sq:
            O[head_all2] = squash(O[head_all2], sq)
        Os[nm_] = O
        kb = ob.shape_key_add(name=key_name(nm_), from_mix=False)
        kb.data.foreach_set("co", O.ravel())
        kb.slider_min, kb.slider_max, kb.value = 0.0, 1.0, 0.0
    me.shape_keys.use_relative = True
    fang_all = {int(idx_new[r[1]]) for refs in geo["fang_groups"].values() for r in refs}

    # lip faces whose Basis normal does not face the front (folds)
    lip_back = []
    for fi in range(nf1):
        if orig[fi] < 0 and mat1[fi] == 0:
            f = faces2[fi]
            P = B0[list(f)]
            a_ = np.zeros(3)
            for k_ in range(1, len(f) - 1):
                a_ += np.cross(P[k_] - P[0], P[k_ + 1] - P[0])
            if np.linalg.norm(a_) > 0 and a_[1] / np.linalg.norm(a_) > -0.2:
                lip_back.append(fi)
    print(f"[s02e] LIP faces not facing front at Basis (normal y > -0.2): {len(lip_back)}")

    # ---- T122 (G2.9): the whole GOB_mesh UV layout is redone with s02d's method now that the mouth exists
    # (Smart UV project -> seams -> angle-based unwrap -> average island scale -> islands holding head-front faces
    # (head part, normal . -Y > HEAD_FRONT_DOT, the same rule as check_g2_static) x HEAD_FRONT_SCALE -> pack);
    # geometry, shape keys, materials and normals are not touched
    import s02d_finish as D
    n_front_isl = D.make_uvs(ob, head_front_value=head_id)
    uvr_full = D.uv_report(ob, head_id, parts["body"])
    uvr_full.pop("_owner", None)
    print(f"[s02e] UV relayout (s02d make_uvs): head-front islands scaled={n_front_isl} (x{D.HEAD_FRONT_SCALE} linear) "
          f"{uvr_full}")

    # ---- save
    OUT_BLEND.parent.mkdir(parents=True, exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)

    # ---- measures (rest)
    keep = np.ones(nv2, bool)
    keep[idx_new] = False
    keep[nv0:] = False
    dev = float(np.abs(B0[:nv0][keep[:nv0]] - V[keep[:nv0]]).max())
    topo1 = topo_report(B0, faces2, pid2, head_id)
    me.calc_loop_triangles()
    tp1 = np.empty(len(me.loop_triangles), dtype=np.int32)
    me.loop_triangles.foreach_get("polygon_index", tp1)
    head_tris0 = int(np.isin(tp0, face.head_faces).sum())
    head_tris1 = int((pid2[tp1] == head_id).sum())
    mat_idx = mat1
    groups = {}
    for k, t in enumerate(geo["tags"]):
        g = t if not t.startswith("lip_ring") else "lip_ring"
        groups.setdefault(g, []).append(int(idx_new[k]))
    for g, refs in geo["fang_groups"].items():
        groups[g] = [int(idx_new[r[1]]) for r in refs]
    groups = {g: sorted(set(v)) for g, v in groups.items()}
    fang_ids = sorted(fang_all)
    groups["cavity"] = sorted(set(groups.get("cavity", [])) - set(fang_ids))
    # cavity + closed upper fangs + tongue inside the head skin at 0
    skin_faces = [f for i, f in enumerate(faces2) if pid2[i] == head_id and mat_idx[i] == 0]
    tree_skin = BVHTree.FromPolygons(B0.tolist(), skin_faces)
    hid_ids = groups["cavity"] + groups["fang_upper_l"] + groups["fang_upper_r"] + groups["tongue"]
    outside_ids = [v for v in hid_ids if not inside(tree_skin, B0[v])]
    cam = goblib.load_json("swing_cam.json")["front"]
    rim_all = set(groups["rim_upper"]) | set(groups["rim_lower"]) | set(groups["rim_corner"])
    skin_v = np.array(sorted({v for i, f in enumerate(faces2) if pid2[i] == head_id and mat_idx[i] == 0 for v in f}
                             - rim_all))
    vnb = np.empty(nv2 * 3)
    me.vertices.foreach_get("normal", vnb)
    vnb = vnb.reshape(-1, 3)
    fsel = skin_v[(vnb[skin_v, 1] < -0.7) & (np.abs(B0[skin_v, 2] - det["z_m"]) < 0.05)]
    face_w = float(np.ptp(B0[fsel, 0]))
    skin_faces_b = [f for i, f in enumerate(faces2) if pid2[i] == head_id and mat_idx[i] == 0]
    tree_b = BVHTree.FromPolygons(B0.tolist(), skin_faces_b)

    def protrusion(P):
        """Per vertex: basis head skin surface y (first +Y ray hit at the vertex x, z) - vertex y (> 0 = ahead)."""
        out = np.zeros(len(P))
        for k, p in enumerate(P):
            h = tree_b.ray_cast(Vector((p[0], -2.0, p[2])), Vector((0.0, 1.0, 0.0)))
            out[k] = (h[0].y - p[1]) if h[0] is not None else 0.0
        return out
    lower_skin = skin_v[B0[skin_v, 2] < det["z_m"]]
    eye_ids = np.nonzero(det["eye_v"])[0]
    # T111e surface metrics (report-only): closed patch skin -> original surface (fang / eye-free, fan-filled);
    # open: lip-ring skin verts (outside the rim) -> fitted surface pf at the un-squashed, un-remapped location
    patch_ids = np.array(sorted(set(groups["rim_upper"]) | set(groups["rim_lower"]) | set(groups["rim_corner"])
                                | set(groups["lip_ring"])))
    dcl = np.array([(Vector(B0[v]) - face.tree.find_nearest(Vector(B0[v]))[0]).length for v in patch_ids])
    closed_dist = {"n": int(len(patch_ids)), "max_mm": float(dcl.max() * 1000), "p95_mm": float(np.percentile(dcl, 95) * 1000),
                   "note": "Basis rim + lip-ring verts, nearest distance to the original head surface without the "
                           "fang / eye protrusions (their holes fan-filled)"}
    ring_ids = np.array(groups["lip_ring"])

    def unsquash(P, sq):
        P = P.copy()
        P[:, 2] = hz0 + (P[:, 2] - hz0) / sq["sz"]
        f = smoothstep((P[:, 2] - hz0 - sq["start"]) / sq["fade"])
        P[:, 0] = hcx + (P[:, 0] - hcx) / (1.0 - (1.0 - sq["sx"]) * f)
        return P
    def open_fit_dist(nm_):
        var_ = next(v for v in VARIANTS if v["name"] == nm_)
        P = Os[nm_][ring_ids]
        if var_.get("squash"):
            P = unsquash(P, var_["squash"])
        d_ = np.array([abs(p[1] - face.pf(p[0], dess[nm_].zsrc(p[2]))) for p in P])
        return {"n": int(len(ring_ids)), "max_mm": float(d_.max() * 1000), "p95_mm": float(np.percentile(d_, 95) * 1000)}
    per_var = {}
    for nm_ in names:
        O = Os[nm_]
        disp = np.linalg.norm(O - B0, axis=1)
        dy = O[skin_v, 1] - B0[skin_v, 1]
        rim_l = sorted(rim_all)
        per_var[nm_] = {
            "variant": next(v for v in VARIANTS if v["name"] == nm_),
            "key": key_name(nm_),
            "topology_open": topo_report(O, faces2, pid2, head_id),
            "max_disp_m": float(disp.max()), "max_disp_vertex": int(disp.argmax()),
            "max_disp_original_outside_region_m": float(disp[:nv0][keep[:nv0]].max()),
            "moved_original_outside_region": int((disp[:nv0][keep[:nv0]] > 1e-9).sum()),
            "per_group_max_m": {g: float(disp[v].max()) for g, v in groups.items()},
            "skin_outside_rim_dy_m": {"max_forward_minus_y": float(max(0.0, -dy.min())),
                                      "max_backward_plus_y": float(max(0.0, dy.max())),
                                      "vertex_forward": int(skin_v[np.argmin(dy)]), "n_verts": int(len(skin_v))},
            "open_ring_dist_to_fitted_surface_mm": open_fit_dist(nm_),
            "skin_protrusion_ahead_of_basis_surface_pre_squash_m": float(protrusion(Opre[nm_][skin_v]).max()),
            "profile_frontmost_y_shift_m": {"all_skin": float(O[skin_v, 1].min() - B0[skin_v, 1].min()),
                                            "below_mouth_line": float(O[lower_skin, 1].min() - B0[lower_skin, 1].min())},
            "head_bbox_scale": {"x": float(np.ptp(O[head_all2, 0]) / np.ptp(B0[head_all2, 0])),
                                "y": float(np.ptp(O[head_all2, 1]) / np.ptp(B0[head_all2, 1])),
                                "z": float(np.ptp(O[head_all2, 2]) / np.ptp(B0[head_all2, 2])),
                                "z_min_shift_m": float(O[head_all2, 2].min() - B0[head_all2, 2].min())},
            "eye_lift_m": {"pre_squash_mean_dz": float((Opre[nm_][eye_ids, 2] - B0[eye_ids, 2]).mean()),
                           "pre_squash_dz_range": [float((Opre[nm_][eye_ids, 2] - B0[eye_ids, 2]).min()),
                                                   float((Opre[nm_][eye_ids, 2] - B0[eye_ids, 2]).max())],
                           "final_mean_dz": float((O[eye_ids, 2] - B0[eye_ids, 2]).mean())},
            "mouth_rest_wh_m": [float(np.ptp(O[rim_l, 0])), float(np.ptp(O[rim_l, 2]))],
            "mouth_w_over_face_front_w": float(np.ptp(O[rim_l, 0]) / face_w),
            "face_front_width_m": face_w,
            "measure_rest": front_measure(cam, B0, O, faces2, pid2, head_id, parts, groups, geo, vref, idx_new)}
    uv_rep = {"method": "T122: s02d_finish.make_uvs on the whole GOB_mesh after the mouth (Smart UV project "
                        f"{round(math.degrees(D.UV_ANGLE_LIMIT))} deg -> seams from islands -> angle-based unwrap -> "
                        "average island scale -> islands holding head-front faces (head part, normal . -Y > "
                        f"{D.HEAD_FRONT_DOT}) x {D.HEAD_FRONT_SCALE} linear -> pack, margin {D.PACK_MARGIN}); the "
                        "mouth-local islands placed above are replaced",
              "head_front_islands_scaled": n_front_isl, "report_s02d_uv_report": uvr_full,
              "mouth_local_packing_replaced": {"islands": len(place), "density_used": k_used, "shrink_retries": tries}}
    newt = orig[tp1] < 0

    counts = {"before": {"verts": nv0, "faces": nf0, "tris": ntri0, "head_faces": len(face.head_faces),
                         "head_tris": head_tris0, "head_verts": int(len(face.head_verts))},
              "after": {"verts": nv2, "faces": nf1, "tris": len(me.loop_triangles),
                        "head_faces": int((pid2 == head_id).sum()), "head_tris": head_tris1},
              "delta_tris": len(me.loop_triangles) - ntri0,
              "region_tris_removed": int(np.isin(tp0, region["faces"]).sum()),
              "new_tris": int(newt.sum())}
    doc = {
        "_doc": "T111b / d-29 R1: GOB_mesh mouth (s02e_mouth.py). Vertex indices refer to GOB_mesh of "
                "rig/%s. Original vertices 0..%d keep their indices; region vertices are "
                "re-used for new geometry; appended verts %d..%d. Basis = closed; one open key per variant "
                "(%s); the chosen one is renamed mouth_open later." % (goblib._rel(OUT_BLEND), nv0 - 1, nv0, nv2 - 1,
                                                                        ", ".join(key_name(n) for n in names)),
        "input": {"path": goblib._rel(IN_BLEND), "sha256": goblib.sha256(IN_BLEND)},
        "output": {"blend": goblib._rel(OUT_BLEND), "sha256": goblib.sha256(OUT_BLEND)},
        "region": {"definition": {"part": "head", "all_verts": {"y_max": REGION_Y_MAX, "abs_x_minus_cx_max": REGION_X,
                                                                  "z_range": [REGION_Z_MIN, REGION_Z_MAX],
                                                                  "not_eye_protrusion": {"disc_r": EYE_DISC, "residual_lt": -EYE_RES},
                                                                  "not_eye_undercut": REGION_UNDERCUT},
                                   "face_normal_y_max": REGION_NY,
                                   "plus_fang_faces": {"zone": FANG_ZONE, "residual_lt": -FANG_RES},
                                   "component": "holding the mouth centre, enclosed holes filled",
                                   "fit": {"terms": "1 x z x2 xz z2 x3 x2z xz2 z3 x4 x2z2 z4", "coef": face.c,
                                           "n": face.fit_n, "rms_m": face.fit_rms}},
                   "faces_removed": len(region["faces"]), "fang_faces": len(region["fang_faces"]),
                   "verts": region["verts"], "boundary_loop": region["B"],
                   # HR2 contract: world-space region (GOB_mesh identity) and the output faces / verts inside it
                   "boundary_loop_co": np.round(V[region["B"]], 7).tolist(),
                   "removed_region_verts_co": np.round(V[region["verts"]], 7).tolist(),
                   "output_faces": {"all_new": [int(i) for i in np.nonzero(orig < 0)[0]],
                                    "by_group": {g: [int(i) for i in np.nonzero((orig < 0) & (mat1 == k))[0]]
                                                 for k, g in enumerate(("lip_patch", "cavity", "teeth", "tongue"))}},
                   "output_verts": sorted({int(i) for i in idx_new} | set(range(nv0, nv2))),
                   "output_note": "indices in GOB_mesh of the output blend; all_new = every face inside the boundary "
                                  "loop (the rebuilt patch + cavity + teeth + tongue, all part_id head); by_group "
                                  "follows the material slot (0 skin = lip patch, 1 cavity, 2 teeth, 3 tongue)"},
        "detected": {"cx": det["cx"], "z_m": det["z_m"], "closed_half_width": det["half_c"],
                     "eye_verts": [int(i) for i in np.nonzero(det["eye_v"])[0]],
                     "fangs": {k: {kk: vv for kk, vv in v.items() if kk != "plate_verts"} for k, v in det["fangs"].items()},
                     "eyes": det["eyes"]},
        "design": {"gap": GAP, "n_seg": N_SEG, "ring_t": RING_T, "variants": list(VARIANTS),
                   "jaw_field": {"x0x1": [JAW_X0, JAW_X1], "y0y1": [JAW_Y0, JAW_Y1], "ang0_deg": math.degrees(JAW_ANG0)},
                   "pocket_closed": POCKET_CLOSED, "pocket_open": POCKET_OPEN,
                   "fang": {"closed_depth": FANG_DEPTH, "open_depth": FANG_OPEN_DEPTH, "embed": FANG_EMBED,
                            "proud": FANG_PROUD, "upper_closed_wh": [UF_CLOSED_W, UF_CLOSED_H]},
                   "tongue": {"segments": TONGUE_NS, "depth_r": TONGUE_DEPTH_R, "height": TONGUE_H, "back": TONGUE_BACK,
                              "base": TONGUE_BASE, "closed": TONGUE_CLOSED},
                   "roar_head_x_deg": ROAR_HEAD_X},
        "new_vertices": {"reused_region_slots": [int(i) for i in reuse[:n_new]], "appended_range": [nv0, nv2 - 1],
                         "groups": groups},
        "counts": counts,
        "topology": {"before": topo0, "after_basis": topo1},
        "basis_deviation_outside_region_m": dev,
        "patch_closed_dist_to_original": closed_dist,
        "patch_fit": face.pfit,
        "head_squash_anchor": {"z0": hz0, "cx": hcx},
        "lip_faces_not_front_at_basis": len(lip_back),
        "variants": per_var,
        "materials": {"slots": list(MAT_ORDER), "rgba": MAT_RGBA,
                      "faces_per_slot": [int((mat_idx == k).sum()) for k in range(len(MAT_ORDER))],
                      "note": "the input mesh had no material; slot 0 GOB_skin was added for the other faces; the two "
                              "closed lower fangs are now GOB_teeth (cream)"},
        "normals": {"original_corners_angle_err_deg_max": float(ang_err.max()), "new_sharp_edges": n_sharp_new,
                    "new_face_corners_on_B_given_original_normal": n_b_corners,
                    "new_lip_corners_given_interpolated_original_normal": n_lip_corners,
                    "fang_footprint_verts_renormalised": sorted(fpn), "fang_footprint_corners": n_fp_corners,
                    "boundary_seam_verts_fitted_normal": sorted(bseam), "boundary_seam_corners": n_seam_corners,
                    "note_fp": "original vertices behind the closed fang plates (footprint + FP_MARGIN): corner "
                               "normals set to the fitted face-surface normal (hidden at Basis by the fangs; "
                               "original_corners_angle_err excludes them)"},
        "uv": uv_rep,
        "hidden_at_0": {"checked_verts": len(hid_ids), "outside_head_skin": len(outside_ids),
                        "outside_ids": outside_ids[:50],
                        "note": "cavity + closed upper fang + tongue verts, ray parity (3 dirs, majority) against the "
                                "head skin faces (material 0); the rim loop is on the skin"},
    }
    print(f"[s02e] OUTPUT blend={goblib._rel(OUT_BLEND)}")
    print(f"[s02e] COUNTS {counts}")
    print(f"[s02e] TOPOLOGY before {topo0}")
    print(f"[s02e] TOPOLOGY after basis {topo1}")
    print(f"[s02e] BASIS deviation of original verts outside the region: max {dev:.3e} m")
    print(f"[s02e] PATCH closed distance to the original surface {closed_dist}")
    for nm_, r in per_var.items():
        print(f"[s02e] {r['key']}: open topology {r['topology_open']}; max disp {mm(r['max_disp_m'])} mm; originals "
              f"outside region moved {r['moved_original_outside_region']} max {mm(r['max_disp_original_outside_region_m'])} mm")
        print(f"[s02e] {r['key']} FRONT (rest) {r['measure_rest']['open']}")
        print(f"[s02e] {r['key']} open lip-ring distance to the fitted surface {r['open_ring_dist_to_fitted_surface_mm']}")
        print(f"[s02e] {r['key']} protrusion ahead of the basis surface (pre-squash) "
              f"{mm(r['skin_protrusion_ahead_of_basis_surface_pre_squash_m'])} mm; profile frontmost y shift "
              f"{ {k: mm(v) for k, v in r['profile_frontmost_y_shift_m'].items()} } mm")
        print(f"[s02e] {r['key']} skin dy {r['skin_outside_rim_dy_m']}; head bbox scale {r['head_bbox_scale']}; mouth rest "
              f"w x h {[mm(v) for v in r['mouth_rest_wh_m']]} mm, w / face front width ({mm(face_w)} mm) "
              f"{r['mouth_w_over_face_front_w']:.3f}; eye lift {r['eye_lift_m']}")
    print(f"[s02e] MATERIALS {doc['materials']['slots']} faces {doc['materials']['faces_per_slot']}")
    print(f"[s02e] NORMALS {doc['normals']}")
    print(f"[s02e] UV {uv_rep}")
    print(f"[s02e] HIDDEN at 0: {len(outside_ids)} of {len(hid_ids)} cavity / closed upper fang / tongue verts outside the head skin")
    return doc, V, nv0, idx_new, reuse


def front_measure(cam, B0, O, faces, pid, head_id, parts, groups, geo, vref, idx_new, body_tree=None):
    """Front camera: head width, visible head height (lowest head skin vert not occluded by / inside the body),
    open mouth width / height (rim loop), in px and ratios; face front width."""
    head_skin = sorted({v for i, f in enumerate(faces) if pid[i] == head_id for v in f}
                       - set(groups.get("cavity", [])) - set(groups.get("tongue", []))
                       - {v for g, vs in groups.items() if g.startswith("fang_") for v in vs})
    body_faces = [f for i, f in enumerate(faces) if pid[i] == parts["body"]]
    out = {}
    d = np.array(cam["view_dir"])
    rim_ids = sorted(set(groups["rim_upper"]) | set(groups["rim_lower"]) | set(groups["rim_corner"]))
    for st, P in (("closed", B0), ("open", O)):
        tree = BVHTree.FromPolygons(P.tolist(), body_faces)
        H = P[head_skin]
        uv, spx = cam_project(cam, H)
        vis = []
        for k, p in enumerate(H):
            hit = tree.ray_cast(Vector(p) - Vector(d) * 1e-5, Vector(-d))
            vis.append(hit[0] is None)
        vis = np.array(vis)
        top = uv[:, 1].min()
        bot = uv[vis, 1].max()
        hw = np.ptp(uv[:, 0])
        r_uv, _ = cam_project(cam, P[rim_ids])
        mw, mh = np.ptp(r_uv[:, 0]), np.ptp(r_uv[:, 1])
        out[st] = {"head_width_px": float(hw), "head_visible_height_px": float(bot - top),
                   "mouth_width_px": float(mw), "mouth_height_px": float(mh),
                   "mouth_w_over_head_w": float(mw / hw), "mouth_h_over_head_visible_h": float(mh / (bot - top)),
                   "px_per_m": float(spx)}
    out["note"] = "swing_cam.json front, 896x1152 px; mouth = rim loop extents; visible head height = head top to " \
                  "the lowest head skin vertex whose ray to the camera misses the body"
    return out


# ================================================================ renders / ready pose
def resize(img, h):
    H, W = img.shape[:2]
    w = max(1, int(round(W * h / H)))
    ys = (np.arange(h) + 0.5) * H / h - 0.5
    xs = (np.arange(w) + 0.5) * W / w - 0.5
    y0 = np.clip(np.floor(ys).astype(int), 0, H - 1)
    x0 = np.clip(np.floor(xs).astype(int), 0, W - 1)
    y1 = np.clip(y0 + 1, 0, H - 1)
    x1 = np.clip(x0 + 1, 0, W - 1)
    fy = np.clip(ys - y0, 0, 1)[:, None, None]
    fx = np.clip(xs - x0, 0, 1)[None, :, None]
    if H > 2 * h:   # pre-average for strong downscale
        k = H // h
        img = img[:H // k * k, :W // k * k].reshape(H // k, k, W // k, k, -1).mean((1, 3))
        return resize(img, h)
    a = img[y0][:, x0] * (1 - fx) + img[y0][:, x1] * fx
    b = img[y1][:, x0] * (1 - fx) + img[y1][:, x1] * fx
    return a * (1 - fy) + b * fy


def pose_phase(doc, V_orig, nv0, idx_new, reuse, do_render):
    """gob_r04_ctrl.blend in memory (never saved): new mesh data + weights, poses 'ready' (goblin.reset_rig +
    goblin.ready_pose + data/clips/idle.json pose frame 1, with the club) and 'roar' (the same + CTRL_head rot X
    ROAR_HEAD_X); per pose and variant: body penetration, front-camera measures, renders."""
    import s10a_swing_keyposes as S
    bpy.ops.wm.open_mainfile(filepath=str(CTRL_BLEND))
    parts = goblib.load_json("parts.json")
    head_id = parts["head"]
    rig = bpy.data.objects[S.ARM]
    gm = bpy.data.objects[S.MESH]
    old = gm.data
    Vc = np.empty(len(old.vertices) * 3)
    old.vertices.foreach_get("co", Vc)
    Vc = Vc.reshape(-1, 3)
    same = len(Vc) == nv0 and float(np.abs(Vc - V_orig).max()) < 1e-7
    gnames = [g.name for g in gm.vertex_groups]
    wts = [[(g.group, g.weight) for g in v.groups] for v in old.vertices]
    with bpy.data.libraries.load(str(OUT_BLEND), link=False) as (src, dst):
        dst.meshes = [MESH]
    new_me = dst.meshes[0]
    gm.data = new_me
    for name in gnames:
        if name not in gm.vertex_groups:
            gm.vertex_groups.new(name=name)
    gidx = {i: gm.vertex_groups[n].index for i, n in enumerate(gnames)}
    nv = len(new_me.vertices)
    Vn = np.empty(nv * 3)
    new_me.vertices.foreach_get("co", Vn)
    Vn = Vn.reshape(-1, 3)
    pid = np.empty(len(new_me.polygons), dtype=np.int32)
    new_me.attributes["part_id"].data.foreach_get("value", pid)
    # T114: gob_r04_ctrl still carries the pre-HR mesh, so weights come from the nearest vertex of the same part
    # (part of the vertex's first face) in r04's GOB_mesh at rest (head part: DEF head 100 % there)
    from mathutils.kdtree import KDTree
    old_pid = np.empty(len(old.polygons), dtype=np.int32)
    old.attributes["part_id"].data.foreach_get("value", old_pid)
    vpart_old = np.full(len(Vc), -1)
    for p_ in old.polygons:
        for v_ in p_.vertices:
            if vpart_old[v_] < 0:
                vpart_old[v_] = old_pid[p_.index]
    vpart_new = np.full(nv, -1)
    for p_ in new_me.polygons:
        for v_ in p_.vertices:
            if vpart_new[v_] < 0:
                vpart_new[v_] = pid[p_.index]
    kds = {}
    for part_ in set(vpart_old.tolist()):
        ids_ = np.nonzero(vpart_old == part_)[0]
        kd = KDTree(len(ids_))
        for v_ in ids_:
            kd.insert(Vector(Vc[v_]), int(v_))
        kd.balance()
        kds[part_] = kd
    per_group = {}
    wdist = []
    for v in range(nv):
        _co, src_v, dd = kds[int(vpart_new[v])].find(Vector(Vn[v]))
        wdist.append(dd)
        for g, w in wts[src_v]:
            per_group.setdefault((g, w), []).append(v)
    for (g, w), vs in per_group.items():
        gm.vertex_groups[gidx[g]].add(vs, w, "REPLACE")
    wdist = np.array(wdist)
    # rig
    sc = bpy.context.scene
    bpy.context.view_layer.objects.active = rig
    rig.data.pose_position = "POSE"
    mod = bpy.data.texts[S.TEXT_UI].as_module()
    mod.register()
    sc.tool_settings.use_keyframe_insert_auto = False
    if rig.animation_data is not None:
        rig.animation_data.action = None
    for op in ("reset_rig", "ready_pose"):
        r = getattr(bpy.ops.goblin, op)()
        if "FINISHED" not in r:
            raise RuntimeError(f"goblin.{op} returned {r}")
    S.refresh(rig)
    base_c, base_p = S.snapshot(rig)
    man = goblib.load_json("ctrl_manifest.json")
    idle = goblib.load_json("clips/idle.json")
    f1 = next(p for p in idle["poses"] if int(p["frame"]) == 1)
    ctrl_ready = {k: dict(v) for k, v in f1.get("ctrl_overrides", {}).items()}
    ctrl_roar = {k: dict(v) for k, v in ctrl_ready.items()}
    hd = ctrl_roar.get("CTRL_head", {"loc": [0.0, 0.0, 0.0], "rot": [0.0, 0.0, 0.0]})
    hd["rot"] = [ROAR_HEAD_X, float(hd.get("rot", [0, 0, 0])[1]), float(hd.get("rot", [0, 0, 0])[2])]
    ctrl_roar["CTRL_head"] = hd
    poses = {"ready": ctrl_ready, "roar": ctrl_roar}
    props = f1.get("props_overrides", {})
    keys = new_me.shape_keys.key_blocks
    names = [v["name"] for v in VARIANTS]

    def set_state(pose, var):
        S.apply_state(rig, man, base_c, base_p, {"name": f"R1 {pose}", "ctrl": poses[pose], "props": props})
        for nm_ in names:
            keys[key_name(nm_)].value = 1.0 if nm_ == var else 0.0
        S.refresh(rig)

    def evaluated():
        dg = bpy.context.evaluated_depsgraph_get()
        me_e = gm.evaluated_get(dg).data
        P = np.empty(len(me_e.vertices) * 3)
        me_e.vertices.foreach_get("co", P)
        return P.reshape(-1, 3) @ np.array(gm.matrix_world)[:3, :3].T + np.array(gm.matrix_world)[:3, 3]

    states = {}
    for pose in poses:
        for var in [None] + names:
            set_state(pose, var)
            states[(pose, var)] = evaluated()
    faces = [tuple(p.vertices) for p in new_me.polygons]
    body_faces = [f for i, f in enumerate(faces) if pid[i] == parts["body"]]
    head_all = sorted({v for i, f in enumerate(faces) if pid[i] == head_id for v in f})
    groups = doc["new_vertices"]["groups"]
    gof = {}
    for g, vs in groups.items():
        for v in vs:
            gof[v] = g
    cam = goblib.load_json("swing_cam.json")
    res = {}
    for pose in poses:
        S0 = states[(pose, None)]
        t0 = BVHTree.FromPolygons(S0.tolist(), body_faces)
        for nm_ in names:
            S1 = states[(pose, nm_)]
            t1 = BVHTree.FromPolygons(S1.tolist(), body_faces)
            newly, depth = [], []
            for v in head_all:
                if np.linalg.norm(S1[v] - S0[v]) <= 1e-7:
                    continue
                if inside(t1, S1[v]) and not inside(t0, S0[v]):
                    newly.append(v)
                    depth.append(t1.find_nearest(Vector(S1[v]))[3])
            by_group = {}
            for v, dd in zip(newly, depth):
                by_group.setdefault(gof.get(v, "original_head" if v < nv0 else "new"), []).append(dd)
            pen = {"newly_inside_body": len(newly), "max_depth_m": float(max(depth)) if depth else 0.0,
                   "by_group": {g: {"n": len(d), "max_depth_m": float(max(d))} for g, d in by_group.items()},
                   "vertices": newly[:60]}
            meas = front_measure(cam["front"], S0, S1, faces, pid, head_id, parts, groups, None, None, idx_new)
            res.setdefault(nm_, {})[pose] = {"penetration": pen, "measure_front": meas}
            print(f"[s02e] {key_name(nm_)} {pose}: PENETRATION {pen['newly_inside_body']} max "
                  f"{mm(pen['max_depth_m'])} mm {dict((g, d['n']) for g, d in pen['by_group'].items())}; FRONT open "
                  f"w/headw {meas['open']['mouth_w_over_head_w']:.3f} h/head_h_vis "
                  f"{meas['open']['mouth_h_over_head_visible_h']:.3f} (mouth {meas['open']['mouth_width_px']:.1f} x "
                  f"{meas['open']['mouth_height_px']:.1f} px, h/w {meas['open']['mouth_height_px'] / meas['open']['mouth_width_px']:.3f})")
    doc["poses"] = {"weights_from_r04_by_index": same,
                    "weights_nearest_same_part": {"max_dist_m": float(wdist.max()),
                                                  "p95_dist_m": float(np.percentile(wdist, 95))}, "ready": "goblin.reset_rig + goblin.ready_pose + "
                    "data/clips/idle.json pose frame 1 (ctrl + props overrides)",
                    "roar": f"ready + CTRL_head rot X {ROAR_HEAD_X} deg",
                    "penetration_note": "head verts that move with the key and are inside the body shell (ray parity, "
                                        "3 dirs, majority) with the key at 1 but not at 0, same pose; depth = distance "
                                        "to the body surface",
                    "per_variant": res}
    print(f"[s02e] POSES r04 GOB_mesh matches r01 by index: {same}; groups copied={len(gnames)}")
    if not do_render:
        return doc
    # ---- renders (T114): states closed (ready) and open (roar-like); plain + matcap, 5 views
    S.setup_render()
    sh = sc.display.shading
    sh.color_type = "MATERIAL"
    for m in new_me.materials:
        base = next(n for n in MAT_ORDER if m.name == n or m.name.startswith(n + "."))
        m.diffuse_color = MAT_RGBA[base]
    club = bpy.data.objects[S.CLUB]
    cm = bpy.data.materials.new("_r1_club")
    cm.diffuse_color = S.CLUB_RGBA
    club.data.materials.append(cm)
    INSP.mkdir(parents=True, exist_ok=True)
    hv = np.array(head_all)

    def cam_mw(c, center, back=6.0):
        M = np.array(c["matrix_world"])
        d = np.array(c["view_dir"])
        M[:3, 3] = center - d * back
        return M

    def dir_mw(d, center, back=6.0):
        """Camera looking along d (world up +Z): columns right = d x Z, up = right x d, -d."""
        d = np.asarray(d, float) / np.linalg.norm(d)
        rt = np.cross(d, [0.0, 0.0, 1.0])
        rt /= np.linalg.norm(rt)
        up = np.cross(rt, d)
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = rt, up, -d, center - d * back
        return M
    vtq = np.array(cam["three_quarter"]["view_dir"])
    d_tql = np.array([-vtq[0], vtq[1], vtq[2]])        # 3/4 from the character's left (+X)
    d_side = np.array([-1.0, 0.0, 0.0])                # side profile from the character's left (+X)

    def shot(mw, ortho, res_, out):
        r = sc.render
        r.resolution_x, r.resolution_y = res_
        cd = bpy.data.cameras.new("_r1_cam")
        cd.type, cd.sensor_fit, cd.ortho_scale = "ORTHO", "VERTICAL", ortho
        cd.clip_start, cd.clip_end = 0.01, 20.0
        co = bpy.data.objects.new("_r1_cam", cd)
        sc.collection.objects.link(co)
        co.matrix_world = Matrix(mw.tolist())
        sc.camera = co
        try:
            bpy.ops.render.render(write_still=False)
            bpy.data.images["Render Result"].save_render(str(out))
        finally:
            bpy.data.objects.remove(co, do_unlink=True)
            bpy.data.cameras.remove(cd)
        img = S.over_white(S.load_rgba(out))
        S.save_rgb(img, out)
        return img

    mats = [l_.name for l_ in bpy.context.preferences.studio_lights if l_.type == "MATCAP"]
    mc = "check_reflection_horizontal.exr" if "check_reflection_horizontal.exr" in mats else (mats[0] if mats else None)

    def views(tag, center, full=True):
        out = {"fc": shot(cam_mw(cam["front"], center), 0.72, (700, 600), INSP / f"hr1_{tag}_front_close.png"),
               "tl": shot(dir_mw(d_tql, center), 0.72, (700, 600), INSP / f"hr1_{tag}_tq_left_close.png"),
               "tc": shot(cam_mw(cam["three_quarter"], center), 0.72, (700, 600), INSP / f"hr1_{tag}_tq_right_close.png"),
               "sd": shot(dir_mw(d_side, center), 0.72, (700, 600), INSP / f"hr1_{tag}_side_close.png")}
        if full:
            out["ff"] = shot(np.array(cam["front"]["matrix_world"]), cam["front"]["ortho_scale"],
                             tuple(cam["front"]["resolution"]), INSP / f"hr1_{tag}_full_front.png")
        if mc:
            light0, sl0, ct0, sc0 = sh.light, sh.studio_light, sh.color_type, tuple(sh.single_color)
            sh.light, sh.studio_light, sh.color_type, sh.single_color = "MATCAP", mc, "SINGLE", (0.8, 0.8, 0.8)
            out["mc"] = shot(cam_mw(cam["front"], center), 0.72, (700, 600), INSP / f"hr1_{tag}_front_close_matcap.png")
            shot(dir_mw(d_tql, center), 0.72, (700, 600), INSP / f"hr1_{tag}_tq_left_close_matcap.png")
            shot(cam_mw(cam["three_quarter"], center), 0.72, (700, 600), INSP / f"hr1_{tag}_tq_right_close_matcap.png")
            sh.light, sh.studio_light, sh.color_type, sh.single_color = light0, sl0, ct0, sc0
        return out

    imgs, dark = {}, {}
    nm1 = names[0]
    for pose, var, tag in (("ready", None, "closed"), ("roar", nm1, "open")):
        allz = np.vstack([states[(pose, v)][hv] for v in [None] + names])
        Hc = 0.5 * (allz.min(0) + allz.max(0))
        set_state(pose, var)
        imgs[tag] = views(tag, Hc)
        ff = imgs[tag]["ff"]
        huv, _ = cam_project(cam["front"], states[(pose, var)][hv])
        u0, v0 = np.floor(huv.min(0)).astype(int)
        u1, v1 = np.ceil(huv.max(0)).astype(int)
        box = ff[v0:v1 + 1, u0:u1 + 1]
        dm = box.sum(2) < 0.25
        tg = (box[:, :, 0] > 1.6 * box[:, :, 1]) & (box[:, :, 0] > 0.12) & (box[:, :, 1] < 0.25)
        ys, xs = np.nonzero(dm | tg)
        dark[f"{pose}_{tag}"] = {"cavity_px": int(dm.sum()), "tongue_px": int(tg.sum()),
                                 "mouth_bbox_wh_px": [int(np.ptp(xs)) + 1, int(np.ptp(ys)) + 1] if len(xs) else None,
                                 "head_box_w_px": int(u1 - u0),
                                 "mouth_w_over_head_w": float((np.ptp(xs) + 1) / (u1 - u0)) if len(xs) else None}
    # source row: SRC_hi / SRC_club_hi (rig/gob_r00_source.blend, appended read-only), rest position
    src_path = goblib.RIG / "gob_r00_source.blend"
    with bpy.data.libraries.load(str(src_path), link=False) as (src_, dst_):
        dst_.objects = [n for n in src_.objects if n in ("SRC_hi", "SRC_club_hi")]
    hidden = []
    for o in sc.objects:
        if not o.hide_render:
            o.hide_render = True
            hidden.append(o)
    srcm = bpy.data.materials.new("_r1_src")
    srcm.diffuse_color = MAT_RGBA[SKIN_MAT]
    for o in dst_.objects:
        if o is None:
            continue
        sc.collection.objects.link(o)
        o.data.materials.clear()
        o.data.materials.append(srcm if o.name.startswith("SRC_hi") else cm)
    Vs = np.array([v.co[:] for v in bpy.data.objects["SRC_hi"].data.vertices])
    mw_s = np.array(bpy.data.objects["SRC_hi"].matrix_world)
    Vs = Vs @ mw_s[:3, :3].T + mw_s[:3, 3]
    hsel = Vs[:, 2] > 0.94
    Hs = 0.5 * (Vs[hsel].min(0) + Vs[hsel].max(0)) + np.array([0.0, 0.0, -0.02])
    imgs["source"] = views("source", Hs)
    for o in dst_.objects:
        if o is not None:
            bpy.data.objects.remove(o, do_unlink=True)
    for o in hidden:
        o.hide_render = False
    set_state("ready", None)
    doc["poses"]["render_mouth_px_full_front"] = dark
    doc["poses"]["matcap"] = mc
    print(f"[s02e] RENDER mouth px (full front 896x1152): {dark}")
    # ---- sheet: rows source / closed (ready) / open (roar-like); columns reference | front plain | front matcap |
    # left 3/4 | right 3/4 | side | full front
    ref_c = S.load_rgba(REF_CLOSEUP)[:, :, :3]
    Hrow = 480
    rows_img = []
    for tag, x0 in (("source", None), ("closed", 0), ("open", 260)):
        refc = resize(ref_c[:, x0:x0 + 260], Hrow) if x0 is not None else np.ones((Hrow, 624, 3), np.float32)
        cells = [refc] + [resize(imgs[tag][k], Hrow) for k in ("fc", "mc", "tl", "tc", "sd", "ff") if k in imgs[tag]]
        sp = np.ones((Hrow, 8, 3), np.float32)
        rows_img.append(np.concatenate(sum(([c, sp] for c in cells), [])[:-1], axis=1))
    wmax = max(r.shape[1] for r in rows_img)
    rows_img = [np.concatenate([r, np.ones((Hrow, wmax - r.shape[1], 3), np.float32)], axis=1) for r in rows_img]
    sep = np.ones((8, wmax, 3), np.float32)
    S.save_rgb(np.concatenate(sum(([r, sep] for r in rows_img), [])[:-1], axis=0), INSP / "hr1_mouth_sheet.png")
    doc["renders"] = sorted(goblib._rel(p_) for p_ in INSP.glob("hr1_*.png"))
    print(f"[s02e] RENDERS {doc['renders']}")
    print("[s02e] SHEET inspect/R1/hr1_mouth_sheet.png rows source (SRC_hi, rest) / closed (ready) / open (roar-like); "
          "columns reference (f7 / f15; empty for the source row) | front close-up plain | front close-up matcap "
          f"({mc}) | 3/4 from the character's left | 3/4 from the character's right | side profile (character's "
          "left) | full front")
    return doc


def wire_renders(doc):
    ob = bpy.data.objects[MESH]
    keys = ob.data.shape_keys.key_blocks
    INSP.mkdir(parents=True, exist_ok=True)
    bb = ((-0.32, -0.30, 0.80), (0.32, 0.26, 1.40))
    names = [v["name"] for v in VARIANTS]
    for var in [None] + names:
        for nm_ in names:
            keys[key_name(nm_)].value = 1.0 if nm_ == var else 0.0
        bpy.context.view_layer.update()
        tag = "hr1_" + ("closed" if var is None else "open")
        goblib.ortho_render([ob], "front", INSP / f"wire_rest_{tag}_front.png", res_h=900, frame_bbox=bb, wire=True)
        goblib.ortho_render([ob], "three_quarter", INSP / f"wire_rest_{tag}_tq.png", res_h=900, frame_bbox=bb, wire=True)
        culled_renders(ob, "closed" if var is None else "open")
    for nm_ in names:
        keys[key_name(nm_)].value = 0.0
    bpy.context.view_layer.update()


def culled_renders(ob, state):
    """T124: rest pose, Workbench with back-face culling (like Unity's default shading) on a magenta background, so a
    face seen from its back (or a missing face) shows as a magenta hole: inspect/R1/culled_<state>_{front,tq}.png
    (swing_cam.json front / three_quarter view directions, head close-up)."""
    cams = goblib.load_json("swing_cam.json")
    sc = bpy.context.scene
    r, sh = sc.render, sc.display.shading
    if sc.world is None:
        sc.world = bpy.data.worlds.new("_r1_world")
    saved = (r.engine, r.resolution_x, r.resolution_y, r.resolution_percentage, r.film_transparent, r.filepath,
             sh.light, sh.color_type, sh.show_backface_culling, tuple(sc.world.color), sc.camera)
    dg = bpy.context.evaluated_depsgraph_get()
    me_e = ob.evaluated_get(dg).data
    P = np.array([v.co[:] for v in me_e.vertices])
    pid = np.empty(len(me_e.polygons), dtype=np.int32)
    me_e.attributes["part_id"].data.foreach_get("value", pid)
    hv = sorted({v for p in me_e.polygons if pid[p.index] == 1 for v in p.vertices})
    ctr = 0.5 * (P[hv].min(0) + P[hv].max(0))
    try:
        r.engine = "BLENDER_WORKBENCH"
        r.resolution_x, r.resolution_y, r.resolution_percentage = 700, 600, 100
        r.film_transparent = False
        sh.light, sh.color_type, sh.show_backface_culling = "STUDIO", "MATERIAL", True
        sc.world.color = (1.0, 0.0, 1.0)
        for view, tag in (("front", "front"), ("three_quarter", "tq")):
            d = np.array(cams[view]["view_dir"], float)
            rt = np.cross(d, [0.0, 0.0, 1.0])
            rt /= np.linalg.norm(rt)
            up = np.cross(rt, d)
            M = np.eye(4)
            M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = rt, up, -d, ctr - d * 6.0
            cd = bpy.data.cameras.new("_r1_cull")
            cd.type, cd.sensor_fit, cd.ortho_scale, cd.clip_start, cd.clip_end = "ORTHO", "VERTICAL", 0.72, 0.01, 20.0
            co = bpy.data.objects.new("_r1_cull", cd)
            sc.collection.objects.link(co)
            co.matrix_world = Matrix(M.tolist())
            sc.camera = co
            try:
                out = INSP / f"culled_{state}_{tag}.png"
                bpy.ops.render.render(write_still=False)
                bpy.data.images["Render Result"].save_render(str(out))
            finally:
                bpy.data.objects.remove(co, do_unlink=True)
                bpy.data.cameras.remove(cd)
    finally:
        (r.engine, r.resolution_x, r.resolution_y, r.resolution_percentage, r.film_transparent, r.filepath,
         sh.light, sh.color_type, sh.show_backface_culling, wc, sc.camera) = saved
        sc.world.color = wc


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    do_render = "--no-render" not in args
    doc, V, nv0, idx_new, reuse = model()
    if do_render:
        wire_renders(doc)
    if "--pose" in args:   # HR2: needs rig/gob_r04_ctrl.blend (downstream of this chain), so opt-in
        doc = pose_phase(doc, V, nv0, idx_new, reuse, do_render)
    else:
        print("[s02e] POSE phase skipped (no --pose): no gob_r04_ctrl.blend read, no hr1_* pose renders")
    p = goblib.save_json("mouth.json", doc)
    print(f"[s02e] JSON {goblib._rel(p)}")
    print("[s02e] done")


if __name__ == "__main__":
    goblib.run_main(main)
