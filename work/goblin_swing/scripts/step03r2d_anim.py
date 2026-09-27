"""STEP 03R2 round 2 - evaluate the REAL blocking action on a given build.

Run: blender -b <blend> --python step03r2d_anim.py -- --tag v05c [--mesh GOB_body]
     [--frames 9,11,12,13,15,18,24,27,29]
Appends GOB_swing from tmp\\v06_read.blend (a copy of the read-only blocking
file), evaluates the frames, renders through REF_CAM, and measures the same
honest silhouette / crease metrics as step03r2b_gate.  NEVER saves.
"""
import bpy, json, math, os, sys, time
import numpy as np
from mathutils import Vector

sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
SRC = os.path.join(ROOT, "tmp", "v06_read.blend")
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(n, d=None):
    return argv[argv.index(n) + 1] if n in argv else d


TAG = opt("--tag", "x")
MESH = opt("--mesh", "GOB_body_lo")
FRAMES = [int(x) for x in (opt("--frames", "9,11,12,13,15,18,24,27,29")).split(",")]
OUT = os.path.join(ROOT, "inspect", "step03r2", "anim")
os.makedirs(OUT, exist_ok=True)
CELL = 0.002
SE = {"R": 0.020, "L": -0.030}

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
hi = bpy.data.objects["GOB_body"]
lo = bpy.data.objects["GOB_body_lo"]
for c, vis in (("GOB_hi", MESH == "GOB_body"), ("GOB_lo", MESH == "GOB_body_lo")):
    cc = bpy.data.collections.get(c)
    if cc:
        cc.hide_viewport = not vis
        cc.hide_render = not vis
hi.hide_viewport = MESH != "GOB_body"; hi.hide_render = hi.hide_viewport
lo.hide_viewport = MESH != "GOB_body_lo"; lo.hide_render = lo.hide_viewport
for lc in bpy.context.view_layer.layer_collection.children:
    lc.exclude = False
    lc.hide_viewport = not ((lc.name == "GOB_hi") == (MESH == "GOB_body"))

# ---------------------------------------------------------------- the action
before = set(a.name for a in bpy.data.actions)
with bpy.data.libraries.load(SRC, link=False) as (src, dst):
    assert "GOB_swing" in src.actions, list(src.actions)
    dst.actions = ["GOB_swing"]
act = [a for a in bpy.data.actions if a.name not in before][0]
if rig.animation_data is None:
    rig.animation_data_create()
rig.animation_data.action = act
try:
    sl = rig.animation_data.action_suitable_slots
    if len(sl):
        rig.animation_data.action_slot = sl[0]
except Exception as e:
    print("slot note:", e)

ob = bpy.data.objects[MESH]
me = ob.data
N = len(me.vertices)
P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)
me.calc_loop_triangles()
TRI = np.array([t.vertices[:] for t in me.loop_triangles], dtype=np.int32)
R = {"tag": TAG, "mesh": MESH, "action": act.name,
     "action_frame_range": [round(float(x), 1) for x in act.frame_range],
     "modifiers": [(m.name, m.type) for m in ob.modifiers]}


def poly_d(pts):
    pts = np.asarray(pts, float); A, B = pts[:-1], pts[1:]; AB = B - A
    L2 = (AB ** 2).sum(1); bd = np.full(N, 1e9)
    for k in range(len(A)):
        w = P0 - A[k]
        tt = np.clip((w * AB[k]).sum(1) / L2[k], 0, 1)
        bd = np.minimum(bd, np.linalg.norm(w - tt[:, None] * AB[k], axis=1))
    return bd


def bh(n):
    return np.array(arm.bones[n].head_local)


def bt(n):
    return np.array(arm.bones[n].tail_local)


CH = {S: poly_d([bh("UPPERARM_" + S), bh("FOREARM_" + S), bh("HAND_" + S), bt("HAND_" + S)])
      for S in ("R", "L")}
SC = {}
for S in ("R", "L"):
    u = bt("UPPERARM_" + S) - bh("UPPERARM_" + S); u /= np.linalg.norm(u)
    SC[S] = (P0 - bh("UPPERARM_" + S)) @ u
AXd = np.array([-0.10609, 0.03071, 0.99388]); AXd /= np.linalg.norm(AXd)
V = P0 - np.array([-0.6661, -0.3447, 0.9910]); S_AX = V @ AXd
D_AX = np.linalg.norm(V - S_AX[:, None] * AXd, axis=1)
CLUB = (S_AX > -0.37) & (S_AX < 0.92) & (np.abs(S_AX) > 0.1134) & (D_AX < 0.22)
HANDR = np.linalg.norm(P0 - np.array([-0.676, -0.364, 0.992]), axis=1) <= 0.155
HANDL = np.linalg.norm(P0 - np.array([0.772, 0.056, 0.720]), axis=1) <= 0.155
TUBEV = CLUB | HANDR | HANDL
for S in ("R", "L"):
    TUBEV |= (CH[S] < 0.10) & (SC[S] > SE[S])
TRI_TUBE = np.nonzero(TUBEV[TRI].any(axis=1))[0]
TRI_EGG = np.nonzero((~TUBEV)[TRI].all(axis=1))[0]

WEB = np.zeros(N, dtype=bool)
for S in ("R", "L"):
    WEB |= ((np.linalg.norm(P0 - bh("UPPERARM_" + S), axis=1) < 0.30)
            & (CH[S] > 0.045) & (CH[S] < 0.22))
WEB &= ~CLUB & ~HANDR & ~HANDL & (P0[:, 2] < 1.30) & (P0[:, 2] > 0.80)
toe = {}
for ti, t in enumerate(TRI):
    for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
        k = (a, b) if a < b else (b, a)
        toe.setdefault(k, []).append(ti)
WE_T = np.array([[v[0], v[1]] for k, v in toe.items()
                 if len(v) == 2 and WEB[k[0]] and WEB[k[1]]], dtype=np.int32)


def dihedral(Q):
    if not len(WE_T):
        return None
    a, b, c = Q[TRI[:, 0]], Q[TRI[:, 1]], Q[TRI[:, 2]]
    nr = np.cross(b - a, c - a)
    nr = nr / np.maximum(np.linalg.norm(nr, axis=1), 1e-12)[:, None]
    return np.degrees(np.arccos(np.clip((nr[WE_T[:, 0]] * nr[WE_T[:, 1]]).sum(1), -1, 1)))


D0 = dihedral(P0)
_BA = []
K = 7
for i in range(K + 1):
    for j in range(K + 1 - i):
        _BA.append((i / K, j / K, 1 - i / K - j / K))
_BA = np.array(_BA)


def fp(p2, tris, shape, org):
    g = np.zeros(shape, bool)
    if not len(tris):
        return g
    A, B, C = p2[TRI[tris, 0]], p2[TRI[tris, 1]], p2[TRI[tris, 2]]
    for w in _BA:
        Pt = w[0] * A + w[1] * B + w[2] * C
        ij = np.floor((Pt - org) / CELL).astype(np.int64)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[1])
        g[ij[ok, 0], ij[ok, 1]] = True
    return g


def dil(g, k=1):
    o = g.copy()
    for _ in range(k):
        t = o.copy()
        t[1:] |= o[:-1]; t[:-1] |= o[1:]; t[:, 1:] |= o[:, :-1]; t[:, :-1] |= o[:, 1:]
        o = t
    return o


refcam = bpy.data.objects.get("REF_CAM")
CM = np.array(refcam.matrix_world.inverted())
DREF = float(-(np.array([0.0, 0.088, 0.95, 1.0]) @ CM.T)[2])


def proj_ref(Q):
    H = np.concatenate([Q, np.ones((len(Q), 1))], axis=1)
    C = H @ CM.T
    zc = np.maximum(-C[:, 2], 1e-6)
    return np.stack([C[:, 0] / zc, C[:, 1] / zc], axis=1) * DREF


cam, cd = RL.setup(res=1024, plain=True)
sc = bpy.context.scene
TESTS = {}
t0 = time.time()
for f in FRAMES:
    sc.frame_set(f)
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
    ev.to_mesh_clear()
    pr_r, pr_q = proj_ref(P0), proj_ref(Q)
    allp = np.concatenate([pr_r, pr_q]); org = allp.min(0) - 0.02
    shape = tuple(np.ceil((allp.max(0) + 0.02 - org) / CELL).astype(int))
    ideal = dil(fp(pr_r, TRI_EGG, shape, org) | fp(pr_q, TRI_TUBE, shape, org))
    actual = fp(pr_q, np.arange(len(TRI)), shape, org)
    outm = actual & ~ideal
    cells = 0
    if outm.any():
        grown = ideal.copy()
        for k in range(1, 61):
            grown = dil(grown)
            if not (outm & ~grown).any():
                cells = k
                break
        else:
            cells = 60
    row = {"protrusion_mm": round(cells * CELL * 1000, 1),
           "protrusion_cm2": round(int(outm.sum()) * (CELL * 100) ** 2, 2)}
    D = dihedral(Q)
    if D is not None:
        dD = np.abs(D - D0)
        row["web_crease_p95_deg"] = round(float(np.percentile(dD, 95)), 2)
        row["web_crease_max_deg"] = round(float(dD.max()), 2)
        row["web_edges_creased_gt20deg"] = int((dD > 20).sum())
    # edge stretch in the shoulder region (tearing / smearing indicator)
    E = np.empty(len(me.edges) * 2, dtype=np.int32); me.edges.foreach_get("vertices", E)
    E = E.reshape(-1, 2)
    loc = WEB[E[:, 0]] | WEB[E[:, 1]]
    L0 = np.linalg.norm(P0[E[loc, 0]] - P0[E[loc, 1]], axis=1)
    L1 = np.linalg.norm(Q[E[loc, 0]] - Q[E[loc, 1]], axis=1)
    rt = L1 / np.maximum(L0, 1e-9)
    row["web_edge_stretch_p99"] = round(float(np.percentile(rt, 99)), 3)
    row["web_edge_stretch_max"] = round(float(rt.max()), 3)
    TESTS["f%04d" % f] = row
    old = sc.camera
    sc.camera = refcam
    sc.render.filepath = os.path.join(OUT, "%s_f%04d_ref.png" % (TAG, f))
    bpy.ops.render.render(write_still=True)
    sc.camera = old
    print("### f%04d %s" % (f, json.dumps(row)))

rig.animation_data.action = None
R["tests"] = TESTS
R["seconds"] = round(time.time() - t0, 1)
with open(os.path.join(ROOT, "inspect", "step03r2d_anim_%s.json" % TAG), "w") as f_:
    json.dump(R, f_, indent=1, default=str)
print("@@@ANIM_DONE@@@", TAG, R["seconds"])
