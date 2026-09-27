"""STEP 03 - skin GOB_body to GOB_rig.

Run: blender -b goblin_v03_controls.blend --python step03_skin.py -- [--dq|--linear]

Route:
  1. run Blender's automatic (bone-heat) weights and MEASURE them (recorded in the
     report as evidence);
  2. discard them and rebuild the weights deterministically:
       hard geometric labels  ->  constrained harmonic diffusion in narrow bands
     which is what gives "near-rigid segments + compact smooth blend bands".
Writes goblin_v04_skinned.blend.  Never touches goblin_v03_controls.blend.
"""
import bpy, json, math, os, sys, time, heapq
import numpy as np
from mathutils import Vector

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
OUTB = os.path.join(ROOT, "goblin_v04_skinned.blend")
INSP = os.path.join(ROOT, "inspect")
os.makedirs(INSP, exist_ok=True)

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
PRESERVE_VOLUME = "--linear" not in argv
TAG = "dq" if PRESERVE_VOLUME else "lin"
if "--out" in argv:
    OUTB = argv[argv.index("--out") + 1]

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
body = bpy.data.objects["GOB_body"]
me = body.data
N = len(me.vertices)

REP = {"preserve_volume": PRESERVE_VOLUME}

# ============================================================ numpy geometry
P = np.empty(N * 3, dtype=np.float64)
me.vertices.foreach_get("co", P)
P = P.reshape(N, 3)
E = np.empty(len(me.edges) * 2, dtype=np.int32)
me.edges.foreach_get("vertices", E)
E = E.reshape(-1, 2)

BONES = ["HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]
BI = {n: i for i, n in enumerate(BONES)}
NB = len(BONES)
assert sorted(BONES) == sorted([b.name for b in arm.bones if b.use_deform])

SEG = {}
for n in BONES:
    b = arm.bones[n]
    SEG[n] = (np.array(b.head_local), np.array(b.tail_local))


def poly_td(pts):
    """arc-length t and distance d from every vertex to the polyline `pts`."""
    pts = np.asarray(pts, dtype=np.float64)
    A, B = pts[:-1], pts[1:]
    AB = B - A
    L2 = (AB ** 2).sum(1)
    L = np.sqrt(L2)
    cum = np.concatenate([[0.0], np.cumsum(L)])
    bd = np.full(N, 1e9)
    bt = np.zeros(N)
    for k in range(len(A)):
        w = P - A[k]
        tt = np.clip((w * AB[k]).sum(1) / L2[k], 0.0, 1.0)
        d = np.linalg.norm(w - tt[:, None] * AB[k], axis=1)
        m = d < bd
        bd[m] = d[m]
        bt[m] = cum[k] + tt[m] * L[k]
    return bt, bd


def seg_d(name):
    a, b = SEG[name]
    ab = b - a
    L2 = (ab ** 2).sum()
    w = P - a
    tt = np.clip((w * ab).sum(1) / L2, 0.0, 1.0)
    return np.linalg.norm(w - tt[:, None] * ab, axis=1)


# ============================================================ club / hand balls
AXd = np.array([-0.10609, 0.03071, 0.99388])
AXd /= np.linalg.norm(AXd)
GP = np.array([-0.6661, -0.3447, 0.9910])
HC_R = np.array([-0.676, -0.364, 0.992])
HC_L = np.array([0.772, 0.056, 0.720])

V = P - GP
S_AX = V @ AXd
D_AX = np.linalg.norm(V - S_AX[:, None] * AXd, axis=1)
DHR = np.linalg.norm(P - HC_R, axis=1)
DHL = np.linalg.norm(P - HC_L, axis=1)

S_BALL = 0.1134          # sphere(0.126) x cylinder(0.055) intersection half-height
club_m = ((S_AX > -0.37) & (S_AX < 0.92) & (np.abs(S_AX) > S_BALL) & (D_AX < 0.22))

# adjacency (for components + diffusion)
adj_head = np.concatenate([E[:, 0], E[:, 1]])
adj_tail = np.concatenate([E[:, 1], E[:, 0]])
order = np.argsort(adj_head, kind="stable")
adj_head = adj_head[order]
adj_tail = adj_tail[order]
adj_start = np.searchsorted(adj_head, np.arange(N + 1))
DEG = np.diff(adj_start).astype(np.float64)


def components(mask):
    seen = np.zeros(N, dtype=bool)
    out = []
    idx = np.nonzero(mask)[0]
    for i0 in idx:
        if seen[i0]:
            continue
        st = [i0]
        seen[i0] = True
        c = []
        while st:
            v = st.pop()
            c.append(v)
            for k in range(adj_start[v], adj_start[v + 1]):
                w = adj_tail[k]
                if mask[w] and not seen[w]:
                    seen[w] = True
                    st.append(w)
        out.append(np.array(c))
    out.sort(key=len, reverse=True)
    return out


cc = components(club_m)
CLUB = np.zeros(N, dtype=bool)
for c in cc:
    if len(c) >= 100:
        CLUB[c] = True
REP["club_components"] = [int(len(c)) for c in cc[:6]]
REP["club_verts"] = int(CLUB.sum())

HANDRB = (DHR <= 0.155) & (~CLUB)
HANDLB = (DHL <= 0.155)
REP["handR_ball_verts"] = int(HANDRB.sum())
REP["handL_ball_verts"] = int(HANDLB.sum())

# ============================================================ limb centerlines
ARM_CHAIN = {}
for S in ("L", "R"):
    ARM_CHAIN[S] = [SEG["UPPERARM_" + S][0], SEG["FOREARM_" + S][0],
                    SEG["HAND_" + S][0], SEG["HAND_" + S][1]]
LEG_CHAIN = {}
for S in ("L", "R"):
    LEG_CHAIN[S] = [SEG["THIGH_" + S][0], SEG["SHIN_" + S][0],
                    SEG["FOOT_" + S][0], SEG["FOOT_" + S][1]]

tA, dA, tL, dL = {}, {}, {}, {}
for S in ("L", "R"):
    tA[S], dA[S] = poly_td(ARM_CHAIN[S])
    tL[S], dL[S] = poly_td(LEG_CHAIN[S])


def flood(seed, allow):
    out = np.zeros(N, dtype=bool)
    st = list(np.nonzero(seed & allow)[0])
    out[st] = True
    while st:
        v = st.pop()
        for k in range(adj_start[v], adj_start[v + 1]):
            w = adj_tail[k]
            if allow[w] and not out[w]:
                out[w] = True
                st.append(w)
    return out

# ============================================================ hard labels
rT = np.hypot(P[:, 0], P[:, 1] - 0.088)
Z = P[:, 2]
lab = np.full(N, -1, dtype=np.int32)

lab[CLUB] = BI["WEAPON"]
lab[HANDRB & (lab < 0)] = BI["HAND_R"]
lab[HANDLB & (lab < 0)] = BI["HAND_L"]

# feet: the flat shoe ovals sit below z = 0.168 (measured: shoe top 0.165)
FOOT_Z = 0.168
foot = (Z <= FOOT_Z) & (lab < 0)
lab[foot & (P[:, 0] < 0)] = BI["FOOT_R"]
lab[foot & (P[:, 0] >= 0)] = BI["FOOT_L"]

# arms: flood-fill outwards from the hand ball through the tube only.
# The flood guarantees the arm region is mesh-connected to the hand, so it can
# never jump onto the head / torso the way a pure distance test can.
ARM_MASK = {}
BALL = {"L": HANDLB, "R": HANDRB}
for S in ("L", "R"):
    sh = SEG["UPPERARM_" + S][0]
    u = SEG["UPPERARM_" + S][1] - sh
    u /= np.linalg.norm(u)
    outb = (P - sh) @ u
    allow = ((dA[S] < 0.105) & (Z < 1.345) & (outb > 0.030) &
             (~CLUB) & (~BALL["L" if S == "R" else "R"])) | BALL[S]
    ARM_MASK[S] = flood(BALL[S], allow) & (lab < 0)
    m = ARM_MASK[S]
    sub = np.stack([seg_d("UPPERARM_" + S), seg_d("FOREARM_" + S), seg_d("HAND_" + S)])
    pick = np.argmin(sub, axis=0)
    for k, bn in enumerate(("UPPERARM_" + S, "FOREARM_" + S, "HAND_" + S)):
        lab[m & (pick == k)] = BI[bn]

# legs: flood from the shoe up the tube, stopping at the torso underside
LEG_Z = 0.355
for S in ("L", "R"):
    shoe = (lab == BI["FOOT_" + S])
    allow = ((dL[S] < 0.105) & (Z < LEG_Z)) | shoe
    m = flood(shoe, allow) & (lab < 0)
    sub = np.stack([seg_d("THIGH_" + S), seg_d("SHIN_" + S)])
    pick = np.argmin(sub, axis=0)
    for k, bn in enumerate(("THIGH_" + S, "SHIN_" + S)):
        lab[m & (pick == k)] = BI[bn]

# head above the crease (measured narrowest cross-section at z = 1.295-1.30)
HEAD_Z = 1.305
lab[(Z >= HEAD_Z) & (lab < 0)] = BI["HEAD"]
# neck: the crease column only
lab[(Z >= 1.250) & (Z < HEAD_Z) & (rT < 0.32) & (lab < 0)] = BI["NECK"]

# torso remainder
rest = lab < 0
lab[rest & (Z < 0.84)] = BI["HIPS"]
lab[rest & (Z >= 0.84) & (Z < 1.01)] = BI["SPINE_01"]
lab[rest & (Z >= 1.01)] = BI["CHEST"]

assert (lab >= 0).all(), "unlabelled verts: %d" % int((lab < 0).sum())
REP["label_counts_raw"] = {BONES[i]: int((lab == i).sum()) for i in range(NB)}

# ---- despeckle: tiny islands of a label inside another region are relabelled
PROT = CLUB | HANDRB | HANDLB
cleaned = 0
for _ in range(3):
    moved = 0
    for j in range(NB):
        for c in components((lab == j) & (~PROT)):
            if len(c) >= 40:
                continue
            cnt = {}
            for v in c:
                for k in range(adj_start[v], adj_start[v + 1]):
                    w = adj_tail[k]
                    if lab[w] != j:
                        cnt[lab[w]] = cnt.get(lab[w], 0) + 1
            if cnt:
                lab[c] = max(cnt.items(), key=lambda kv: kv[1])[0]
                moved += len(c)
    cleaned += moved
    if moved == 0:
        break
REP["despeckled_verts"] = int(cleaned)
REP["label_counts"] = {BONES[i]: int((lab == i).sum()) for i in range(NB)}
REP["label_components"] = {BONES[i]: [int(len(c)) for c in components(lab == i)[:4]]
                           for i in range(NB)}
REP["label_zrange"] = {BONES[i]: [round(float(Z[lab == i].min()), 3),
                                  round(float(Z[lab == i].max()), 3)]
                       for i in range(NB) if (lab == i).any()}

# diagnostics: does the arm label leak into the torso envelope?
for S in ("L", "R"):
    am = (lab == BI["UPPERARM_" + S])
    REP["shoulder_leak_" + S] = {
        "upperarm_verts": int(am.sum()),
        "with_rT_lt_0.34": int((am & (rT < 0.34)).sum()),
        "min_rT": round(float(rT[am].min()), 4),
        "max_z": round(float(Z[am].max()), 4)}

# ============================================================ locks + band width
LOCK = CLUB | HANDRB | HANDLB | (lab == BI["FOOT_L"]) | (lab == BI["FOOT_R"])
LOCK |= (Z >= 1.375) & (lab == BI["HEAD"])
BELT = (Z >= 0.578) & (Z <= 0.782) & (lab == BI["HIPS"])
LOCK |= BELT
REP["locked_verts"] = int(LOCK.sum())
REP["belt_verts"] = int(BELT.sum())

W = np.zeros(N)
for S in ("L", "R"):
    for bn in ("UPPERARM_", "FOREARM_", "HAND_"):
        W[lab == BI[bn + S]] = 0.035
    # the armpit has to absorb the whole shoulder rotation - give it more room
    W[(lab == BI["UPPERARM_" + S]) & (rT > 0.30) & (Z > 1.05)] = 0.060
    for bn in ("THIGH_", "SHIN_"):
        W[lab == BI[bn + S]] = 0.030
W[lab == BI["HEAD"]] = 0.040
W[lab == BI["NECK"]] = 0.040
W[lab == BI["CHEST"]] = 0.070
W[(lab == BI["CHEST"]) & (Z > 1.05) & (rT > 0.28)] = 0.090
W[lab == BI["SPINE_01"]] = 0.080
W[lab == BI["HIPS"]] = 0.070
W[(lab == BI["HIPS"]) & (Z < 0.50)] = 0.035     # leg root: keep it tight

# the mesh density is very uneven (dense head/feet/belt, sparse tubes & torso):
# widen the band where the local edges are long so every joint still gets a ramp
ELEN_ALL = np.linalg.norm(P[adj_head] - P[adj_tail], axis=1)
LOCAL_E = np.bincount(adj_head, weights=ELEN_ALL, minlength=N) / DEG
REP["local_edge_len"] = {"min": round(float(LOCAL_E.min()), 5),
                         "median": round(float(np.median(LOCAL_E)), 5),
                         "max": round(float(LOCAL_E.max()), 5)}
BLEN = np.array([arm.bones[n].length for n in BONES])
WCAP = np.minimum(2.0 * W, 0.25 * BLEN[lab])
W = np.clip(np.maximum(W, np.where(W > 0, 1.8 * LOCAL_E, 0.0)), 0.0, WCAP)

# ---- forbidden blends: only skeleton-adjacent bones may share a vertex.
# Any other label boundary (head vs. shoulder, hips vs. shin, club vs. hand ...)
# is made HARD so proximity can never turn into cross-bleed.
ALLOWED = set()
for a, b in [("HIPS", "SPINE_01"), ("SPINE_01", "CHEST"), ("CHEST", "NECK"),
             ("NECK", "HEAD"), ("CHEST", "HEAD"),
             ("CHEST", "UPPERARM_L"), ("CHEST", "UPPERARM_R"),
             ("SPINE_01", "UPPERARM_L"), ("SPINE_01", "UPPERARM_R"),
             ("UPPERARM_L", "FOREARM_L"), ("FOREARM_L", "HAND_L"),
             ("UPPERARM_R", "FOREARM_R"), ("FOREARM_R", "HAND_R"),
             ("HIPS", "THIGH_L"), ("HIPS", "THIGH_R"),
             ("THIGH_L", "SHIN_L"), ("SHIN_L", "FOOT_L"),
             ("THIGH_R", "SHIN_R"), ("SHIN_R", "FOOT_R"),
             # HEAD <-> UPPERARM is deliberately NOT allowed: the head's lower rim
             # touches the shoulder, and blending there makes the ARM follow the
             # head (measured: 0.12 m of arm travel on a 40 deg head yaw).  The
             # price is a visible crease where a raised arm passes the head.
             ("HAND_R", "WEAPON")]:
    ALLOWED.add((BI[a], BI[b]))
    ALLOWED.add((BI[b], BI[a]))

la, lb = lab[adj_head], lab[adj_tail]
diff = la != lb
pairs = {}
bad_edge = np.zeros(len(adj_head), dtype=bool)
for k in np.nonzero(diff)[0]:
    key = (int(la[k]), int(lb[k]))
    if key not in ALLOWED:
        bad_edge[k] = True
        kk = tuple(sorted(key))
        pairs[kk] = pairs.get(kk, 0) + 1
REP["forbidden_pairs"] = {BONES[a] + "|" + BONES[b]: v // 2
                          for (a, b), v in sorted(pairs.items(), key=lambda kv: -kv[1])}
HARD = np.zeros(N, dtype=bool)
HARD[adj_head[bad_edge]] = True
HARD[adj_tail[bad_edge]] = True
LOCK = LOCK | HARD
REP["hard_boundary_verts"] = int(HARD.sum())
W[LOCK] = 0.0

# ============================================================ geodesic from boundary
t0 = time.time()
bnd = lab[adj_head] != lab[adj_tail]
BSET = np.unique(adj_head[bnd])
ELEN = np.linalg.norm(P[adj_head] - P[adj_tail], axis=1)
g = np.full(N, 1e9)
g[BSET] = 0.0
pq = [(0.0, int(i)) for i in BSET]
heapq.heapify(pq)
WMAX = float(W.max())
while pq:
    du, u = heapq.heappop(pq)
    if du > g[u] + 1e-12 or du > WMAX:
        continue
    for k in range(adj_start[u], adj_start[u + 1]):
        v = adj_tail[k]
        nd = du + ELEN[k]
        if nd < g[v]:
            g[v] = nd
            if nd <= WMAX:
                heapq.heappush(pq, (nd, int(v)))
REP["geodesic_seconds"] = round(time.time() - t0, 1)
REP["boundary_verts"] = int(len(BSET))

# Every label region must keep a CLAMPED CORE: if a region's blend bands met in
# the middle (the thigh is only 7.6 cm of tube) a distant bone could diffuse
# straight through it.  A clamped core is a Dirichlet wall that stops that, and
# unlike a per-vertex "allowed bones" mask it introduces no discontinuity.
gcap = np.full(N, 1e9)
for j in range(NB):
    for c in components(lab == j):
        gcap[c] = 0.60 * float(g[c].max()) if len(c) else 0.0
W = np.minimum(W, gcap)
W[LOCK] = 0.0
REP["band_width"] = {BONES[j]: [round(float(W[lab == j].min()), 4),
                                round(float(W[lab == j].max()), 4)]
                     for j in range(NB) if (lab == j).any()}

FREE = (g < W) & (~LOCK)
REP["free_verts"] = int(FREE.sum())
REP["clamped_core_per_label"] = {BONES[j]: int(((lab == j) & (~FREE)).sum())
                                 for j in range(NB)}

# ============================================================ constrained diffusion
# a vertex may only ever carry its own bone + the bones adjacent to it in the
# skeleton.  Enforced every iteration, so nothing can creep across two joints.
ALLOWM = np.zeros((NB, NB), dtype=bool)
for j in range(NB):
    ALLOWM[j, j] = True
for a, b in ALLOWED:
    ALLOWM[a, b] = True
VALLOW = ALLOWM[lab]
REP["allowed_bones_per_vert"] = {BONES[j]: [BONES[k] for k in range(NB) if ALLOWM[j, k]]
                                 for j in range(NB)}

Wt = np.zeros((N, NB), dtype=np.float32)
Wt[np.arange(N), lab] = 1.0
W0 = Wt.copy()
CLAMP = ~FREE
t0 = time.time()
ITER = 320
for it in range(ITER):
    S_ = np.empty_like(Wt)
    for j in range(NB):
        S_[:, j] = np.bincount(adj_head, weights=Wt[adj_tail, j], minlength=N)
    Wt = (S_.T / DEG).T.astype(np.float32)
    Wt[CLAMP] = W0[CLAMP]
REP["diffusion_seconds"] = round(time.time() - t0, 1)

# ============================================================ prune / normalize
# ---------------------------------------------------------------- legs
# The leg tubes carry only ~3 vertex rings per segment, far too few for a
# diffused blend to stay monotone (HIPS leaked all the way to the shin).
# Replace them with an explicit smoothstep ladder along the leg chain, which is
# monotone by construction: HIPS -> THIGH -> SHIN -> FOOT, never skipping one.
def smoothstep(x, a, b):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


LR = (argv[argv.index("--legramp") + 1] if "--legramp" in argv else "conic")
CFG = {
    "wide":   {"root": (0.025, 0.085), "knee": (0.098, 0.148), "ankle": (0.215, 0.250),
               "cone": 0.0, "dz": 0.16, "zmax": 0.46},
    "narrow": {"root": (0.043, 0.075), "knee": (0.108, 0.140), "ankle": (0.233, 0.257),
               "cone": 0.0, "dz": 0.16, "zmax": 0.46},
    "rigid":  {"root": (0.058, 0.060), "knee": (0.122, 0.124), "ankle": (0.246, 0.248),
               "cone": 0.0, "dz": 0.16, "zmax": 0.46},
    # the tube carries ~3 vertex rings per segment; a short band tears it, so the
    # bend is spread over the whole tube and feathered conically into the
    # torso underside (cone = how fast the weight dies off sideways).
    "conic":  {"root": (0.005, 0.095), "knee": (0.080, 0.170), "ankle": (0.215, 0.262),
               "cone": 1.2, "dz": 0.28, "zmax": 0.50},
}[LR]
LEGRAMP = {k: CFG[k] for k in ("root", "knee", "ankle")}
REP["legramp_mode"] = LR
REP["legramp_cfg"] = CFG
REP["leg_ramps"] = LEGRAMP
legfix = 0
for S in ("L", "R"):
    zone = (dL[S] < CFG["dz"]) & (Z < CFG["zmax"]) & (lab != BI["FOOT_" + S]) & \
           (lab != BI["FOOT_" + ("R" if S == "L" else "L")]) & \
           (lab != BI["SHIN_" + ("R" if S == "L" else "L")]) & \
           (lab != BI["THIGH_" + ("R" if S == "L" else "L")])
    t = tL[S][zone] - CFG["cone"] * np.clip(dL[S][zone] - 0.075, 0.0, None)
    g1 = smoothstep(t, *LEGRAMP["root"])
    g2 = smoothstep(t, *LEGRAMP["knee"])
    g3 = smoothstep(t, *LEGRAMP["ankle"])
    Wt[zone] = 0.0
    Wt[zone, BI["HIPS"]] = 1.0 - g1
    Wt[zone, BI["THIGH_" + S]] = g1 * (1.0 - g2)
    Wt[zone, BI["SHIN_" + S]] = g1 * g2 * (1.0 - g3)
    Wt[zone, BI["FOOT_" + S]] = g1 * g2 * g3
    legfix += int(zone.sum())
    REP["legzone_" + S] = {"n": int(zone.sum()),
                           "t_range": [round(float(t.min()), 4), round(float(t.max()), 4)],
                           "max_hips_on_shin_label": round(float(
                               Wt[(lab == BI["SHIN_" + S]), BI["HIPS"]].max()), 6)}
REP["leg_override_verts"] = legfix

# ---------------------------------------------------------------- torso
# The torso is one smooth egg.  Near-rigid HIPS/SPINE_01/CHEST chunks joined by
# 5 cm bands shear visibly under a 45 deg chest twist, so the three spine bones
# are re-distributed as a continuous gradient in Z.  Only their SHARE changes -
# the mass already held by arm / leg / neck / head bones is left untouched, so
# the result stays continuous everywhere, including at the shoulder and belt.
TZ = {"hips_spine": (0.780, 1.000), "spine_chest": (0.960, 1.220)}
REP["torso_ramps"] = TZ
SPINE3 = [BI["HIPS"], BI["SPINE_01"], BI["CHEST"]]
mt = Wt[:, SPINE3].sum(1)
tm = mt > 1e-6
gA = smoothstep(Z[tm], *TZ["hips_spine"])
gB = smoothstep(Z[tm], *TZ["spine_chest"])
Wt[tm, BI["HIPS"]] = mt[tm] * (1.0 - gA)
Wt[tm, BI["SPINE_01"]] = mt[tm] * gA * (1.0 - gB)
Wt[tm, BI["CHEST"]] = mt[tm] * gA * gB
REP["torso_override_verts"] = int(tm.sum())
REP["belt_still_pure_hips"] = round(float(Wt[BELT, BI["HIPS"]].min()), 6)

REP["pre_prune_influences"] = [int(((Wt > 0.004).sum(1) == k).sum()) for k in range(9)]
Wt[Wt < 0.004] = 0.0
if NB > 4:
    part = np.argpartition(-Wt, 4, axis=1)[:, 4:]
    REP["max_dropped_weight"] = float(np.take_along_axis(Wt, part, axis=1).max())
    np.put_along_axis(Wt, part, 0.0, axis=1)
ssum = Wt.sum(1)
bad = ssum <= 1e-9
if bad.any():
    Wt[bad] = 0.0
    Wt[bad, lab[bad]] = 1.0
    ssum = Wt.sum(1)
Wt = (Wt.T / ssum).T
Wt = np.round(Wt.astype(np.float64), 5)
dom = np.argmax(Wt, axis=1)
Wt[np.arange(N), dom] = 0.0
Wt[np.arange(N), dom] = 1.0 - Wt.sum(1)

nz = Wt > 0
REP["influence_hist"] = [int((nz.sum(1) == k).sum()) for k in range(6)]
REP["zero_weight_verts"] = int((Wt.sum(1) < 1e-6).sum())
REP["max_sum_error"] = float(np.abs(Wt.sum(1) - 1.0).max())
REP["min_weight"] = float(Wt[nz].min())
REP["verts_per_group"] = {BONES[j]: int(nz[:, j].sum()) for j in range(NB)}

# ============================================================ purity checks
def purity(mask, bone, tag):
    w = Wt[mask]
    other = w.sum(1) - w[:, BI[bone]]
    return {tag: {"n": int(mask.sum()),
                  "min_w_" + bone: round(float(w[:, BI[bone]].min()), 6),
                  "max_other": round(float(other.max()), 6)}}


pc = {}
pc.update(purity(CLUB, "WEAPON", "club"))
pc.update(purity(HANDRB, "HAND_R", "ring_hand"))
pc.update(purity(HANDLB, "HAND_L", "hand_L_ball"))
pc.update(purity((Z >= 1.375) & (lab == BI["HEAD"]), "HEAD", "head_core"))
pc.update(purity(lab == BI["FOOT_R"], "FOOT_R", "foot_R"))
pc.update(purity(lab == BI["FOOT_L"], "FOOT_L", "foot_L"))
pc.update(purity(BELT, "HIPS", "belt"))
REP["purity"] = pc

TORSOHEAD = (lab == BI["HIPS"]) | (lab == BI["SPINE_01"]) | (lab == BI["CHEST"]) | \
            (lab == BI["NECK"]) | (lab == BI["HEAD"])
REP["bleed"] = {
    "torso_head_max_WEAPON": round(float(Wt[TORSOHEAD, BI["WEAPON"]].max()), 6),
    "torso_head_max_HAND_R": round(float(Wt[TORSOHEAD, BI["HAND_R"]].max()), 6),
    "torso_head_max_HAND_L": round(float(Wt[TORSOHEAD, BI["HAND_L"]].max()), 6),
    "club_max_nonweapon": round(float((Wt[CLUB].sum(1) - Wt[CLUB, BI["WEAPON"]]).max()), 6),
    "handL_ball_max_HIPS": round(float(Wt[HANDLB, BI["HIPS"]].max()), 6),
    "shin_max_HIPS": round(float(Wt[(lab == BI["SHIN_L"]) | (lab == BI["SHIN_R"]),
                                   BI["HIPS"]].max()), 6),
    "head_max_UPPERARM_R": round(float(Wt[lab == BI["HEAD"], BI["UPPERARM_R"]].max()), 6),
    "head_max_UPPERARM_L": round(float(Wt[lab == BI["HEAD"], BI["UPPERARM_L"]].max()), 6),
    "torso_max_UPPERARM_R": round(float(Wt[(lab == BI["CHEST"]) | (lab == BI["SPINE_01"]),
                                           BI["UPPERARM_R"]].max()), 6),
    "torsobottom_max_THIGH_R": round(float(Wt[(lab == BI["HIPS"]) & (Z > 0.50),
                                              BI["THIGH_R"]].max()), 6),
}
# geometric (label-independent) cross-bleed checks
ARMW = Wt[:, BI["UPPERARM_L"]] + Wt[:, BI["FOREARM_L"]] + Wt[:, BI["HAND_L"]] + \
       Wt[:, BI["UPPERARM_R"]] + Wt[:, BI["FOREARM_R"]] + Wt[:, BI["HAND_R"]]
LEGW = sum(Wt[:, BI[n]] for n in ("THIGH_L", "SHIN_L", "FOOT_L",
                                  "THIGH_R", "SHIN_R", "FOOT_R"))
TORSO_GEO = (rT < 0.34) & (Z > 0.45) & (Z < 1.30)      # inside the egg envelope
HEAD_GEO = (Z > 1.34) & (~CLUB)
REP["bleed_geo"] = {
    "torso_surface_max_arm_w": round(float(ARMW[TORSO_GEO].max()), 6),
    "torso_surface_n_arm_gt_0.05": int((ARMW[TORSO_GEO] > 0.05).sum()),
    "head_max_arm_w": round(float(ARMW[HEAD_GEO].max()), 6),
    "head_max_weapon_w": round(float(Wt[HEAD_GEO, BI["WEAPON"]].max()), 6),
    "torso_above_0.50_max_leg_w": round(float(LEGW[(rT < 0.40) & (Z > 0.50)].max()), 6),
    "shin_geo_max_HIPS": round(float(Wt[(Z < 0.30) & (Z > 0.17), BI["HIPS"]].max()), 6),
    "handL_ball_max_torso_w": round(float((Wt[HANDLB, BI["HIPS"]] +
                                           Wt[HANDLB, BI["SPINE_01"]]).max()), 6),
}

# ============================================================ bone heat (evidence)
for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
body.select_set(True)
t0 = time.time()
heat_err = None
try:
    bpy.ops.object.parent_set(type='ARMATURE_AUTO')
except Exception as e:
    heat_err = str(e)
REP["bone_heat"] = {"seconds": round(time.time() - t0, 1), "error": heat_err,
                    "groups": len(body.vertex_groups)}
if body.vertex_groups:
    gi = {g_.index: g_.name for g_ in body.vertex_groups}
    hw = np.zeros((N, NB), dtype=np.float32)
    for v in me.vertices:
        for gg in v.groups:
            nm = gi[gg.group]
            if nm in BI:
                hw[v.index, BI[nm]] = gg.weight
    REP["bone_heat"]["bleed"] = {
        "club_max_nonweapon": round(float((hw[CLUB].sum(1) - hw[CLUB, BI["WEAPON"]]).max()), 4),
        "ring_hand_max_WEAPON": round(float(hw[HANDRB, BI["WEAPON"]].max()), 4),
        "head_max_UPPERARM_R": round(float(hw[lab == BI["HEAD"], BI["UPPERARM_R"]].max()), 4),
        "torso_head_max_WEAPON": round(float(hw[TORSOHEAD, BI["WEAPON"]].max()), 4),
        "max_influences": int((hw > 1e-4).sum(1).max()),
    }
# discard bone heat entirely
for g_ in list(body.vertex_groups):
    body.vertex_groups.remove(g_)
for m in list(body.modifiers):
    body.modifiers.remove(m)
body.parent = None
body.matrix_world = body.matrix_world.Identity(4)

# ============================================================ write groups
VG = {n: body.vertex_groups.new(name=n) for n in BONES}
t0 = time.time()
for j, n in enumerate(BONES):
    col = Wt[:, j]
    idx = np.nonzero(col)[0]
    # bucket identical (rounded) weights so we issue few add() calls
    order2 = np.argsort(col[idx], kind="stable")
    idx = idx[order2]
    vals = col[idx]
    starts = np.nonzero(np.diff(vals))[0] + 1
    for a, b in zip(np.concatenate([[0], starts]),
                    np.concatenate([starts, [len(idx)]])):
        VG[n].add([int(x) for x in idx[a:b]], float(vals[a]), 'REPLACE')
REP["vgroup_write_seconds"] = round(time.time() - t0, 1)

# parent to the rig + armature modifier
body.parent = rig
body.matrix_parent_inverse = rig.matrix_world.inverted()
md = body.modifiers.new("Armature", 'ARMATURE')
md.object = rig
md.use_vertex_groups = True
md.use_bone_envelopes = False
md.use_deform_preserve_volume = PRESERVE_VOLUME
REP["modifier"] = {"type": md.type, "object": md.object.name,
                   "use_vertex_groups": md.use_vertex_groups,
                   "use_bone_envelopes": md.use_bone_envelopes,
                   "use_deform_preserve_volume": md.use_deform_preserve_volume}

# ============================================================ rest check
bpy.context.view_layer.update()
dg = bpy.context.evaluated_depsgraph_get()
ev = body.evaluated_get(dg)
dmesh = ev.to_mesh()
Q = np.empty(len(dmesh.vertices) * 3)
dmesh.vertices.foreach_get("co", Q)
Q = Q.reshape(-1, 3)
dev = np.linalg.norm(Q - P, axis=1)
REP["rest_max_deviation_m"] = float(dev.max())
REP["rest_mean_deviation_m"] = float(dev.mean())
wi = np.argmax(dev)
REP["rest_worst_vert"] = {"index": int(wi), "co": [round(float(c), 4) for c in P[wi]],
                          "n_infl": int((Wt[wi] > 0).sum()),
                          "bones": {BONES[j]: round(float(Wt[wi, j]), 4)
                                    for j in range(NB) if Wt[wi, j] > 0}}
REP["rest_dev_over_1e5"] = int((dev > 1e-5).sum())
REP["rest_dev_by_infl"] = {str(k): round(float(dev[(Wt > 0).sum(1) == k].max()), 9)
                           for k in range(1, 5) if ((Wt > 0).sum(1) == k).any()}
ev.to_mesh_clear()

REP["has_action"] = bool(rig.animation_data and rig.animation_data.action)
REP["n_actions"] = len(bpy.data.actions)
REP["vgroup_names"] = [g_.name for g_ in body.vertex_groups]
REP["nondeform_groups"] = [n for n in REP["vgroup_names"]
                           if n not in BONES]

np.save(os.path.join(INSP, "step03_labels.npy"), lab)
with open(os.path.join(INSP, "step03_skin_%s.json" % TAG), "w") as f:
    json.dump(REP, f, indent=1, default=str)
print("@@@JSON_START@@@")
print(json.dumps(REP, indent=1, default=str))
print("@@@JSON_END@@@")

bpy.ops.wm.save_as_mainfile(filepath=OUTB)
print("SAVED", OUTB)
