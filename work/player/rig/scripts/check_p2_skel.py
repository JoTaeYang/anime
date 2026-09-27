"""check_p2_skel - Player P2.3 skeleton checker (task T231; spec work/player/d-02-player-rig.md section 2 gate P2.3,
d-01 section 2 skeleton v0 contract; checker side).

Inputs (defaults; override after "--"):
  --canon  work/player/rig/data/canonical_skeleton.json  bones [{name, parent, head, tail, roll, rest_matrix,
                                                          deform (or use_deform), use_connect?}] (list or {name: {}})
  --blend  work/player/rig/pl_r02_skeleton.blend          armature PL_rig + PL_mesh (FACE INT part_id)
  --armature <name>  default PL_rig, else the only armature;  --mesh <name> default the only mesh with part_id
  --parts / --loops / --p0b   parts.json, retopo_loops.json (P2a rings), P0b landmarks
  --out    work/player/inspect/P2/check_p2_skel.json

Methods copied / adapted (goblin files not imported):
  check_g4_skeleton.py  canonical record reading (_canon_bones: list or dict, rest matrix key aliases), world bone
                        data = matrix_world @ head_local / tail_local / matrix_local, roll =
                        Bone.AxisRollFromMatrix(matrix_local 3x3) (G4.8), ring centre = mean of the ring's mesh
                        vertices (G4.3), socket head vs reference point (G4.6)
  check_g2_static.py    least-squares sphere (_fit_sphere, G2.16) for the fist centre
  check_p1.py           v0 contract tree (41 bones), evidence format
  check_p2_deform.py    generic helpers (sha256 / rel / jsonable / mm / rnd / family / bvh_of), imported
Rows: P2.3a contract, P2.3b blend = JSON, P2.3c unconnected, P2.3d symmetry (draft 2 mm, calibration), P2.3e joint
positions (report), P2.3f sockets (WeaponSocket_* at the fist centre <= 2 mm; head top / back surface report).
Nothing is saved: the blend is opened read-only in memory; only the evidence JSON is written.
Run:    blender --background --factory-startup --python check_p2_skel.py [-- <overrides>]
Exit:   0 evidence written; 2 script error or missing input (evidence still written with blocked rows).
Coordinates: front -Y, up +Z, character left (_L) +X.  Lengths in m, reported in mm.
"""
import json
import math
import sys
import traceback
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True   # no __pycache__ next to the scripts
import check_p2_deform as p2d  # noqa: E402  (generic helpers only)

PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
GATE, CHECKER, TASK = "P2.3", "check_p2_skel", "T231"
CALIB = p2d.CALIB
sha256, rel, jsonable, mm, rnd, family = p2d.sha256, p2d.rel, p2d.jsonable, p2d.mm, p2d.rnd, p2d.family

P = {"canon": PLAYER_RIG / "data" / "canonical_skeleton.json",
     "blend": PLAYER_RIG / "pl_r02_skeleton.blend",
     "parts": PLAYER_RIG / "data" / "parts.json",
     "loops": PLAYER_RIG / "data" / "retopo_loops.json",
     "p0b": REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json",
     "armature": None, "mesh": None,
     "out": REPO / "work" / "player" / "inspect" / "P2" / "check_p2_skel.json"}
ARM_DEFAULT = "PL_rig"

POS_TOL = 1e-6            # P2.3b head / tail (m)
ROLL_TOL = 1e-4           # P2.3b roll (rad)
SYM_TOL = 0.002           # P2.3d draft (spec italic 2 mm) -> calibration
SOCKET_TOL = 0.002        # P2.3f WeaponSocket_* head vs fist sphere centre (m)
SKIRT_DIRS = ("F", "FL", "L", "BL", "B", "BR", "R", "FR")
SKIRT_MIRROR = {"FL": "FR", "L": "R", "BL": "BR"}
NON_DEFORM = ("Root", "HeadEquipmentSocket", "WeaponSocket_L", "WeaponSocket_R", "BackSocket", "BackWeaponSocket")


def contract():
    """d-01 section 2 v0 tree (copied from check_p1.contract)."""
    t = [("Root", None), ("Pelvis", "Root"), ("Spine_01", "Pelvis"), ("Spine_02", "Spine_01"),
         ("Neck", "Spine_02"), ("Head", "Neck"), ("HeadEquipmentSocket", "Head")]
    for s in ("L", "R"):
        t += [(f"Clavicle_{s}", "Spine_02"), (f"Arm_{s}", f"Clavicle_{s}"), (f"Forearm_{s}", f"Arm_{s}"),
              (f"Hand_{s}", f"Forearm_{s}"), (f"WeaponSocket_{s}", f"Hand_{s}")]
    t += [("BackSocket", "Spine_02"), ("BackWeaponSocket", "Spine_02")]
    for s in ("L", "R"):
        t += [(f"Thigh_{s}", "Pelvis"), (f"Calf_{s}", f"Thigh_{s}"), (f"Foot_{s}", f"Calf_{s}")]
    for d in SKIRT_DIRS:
        t += [(f"Skirt_{d}_01", "Pelvis"), (f"Skirt_{d}_02", f"Skirt_{d}_01")]
    return dict(t)


PARENT = contract()
NAMES = list(PARENT)

# joint positions (P2.3e report): bone head vs ring centre (goblin G4.3: elbow_x_1 / wrist_x_1 / knee_x_1 /
# ankle_x_1) and vs P0b landmarks
RING_REF = {"Forearm": "elbow_{s}_1", "Hand": "wrist_{s}_1", "Calf": "knee_{s}_1", "Foot": "ankle_{s}_1"}
LM_REF = {"Arm": "shoulder_{s}", "Forearm": "elbow_{s}", "Hand": "wrist_{s}", "Thigh": "hip_{s}", "Calf": "knee_{s}",
          "Foot": "ankle_{s}", "WeaponSocket": "fist_center_{s}"}
LM_CENTRE = {"Pelvis": "pelvis", "Neck": "neck_base", "Head": "head_base"}


class Blocked(Exception):
    pass


def tb_tail(n=4):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


def mmv(v):
    return [mm(c) for c in v]


def fit_sphere(p):
    """least-squares sphere (goblin check_g2_static._fit_sphere)."""
    A = np.column_stack([2 * p, np.ones(len(p))])
    b = (p ** 2).sum(1)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    c = sol[:3]
    return c, float(math.sqrt(max(sol[3] + c @ c, 0.0)))


def canon_bones(canon):
    """goblin check_g4_skeleton._canon_bones, adapted: 'deform' or 'use_deform'; use_connect optional."""
    src = canon.get("bones", canon) if isinstance(canon, dict) else canon
    items = []
    if isinstance(src, dict):
        for name, b in src.items():
            if isinstance(b, dict):
                d = dict(b)
                d.setdefault("name", name)
                items.append(d)
    elif isinstance(src, list):
        items = [dict(b) for b in src if isinstance(b, dict)]
    if not items:
        raise Blocked("canonical_skeleton.json: no bone records (expected 'bones' list / dict)")
    out, missing = [], {}
    for b in items:
        name = b.get("name")
        dk = "deform" if "deform" in b else "use_deform"
        need = [k for k in ("name", "parent", "head", "tail", "roll", dk) if k not in b]
        if need:
            missing[str(name)] = need
            continue
        mat = next((b[k] for k in ("rest_matrix", "matrix_local", "matrix", "rest") if b.get(k) is not None), None)
        con = b.get("use_connect", b.get("connect"))
        out.append({"name": name, "parent": b.get("parent") or None, "head": np.array(b["head"], dtype=np.float64),
                    "tail": np.array(b["tail"], dtype=np.float64), "roll": float(b["roll"]),
                    "deform": bool(b[dk]), "connect": None if con is None else bool(con), "matrix": mat})
    return out, missing


class Ctx:
    def __init__(self):
        self.canon = self.parts = self.loops = self.p0b = None
        self.arm = self.mesh = None
        self.mw = None
        self.V = None
        self.part = None


def bone_world(ctx, n):
    b = ctx.arm.data.bones[n]
    h = np.array(ctx.mw @ b.head_local)
    t = np.array(ctx.mw @ b.tail_local)
    return h, t


def setup(ctx):
    arms = [o for o in bpy.data.objects if o.type == "ARMATURE"]
    if P["armature"]:
        arm = bpy.data.objects.get(P["armature"])
    else:
        arm = bpy.data.objects.get(ARM_DEFAULT)
        if arm is None and len(arms) == 1:
            arm = arms[0]
    if arm is None or arm.type != "ARMATURE":
        raise Blocked(f"armature {P['armature'] or ARM_DEFAULT!r} not found (armatures {[a.name for a in arms]})")
    ctx.arm, ctx.mw = arm, arm.matrix_world.copy()
    if P["mesh"]:
        mo = bpy.data.objects.get(P["mesh"])
    else:
        c = [o for o in bpy.data.objects if o.type == "MESH" and o.data.attributes.get("part_id") is not None]
        mo = c[0] if len(c) == 1 else None
    ctx.mesh = mo
    if mo is not None:
        me = mo.data
        co = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", co)
        M = np.array(mo.matrix_world, dtype=np.float64)
        ctx.V = co.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]
        att = me.attributes.get("part_id")
        if att is not None and att.domain == "FACE":
            pv = np.empty(len(me.polygons), dtype=np.int64)
            att.data.foreach_get("value", pv)
            ctx.part = pv


def part_verts(ctx, name):
    pid = (ctx.parts or {}).get(name)
    if pid is None or ctx.part is None:
        return np.zeros(0, np.int64)
    me = ctx.mesh.data
    return np.unique([v for p in me.polygons if ctx.part[p.index] == pid for v in p.vertices]).astype(np.int64)


# ---------------------------------------------------------------- rows
def p23a(ctx):
    bones, missing = canon_bones(ctx.canon)
    names = [b["name"] for b in bones]
    par = {b["name"]: b["parent"] for b in bones}
    dfm = {b["name"]: b["deform"] for b in bones}
    only_c = sorted(set(NAMES) - set(names))
    only_j = sorted(set(names) - set(NAMES))
    pm = {n: {"contract": PARENT[n], "json": par[n]} for n in NAMES if n in par and par[n] != PARENT[n]}
    dup = sorted({n for n in names if names.count(n) > 1})
    dbad = {n: {"contract": n not in NON_DEFORM, "json": dfm[n]} for n in NAMES if n in dfm and
            dfm[n] != (n not in NON_DEFORM)}
    rec = {"n_bones": len(names), "n_contract": len(NAMES), "only_contract": only_c, "only_json": only_j,
           "parent_mismatch": pm, "duplicates": dup, "deform_mismatch": dbad, "records_missing_fields": missing,
           "non_deform_contract": list(NON_DEFORM),
           "json_top_level_keys_report": sorted(ctx.canon) if isinstance(ctx.canon, dict) else "list"}
    ok = len(names) == 41 and not only_c and not only_j and not pm and not dup and not dbad and not missing
    return rec, ok, bones


def p23b(ctx, bones):
    ab = ctx.arm.data.bones
    per, worst = {}, {"head_m": (0.0, None), "tail_m": (0.0, None), "roll_rad": (0.0, None)}
    field_mis, mat_d, not_in_rig = {}, {}, []
    for b in bones:
        n = b["name"]
        if n not in ab:
            not_in_rig.append(n)
            continue
        h, t = bone_world(ctx, n)
        _axis, roll = bpy.types.Bone.AxisRollFromMatrix(ab[n].matrix_local.to_3x3())
        dr = abs((float(roll) - b["roll"] + math.pi) % (2 * math.pi) - math.pi)
        dh, dt = float(np.abs(h - b["head"]).max()), float(np.abs(t - b["tail"]).max())
        per[n] = (dh, dt, dr)
        for k, v in (("head_m", dh), ("tail_m", dt), ("roll_rad", dr)):
            if v > worst[k][0]:
                worst[k] = (v, n)
        got = {"parent": ab[n].parent.name if ab[n].parent else None, "deform": bool(ab[n].use_deform)}
        want = {"parent": b["parent"], "deform": b["deform"]}
        diff = {k: {"json": want[k], "rig": got[k]} for k in got if got[k] != want[k]}
        if diff:
            field_mis[n] = diff
        if b["matrix"] is not None:
            try:
                m = np.array(b["matrix"], dtype=np.float64)
                mat_d[n] = float(np.abs(m - np.array(ctx.mw @ ab[n].matrix_local)).max()) if m.shape == (4, 4) \
                    else "not 4x4"
            except (TypeError, ValueError) as e:
                mat_d[n] = f"unreadable: {e}"
    only_rig = sorted(set(ab.keys()) - {b["name"] for b in bones})
    over = {n: {"head_mm": mm(v[0]), "tail_mm": mm(v[1]), "roll_rad": float(f"{v[2]:.3e}")}
            for n, v in per.items() if v[0] > POS_TOL or v[1] > POS_TOL or v[2] > ROLL_TOL}
    num = [v for v in mat_d.values() if isinstance(v, float)]
    mwd = float(np.abs(np.array(ctx.mw) - np.eye(4)).max())
    rec = {"armature": ctx.arm.name, "n_rig_bones": len(ab), "n_compared": len(per),
           "max_head_diff_m": float(f"{worst['head_m'][0]:.3e}"), "worst_head": worst["head_m"][1],
           "max_tail_diff_m": float(f"{worst['tail_m'][0]:.3e}"), "worst_tail": worst["tail_m"][1],
           "max_roll_diff_rad": float(f"{worst['roll_rad'][0]:.3e}"), "worst_roll": worst["roll_rad"][1],
           "over_tol": over, "field_mismatch": field_mis, "json_bones_not_in_rig": not_in_rig,
           "rig_bones_not_in_json": only_rig,
           "rest_matrix_max_abs_diff_report": float(f"{max(num):.3e}") if num else None,
           "rest_matrix_n": len(mat_d), "armature_matrix_world_dev_from_identity": float(f"{mwd:.3e}")}
    ok = not over and not field_mis and not not_in_rig and not only_rig and len(per) == len(bones)
    return rec, ok


def p23c(ctx, bones):
    rig = sorted(b.name for b in ctx.arm.data.bones if b.use_connect)
    js = sorted(b["name"] for b in bones if b["connect"])
    return {"rig_connected": rig, "json_connected": js,
            "json_connect_field_present": sum(1 for b in bones if b["connect"] is not None)}, not rig and not js


def p23d(ctx):
    ab = ctx.arm.data.bones
    pairs, rows = [], {}
    for n in ab.keys():
        if n.endswith("_L"):
            pairs.append((n, n[:-2] + "_R"))
    for a, b in SKIRT_MIRROR.items():
        for k in ("01", "02"):
            pairs.append((f"Skirt_{a}_{k}", f"Skirt_{b}_{k}"))
    vals = []
    for a, b in pairs:
        if a not in ab or b not in ab:
            rows[f"{a}|{b}"] = "missing"
            continue
        ha, ta = bone_world(ctx, a)
        hb, tb = bone_world(ctx, b)
        m = np.array([-1.0, 1.0, 1.0])
        dh, dt = float(np.linalg.norm(ha * m - hb)), float(np.linalg.norm(ta * m - tb))
        rows[f"{a}|{b}"] = {"head_mm": mm(dh), "tail_mm": mm(dt)}
        vals += [dh, dt]
    centre = {}
    for n in ab.keys():
        if not n.endswith(("_L", "_R")) and not any(n.startswith(f"Skirt_{d}_") for d in SKIRT_MIRROR.values()) \
                and not any(n.startswith(f"Skirt_{d}_") for d in SKIRT_MIRROR):
            h, t = bone_world(ctx, n)
            centre[n] = {"head_x_mm": mm(h[0]), "tail_x_mm": mm(t[0])}
            vals += [abs(h[0]), abs(t[0])]
    mx = max(vals) if vals else None
    return {"max_mm": mm(mx), "draft_threshold_mm": SYM_TOL * 1000, "within_draft": mx is not None and mx <= SYM_TOL,
            "pairs": rows, "centre_bones_abs_x": centre,
            "rule": "mirror x -> -x of the _L (Skirt_FL/L/BL) head / tail vs the _R (FR/R/BR) one; centre bones |x|"}, True


def ring_centre(ctx, name):
    r = ((ctx.loops or {}).get("rings") or {}).get(name)
    if not isinstance(r, dict) or ctx.V is None:
        return None
    vs = np.array(r.get("verts") or [], dtype=np.int64)
    if not len(vs) or vs.max() >= len(ctx.V):
        return None
    return ctx.V[vs].mean(axis=0)


def p23e(ctx):
    lm = (ctx.p0b or {}).get("landmarks") or {}
    rows = {}
    for s, x in (("L", "l"), ("R", "r")):
        for b in ("Arm", "Forearm", "Hand", "Thigh", "Calf", "Foot", "WeaponSocket"):
            n = f"{b}_{s}"
            if n not in ctx.arm.data.bones:
                rows[n] = "missing bone"
                continue
            h, _t = bone_world(ctx, n)
            r = {"head_mm": mmv(h)}
            if b in RING_REF:
                rn = RING_REF[b].format(s=x)
                c = ring_centre(ctx, rn)
                r["ring"] = rn
                r["ring_centre_dist_mm"] = mm(np.linalg.norm(h - c)) if c is not None else "ring unavailable"
                if c is not None:
                    r["ring_centre_delta_mm"] = mmv(h - c)
            if b in LM_REF and LM_REF[b].format(s=x) in lm:
                ln = LM_REF[b].format(s=x)
                r["p0b_landmark"] = ln
                r["p0b_dist_mm"] = mm(np.linalg.norm(h - np.array(lm[ln]["pos"])))
            rows[n] = r
    for b, ln in LM_CENTRE.items():
        if b in ctx.arm.data.bones and ln in lm:
            h, _t = bone_world(ctx, b)
            rows[b] = {"head_mm": mmv(h), "p0b_landmark": ln, "p0b_dist_mm": mm(np.linalg.norm(h - np.array(lm[ln]["pos"])))}
    # chain straightness and bend direction of local +X (main request, T231 run)
    chains = {}
    ab = ctx.arm.data.bones
    for s in ("L", "R"):
        for prox, dist in (("Arm", "Forearm"), ("Thigh", "Calf")):
            a, b = f"{prox}_{s}", f"{dist}_{s}"
            if a not in ab or b not in ab:
                continue
            ha, ta = bone_world(ctx, a)
            hb, tb = bone_world(ctx, b)
            da, db = p2d.unit(ta - ha), p2d.unit(tb - hb)
            z = np.array((ctx.mw.to_3x3() @ ab[b].matrix_local.to_3x3()).col[2])
            z = p2d.unit(z)
            chains[f"{a}->{b}"] = {
                "bend_deg": rnd(p2d.vec_angle_deg(da, db), 3),
                f"{b}_plus_X_rotation_moves_tail_toward": [rnd(c, 4) for c in z],
                f"{b}_plus_X_tail_moves": "forward (-Y)" if z[1] < 0 else "back (+Y)",
                "goblin_G4.4_convention": "lowerarm +X: hand -Y; calf +X: foot +Y"}
    return {"bones": rows, "chains": chains, "mesh": ctx.mesh.name if ctx.mesh else None,
            "ring_rule": "ring centre = mean of the ring's mesh vertices (retopo_loops.json indices, goblin G4.3)",
            "chain_rule": "bend = angle between the proximal and distal bone head->tail directions; a positive "
                          "rotation about the distal bone local +X moves its tail along its local +Z (world)"}, True


def p23f(ctx):
    rows, ok = {}, True
    for s, x in (("L", "l"), ("R", "r")):
        n = f"WeaponSocket_{s}"
        fv = part_verts(ctx, f"fist_{x}")
        if n not in ctx.arm.data.bones or len(fv) < 4:
            rows[n] = f"blocked: bone present {n in ctx.arm.data.bones}, fist_{x} verts {len(fv)}"
            ok = False
            continue
        c, r = fit_sphere(ctx.V[fv])
        h, _t = bone_world(ctx, n)
        err = float(np.linalg.norm(h - c))
        rows[n] = {"head_mm": mmv(h), "fist_sphere_centre_mm": mmv(c), "fist_sphere_r_mm": mm(r),
                   "err_mm": mm(err), "within": err <= SOCKET_TOL}
        ok = ok and err <= SOCKET_TOL
    hv = part_verts(ctx, "head")
    n = "HeadEquipmentSocket"
    if n in ctx.arm.data.bones and len(hv):
        h, _t = bone_world(ctx, n)
        top = ctx.V[hv][np.argmax(ctx.V[hv][:, 2])]
        rows[n] = {"report": True, "head_mm": mmv(h), "head_top_vertex_mm": mmv(top),
                   "dist_to_top_vertex_mm": mm(np.linalg.norm(h - top)), "dz_mm": mm(h[2] - top[2])}
    tp = []
    tpid = (ctx.parts or {}).get("tunic")
    if tpid is not None and ctx.part is not None:
        me = ctx.mesh.data
        me.calc_loop_triangles()
        tv = np.array([t.vertices[:] for t in me.loop_triangles if ctx.part[t.polygon_index] == tpid])
        tp = tv
    if len(tp):
        bvh = p2d.bvh_of(ctx.V, tp)
        for n in ("BackSocket", "BackWeaponSocket"):
            if n not in ctx.arm.data.bones:
                continue
            h, t = bone_world(ctx, n)
            loc, nrm, _i, d = bvh.find_nearest(Vector(h))
            side = float((h - np.array(loc)) @ np.array(nrm)) if loc is not None else None
            rows[n] = {"report": True, "head_mm": mmv(h), "nearest_tunic_mm": mmv(loc) if loc else None,
                       "dist_to_tunic_mm": mm(d) if loc else None,
                       "outside_side_of_surface": (side > 0) if side is not None else None,
                       "surface_normal": [rnd(c, 4) for c in nrm] if nrm else None,
                       "tail_dir": [rnd(c, 4) for c in p2d.unit(t - h)]}
    return {"sockets": rows, "weapon_tol_mm": SOCKET_TOL * 1000,
            "rule": "fist centre = least-squares sphere of the fist_x part vertices (goblin G2.16 fit); head top = "
                    "highest head-part vertex; back sockets: nearest tunic surface point, side = sign of (head - "
                    "nearest) . face normal"}, ok


IDS = (
    ("P2.3a", "canonical JSON = d-01 section 2 v0 contract: 41 names, parents, deform flags (Root + 5 sockets "
              "non-deform), every record with name / parent / head / tail / roll / deform", ""),
    ("P2.3b", f"blend armature = JSON: head / tail <= {POS_TOL:g} m (world), roll <= {ROLL_TOL:g} rad "
              "(Bone.AxisRollFromMatrix of matrix_local), parent and deform equal, same bone set",
     "rest_matrix vs matrix_world @ matrix_local reported; armature transform deviation reported"),
    ("P2.3c", "every bone use_connect False (rig; JSON use_connect / connect when given)", ""),
    ("P2.3d", f"(draft) L/R symmetry <= {SYM_TOL * 1000:g} mm (mirrored head / tail; centre bones |x|)", CALIB),
    ("P2.3e", "(report) joint heads vs P2a ring centres (elbow / wrist / knee / ankle _1) and P0b landmarks", CALIB),
    ("P2.3f", f"WeaponSocket_L/R head within {SOCKET_TOL * 1000:g} mm of the fist sphere centre; HeadEquipmentSocket "
              "vs head top and back sockets vs the back surface reported", ""),
)


class Evidence(p2d.Evidence):
    def write(self):
        out = Path(P["out"])
        out.parent.mkdir(parents=True, exist_ok=True)
        order = {i: k for k, (i, _t, _n) in enumerate(IDS)}
        self.criteria.sort(key=lambda c: order.get(c["id"], 99))
        doc = {"gate": GATE, "checker": CHECKER, "task": TASK, "blender": bpy.app.version_string,
               **jsonable(self.extra), "inputs": self.inputs, "criteria": self.criteria}
        with open(out, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']} measured={json.dumps(c['measured'])[:400]}")
        print(f"[{GATE}/{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


_STATE = {"missing": []}


def parse_args(argv):
    keys = {"--canon": "canon", "--blend": "blend", "--parts": "parts", "--loops": "loops", "--p0b": "p0b",
            "--armature": "armature", "--mesh": "mesh", "--out": "out"}
    i = 0
    while i < len(argv):
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: {' '.join(k + ' <value>' for k in keys)}")
        k = keys[argv[i]]
        P[k] = argv[i + 1] if k in ("armature", "mesh") else Path(argv[i + 1])
        i += 2


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    ev.add_input(__file__)
    ev.add_input(p2d.__file__)
    ctx = Ctx()
    th = {i: t for i, t, _n in IDS}
    nt = {i: n for i, _t, n in IDS}
    ids = [i for i, _t, _n in IDS]
    done = set()

    def put(cid, measured, ok):
        ev.criterion(cid, measured, th[cid], ok, nt[cid])
        done.add(cid)

    def block(cids, reason):
        for c in cids:
            if c not in done:
                put(c, f"blocked: {reason}", False)

    try:
        miss = [rel(P[k]) for k in ("canon", "blend", "parts", "loops", "p0b") if not Path(P[k]).exists()]
        for k in ("canon", "blend", "parts", "loops", "p0b"):
            ev.add_input(P[k])
        for q in sorted(HERE.glob("p2*.py")):
            ev.add_input(q)
        _STATE["missing"] = miss
        if not Path(P["canon"]).exists() or not Path(P["blend"]).exists():
            block(ids, "missing input " + ", ".join(miss))
            return
        ctx.canon = p2d.load_json(P["canon"])
        raw = p2d.load_json(P["parts"]) if Path(P["parts"]).exists() else None
        ctx.parts = raw["part_id"] if isinstance(raw, dict) and isinstance(raw.get("part_id"), dict) else raw
        ctx.loops = p2d.load_json(P["loops"]) if Path(P["loops"]).exists() else None
        ctx.p0b = p2d.load_json(P["p0b"]) if Path(P["p0b"]).exists() else None
        bpy.ops.wm.open_mainfile(filepath=str(P["blend"]), load_ui=False)
        try:
            setup(ctx)
        except Blocked as e:
            block(ids, str(e))
            return
        ev.extra["objects"] = {"armature": ctx.arm.name, "mesh": ctx.mesh.name if ctx.mesh else None}
        try:
            rec, ok, bones = p23a(ctx)
            put("P2.3a", rec, ok)
        except Blocked as e:
            block(ids, str(e))
            return
        for cid, fn, args in (("P2.3b", p23b, (ctx, bones)), ("P2.3c", p23c, (ctx, bones)), ("P2.3d", p23d, (ctx,)),
                              ("P2.3e", p23e, (ctx,)), ("P2.3f", p23f, (ctx,))):
            try:
                put(cid, *fn(*args))
            except Blocked as e:
                block([cid], str(e))
    finally:
        try:
            block(ids, "checker aborted before this criterion: " + tb_tail(2) if sys.exc_info()[0] else
                  "checker aborted before this criterion")
        finally:
            ev.write()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(2)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
