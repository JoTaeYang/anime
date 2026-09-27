"""check_p2_skin - Player P2.4 skin checker (task T231; spec work/player/d-02-player-rig.md section 2 gate P2.4,
checker side).

Inputs (defaults; override after "--"):
  --blend  work/player/rig/pl_r03_skinned.blend   PL_mesh (FACE INT part_id, vertex groups, one Armature modifier)
                                                   + armature (PL_rig)
  --parts / --loops / --p0b   parts.json, retopo_loops.json (tunic rows / columns / inner-outer pairs), P0b
  --mesh / --armature <name>  default: the only mesh with part_id / its Armature modifier object
  --out    work/player/inspect/P2/check_p2_skin.json   (sheet: <out dir>/P2b_joint_sheet.png)

Weight rows (obj.data vertex groups; nonzero = weight > 0, 'carries' = weight > 1e-4):
  P2.4a  every vertex weighted, sum 1 +- 1e-6, max influences <= 4, vertex groups = deform bones only, non-deform
         bones (Root, sockets) weight 0, Armature modifier -> the armature with vertex groups
         (goblin check_g5_skin.check_g51, tolerances from the task)
  P2.4b  rigid parts 100 % on one bone, table = p21_temprig.RIGID (head, eyes -> Head; fists -> Hand_x; shoes, cuffs
         -> Foot_x; belt, pouch -> Pelvis; sleeves -> Arm_x; scarf, scarf_tail -> Spine_02); expected >= 1 - 1e-4,
         every other <= 1e-4 (goblin check_g5_skin.check_g52 RIGID_TOL)
  P2.4c  tunic weights only on Spine_01 / Spine_02 / Pelvis / Neck / Clavicle_L / Clavicle_R / Skirt_*
  P2.4d  skirt ring vertices (retopo_loops.json tunic outer / inner rows hem_outer, hem_inner, skirt_<k>[_inner])
         only on Skirt_*; belt_bot row Skirt_* + Pelvis; per azimuth column (tunic.columns) bones reported
  P2.4e  inner wall = outer partner weights (tunic.inner_rows paired_outer_row, same position in the row):
         max |w_inner - w_outer| over all bones <= 1e-6
  P2.4f  Neck, Clavicle_L, Clavicle_R carry weights; weighted vertex count of every deform bone reported
  P2.4g  rest deformation (evaluated mesh at rest pose vs obj.data) <= 0.001 mm
Deform rows on the final skin (P2.2 metrics, functions imported from check_p2_deform.py, same pose set, same
  definitions, T223 visible / hidden / designed rules): P2.4h poses (= P2.2a), P2.4i_* tube ratio (= P2.2b; draft 0.8
  calibration, goblin final reference >= 0.75 at 60 deg reported), P2.4j_* visible self-intersection (= P2.2c), P2.4k_*
  shoulder sleeve vs tunic / scarf visible / hidden with depth and rest (= P2.2d), P2.4l_* hip leg vs skirt incl.
  poke-out (= P2.2e), P2.4m [U] <out dir>/P2b_joint_sheet.png (= P2.2f layout).
Nothing is saved: the blend is opened read-only, poses in memory only; only the evidence JSON and the sheet are
written.
Run:    blender --background --factory-startup --python check_p2_skin.py [-- <overrides>]
Exit:   0 evidence written; 2 script error or missing input (evidence still written with blocked rows).
"""
import json
import re
import shutil
import sys
import tempfile
import traceback
from pathlib import Path

import bpy
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True   # no __pycache__ next to the scripts
import check_p2_deform as p2d  # noqa: E402  (P2.2 metrics, reused unchanged)

PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
GATE, CHECKER, TASK = "P2.4", "check_p2_skin", "T231"
CALIB = p2d.CALIB
rel, jsonable, mm, rnd = p2d.rel, p2d.jsonable, p2d.mm, p2d.rnd

P = {"blend": PLAYER_RIG / "pl_r03_skinned.blend",
     "parts": PLAYER_RIG / "data" / "parts.json",
     "loops": PLAYER_RIG / "data" / "retopo_loops.json",
     "p0b": REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json",
     "mesh": None, "armature": None,
     "out": REPO / "work" / "player" / "inspect" / "P2" / "check_p2_skin.json"}
SHEET_NAME = "P2b_joint_sheet.png"

MAX_INF = 4
SUM_TOL = 1e-6
CARRY = 1e-4
RIGID_TOL = 1e-4
PAIR_TOL = 1e-6
REST_TOL = 1e-6           # m (0.001 mm)
GOBLIN_60 = 0.75          # goblin G5.5 final 60 deg (user decision 2026-09-25), reference only
NON_DEFORM = ("Root", "HeadEquipmentSocket", "WeaponSocket_L", "WeaponSocket_R", "BackSocket", "BackWeaponSocket")
RIGID = {"head": "Head", "eye_l": "Head", "eye_r": "Head", "fist_l": "Hand_L", "fist_r": "Hand_R",
         "shoe_l": "Foot_L", "shoe_r": "Foot_R", "cuff_l": "Foot_L", "cuff_r": "Foot_R",
         "belt": "Pelvis", "pouch": "Pelvis", "sleeve_l": "Arm_L", "sleeve_r": "Arm_R",
         "scarf": "Spine_02", "scarf_tail": "Spine_02"}          # p21_temprig.RIGID
TUNIC_OK = ("Spine_01", "Spine_02", "Pelvis", "Neck", "Clavicle_L", "Clavicle_R")
SKIRT_ROW = re.compile(r"^(hem_outer|hem_inner|skirt_\d+|skirt_\d+_inner)$")
CARRY_BONES = ("Neck", "Clavicle_L", "Clavicle_R")
ID_MAP = {"P2.2a": "P2.4h", "P2.2b": "P2.4i", "P2.2c": "P2.4j", "P2.2d": "P2.4k", "P2.2e": "P2.4l", "P2.2f": "P2.4m"}


class Blocked(Exception):
    pass


def tb_tail(n=4):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


def read_weights(mo):
    """list per vertex of {group name: weight} (goblin check_g5_skin.read_weights)."""
    names = {vg.index: vg.name for vg in mo.vertex_groups}
    return [{names.get(g.group, f"#{g.group}"): float(g.weight) for g in v.groups} for v in mo.data.vertices]


def vertex_parts(mo, parts):
    me = mo.data
    pv = np.empty(len(me.polygons), dtype=np.int64)
    me.attributes["part_id"].data.foreach_get("value", pv)
    inv = {v: k for k, v in parts.items()}
    out = {}
    for poly in me.polygons:
        n = inv.get(int(pv[poly.index]), f"part_{int(pv[poly.index])}")
        for v in poly.vertices:
            out.setdefault(n, set()).add(v)
    return {k: np.array(sorted(v), dtype=np.int64) for k, v in out.items()}


def wstr(d):
    return {k: rnd(w, 4) for k, w in sorted(d.items(), key=lambda kv: -kv[1]) if w > CARRY}


# ---------------------------------------------------------------- weight rows
def p24a(ctx, W):
    arm, mo = ctx.arm, ctx.mesh
    deform = {b.name for b in arm.data.bones if b.use_deform}
    cnt = np.array([sum(1 for w in d.values() if w > 0) for d in W])
    sums = np.array([sum(d.values()) for d in W])
    over = np.nonzero(cnt > MAX_INF)[0]
    bad = np.nonzero(np.abs(sums - 1.0) > SUM_TOL)[0]
    unw = np.nonzero(cnt == 0)[0]
    vg = [g.name for g in mo.vertex_groups]
    nd = {n: {"group": n in vg, "max_weight": max((d.get(n, 0.0) for d in W), default=0.0)} for n in NON_DEFORM}
    mods = [{"name": m.name, "object": m.object.name if m.object else None, "use_vertex_groups": m.use_vertex_groups,
             "use_bone_envelopes": m.use_bone_envelopes} for m in mo.modifiers if m.type == "ARMATURE"]
    mod_ok = len(mods) == 1 and mods[0]["object"] == arm.name and mods[0]["use_vertex_groups"]
    rec = {"n_verts": len(W), "unweighted": int(len(unw)), "unweighted_examples": unw[:10].tolist(),
           "max_influences": int(cnt.max()) if len(cnt) else None,
           "influence_histogram": {int(k): int(c) for k, c in zip(*np.unique(cnt, return_counts=True))},
           "n_over_4": int(len(over)), "sum_min": float(f"{sums.min():.9f}") if len(sums) else None,
           "sum_max": float(f"{sums.max():.9f}") if len(sums) else None, "n_sum_off": int(len(bad)),
           "sum_off_examples": bad[:10].tolist(), "groups_not_deform_bone": sorted(set(vg) - deform),
           "non_deform_bones": nd, "armature_modifiers": mods,
           "other_modifiers_report": [m.type for m in mo.modifiers if m.type != "ARMATURE"]}
    ok = (not len(unw) and not len(over) and not len(bad) and not rec["groups_not_deform_bone"]
          and all(v["max_weight"] == 0.0 for v in nd.values()) and mod_ok)
    return rec, ok


def p24b(ctx, W, vp):
    res, ok = {}, True
    for part, bone in RIGID.items():
        vs = vp.get(part, np.zeros(0, np.int64))
        n_bad, min_e, max_o, others, ex = 0, 1.0, 0.0, {}, []
        for i in vs.tolist():
            d = W[i]
            e = d.get(bone, 0.0)
            oth = {k: w for k, w in d.items() if k != bone and w > 0.0}
            mo_ = max(oth.values()) if oth else 0.0
            min_e, max_o = min(min_e, e), max(max_o, mo_)
            for k, w in oth.items():
                if w > RIGID_TOL:
                    others[k] = others.get(k, 0) + 1
            if e < 1.0 - RIGID_TOL or mo_ > RIGID_TOL:
                n_bad += 1
                if len(ex) < 10:
                    ex.append(i)
        good = len(vs) > 0 and n_bad == 0
        ok &= good
        res[part] = {"bone": bone, "n_verts": int(len(vs)), "n_bad": n_bad, "min_expected_weight": rnd(min_e, 7),
                     "max_other_weight": rnd(max_o, 7), "other_bones_nverts": others, "bad_examples": ex}
    return res, ok


def p24c(ctx, W, vp):
    vs = vp.get("tunic", np.zeros(0, np.int64))
    bad, by = [], {}
    for i in vs.tolist():
        off = [k for k, w in W[i].items() if w > 0.0 and k not in TUNIC_OK and not k.startswith("Skirt_")]
        for k in off:
            by[k] = by.get(k, 0) + 1
        if off:
            bad.append(i)
    used = {}
    for i in vs.tolist():
        for k, w in W[i].items():
            if w > 0.0:
                used[k] = used.get(k, 0) + 1
    return {"n_tunic_verts": int(len(vs)), "n_verts_other_bones": len(bad), "other_bones_nverts": by,
            "examples": bad[:10], "bones_used_nverts": dict(sorted(used.items())),
            "allowed": list(TUNIC_OK) + ["Skirt_*"]}, len(vs) > 0 and not bad


def p24d(ctx, W):
    tun = (ctx.loops or {}).get("tunic") or {}
    rows = {}
    bad_n = 0
    for key in ("outer_rows", "inner_rows"):
        for r in tun.get(key) or []:
            name = str(r.get("name"))
            if not (SKIRT_ROW.match(name) or name == "belt_bot"):
                continue
            allow_pel = name == "belt_bot"
            bones, bad = {}, []
            for v in r.get("verts") or []:
                for k, w in W[v].items():
                    if w > 0.0:
                        bones[k] = bones.get(k, 0) + 1
                        if not (k.startswith("Skirt_") or (allow_pel and k == "Pelvis")):
                            bad.append(v)
            bad = sorted(set(bad))
            bad_n += len(bad)
            rows[f"{key}:{name}"] = {"z": r.get("z"), "n_verts": len(r.get("verts") or []), "bones_nverts": bones,
                                     "n_verts_off": len(bad), "examples": bad[:5],
                                     "rule": "Skirt_* + Pelvis" if allow_pel else "Skirt_* only"}
    cols = {}
    for tag, c in (tun.get("columns") or {}).items():
        cols[tag] = {"azimuth_deg": c.get("azimuth_deg"),
                     "outer_hem_to_belt_bot": [wstr(W[v]) for v in c.get("outer_hem_to_belt_bot") or []],
                     "inner_hem_to_ceiling": [wstr(W[v]) for v in c.get("inner_hem_to_ceiling") or []]}
    ceil = [r for r in tun.get("inner_rows") or [] if r.get("name") == "ceiling_ring"]
    ceil_b = {}
    for r in ceil:
        for v in r.get("verts") or []:
            for k, w in W[v].items():
                if w > 0.0:
                    ceil_b[k] = ceil_b.get(k, 0) + 1
    return {"rows": rows, "n_verts_off_total": bad_n, "columns_report": cols,
            "ceiling_ring_bones_nverts_report": ceil_b}, bool(rows) and bad_n == 0


def p24e(ctx, W):
    tun = (ctx.loops or {}).get("tunic") or {}
    outer = tun.get("outer_rows") or []
    worst, n_pairs, n_over, ex = 0.0, 0, 0, []
    for r in tun.get("inner_rows") or []:
        k = r.get("paired_outer_row")
        if k is None or k >= len(outer):
            continue
        for vi, vo in zip(r.get("verts") or [], outer[k].get("verts") or []):
            a, b = W[vi], W[vo]
            d = max((abs(a.get(n, 0.0) - b.get(n, 0.0)) for n in set(a) | set(b)), default=0.0)
            n_pairs += 1
            worst = max(worst, d)
            if d > PAIR_TOL:
                n_over += 1
                if len(ex) < 10:
                    ex.append({"inner": vi, "outer": vo, "diff": float(f"{d:.3e}"), "inner_w": wstr(a),
                               "outer_w": wstr(b)})
    return {"n_pairs": n_pairs, "max_abs_diff": float(f"{worst:.3e}"), "n_over_tol": n_over, "examples": ex,
            "pairing": "tunic.inner_rows[i].verts[j] <-> tunic.outer_rows[paired_outer_row].verts[j]"}, \
        n_pairs > 0 and n_over == 0


def p24f(ctx, W):
    deform = [b.name for b in ctx.arm.data.bones if b.use_deform]
    cnt = {n: sum(1 for d in W if d.get(n, 0.0) > CARRY) for n in deform}
    mx = {n: rnd(max((d.get(n, 0.0) for d in W), default=0.0), 4) for n in CARRY_BONES}
    return {"carry_bones": {n: {"n_verts": cnt.get(n, 0), "max_weight": mx[n]} for n in CARRY_BONES},
            "deform_bones_nverts": cnt, "deform_bones_without_weight": sorted(n for n, c in cnt.items() if c == 0)}, \
        all(cnt.get(n, 0) > 0 for n in CARRY_BONES)


def p24g(ctx):
    mo = ctx.mesh
    me = mo.data
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    M = np.array(mo.matrix_world, dtype=np.float64)
    V0 = co.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]
    p2d.apply_pose(ctx.p2, None)
    Ve = p2d.eval_verts(mo)
    d = np.linalg.norm(Ve - V0, axis=1)
    return {"max_mm": float(f"{d.max() * 1000:.6f}"), "argmax_vertex": int(d.argmax()),
            "notes": ctx.p2.notes}, float(d.max()) <= REST_TOL


IDS = (
    ("P2.4a", f"every vertex weighted; sum 1 +- {SUM_TOL:g}; influences (> 0) <= {MAX_INF}; vertex groups = deform "
              "bones; Root / sockets weight 0; one Armature modifier -> the armature with vertex groups", ""),
    ("P2.4b", f"rigid parts 100 % on one bone (p21_temprig RIGID table): expected >= 1 - {RIGID_TOL:g}, others <= "
              f"{RIGID_TOL:g}", ""),
    ("P2.4c", "tunic weights only on Spine_01 / Spine_02 / Pelvis / Neck / Clavicle_L / Clavicle_R / Skirt_*", ""),
    ("P2.4d", "skirt ring vertices (hem / skirt_<k> outer and inner rows) only on Skirt_*; belt_bot row Skirt_* + "
              "Pelvis", "per azimuth column bones (weight > 1e-4) reported; ceiling ring bones reported"),
    ("P2.4e", f"inner wall = paired outer weights: max |dw| <= {PAIR_TOL:g}", ""),
    ("P2.4f", f"Neck, Clavicle_L, Clavicle_R each carry weight (> {CARRY:g}) on >= 1 vertex",
     "weighted vertex count per deform bone reported"),
    ("P2.4g", f"rest deformation (evaluated at rest vs obj.data) <= {REST_TOL * 1000:g} mm", "action unassigned and "
              "pose reset in memory (as check_p2_deform.measure)"),
)
ROW_ORDER = ["P2.4a", "P2.4b", "P2.4c", "P2.4d", "P2.4e", "P2.4f", "P2.4g", "P2.4h", "P2.4i", "P2.4j", "P2.4k",
             "P2.4l", "P2.4m"]


class Evidence(p2d.Evidence):
    def write(self):
        out = Path(P["out"])
        out.parent.mkdir(parents=True, exist_ok=True)
        self.criteria.sort(key=lambda c: ROW_ORDER.index(c["id"].split("_")[0])
                           if c["id"].split("_")[0] in ROW_ORDER else 99)
        doc = {"gate": GATE, "checker": CHECKER, "task": TASK, "blender": bpy.app.version_string,
               **jsonable(self.extra), "inputs": self.inputs, "renders": self.renders, "criteria": self.criteria}
        with open(out, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']} measured={json.dumps(c['measured'])[:300]}")
        print(f"[{GATE}/{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


class Ctx:
    def __init__(self):
        self.parts = self.loops = self.p0b = None
        self.arm = self.mesh = None
        self.p2 = None


_STATE = {"missing": []}


def parse_args(argv):
    keys = {"--blend": "blend", "--parts": "parts", "--loops": "loops", "--p0b": "p0b", "--mesh": "mesh",
            "--armature": "armature", "--out": "out"}
    i = 0
    while i < len(argv):
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: {' '.join(k + ' <value>' for k in keys)}")
        k = keys[argv[i]]
        P[k] = argv[i + 1] if k in ("mesh", "armature") else Path(argv[i + 1])
        i += 2


def all_ids():
    ids = [i for i, _t, _n in IDS] + ["P2.4h"]
    ids += [f"P2.4i_{j}_{x}" for j in ("elbow", "knee", "wrist") for x in p2d.SIDES]
    ids += [f"P2.4j_{j}_{x}" for j in ("elbow", "knee", "shoulder", "hip", "wrist") for x in p2d.SIDES] + ["P2.4j_rest"]
    ids += [f"P2.4k_shoulder_{x}" for x in p2d.SIDES] + [f"P2.4l_hip_{x}" for x in p2d.SIDES] + ["P2.4m"]
    return ids


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    ev.add_input(__file__)
    ev.add_input(p2d.__file__)
    ctx = Ctx()
    th = {i: t for i, t, _n in IDS}
    nt = {i: n for i, _t, n in IDS}
    ids = all_ids()
    done = set()

    def put(cid, measured, ok, threshold=None, note=None):
        ev.criterion(cid, measured, threshold if threshold is not None else th.get(cid, ""), ok,
                     note if note is not None else nt.get(cid, ""))
        done.add(cid)

    def put2(cid, measured, ok, threshold=None, note=None):
        """P2.2 row from check_p2_deform.write_rows -> P2.4 id, same threshold / note."""
        base = cid.split("_")[0]
        new = ID_MAP[base] + cid[len(base):]
        t, n = p2d.doc_of(cid)
        if base == "P2.2b" and isinstance(measured, dict):
            m60 = [v["min_ratio"] for k, v in measured.get("poses", {}).items() if k.endswith("_60")]
            measured["goblin_final_reference_60deg"] = {"min_60": min((v for v in m60 if v is not None), default=None),
                                                        "reference": GOBLIN_60}
        put(new, measured, ok, threshold if threshold is not None else t,
            (note if note is not None else n) + f"; re-run of P2.2 {cid} on the final skin")

    def block(cids, reason):
        for c in cids:
            if c not in done:
                put(c, f"blocked: {reason}", False)

    tmp = None
    try:
        miss = [rel(P[k]) for k in ("blend", "parts", "loops", "p0b") if not Path(P[k]).exists()]
        for k in ("blend", "parts", "loops", "p0b"):
            ev.add_input(P[k])
        for q in sorted(HERE.glob("p2*.py")):
            ev.add_input(q)
        _STATE["missing"] = miss
        if miss:
            block(ids, "missing input " + ", ".join(miss))
            return
        raw = p2d.load_json(P["parts"])
        ctx.parts = raw["part_id"] if isinstance(raw, dict) and isinstance(raw.get("part_id"), dict) else raw
        ctx.loops = p2d.load_json(P["loops"])
        ctx.p0b = p2d.load_json(P["p0b"])
        bpy.ops.wm.open_mainfile(filepath=str(P["blend"]), load_ui=False)
        # P2.2 context (objects, topology, rings) through check_p2_deform
        p2d.P["mesh"], p2d.P["armature"] = P["mesh"], P["armature"]
        c2 = p2d.Ctx()
        c2.parts, c2.loops, c2.p0b = ctx.parts, ctx.loops, ctx.p0b
        try:
            p2d.check_setup(c2)
        except p2d.Blocked as e:
            block(ids, str(e))
            return
        ctx.p2, ctx.arm, ctx.mesh = c2, c2.arm, c2.mesh
        ev.extra["objects"] = {"mesh": ctx.mesh.name, "armature": ctx.arm.name, "rings": c2.ring_note}
        arm = ctx.arm
        if arm.animation_data is not None and arm.animation_data.action is not None:
            c2.notes.append(f"armature action {arm.animation_data.action.name} unassigned in memory")
            arm.animation_data.action = None
        if arm.data.pose_position != "POSE":
            c2.notes.append(f"armature pose_position {arm.data.pose_position} -> POSE in memory")
            arm.data.pose_position = "POSE"
        W = read_weights(ctx.mesh)
        vp = vertex_parts(ctx.mesh, ctx.parts)
        for cid, fn, args in (("P2.4a", p24a, (ctx, W)), ("P2.4b", p24b, (ctx, W, vp)), ("P2.4c", p24c, (ctx, W, vp)),
                              ("P2.4d", p24d, (ctx, W)), ("P2.4e", p24e, (ctx, W)), ("P2.4f", p24f, (ctx, W)),
                              ("P2.4g", p24g, (ctx,))):
            try:
                put(cid, *fn(*args))
            except (Blocked, KeyError, IndexError) as e:
                block([cid], f"{type(e).__name__}: {e}")
        # P2.2 metrics on the final skin
        tmp = Path(tempfile.mkdtemp(prefix="p2b_cells_"))
        try:
            res, poses, cells = p2d.measure(c2, tmp)
        except p2d.Blocked as e:
            block(ids, str(e))
            return
        p2d.write_rows(put2, c2, res, poses)
        grid = []
        for kind in p2d.SHEET_ROWS:
            row = []
            for x in p2d.SIDES:
                angs = sorted({a for (k, s, a) in cells if k == kind and s == x})
                row += [cells.get((kind, x, a)) for a in angs] + [None] * (3 - len(angs))
            grid.append(row)
        sheet = p2d.compose_sheet(grid, Path(P["out"]).parent / SHEET_NAME)
        ev.render(sheet)
        t, n = p2d.doc_of("P2.2f")
        put("P2.4m", {"sheet": rel(sheet), "rows": p2d.SHEET_ROWS,
                      "cols": {kind: {x: sorted(a for (k, s, a) in cells if k == kind and s == x) for x in p2d.SIDES}
                               for kind in p2d.SHEET_ROWS}}, sheet.exists(), t, n + "; final skin")
    finally:
        if tmp is not None:
            shutil.rmtree(tmp, ignore_errors=True)
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
