"""check_p1 - Player P1 walking-skeleton checker (task T211; spec work/player/d-01-player-walking-skeleton.md
section 2 skeleton contract, section 5 gates P1.1 .. P1.4, checker side).

Inputs (defaults; override after "--" with --blend / --export-dir / --report / --stage / --preset / --p0b / --out):
  work/player/rig/pl_p1_crude.blend             crude work rig + skinned mesh + action p1test
  work/player/rig/export/Player.fbx             rest FBX
  work/player/rig/export/Player@p1test.fbx      clip FBX
  unity/AvatarCheck/player_p1_report.json       Unity report (PlayerP1Check.cs), Unity world coordinates
  export preset (reimport options + axis_mapping.blender_to_unity): work/player/rig/data/export_preset.json or
      work/player/rig/export/export_preset.json when present, else work/goblin_swing/rig/data/export_preset.json
  work/player/inspect/P0b/p0b_measure.json      part centroids (P1.1f per-part dominant bone; identification only)
Blender stage (reference for P1.2b / P1.3e / P1.3f): first found of
  1. --stage <blend> or work/player/rig/export/stage*.blend: an armature holding every section 2 bone
  2. pl_p1_crude.blend: an armature holding every section 2 bone whose name contains "export" (else, when several
     armatures hold the section 2 bones, the one with no connected bone; else the only one = the work rig)
  the stage action = its assigned action when named p1test, else bpy.data.actions['p1test'] assigned in memory.

Methods copied from the goblin checkers (check_g8_fbx / check_a3_clip, not imported):
  FBX reimport into an empty factory scene with the preset blender_reimport.import_scene_fbx options, then every bone
  use_connect False in edit mode (importer auto-connect pitfall); FBX frame g0+i vs stage frame s0+i (anim_offset
  shift corrected by aligning range starts); world = matrix_world @ pose_bone.matrix (evaluated);
  position error = |t1 - t2|, rotation error = degrees(2 acos |q1.q2|) evaluated in float64 as
  2 asin(|R1 - R2|_F / (2 sqrt 2)) (columns normalized; removes the float32 acos noise floor ~0.03-0.3 deg).
  Unity: p_u vs M p_b with M = axis_mapping.blender_to_unity (unity.x = -blender.x, unity.y = blender.z,
  unity.z = -blender.y); rotation rest-relative dR = R R_ref^T, error = angle(dR_u, M dR_b M^T) where R_ref = the
  report 'rest' block when present, else the first matched frame (first-frame-relative); the absolute
  angle(R_u, M R_b M^T) is reported beside it.

Rows: P1.1a-g, P1.2a-b, P1.3a-f, P1.4a-c.  Rows marked (report) hold a first-run measured distribution: ok = True,
note "calibration run: threshold set after measurement".  A row that cannot be measured is "blocked: ..." ok False.
Nothing is saved: blends are opened read-only in memory; only the evidence JSON is written.
Run:    blender --background --factory-startup --python check_p1.py [-- <overrides>]
Output: work/player/inspect/P1/check_p1.json
Exit:   0 evidence written; 2 script error or missing input (evidence still written with blocked rows).
"""
import hashlib
import json
import math
import sys
import traceback
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

HERE = Path(__file__).resolve().parent
PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
GATE, CHECKER, TASK = "P1", "check_p1", "T211"
CALIB = "calibration run: threshold set after measurement"

P = {"blend": PLAYER_RIG / "pl_p1_crude.blend",
     "export_dir": PLAYER_RIG / "export",
     "report": REPO / "unity" / "AvatarCheck" / "player_p1_report.json",
     "stage": None,
     "preset": None,
     "p0b": REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json",
     "out": REPO / "work" / "player" / "inspect" / "P1" / "check_p1.json"}
PRESET_CANDIDATES = (PLAYER_RIG / "data" / "export_preset.json", PLAYER_RIG / "export" / "export_preset.json",
                     REPO / "work" / "goblin_swing" / "rig" / "data" / "export_preset.json")
M_SPEC = [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]
ACTION = "p1test"
SCALE_TOL = 1e-6

# ---------------------------------------------------------------- section 2 contract
SKIRT_DIRS = ("F", "FL", "L", "BL", "B", "BR", "R", "FR")


def contract():
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
N_BONES = len(NAMES)          # 41
SKIRT = [n for n in NAMES if n.startswith("Skirt_")]
BODY = [n for n in NAMES if not n.startswith("Skirt_")]
# section 3 rigid parts (P0b part name -> expected bone), report only
RIGID = {"head": "Head", "eye_l": "Head", "eye_r": "Head", "fist_l": "Hand_L", "fist_r": "Hand_R",
         "shoe_l": "Foot_L", "shoe_r": "Foot_R", "cuff_l": "Foot_L", "cuff_r": "Foot_R", "belt": "Pelvis",
         "pouch": "Pelvis", "sleeve_l": "Arm_L", "sleeve_r": "Arm_R", "scarf": "Spine_02", "scarf_tail": "Spine_02"}


# ---------------------------------------------------------------- helpers
class Blocked(Exception):
    pass


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def rel(p):
    p = Path(p).resolve()
    try:
        return p.relative_to(REPO).as_posix()
    except ValueError:
        return p.as_posix()


def jsonable(v):
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
        return rel(v)
    if isinstance(v, np.ndarray):
        return jsonable(v.tolist())
    if isinstance(v, dict):
        return {str(k): jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple, set)) or isinstance(v, (Vector, Quaternion)):
        return [jsonable(x) for x in v]
    if isinstance(v, Matrix):
        return [jsonable(list(r)) for r in v]
    return str(v)


def mm(x):
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=5):
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def tb_tail(n=4):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


def rot3n(R):
    """3x3 rotation (float64, columns normalized) from a mathutils / numpy 3x3 or 4x4."""
    A = np.array([list(r) for r in R], dtype=np.float64)[:3, :3]
    return A / np.linalg.norm(A, axis=0)


def rang(Ra, Rb):
    """Rotation angle (deg) between two rotations, float64: 2 asin(|Ra - Rb|_F / (2 sqrt 2)) (same angle as
    2 acos|q1.q2|, without the acos round-off near 0 that float32 quaternions give)."""
    d = float(np.linalg.norm(rot3n(Ra) - rot3n(Rb))) / (2.0 * math.sqrt(2.0))
    return math.degrees(2.0 * math.asin(min(1.0, d)))


def dist(vals, scale=1.0, n=4, keys=None):
    """values -> n / max / p95 / p50 / mean (+ worst key)."""
    if not len(vals):
        return {"n": 0}
    v = np.asarray(vals, dtype=np.float64) * scale
    i = int(v.argmax())
    out = {"n": int(v.size), "max": round(float(v.max()), n), "p95": round(float(np.percentile(v, 95)), n),
           "p50": round(float(np.percentile(v, 50)), n), "mean": round(float(v.mean()), n)}
    if keys is not None:
        out["worst"] = keys[i]
    return out


def fcurves_of(action):
    out = []
    for layer in getattr(action, "layers", ()):
        for strip in layer.strips:
            for cb in getattr(strip, "channelbags", ()):
                out.extend(cb.fcurves)
    if not out and hasattr(action, "fcurves"):
        out.extend(action.fcurves)
    return out


def assign_action(obj, act):
    """In memory only (never saved)."""
    ad = obj.animation_data or obj.animation_data_create()
    if ad.action != act:
        ad.action = act
    if hasattr(ad, "action_slot") and ad.action_slot is None:
        from bpy_extras import anim_utils
        slot = anim_utils.action_get_first_suitable_slot(act, "OBJECT")
        if slot is not None:
            ad.action_slot = slot
    slot = getattr(ad, "action_slot", None)
    return getattr(slot, "identifier", None) if slot is not None else None


def open_blend(path):
    bpy.ops.wm.open_mainfile(filepath=str(path), load_ui=False)


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def hier(arm):
    """(names, parent map, connected, *_end) of an armature object."""
    bones = arm.data.bones
    names = [b.name for b in bones]
    par = {b.name: (b.parent.name if b.parent else None) for b in bones}
    return names, par, sorted(b.name for b in bones if b.use_connect)


def hier_diff(names, par):
    only_c = sorted(set(NAMES) - set(names))
    only_x = sorted(set(names) - set(NAMES))
    pm = {n: {"spec": PARENT[n], "found": par[n]} for n in NAMES if n in par and par[n] != PARENT[n]}
    dup = sorted({n for n in names if names.count(n) > 1})
    ok = len(names) == N_BONES and not only_c and not only_x and not pm and not dup
    return {"n_bones": len(names), "n_spec": N_BONES, "only_spec": only_c, "only_found": only_x,
            "parent_mismatch": pm, "duplicates": dup}, ok


def scale_rec(o):
    return {"object_scale": [rnd(s, 8) for s in o.scale], "world_scale": [rnd(s, 8) for s in o.matrix_world.to_scale()],
            "max_dev": float(max(max(abs(s - 1.0) for s in o.scale),
                                 max(abs(s - 1.0) for s in o.matrix_world.to_scale())))}


def sample_world(arm, frames, names):
    """(F,B,4,4) evaluated world matrices of pose bones."""
    sc = bpy.context.scene
    out = np.zeros((len(frames), len(names), 4, 4))
    for i, f in enumerate(frames):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = arm.evaluated_get(dg)
        mw = oe.matrix_world
        for j, n in enumerate(names):
            out[i, j] = np.array(mw @ oe.pose.bones[n].matrix, dtype=np.float64)
    return out


def npm(A):
    return Matrix(np.asarray(A).tolist())


def pos_rot_err(A, B):
    """A, B 4x4 numpy world matrices -> (pos m, rot deg) (decompose rotation)."""
    A, B = np.asarray(A, dtype=np.float64), np.asarray(B, dtype=np.float64)
    return float(np.linalg.norm(A[:3, 3] - B[:3, 3])), rang(A, B)


def fps_of(sc):
    return rnd(sc.render.fps / (sc.render.fps_base or 1.0), 4)


# ---------------------------------------------------------------- context
class Ctx:
    def __init__(self):
        self.preset = self.preset_path = None
        self.M = Matrix(M_SPEC)
        self.opts = None
        self.stage_src = None        # description
        self.stage_frames = None
        self.stage_W = None          # (F,B,4,4) over NAMES
        self.stage_rest = None       # {name: 4x4}
        self.stage_info = {}


def pick_preset(ctx, ev):
    cands = [P["preset"]] if P["preset"] else list(PRESET_CANDIDATES)
    for c in cands:
        if c and Path(c).exists():
            ctx.preset_path = Path(c)
            ctx.preset = load_json(c)
            ev.add_input(c)
            break
    if ctx.preset is None:
        raise Blocked(f"no export preset among {[rel(c) for c in cands if c]}")
    am = (ctx.preset.get("axis_mapping") or {}).get("blender_to_unity")
    note = {"preset": rel(ctx.preset_path)}
    if am is not None:
        dev = float(np.abs(np.array(am, dtype=np.float64) - np.array(M_SPEC)).max())
        note["preset_blender_to_unity"] = am
        note["preset_vs_instruction_mapping_max_abs_diff"] = dev
        if dev <= 1e-9:
            ctx.M = Matrix(am)
    note["M_used"] = [list(r) for r in ctx.M]
    pr = (ctx.preset.get("blender_reimport") or {}).get("import_scene_fbx")
    if not isinstance(pr, dict) or not pr:
        raise Blocked(f"{rel(ctx.preset_path)} has no blender_reimport.import_scene_fbx")
    ctx.opts = dict(pr)
    note["import_options"] = ctx.opts
    return note


# ---------------------------------------------------------------- P1.1
def classify_armatures():
    arms = [o for o in bpy.data.objects if o.type == "ARMATURE"]
    full = [a for a in arms if all(n in a.data.bones for n in NAMES)]
    exp = [a for a in full if "export" in a.name.lower()]
    if exp:
        stage = exp[0]
        how = "name contains 'export'"
    elif len(full) > 1:
        unc = [a for a in full if not any(b.use_connect for b in a.data.bones)]
        stage = unc[0] if unc else None
        how = "several full armatures: the one with no connected bone" if unc else None
    elif len(full) == 1:
        stage, how = full[0], "only armature with all section 2 bones (work rig = stage)"
    else:
        stage, how = None, None
    works = [a for a in arms if a is not stage]
    if stage is not None and len(full) == 1 and not exp:
        works = [stage]
    return arms, stage, works, how


def mesh_components(me):
    n = len(me.vertices)
    e = np.empty(len(me.edges) * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", e)
    e = e.reshape(-1, 2)
    lab = np.arange(n)
    while True:
        m = np.minimum(lab[e[:, 0]], lab[e[:, 1]])
        new = lab.copy()
        np.minimum.at(new, e[:, 0], m)
        np.minimum.at(new, e[:, 1], m)
        new = new[new]
        if np.array_equal(new, lab):
            break
        lab = new
    _u, inv = np.unique(lab, return_inverse=True)
    return inv


def skin_facts(arm, p0b_parts):
    """P1.1e/f/g over every mesh deformed by `arm` (Armature modifier object)."""
    bone_set = set(arm.data.bones.keys())
    meshes, unskinned = [], []
    for o in bpy.data.objects:
        if o.type != "MESH":
            continue
        mods = [m for m in o.modifiers if m.type == "ARMATURE"]
        if any(m.object == arm for m in mods):
            meshes.append(o)
        else:
            unskinned.append({"name": o.name, "armature_modifiers": [m.object.name if m.object else None for m in mods],
                              "parent": o.parent.name if o.parent else None, "n_verts": len(o.data.vertices)})
    if not meshes:
        raise Blocked(f"no mesh with an Armature modifier on {arm.name}")
    e_rec, f_rec, g_rec = {}, {}, {}
    total_unw = 0
    hist_all = {}
    for o in meshes:
        me = o.data
        vg = {g.index: g.name for g in o.vertex_groups}
        non_bone = sorted(n for n in vg.values() if n not in bone_set)
        nv = len(me.vertices)
        cnt = np.zeros(nv, dtype=np.int64)
        wsum = np.zeros(nv)
        W = {}
        for v in me.vertices:
            for g in v.groups:
                nm = vg.get(g.group)
                if nm in bone_set and g.weight > 0.0:
                    cnt[v.index] += 1
                    wsum[v.index] += g.weight
                    W.setdefault(nm, np.zeros(nv))[v.index] = g.weight
        unw = np.nonzero(cnt == 0)[0]
        total_unw += len(unw)
        e_rec[o.name] = {"n_verts": nv, "unweighted": int(len(unw)), "unweighted_first": unw[:20].tolist(),
                         "weight_sum_[min,max]_weighted": ([rnd(wsum[cnt > 0].min(), 6), rnd(wsum[cnt > 0].max(), 6)]
                                                           if (cnt > 0).any() else None),
                         "non_bone_vertex_groups": non_bone,
                         "modifier_order": [m.type for m in o.modifiers]}
        h = {int(k): int(c) for k, c in zip(*np.unique(cnt, return_counts=True))}
        for k, c in h.items():
            hist_all[k] = hist_all.get(k, 0) + c
        f_rec[o.name] = {"max_influences": int(cnt.max()) if nv else 0, "histogram_influences": h,
                         "verts_over_4": int((cnt > 4).sum())}
        # per-part dominant bone (mesh islands matched to P0b part centroids)
        comp = mesh_components(me)
        co = np.empty(nv * 3)
        me.vertices.foreach_get("co", co)
        mw = np.array(o.matrix_world, dtype=np.float64)
        cw = co.reshape(nv, 3) @ mw[:3, :3].T + mw[:3, 3]
        names_w = list(W)
        Wm = np.stack([W[n] for n in names_w], axis=1) if names_w else np.zeros((nv, 0))
        parts = []
        for c in range(int(comp.max()) + 1 if nv else 0):
            idx = np.nonzero(comp == c)[0]
            cen = cw[idx].mean(0)
            best, bd = None, None
            for pn, pv in (p0b_parts or {}).items():
                d = float(np.linalg.norm(cen - np.array(pv["centroid"])))
                if bd is None or d < bd:
                    best, bd = pn, d
            tot = Wm[idx].sum(0) if Wm.shape[1] else np.zeros(0)
            order = np.argsort(-tot)[:3] if tot.size else []
            s = float(tot.sum()) or 1.0
            top = [[names_w[k], rnd(tot[k] / s, 4)] for k in order if tot[k] > 0]
            rec = {"verts": int(len(idx)), "centroid": [rnd(x, 4) for x in cen],
                   "p0b_part": best, "p0b_centroid_dist_mm": mm(bd) if bd is not None else None,
                   "top_bones_[bone,share]": top}
            if best in RIGID:
                exp = RIGID[best]
                rec["rigid_expected_bone"] = exp
                rec["dominant_is_expected"] = bool(top and top[0][0] == exp)
                rec["verts_not_100pct_expected"] = int((Wm[idx][:, names_w.index(exp)] < 0.999).sum()
                                                       if exp in names_w else len(idx))
            parts.append(rec)
        g_rec[o.name] = parts
    return ({"meshes": e_rec, "unweighted_total": total_unw, "other_meshes_not_skinned_by_this_armature": unskinned},
            total_unw == 0, {"per_mesh": f_rec, "histogram_all": hist_all,
                             "max_influences_all": max(hist_all) if hist_all else 0}, g_rec, meshes)


def orientation(arm, meshes, p0b_parts):
    mw = arm.matrix_world
    head = {n: mw @ arm.data.bones[n].head_local for n in NAMES}
    tail = {n: mw @ arm.data.bones[n].tail_local for n in NAMES}
    lx = {n: rnd(head[n].x, 4) for n in NAMES if n.endswith("_L")}
    rx = {n: rnd(head[n].x, 4) for n in NAMES if n.endswith("_R")}
    side_ok = all(v > 0 for v in lx.values()) and all(v < 0 for v in rx.values())
    zs = {n: rnd(head[n].z, 4) for n in ("Head", "Pelvis", "Foot_L", "Foot_R")}
    up_ok = zs["Head"] > zs["Pelvis"] > max(zs["Foot_L"], zs["Foot_R"])
    rec = {"character_left_is_+X_[bone head x]": {"L": lx, "R": rx, "ok": side_ok},
           "up_is_+Z_[head z]": {"z": zs, "ok": up_ok}}
    # front -Y from the mesh: eye islands (nearest P0b eye centroids) in front of the head island
    front_ok = None
    if p0b_parts and all(k in p0b_parts for k in ("head", "eye_l", "eye_r")):
        pts = []
        for o in meshes:
            me = o.data
            nv = len(me.vertices)
            co = np.empty(nv * 3)
            me.vertices.foreach_get("co", co)
            M4 = np.array(o.matrix_world, dtype=np.float64)
            cw = co.reshape(nv, 3) @ M4[:3, :3].T + M4[:3, 3]
            comp = mesh_components(me)
            for c in range(int(comp.max()) + 1 if nv else 0):
                pts.append(cw[comp == c].mean(0))
        found = {}
        for k in ("head", "eye_l", "eye_r"):
            ref = np.array(p0b_parts[k]["centroid"])
            if pts:
                d = [float(np.linalg.norm(p - ref)) for p in pts]
                i = int(np.argmin(d))
                found[k] = {"centroid": [rnd(x, 4) for x in pts[i]], "dist_mm": mm(d[i])}
        if len(found) == 3 and all(found[k]["dist_mm"] <= 20.0 for k in found):
            front_ok = all(found[e]["centroid"][1] < found["head"]["centroid"][1] for e in ("eye_l", "eye_r"))
        rec["front_is_-Y_[eye islands y < head island y]"] = {"islands": found, "ok": front_ok,
                                                             "note": "islands matched to P0b centroids within 20 mm"}
    # A-pose (report): upper-arm direction below horizontal
    ap = {}
    for s in ("L", "R"):
        d = tail[f"Arm_{s}"] - head[f"Arm_{s}"]
        if d.length > 0:
            ap[f"Arm_{s}_deg_below_horizontal"] = rnd(math.degrees(math.asin(max(-1.0, min(1.0, -d.z / d.length)))), 3)
            ap[f"Arm_{s}_forward_deg_(-Y positive)"] = rnd(math.degrees(math.atan2(-d.y, abs(d.x))), 3)
    rec["a_pose_report"] = ap
    rec["pose_position"] = arm.data.pose_position
    ok = side_ok and up_ok and front_ok is not False
    return rec, ok


def p11(ctx, ev, put, block, p0b_parts):
    open_blend(P["blend"])
    arms, stage, works, how = classify_armatures()
    info = {"blend": rel(P["blend"]), "armatures": [a.name for a in arms],
            "stage_armature": stage.name if stage else None, "stage_pick": how,
            "work_armatures": [a.name for a in works], "actions": [a.name for a in bpy.data.actions],
            "unit_scale_length": rnd(bpy.context.scene.unit_settings.scale_length, 8),
            "unit_system": bpy.context.scene.unit_settings.system}
    if stage is None:
        rows = {}
        for a in arms:
            rows[a.name] = hier_diff(*hier(a)[:2])[0]
        put("P1.1a", {**info, "per_armature": rows}, False)
        block(["P1.1b", "P1.1c", "P1.1d", "P1.1e", "P1.1f", "P1.1g"], "no armature holds every section 2 bone")
        return None
    names, par, conn = hier(stage)
    d, ok = hier_diff(names, par)
    work_h = {a.name: hier_diff(*hier(a)[:2])[0] for a in works if a is not stage}
    put("P1.1a", {**info, "stage": d, "work_rigs_report": work_h}, ok)
    put("P1.1b", {"stage_connected": conn, "work_connected_report": {a.name: hier(a)[2] for a in works}}, not conn)
    skin_arm = stage
    for a in works:   # the skin normally sits on the work rig
        if any(m.type == "ARMATURE" and m.object == a for o in bpy.data.objects if o.type == "MESH" for m in o.modifiers):
            skin_arm = a
            break
    try:
        e, ok_e, f, g, meshes = skin_facts(skin_arm, p0b_parts)
    except Blocked as x:
        meshes = []
        block(["P1.1e", "P1.1f", "P1.1g"], str(x))
    else:
        put("P1.1e", {"skin_armature": skin_arm.name, **e}, ok_e)
        put("P1.1f", {"skin_armature": skin_arm.name, **f}, True)
        put("P1.1g", {"skin_armature": skin_arm.name, "islands_per_mesh": g}, True)
    sc = {o.name: scale_rec(o) for o in [stage, *works, *meshes]}
    put("P1.1c", {"objects": sc, "unit_scale_length": info["unit_scale_length"]},
        all(v["max_dev"] <= SCALE_TOL for v in sc.values())
        and abs(bpy.context.scene.unit_settings.scale_length - 1.0) <= SCALE_TOL)
    o_rec, o_ok = orientation(stage, meshes, p0b_parts)
    put("P1.1d", o_rec, o_ok)
    return stage.name


# ---------------------------------------------------------------- stage reference
def load_stage(ctx, crude_stage_name):
    src = None
    if P["stage"]:
        src = Path(P["stage"])
    else:
        cands = sorted(P["export_dir"].glob("stage*.blend")) if P["export_dir"].exists() else []
        cands = [c for c in cands if ACTION in c.name] or cands
        src = cands[0] if cands else None
    if src is not None:
        open_blend(src)
        arms = [o for o in bpy.data.objects if o.type == "ARMATURE" and all(n in o.data.bones for n in NAMES)]
        if not arms:
            raise Blocked(f"stage {rel(src)} has no armature with every section 2 bone")
        arm = arms[0]
        ctx.stage_src = rel(src)
    else:
        if crude_stage_name is None:
            raise Blocked("no stage blend and no section 2 armature in the crude blend")
        open_blend(P["blend"])
        arm = bpy.data.objects[crude_stage_name]
        ctx.stage_src = f"{rel(P['blend'])}:{arm.name} (no stage*.blend in {rel(P['export_dir'])})"
    ad = arm.animation_data
    act = ad.action if ad and ad.action else None
    assigned = None
    if act is None or (ACTION not in act.name and bpy.data.actions.get(ACTION) is not None):
        a2 = bpy.data.actions.get(ACTION)
        if a2 is None:
            raise Blocked(f"{arm.name}: no action and no bpy.data.actions[{ACTION!r}] "
                          f"(actions {[a.name for a in bpy.data.actions]})")
        assigned = assign_action(arm, a2)
        act = a2
    f0, f1 = (int(round(x)) for x in act.frame_range)
    ctx.stage_frames = list(range(f0, f1 + 1))
    ctx.stage_W = sample_world(arm, ctx.stage_frames, NAMES)
    mw = arm.matrix_world.copy()
    ctx.stage_rest = {n: np.array(mw @ arm.data.bones[n].matrix_local, dtype=np.float64) for n in NAMES}
    sc = bpy.context.scene
    ctx.stage_info = {"source": ctx.stage_src, "armature": arm.name, "action": act.name,
                      "assigned_in_memory_slot": assigned, "frame_range": [f0, f1], "n_frames": len(ctx.stage_frames),
                      "fps": fps_of(sc), "connected_bones": sorted(b.name for b in arm.data.bones if b.use_connect)}
    if src is not None:
        return src
    return None


# ---------------------------------------------------------------- P1.2
def fresh_import(path, opts):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    try:
        res = bpy.ops.import_scene.fbx(filepath=str(path), **opts)
    except Exception:
        raise Blocked(f"import {rel(path)} failed: {tb_tail(3)}")
    if "FINISHED" not in res:
        raise Blocked(f"import {rel(path)} returned {sorted(res)}")
    bpy.context.view_layer.update()


def find_arm():
    arms = [x for x in bpy.data.objects if x.type == "ARMATURE"]
    if len(arms) != 1:
        raise Blocked(f"expected 1 armature in the reimport, found {[a.name for a in arms]}")
    return arms[0]


def disconnect_bones(arm):
    before = sorted(b.name for b in arm.data.bones if b.use_connect)
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    arm.select_set(True)
    vl.objects.active = arm
    bpy.ops.object.mode_set(mode="EDIT")
    for eb in arm.data.edit_bones:
        eb.use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")
    vl.update()
    after = sorted(b.name for b in arm.data.bones if b.use_connect)
    return {"connected_on_import": before, "connected_after": after, "undone": not after}


def fbx_hierarchy(ctx, path):
    fresh_import(path, ctx.opts)
    arm = find_arm()
    names, par, _c = hier(arm)
    d, ok = hier_diff(names, par)
    ends = sorted(n for n in names if n.lower().endswith("_end"))
    conn = disconnect_bones(arm)
    rec = {"objects": sorted(f"{o.name}({o.type})" for o in bpy.data.objects), "armature": arm.name, **d,
           "end_leaf_bones": ends, "importer_connected_bones": conn,
           "object_scales_report": {o.name: scale_rec(o) for o in bpy.data.objects},
           "actions_in_file": [a.name for a in bpy.data.actions]}
    if ctx.stage_rest is not None:
        pe, re_ = {}, {}
        for n in NAMES:
            if n in arm.data.bones:
                W = np.array(arm.matrix_world @ arm.data.bones[n].matrix_local, dtype=np.float64)
                pe[n], re_[n] = pos_rot_err(W, ctx.stage_rest[n])
        if pe:
            rec["rest_vs_stage_rest_report"] = {"pos_mm": dist(list(pe.values()), 1000.0, keys=list(pe)),
                                                "rot_deg": dist(list(re_.values()), keys=list(re_))}
    return rec, ok and not ends and conn["undone"], arm


def p12b(ctx, arm):
    act = arm.animation_data.action if arm.animation_data else None
    if act is None:
        raise Blocked(f"reimported armature {arm.name} has no action")
    times = [k.co.x for fc in fcurves_of(act) for k in fc.keyframe_points]
    if not times:
        raise Blocked(f"action {act.name} has no keys")
    missing = [n for n in NAMES if n not in arm.pose.bones]
    if missing:
        raise Blocked(f"reimported armature lacks {missing}")
    g0, g1 = int(round(min(times))), int(round(max(times)))
    n_fbx = g1 - g0 + 1
    n = min(n_fbx, len(ctx.stage_frames))
    F = sample_world(arm, list(range(g0, g0 + n)), NAMES)
    Pm = np.zeros((n, N_BONES))
    Rd = np.zeros((n, N_BONES))
    for i in range(n):
        for j in range(N_BONES):
            Pm[i, j], Rd[i, j] = pos_rot_err(F[i, j], ctx.stage_W[i, j])
    keys = [[ctx.stage_frames[i], NAMES[j]] for i in range(n) for j in range(N_BONES)]
    sc = bpy.context.scene
    return {"action": act.name, "key_time_range": [rnd(min(times), 4), rnd(max(times), 4)], "n_frames_fbx": n_fbx,
            "n_frames_stage": len(ctx.stage_frames), "frame_count_equal": n_fbx == len(ctx.stage_frames),
            "n_frames_compared": n, "alignment": "fbx frame g0+i vs stage frame s0+i",
            "frame_offset_fbx_minus_stage": g0 - ctx.stage_frames[0],
            "keys_not_on_integer_frame": sum(1 for t in times if abs(t - round(t)) > 1e-4),
            "scene_fps_after_import": fps_of(sc),
            "pos_mm": dist(Pm.ravel(), 1000.0, keys=keys), "rot_deg": dist(Rd.ravel(), keys=keys),
            "per_bone_max_[pos_mm,rot_deg]": {nm: [mm(Pm[:, j].max()), rnd(Rd[:, j].max(), 4)]
                                              for j, nm in enumerate(NAMES)},
            "goblin_reference": "0.0006 mm / 0.1 deg", "stage": ctx.stage_info}


# ---------------------------------------------------------------- P1.3
def tr(v):
    """Unity [px,py,pz,qx,qy,qz,qw] or {"pos","rot"} -> (Vector pos, Matrix rot) or None."""
    if isinstance(v, dict) and "pos" in v and "rot" in v:
        v = list(v["pos"]) + list(v["rot"])
    if not isinstance(v, (list, tuple)) or len(v) != 7:
        return None
    try:
        px, py, pz, qx, qy, qz, qw = (float(x) for x in v)
    except (TypeError, ValueError):
        return None
    return Vector((px, py, pz)), Quaternion((qw, qx, qy, qz)).normalized().to_matrix()


def frame_map(ctx, rep_frames):
    fs = [fr.get("f") for fr in rep_frames if isinstance(fr, dict) and isinstance(fr.get("f"), (int, float))]
    if not fs:
        raise Blocked("report 'frames' has no entry with an integer 'f'")
    s0, s1 = ctx.stage_frames[0], ctx.stage_frames[-1]
    off = 0 if all(s0 <= f <= s1 for f in fs) else s0 - int(min(fs))
    return off, fs


def p13_import(rep):
    imp = rep.get("import")
    if not isinstance(imp, dict):
        raise Blocked("report has no 'import' object")

    def norm(v):
        return str(v).replace(" ", "").replace("_", "").lower()
    at = norm(imp.get("animationType"))
    cp = norm(imp.get("compression"))
    ok = at in ("generic", "2") and cp in ("off", "0")
    return {"import": imp, "animationType_norm": at, "compression_norm": cp, "unity_version": rep.get("unity_version"),
            "timing_s_report": rep.get("timing_s"), "renders_report": rep.get("renders")}, ok


def p13_bones(rep):
    bl = rep.get("bones")
    if not isinstance(bl, list):
        raise Blocked("report has no 'bones' list")
    par = {}
    for b in bl:
        if isinstance(b, dict) and "name" in b:
            par[b["name"]] = b.get("parent")
    missing = [n for n in NAMES if n not in par]
    pm = {n: {"spec": PARENT[n], "unity": par[n]} for n in NAMES if n in par and n != "Root" and par[n] != PARENT[n]}
    extra = sorted(n for n in par if n not in PARENT)
    return {"n_report_bones": len(par), "missing": missing, "parent_mismatch_excl_Root": pm,
            "Root_parent_unity": par.get("Root"), "extra_nodes_report": extra}, not missing and not pm


def p13_pose(ctx, rep, key, names):
    frames = rep.get("frames")
    if not isinstance(frames, list) or not frames:
        raise Blocked("report has no 'frames' list")
    if ctx.stage_W is None:
        raise Blocked("no Blender stage reference")
    off, fs = frame_map(ctx, frames)
    M = ctx.M
    Mt = M.transposed()
    idx = {f: i for i, f in enumerate(ctx.stage_frames)}
    rest = rep.get("rest") if isinstance(rep.get("rest"), dict) else None
    use_rest = rest is not None and all(tr(rest.get(n)) is not None for n in names)
    ref_u, ref_b = {}, {}
    if use_rest:
        for n in names:
            ref_u[n] = tr(rest[n])[1]
            ref_b[n] = npm(ctx.stage_rest[n]).to_3x3().normalized()
    pos, rot, rabs, keys = [], [], [], []
    missing_bones, unmatched = set(), []
    n_with_block = 0
    per_bone = {n: [0.0, 0.0] for n in names}
    for fr in frames:
        if not isinstance(fr, dict) or not isinstance(fr.get("f"), (int, float)):
            continue
        bf = int(fr["f"]) + off
        i = idx.get(bf)
        if i is None:
            unmatched.append(fr["f"])
            continue
        blk = fr.get(key)
        if not isinstance(blk, dict):
            continue
        n_with_block += 1
        for n in names:
            t = tr(blk.get(n))
            if t is None:
                missing_bones.add(n)
                continue
            pu, Ru = t
            Eb = npm(ctx.stage_W[i, NAMES.index(n)])
            Rb = Eb.to_3x3().normalized()
            if n not in ref_u:
                ref_u[n], ref_b[n] = Ru, Rb
            pe = (pu - M @ Eb.translation).length
            dRu = Ru @ ref_u[n].transposed()
            dRb = M @ (Rb @ ref_b[n].transposed()) @ Mt
            re_ = rang(dRu, dRb)
            pos.append(pe)
            rot.append(re_)
            rabs.append(rang(Ru, M @ Rb @ Mt))
            keys.append([bf, n])
            per_bone[n][0] = max(per_bone[n][0], pe)
            per_bone[n][1] = max(per_bone[n][1], re_)
    if not pos:
        raise Blocked(f"no report frame with a '{key}' block matched the stage frames "
                      f"(offset {off}, report f {fs[:3]}..{fs[-3:]})")
    return {"block": key, "n_bones": len(names), "n_report_frames": len(frames),
            "n_frames_with_block": n_with_block, "unity_f_to_stage_frame_offset": off,
            "unmatched_report_frames": unmatched[:20], "missing_bones": sorted(missing_bones),
            "rotation_reference": ("report 'rest' (rest-relative)" if use_rest
                                   else "first matched frame per bone (first-frame-relative)"),
            "pos_mm": dist(pos, 1000.0, keys=keys), "rot_deg": dist(rot, keys=keys),
            "rot_abs_deg_[angle(R_u, M R_b M^T)]_report": dist(rabs, keys=keys),
            "per_bone_max_[pos_mm,rot_deg]": {n: [mm(v[0]), rnd(v[1], 4)] for n, v in per_bone.items()},
            "stage": ctx.stage_info.get("source")}


# ---------------------------------------------------------------- P1.4
def p14_chains(rep):
    sp = rep.get("spring")
    if not isinstance(sp, dict) or not isinstance(sp.get("chains"), list):
        raise Blocked("report has no 'spring.chains' list")
    chains = [c for c in sp["chains"] if isinstance(c, dict)]
    match = {}
    for d in SKIRT_DIRS:
        want = {f"Skirt_{d}_01", f"Skirt_{d}_02"}
        match[d] = [c.get("name") for c in chains if want & set(c.get("bones") or [])]
    ok = all(len(v) == 1 for v in match.values()) and len(chains) == len(SKIRT_DIRS)
    return {"n_chains": len(chains), "chain_bones": {str(c.get("name")): c.get("bones") for c in chains},
            "skirt_dir_to_chain": match, "params_report": sp.get("params"),
            "colliders_report": sp.get("colliders")}, ok, chains


def p14_deflection(rep, chains):
    rows = {}
    for c in chains:
        per = c.get("tip_deflection_m_per_frame")
        mx = c.get("tip_deflection_max_m")
        r = {"tip_deflection_max_mm": mm(mx) if isinstance(mx, (int, float)) else f"missing ({mx!r})",
             "gt_0": bool(isinstance(mx, (int, float)) and mx > 0)}
        if isinstance(per, list) and per:
            v = [float(x) for x in per if isinstance(x, (int, float))]
            r["per_frame_mm"] = dist(v, 1000.0)
            r["n_frames_gt_0"] = sum(1 for x in v if x > 0)
            r["max_of_per_frame_equals_max"] = bool(v and isinstance(mx, (int, float)) and abs(max(v) - mx) <= 1e-9)
        rows[str(c.get("name"))] = r
    # cross-check: skirt_spring vs skirt_raw Skirt_*_02 head distance
    xc = {}
    for fr in rep.get("frames") or []:
        if not isinstance(fr, dict):
            continue
        a, b = fr.get("skirt_spring"), fr.get("skirt_raw")
        if not isinstance(a, dict) or not isinstance(b, dict):
            continue
        for d in SKIRT_DIRS:
            n = f"Skirt_{d}_02"
            ta, tb = tr(a.get(n)), tr(b.get(n))
            if ta and tb:
                xc.setdefault(d, []).append((ta[0] - tb[0]).length)
    return {"per_chain": rows, "all_chains_gt_0": all(r["gt_0"] for r in rows.values()) if rows else False,
            "crosscheck_Skirt_02_pos_spring_minus_raw_mm": {d: dist(v, 1000.0) for d, v in xc.items()}
            or "no frame with both skirt_spring and skirt_raw"}


def p14_penetration(rep):
    pen = rep.get("penetration")
    if not isinstance(pen, dict) or not isinstance(pen.get("per_frame"), list):
        raise Blocked("report has no 'penetration.per_frame' list")
    rows = [(r.get("f"), r.get("leg_vertices_inside_skirt")) for r in pen["per_frame"] if isinstance(r, dict)]
    rows = [(f, int(c)) for f, c in rows if isinstance(c, (int, float))]
    v = [c for _f, c in rows]
    return {"method": pen.get("method"), "n_frames": len(rows), "counts": dist(v, 1.0, 2, keys=[f for f, _c in rows]),
            "n_frames_gt_0": sum(1 for c in v if c > 0), "frames_gt_0_first": [f for f, c in rows if c > 0][:30],
            "note": "skirt still capped in P1"}


# ---------------------------------------------------------------- main
IDS = (
    ("P1.1a", f"crude blend export-stage armature: bone names / parents / count exactly section 2 ({N_BONES} bones, "
              "8 skirt chains x 2)", "stage pick rule in measured.stage_pick; work rigs reported"),
    ("P1.1b", "export-stage armature: every bone use_connect False", "work rig connected bones reported"),
    ("P1.1c", f"stage / work armature + skinned mesh object and world scale = 1 +- {SCALE_TOL:g}; scene "
              "unit scale_length 1", ""),
    ("P1.1d", "rest orientation: *_L bone heads x > 0 and *_R x < 0; Head z > Pelvis z > Foot z; eye islands y < head "
              "island y (front -Y)", "A-pose arm angles reported only; eye/head islands matched to P0b centroids"),
    ("P1.1e", "every vertex of every mesh skinned by the rig has >= 1 bone-group weight > 0",
     "only vertex groups named like a bone of the skin armature count"),
    ("P1.1f", "(report) max bone influences per vertex", CALIB),
    ("P1.1g", "(report) per mesh island: P0b part, top bones and share; rigid parts vs section 3 bone", CALIB),
    ("P1.2a", "reimported Player.fbx and Player@p1test.fbx: bone set / parents = section 2, no *_end bone, "
              "importer auto-connect undone", "preset blender_reimport.import_scene_fbx options"),
    ("P1.2b", "(report) Player@p1test.fbx per-frame bone world pos (mm) / rot (deg) vs Blender stage; goblin "
              "reference 0.0006 mm / 0.1 deg", CALIB),
    ("P1.3a", "Unity import: animationType Generic, compression Off", "avatar / importBlendShapes / timing reported"),
    ("P1.3b", "Unity report bones: every section 2 bone present, parents = section 2 (Root parent reported)", ""),
    ("P1.3c", "Unity bindposes_ok true", ""),
    ("P1.3d", "Unity report errors empty", ""),
    ("P1.3e", "(report) non-skirt bones_world vs Blender stage mapped by M: pos mm / rot deg per frame", CALIB),
    ("P1.3f", "(report) skirt_raw Skirt_* vs Blender stage mapped by M: pos mm / rot deg per frame", CALIB),
    ("P1.4a", "spring chains: exactly one chain per Skirt_{F,FL,L,BL,B,BR,R,FR} chain, 8 chains",
     "chain matched when its bones contain Skirt_<dir>_01 or _02"),
    ("P1.4b", "(report) tip_deflection_max_m per chain (> 0 recorded)", CALIB),
    ("P1.4c", "(report) leg vertices inside skirt per frame", CALIB),
)


class Evidence:
    def __init__(self):
        self.inputs, self.criteria = [], []

    def add_input(self, path):
        r = rel(path)
        if Path(path).exists() and not any(i["path"] == r for i in self.inputs):
            self.inputs.append({"path": r, "sha256": sha256(path)})

    def criterion(self, cid, measured, threshold, ok, note):
        self.criteria.append({"id": cid, "ok": bool(ok), "measured": jsonable(measured),
                              "threshold": threshold, "note": note})

    def write(self):
        out = Path(P["out"])
        out.parent.mkdir(parents=True, exist_ok=True)
        order = {i: k for k, (i, _t, _n) in enumerate(IDS)}
        self.criteria.sort(key=lambda c: order.get(c["id"], 99))
        doc = {"gate": GATE, "checker": CHECKER, "task": TASK, "blender": bpy.app.version_string,
               "inputs": self.inputs, "criteria": self.criteria}
        with open(out, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            s = json.dumps(c["measured"], ensure_ascii=False)
            print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']} measured={s[:600]}")
        print(f"[{GATE}/{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


_STATE = {"missing": []}


def parse_args(argv):
    keys = {"--blend": "blend", "--export-dir": "export_dir", "--report": "report", "--stage": "stage",
            "--preset": "preset", "--p0b": "p0b", "--out": "out"}
    i = 0
    while i < len(argv):
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: {' '.join(k + ' <path>' for k in keys)}")
        P[keys[argv[i]]] = Path(argv[i + 1])
        i += 2


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    ev.add_input(__file__)
    ctx = Ctx()
    th = {i: t for i, t, _n in IDS}
    nt = {i: n for i, _t, n in IDS}
    done = set()

    def put(cid, measured, ok):
        ev.criterion(cid, measured, th[cid], ok, nt[cid])
        done.add(cid)

    def block(cids, reason):
        for c in cids:
            if c not in done:
                put(c, f"blocked: {reason}", False)

    fbx_main = P["export_dir"] / "Player.fbx"
    fbx_clip = P["export_dir"] / f"Player@{ACTION}.fbx"
    miss = [rel(p) for p in (P["blend"], fbx_main, fbx_clip, P["report"]) if not Path(p).exists()]
    for p in (P["blend"], fbx_main, fbx_clip, P["report"], P["p0b"]):
        ev.add_input(p)
    for p in sorted(HERE.glob("*.py")):
        if p.resolve() != Path(__file__).resolve():
            ev.add_input(p)
    for p in sorted((PLAYER_RIG / "data").glob("*.json")) if (PLAYER_RIG / "data").exists() else []:
        ev.add_input(p)
    for p in sorted((REPO / "unity" / "AvatarCheck" / "Assets").rglob("PlayerP1Check.cs")):
        ev.add_input(p)
    _STATE["missing"] = miss
    all_ids = [i for i, _t, _n in IDS]
    try:
        p0b_parts = load_json(P["p0b"]).get("parts") if Path(P["p0b"]).exists() else None
        try:
            pnote = pick_preset(ctx, ev)
        except Blocked as e:
            pnote = f"blocked: {e}"
        crude_stage = None
        if Path(P["blend"]).exists():
            try:
                crude_stage = p11(ctx, ev, put, block, p0b_parts)
            except Blocked as e:
                block([i for i in all_ids if i.startswith("P1.1")], str(e))
        else:
            block([i for i in all_ids if i.startswith("P1.1")], f"missing input {rel(P['blend'])}")
        try:
            sblend = load_stage(ctx, crude_stage) if Path(P["blend"]).exists() or P["stage"] else None
            if sblend is not None:
                ev.add_input(sblend)
            if ctx.stage_W is None:
                raise Blocked("no Blender stage")
        except Blocked as e:
            block(["P1.2b", "P1.3e", "P1.3f"], f"stage: {e}")
        # P1.2
        if ctx.opts is None:
            block(["P1.2a", "P1.2b"], f"preset: {pnote}")
        else:
            a_rec, a_ok = {"import": pnote}, True
            clip_arm = None
            for path in (fbx_main, fbx_clip):
                if not path.exists():
                    a_rec[path.name] = f"blocked: missing input {rel(path)}"
                    a_ok = False
                    continue
                try:
                    r, o, arm = fbx_hierarchy(ctx, path)
                    a_rec[path.name] = r
                    a_ok = a_ok and o
                    if path == fbx_clip:
                        clip_arm = arm
                        if "P1.2b" not in done:
                            put("P1.2b", p12b(ctx, arm), True)
                except Blocked as e:
                    a_rec[path.name] = f"blocked: {e}"
                    a_ok = False
                    if path == fbx_clip:
                        block(["P1.2b"], str(e))
            put("P1.2a", a_rec, a_ok)
            if clip_arm is None:
                block(["P1.2b"], f"missing input {rel(fbx_clip)}")
        # P1.3 / P1.4
        rep = None
        if Path(P["report"]).exists():
            try:
                rep = load_json(P["report"])
            except Exception:
                block([i for i in all_ids if i.startswith(("P1.3", "P1.4"))], f"report unreadable: {tb_tail(2)}")
        else:
            block([i for i in all_ids if i.startswith(("P1.3", "P1.4"))], f"missing input {rel(P['report'])}")
        if isinstance(rep, dict):
            for cid, fn in (("P1.3a", lambda: p13_import(rep)), ("P1.3b", lambda: p13_bones(rep))):
                try:
                    put(cid, *fn())
                except Blocked as e:
                    block([cid], str(e))
            bp = rep.get("bindposes_ok")
            if isinstance(bp, bool):
                put("P1.3c", {"bindposes_ok": bp}, bp)
            else:
                block(["P1.3c"], f"report 'bindposes_ok' missing or not bool ({bp!r})")
            er = rep.get("errors")
            if isinstance(er, list):
                put("P1.3d", {"n_errors": len(er), "errors": er[:30]}, not er)
            else:
                block(["P1.3d"], f"report 'errors' missing or not a list ({type(er).__name__})")
            for cid, key, names in (("P1.3e", "bones_world", BODY), ("P1.3f", "skirt_raw", SKIRT)):
                if cid in done:
                    continue
                try:
                    put(cid, p13_pose(ctx, rep, key, names), True)
                except Blocked as e:
                    block([cid], str(e))
            try:
                m, ok, chains = p14_chains(rep)
                put("P1.4a", m, ok)
                put("P1.4b", p14_deflection(rep, chains), True)
            except Blocked as e:
                block(["P1.4a", "P1.4b"], str(e))
            try:
                put("P1.4c", p14_penetration(rep), True)
            except Blocked as e:
                block(["P1.4c"], str(e))
        elif rep is not None:
            block([i for i in all_ids if i.startswith(("P1.3", "P1.4"))], "report is not a JSON object")
    finally:
        try:
            block(all_ids, "checker aborted before this criterion: " + tb_tail(2) if sys.exc_info()[0] else
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
