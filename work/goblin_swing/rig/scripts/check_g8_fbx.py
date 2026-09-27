"""check_g8_fbx - Gate G8 FBX checker (design doc d-23 sections 6 / 7, G8.1 .. G8.6; plan T34).

Inputs:
  rig/export/goblin.fbx, goblin@rigtest.fbx, goblin_club.fbx   production FBX files (s08_export_fbx.py)
  rig/data/export_preset.json       export settings + Blender->Unity axis mapping 3x3 (written by s08)
  rig/export/stage_rest.blend       G8.4 reference (goblin_mesh corner normals)
  rig/export/stage_rigtest.blend    G8.5 reference (GOB_export baked world transforms, every frame)
                                    (either stage missing -> built with s07b_export_rig.build_export from
                                    gob_r05_rigtest.blend into rig/export/tmp_g8/, deleted at the end)
  rig/data/canonical_skeleton.json  G8.3 reference rest (armature space, GOB_export object identity)
  rig/data/pivots.json              G8.6 grip_r (co, axis)
  rig/gob_r05_rigtest.blend         G8.6 club reference (GOB_club world vertices with GOB_rig at REST pose
                                    position, in memory); the FBX club is placed by the canonical weapon_socket_r
                                    rest_matrix before the comparison

Reimport: every FBX is imported into an empty scene (wm.read_factory_settings(use_empty=True)) with
  automatic_bone_orientation False and the importer's manual orientation set to the preset axes
  (use_manual_orientation True, axis_forward / axis_up = preset values, bake_space_transform = preset value,
  primary / secondary bone axis = preset values or Y / X).  The importer then applies the inverse of the export
  axis conversion, so all comparisons are made in Blender world space (axis correction = importer inverse
  mapping; the preset 3x3 matrix is recorded and compared to Blender's axis_conversion, report only).
  global_scale 1.0: the file UnitScaleFactor is honored (as Unity "Convert Units").

G8.7 (2026-09-26 HR2, rig/data/hr2_contract.md): goblin.fbx and every rig/export/goblin@*.fbx, each reimported the same
  way: goblin_mesh has the shape key mouth_open (BlendShape / BlendShapeChannel, relative) and the 4 contract materials;
  for every clip the reimported mouth_open value (DeformPercent curve / 100; no curve -> the static value, 0) at FBX
  frame g0+i equals the stage goblin_mesh evaluated value at stage frame s0+i (stage = rig/export/stage_<clip>.blend,
  rigtest: the G8.5 stage) within 0.01 on the x100 (Unity weight) scale.

Run:    bl.ps1 -Script check_g8_fbx.py            (no blend)
Output: rig/inspect/G8/check_g8_fbx.json
Exit:   0 when the evidence was written; 2 on a checker script error (run_main) or when an input is missing
        (evidence still written with blocked criteria, then "missing input" is printed).
Nothing is saved: blends / data are never written; temporary stage blends are removed.
Units: distance measured in m, reported in mm; rotation error = degrees(2 acos |q1.q2|).
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import goblib  # noqa: E402,E702

import importlib  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import shutil  # noqa: E402
import traceback  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Vector, kdtree  # noqa: E402
from bpy_extras.io_utils import axis_conversion  # noqa: E402

GATE = "G8"
CHECKER = "check_g8_fbx"
FBX_MAIN = goblib.EXPORT / "goblin.fbx"
FBX_RIG = goblib.EXPORT / "goblin@rigtest.fbx"
FBX_CLUB = goblib.EXPORT / "goblin_club.fbx"
PRESET_JSON = goblib.DATA / "export_preset.json"
CANON_JSON = goblib.DATA / "canonical_skeleton.json"
PIVOTS_JSON = goblib.DATA / "pivots.json"
MANIFEST_JSON = goblib.DATA / "rigtest_manifest.json"   # G8.5 report only: segment names of over-tol ranges
WORK_BLEND = goblib.RIG / "gob_r05_rigtest.blend"
STAGE_REST = goblib.EXPORT / "stage_rest.blend"
STAGE_RIG = goblib.EXPORT / "stage_rigtest.blend"
S07B_NAME = "s07b_export_rig"
S07B = goblib.RIG / "scripts" / f"{S07B_NAME}.py"
TMP = goblib.EXPORT / "tmp_g8"
ACTION = "rigtest"

ARM_NAME = "goblin"
MESH_NAME = "goblin_mesh"
CLUB_NAME = "goblin_club"
STAGE_ARM = "GOB_export"
STAGE_MESH = "goblin_mesh"
STAGE_CLUB = "goblin_club"
WORK_ARM = "GOB_rig"
WORK_CLUB = "GOB_club"
SOCKET = "weapon_socket_r"

N_BONES = 24
SCALE_TOL = 1e-6            # G8.2
REST_POS_TOL = 0.0005       # G8.3 (m)
REST_ROT_TOL = 0.5          # G8.3 (deg)
NORMAL_P99_TOL = 1.0        # G8.4 (deg)
POS_TOL = 0.001             # G8.5 (m)
ROT_TOL = 1.0               # G8.5 (deg)
CLUB_TRI_LIMIT = 1000       # G8.6 (strictly less)
CLUB_VERT_TOL = 0.0005      # G8.6 socket-rest-placed FBX club vertices vs Blender rest club (m)
TOPO_POS_TOL = 1e-5         # G8.4 / G8.6 index correspondence accepted when max vertex distance <= this (m)
CORNER_MATCH_TOL = 1e-4     # G8.4 fallback corner-point match distance (m)
MOUTH_KEY = "mouth_open"    # G8.7 (hr2_contract.md)
MAT_NAMES = ("GOB_skin", "GOB_mouth_inner", "GOB_teeth", "GOB_tongue")   # G8.7 contract material names
MOUTH_PCT_TOL = 0.01        # G8.7 |fbx - stage| x 100 (Unity blend-shape weight scale)
CLIP_GLOB = "goblin@*.fbx"

# G8.1 spec values (plan T35): preset key -> expected value
EXPECT = (("axis_forward", "-Z"), ("axis_up", "Y"), ("add_leaf_bones", False), ("bake_anim_step", 1.0),
          ("bake_anim_simplify_factor", 0.0), ("bake_anim_force_startend_keying", True))
RECORD = ("apply_unit_scale", "apply_scale_options", "bake_space_transform")
REPORT = ("bake_anim", "use_armature_deform_only", "object_types", "mesh_smooth_type", "use_custom_props",
          "primary_bone_axis", "secondary_bone_axis", "global_scale", "use_space_transform",
          "bake_anim_use_all_actions", "bake_anim_use_nla_strips", "bake_anim_use_all_bones", "armature_nodetype",
          "use_mesh_modifiers", "use_triangles", "use_tspace")
AXES = ("X", "Y", "Z", "-X", "-Y", "-Z")

_STATE = {"missing": []}


# ---------------------------------------------------------------- helpers
class Blocked(Exception):
    """A criterion cannot be measured (missing object / action / failed import)."""


def mm(x):
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=5):
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def sci(x):
    x = float(x)
    return float(f"{x:.3e}") if math.isfinite(x) else str(x)


def rel(p):
    return goblib._rel(p)


def tb_tail(n=6):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


def mat_rows(M, n=6):
    return [[rnd(x, n) for x in row] for row in M]


def fcurves_of(action):
    out = []
    for layer in getattr(action, "layers", ()):
        for strip in layer.strips:
            for cb in getattr(strip, "channelbags", ()):
                out.extend(cb.fcurves)
    if not out and hasattr(action, "fcurves"):
        out.extend(action.fcurves)
    return out


def quat(M):
    return M.decompose()[1]


def world_err(A, B):
    """(pos m, rot deg) between two world matrices (decompose rotation)."""
    return (A.translation - B.translation).length, goblib.quat_angle_deg(quat(A), quat(B))


def open_blend(path):
    bpy.ops.wm.open_mainfile(filepath=str(path), load_ui=False)


def inventory():
    out = []
    for o in sorted(bpy.data.objects, key=lambda x: x.name):
        out.append({"name": o.name, "type": o.type, "parent": o.parent.name if o.parent else None,
                    "parent_type": o.parent_type, "scale": [rnd(s, 8) for s in o.scale],
                    "world_scale": [rnd(s, 8) for s in o.matrix_world.to_scale()],
                    "matrix_world": mat_rows(o.matrix_world),
                    "matrix_basis": mat_rows(o.matrix_basis)})
    return out


# ---------------------------------------------------------------- preset (G8.1)
def flatten(d, pre=""):
    out = []
    if isinstance(d, dict):
        for k, v in d.items():
            p = f"{pre}.{k}" if pre else str(k)
            out.append((p, str(k), v))
            out.extend(flatten(v, p))
    return out


def is_3x3(v):
    try:
        a = np.array(v, dtype=np.float64)
    except (TypeError, ValueError):
        return False
    return a.shape == (3, 3) and bool(np.isfinite(a).all())


def same_value(v, exp):
    if isinstance(exp, bool):
        return isinstance(v, bool) and v == exp
    if isinstance(exp, float):
        return isinstance(v, (int, float)) and not isinstance(v, bool) and abs(float(v) - exp) <= 1e-9
    return v == exp


def preset_lookup(flat, key):
    hits = [(p, v) for p, k, v in flat if k == key and not isinstance(v, dict)]
    vals = []
    for _p, v in hits:
        if v not in vals:
            vals.append(v)
    return hits, vals


def read_preset():
    with open(PRESET_JSON, "r", encoding="utf-8") as f:
        return json.load(f)


def export_flat(preset):
    """Flattened export settings: the 'export_scene_fbx' section if present, else the whole preset without the
    reimport / fbx_files records (they repeat some key names with other meanings)."""
    sec = preset.get("export_scene_fbx") if isinstance(preset, dict) else None
    if isinstance(sec, dict):
        return flatten(sec, "export_scene_fbx"), "export_scene_fbx"
    return ([x for x in flatten(preset) if not x[0].startswith(("blender_reimport", "fbx_files"))],
            "whole preset (no export_scene_fbx section)")


def c_g81(preset):
    files = {rel(p): p.exists() for p in (FBX_MAIN, FBX_RIG, FBX_CLUB)}
    sizes = {rel(p): p.stat().st_size for p in (FBX_MAIN, FBX_RIG, FBX_CLUB) if p.exists()}
    rec = {"files_exist": files, "file_sizes_bytes": sizes}
    ok = all(files.values())
    if preset is None:
        rec["preset"] = f"missing: {rel(PRESET_JSON)}"
        return rec, False
    flat, section = export_flat(preset)
    rec["settings_section"] = section
    spec = {}
    for key, exp in EXPECT:
        hits, vals = preset_lookup(flat, key)
        good = len(vals) == 1 and same_value(vals[0], exp)
        spec[key] = {"expected": exp, "found": [[p, v] for p, v in hits], "ok": good}
        ok = ok and good
    rec_v = {}
    for key in RECORD:
        hits, vals = preset_lookup(flat, key)
        good = len(vals) == 1
        rec_v[key] = {"found": [[p, v] for p, v in hits], "recorded_once": good}
        ok = ok and good
    cands = [(p, np.array(v, dtype=np.float64)) for p, _k, v in flatten(preset)
             if is_3x3(v) and ("axis" in p.lower() or "unity" in p.lower())]
    unity = [(p, m) for p, m in cands if "unity" in p.lower()]
    ax = {"candidates": [[p, m.tolist()] for p, m in cands], "blender_to_unity_paths": [p for p, _m in unity]}
    if unity:
        m0 = unity[0][1]
        consistent = all(np.abs(m - m0).max() <= 1e-9 for _p, m in unity)
        ax["blender_to_unity_consistent"] = consistent
        ax["blender_to_unity_det"] = rnd(np.linalg.det(m0), 6)
        ax["blender_to_unity_orthonormal_max_dev"] = sci(np.abs(m0 @ m0.T - np.eye(3)).max())
        fwd = spec["axis_forward"]["found"]
        up = spec["axis_up"]["found"]
        if fwd and up and fwd[0][1] in AXES and up[0][1] in AXES:
            try:
                conv = np.array(axis_conversion(to_forward=fwd[0][1], to_up=up[0][1]), dtype=np.float64)
                flip = np.diag([-1.0, 1.0, 1.0]) @ conv
                ax["axis_conversion_blender_to_fbx_report"] = conv.tolist()
                ax["preset_matrices_equal_axis_conversion_report"] = {
                    p: bool(np.abs(m - conv).max() <= 1e-6) for p, m in cands}
                ax["blender_to_unity_equals_xflip_at_axis_conversion_report"] = bool(
                    np.abs(m0 - flip).max() <= 1e-6)
            except Exception:
                ax["axis_conversion_blender_to_fbx_report"] = f"error: {tb_tail(2)}"
        ok = ok and consistent
    else:
        ok = False
    rep = {}
    for key in REPORT:
        hits, _vals = preset_lookup(flat, key)
        if hits:
            rep[key] = [[p, v] for p, v in hits]
    rec.update({"spec_values": spec, "recorded_values": rec_v, "axis_matrix": ax, "other_settings_report": rep,
                "preset_top_keys": sorted(preset.keys()) if isinstance(preset, dict) else str(type(preset))})
    return rec, ok


def import_options(preset):
    """Importer options = inverse of the preset axis mapping (see module docstring)."""
    flat = export_flat(preset)[0] if preset is not None else []
    notes = []

    def pick(key, default, valid=None):
        _h, vals = preset_lookup(flat, key)
        if len(vals) == 1 and (valid is None or vals[0] in valid):
            return vals[0]
        notes.append(f"{key}: preset value {vals!r} not usable, importer uses {default!r}")
        return default

    fwd = pick("axis_forward", "-Z", AXES)
    up = pick("axis_up", "Y", AXES)
    bst = pick("bake_space_transform", False, (True, False))
    pba = pick("primary_bone_axis", "Y", AXES)
    sba = pick("secondary_bone_axis", "X", AXES)
    opts = dict(use_manual_orientation=True, axis_forward=fwd, axis_up=up, global_scale=1.0,
                bake_space_transform=bool(bst), use_custom_normals=True, use_image_search=False, use_anim=True,
                anim_offset=1.0, use_custom_props=False, ignore_leaf_bones=False, force_connect_children=False,
                automatic_bone_orientation=False, primary_bone_axis=pba, secondary_bone_axis=sba,
                use_prepost_rot=True)
    # report only: differences to the production's own reimport record
    pr = (preset.get("blender_reimport") or {}).get("import_scene_fbx") if isinstance(preset, dict) else None
    if isinstance(pr, dict):
        diff = {k: [v, opts.get(k)] for k, v in pr.items() if k in opts and opts[k] != v}
        notes.append(f"preset blender_reimport.import_scene_fbx differences [preset, checker]: {diff}")
    return opts, notes


def fresh_import(path, opts):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    try:
        res = bpy.ops.import_scene.fbx(filepath=str(path), **opts)
    except Exception:
        raise Blocked(f"import {rel(path)} failed: {tb_tail(3)}")
    if "FINISHED" not in res:
        raise Blocked(f"import {rel(path)} returned {sorted(res)}")
    bpy.context.view_layer.update()


# ---------------------------------------------------------------- canonical / references
def load_canon():
    with open(CANON_JSON, "r", encoding="utf-8") as f:
        canon = json.load(f)
    bones = canon.get("bones") if isinstance(canon, dict) else None
    if not isinstance(bones, list) or not bones:
        raise Blocked("canonical_skeleton.json: no 'bones' list")
    names, parent, mat, length = [], {}, {}, {}
    for b in bones:
        n = b["name"]
        names.append(n)
        parent[n] = b.get("parent") or None
        mat[n] = Matrix(b["rest_matrix"])
        if "head" in b and "tail" in b:
            length[n] = (Vector(b["tail"]) - Vector(b["head"])).length
    return names, parent, mat, length


def mesh_arrays(obj):
    me = obj.data
    nv, nl, npo = len(me.vertices), len(me.loops), len(me.polygons)
    co = np.empty(nv * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(nv, 3)
    vi = np.empty(nl, dtype=np.int64)
    me.loops.foreach_get("vertex_index", vi)
    ls = np.empty(npo, dtype=np.int64)
    lt = np.empty(npo, dtype=np.int64)
    me.polygons.foreach_get("loop_start", ls)
    me.polygons.foreach_get("loop_total", lt)
    nrm = np.empty(nl * 3)
    me.corner_normals.foreach_get("vector", nrm)
    nrm = nrm.reshape(nl, 3)
    M = np.array(obj.matrix_world, dtype=np.float64)
    co_w = co @ M[:3, :3].T + M[:3, 3]
    N3 = np.linalg.inv(M[:3, :3]).T
    nw = nrm @ N3.T
    ln = np.linalg.norm(nw, axis=1)
    n_zero = int((ln < 1e-12).sum())
    nw = nw / np.maximum(ln, 1e-12)[:, None]
    order = np.argsort(ls)
    face_of_loop = np.empty(nl, dtype=np.int64)
    for f in order:
        face_of_loop[ls[f]:ls[f] + lt[f]] = f
    cen = np.zeros((npo, 3))
    np.add.at(cen, face_of_loop, co_w[vi])
    cen /= np.maximum(lt, 1)[:, None]
    return {"name": obj.name, "nv": nv, "nl": nl, "np": npo, "co_w": co_w, "vi": vi, "ls": ls, "lt": lt,
            "n": nw, "n_zero": n_zero, "face_of_loop": face_of_loop, "cen": cen,
            "has_custom_normals": bool(getattr(me, "has_custom_normals", False)),
            "matrix_world": mat_rows(obj.matrix_world)}


def stage_rest_ref(ctx):
    open_blend(ctx.stage_rest)
    mesh = bpy.data.objects.get(STAGE_MESH)
    if mesh is None or mesh.type != "MESH":
        raise Blocked(f"{STAGE_MESH} missing in {rel(ctx.stage_rest)}")
    ctx.ref_mesh = mesh_arrays(mesh)
    club = bpy.data.objects.get(STAGE_CLUB)
    ctx.stage_club_mw = club.matrix_world.copy() if club is not None else None
    if club is not None and club.type == "MESH":
        V = np.empty(len(club.data.vertices) * 3)
        club.data.vertices.foreach_get("co", V)
        M = np.array(club.matrix_world, dtype=np.float64)
        ctx.stage_club_W = V.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]


def stage_rig_ref(ctx):
    open_blend(ctx.stage_rig)
    exp = bpy.data.objects.get(STAGE_ARM)
    if exp is None or exp.type != "ARMATURE":
        raise Blocked(f"{STAGE_ARM} missing in {rel(ctx.stage_rig)}")
    ad = exp.animation_data
    act = ad.action if ad else None
    if act is None:
        raise Blocked(f"{STAGE_ARM} has no action in {rel(ctx.stage_rig)}")
    missing = [n for n in ctx.names if n not in exp.pose.bones]
    if missing:
        raise Blocked(f"{STAGE_ARM} lacks bones {missing}")
    f0, f1 = (int(round(x)) for x in act.frame_range)
    sc = bpy.context.scene
    frames = list(range(f0, f1 + 1))
    S = []
    for f in frames:
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = exp.evaluated_get(dg)
        mw = oe.matrix_world.copy()
        S.append([mw @ oe.pose.bones[n].matrix for n in ctx.names])
    ctx.stage_frames = frames
    ctx.stage_world = S
    ctx.stage_info = {"action": act.name, "frame_range": [f0, f1], "n_frames": len(frames),
                      "fps": rnd(sc.render.fps / (sc.render.fps_base or 1.0), 4)}


def work_club_ref(ctx):
    open_blend(WORK_BLEND)
    club = bpy.data.objects.get(WORK_CLUB)
    if club is None or club.type != "MESH":
        raise Blocked(f"{WORK_CLUB} missing in {rel(WORK_BLEND)}")
    rig = bpy.data.objects.get(WORK_ARM)
    if rig is not None and rig.type == "ARMATURE":
        rig.data.pose_position = "REST"   # in memory only; never saved
    bpy.context.view_layer.update()
    me = club.data
    V = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", V)
    V = V.reshape(-1, 3)
    M = np.array(club.matrix_world, dtype=np.float64)
    ctx.club_ref = {"V_loc": V, "M": M, "W": V @ M[:3, :3].T + M[:3, 3],
                    "tris": int(sum(p.loop_total - 2 for p in me.polygons)),
                    "modifiers": [m.type for m in club.modifiers],
                    "parent": [club.parent.name if club.parent else None, club.parent_type, club.parent_bone],
                    "rig_pose_position": "REST" if rig is not None else "no rig"}


# ---------------------------------------------------------------- G8.2 / G8.3
def c_g82_main():
    inv = inventory()
    by = {o["name"]: o for o in inv}
    expect = {ARM_NAME: "ARMATURE", MESH_NAME: "MESH"}
    missing = [f"{n}({t})" for n, t in expect.items() if n not in by or by[n]["type"] != t]
    extra = [f"{o['name']}({o['type']})" for o in inv if o["name"] not in expect]
    sdev = {}
    for n in expect:
        o = bpy.data.objects.get(n)
        if o is None:
            continue
        sdev[n] = {"object_scale_max_dev": sci(max(abs(s - 1.0) for s in o.scale)),
                   "world_scale_max_dev": sci(max(abs(s - 1.0) for s in o.matrix_world.to_scale()))}
    s_ok = len(sdev) == 2 and all(float(v["object_scale_max_dev"]) <= SCALE_TOL
                                  and float(v["world_scale_max_dev"]) <= SCALE_TOL for v in sdev.values())
    ok = not missing and not extra and s_ok
    return {"objects": inv, "missing_expected": missing, "extra_nodes": extra, "scale_dev": sdev}, ok


def find_arm():
    o = bpy.data.objects.get(ARM_NAME)
    if o is not None and o.type == "ARMATURE":
        return o, None
    arms = [x for x in bpy.data.objects if x.type == "ARMATURE"]
    if len(arms) == 1:
        return arms[0], f"armature object named {arms[0].name!r}, not {ARM_NAME!r}"
    raise Blocked(f"no armature {ARM_NAME!r} (armatures: {[a.name for a in arms]})")


def skeleton(ctx):
    arm, note = find_arm()
    bones = arm.data.bones
    names = [b.name for b in bones]
    ends = sorted(n for n in names if n.lower().endswith("_end"))
    only_c = sorted(set(ctx.names) - set(names))
    only_f = sorted(set(names) - set(ctx.names))
    par_mis = {}
    pos, rot, lens = {}, {}, {}
    mw = arm.matrix_world
    for n in ctx.names:
        b = bones.get(n)
        if b is None:
            continue
        p = b.parent.name if b.parent else None
        if p != ctx.parent[n]:
            par_mis[n] = {"canon": ctx.parent[n], "fbx": p}
        W = mw @ b.matrix_local
        pos[n], rot[n] = world_err(W, ctx.canon_mat[n])
        if n in ctx.length:
            lens[n] = mm(b.length - ctx.length[n])
    wp = max(pos, key=pos.get) if pos else None
    wr = max(rot, key=rot.get) if rot else None
    ok = (len(names) == N_BONES and not only_c and not only_f and not par_mis and not ends and len(pos) == N_BONES
          and pos[wp] <= REST_POS_TOL and rot[wr] <= REST_ROT_TOL)
    rec = {"armature": arm.name, "armature_note": note, "n_bones": len(names), "only_canon": only_c,
           "only_fbx": only_f, "parent_mismatch": par_mis, "end_leaf_bones": ends,
           "rest_pos_mm_max": mm(pos[wp]) if wp else None, "rest_pos_worst": wp,
           "rest_rot_deg_max": rnd(rot[wr], 4) if wr else None, "rest_rot_worst": wr,
           "per_bone_[pos_mm,rot_deg]": {n: [mm(pos[n]), rnd(rot[n], 4)] for n in pos},
           "bone_length_minus_canon_mm_report": lens,
           "armature_matrix_world": mat_rows(mw)}
    return rec, ok


def disconnect_bones():
    """T34c: the FBX importer connects a child whose head lies on the parent's computed tail (the connected
    bone then ignores its location keys; importer behaviour, not file content).  Set every bone use_connect
    False in edit mode, back to object mode.  Returns {"connected_on_import", "connected_after", "undone"}."""
    arm, _note = find_arm()
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


def skeleton_disconnected(ctx):
    """G8.3 record measured after disconnect_bones(); the pre-disconnect values are kept as a report."""
    pre, _o = skeleton(ctx)
    conn = disconnect_bones()
    rec, ok = skeleton(ctx)
    rec["importer_connected_bones"] = conn
    rec["before_disconnect_report"] = {k: pre[k] for k in ("rest_pos_mm_max", "rest_pos_worst", "rest_rot_deg_max",
                                                           "rest_rot_worst", "per_bone_[pos_mm,rot_deg]")}
    return rec, ok, conn


# ---------------------------------------------------------------- G8.4
def c_g84(ctx):
    obj = bpy.data.objects.get(MESH_NAME)
    if obj is None or obj.type != "MESH":
        meshes = [o for o in bpy.data.objects if o.type == "MESH"]
        if len(meshes) != 1:
            raise Blocked(f"no mesh {MESH_NAME!r} in reimported goblin.fbx (meshes {[m.name for m in meshes]})")
        obj = meshes[0]
    A = mesh_arrays(obj)
    R = ctx.ref_mesh
    topo = {"fbx": [A["nv"], A["np"], A["nl"]], "stage_rest": [R["nv"], R["np"], R["nl"]]}
    same = (A["nv"] == R["nv"] and A["np"] == R["np"] and A["nl"] == R["nl"]
            and np.array_equal(A["vi"], R["vi"]) and np.array_equal(A["ls"], R["ls"])
            and np.array_equal(A["lt"], R["lt"]))
    vdev = float(np.linalg.norm(A["co_w"] - R["co_w"], axis=1).max()) if A["nv"] == R["nv"] else float("inf")
    rec = {"mesh": obj.name, "counts_[verts,faces,corners]": topo,
           "vertex_pos_max_dev_mm_by_index": mm(vdev),
           "fbx_has_custom_normals": A["has_custom_normals"], "stage_has_custom_normals": R["has_custom_normals"],
           "zero_length_normals": {"fbx": A["n_zero"], "stage": R["n_zero"]},
           "fbx_matrix_world": A["matrix_world"], "stage_matrix_world": R["matrix_world"]}
    if same and vdev <= TOPO_POS_TOL:
        rec["match"] = "index (same vertices / faces / corner order, vertex positions within " \
                       f"{TOPO_POS_TOL * 1000:g} mm)"
        idx = np.arange(A["nl"])
        un = 0
    else:
        # corner point = 0.8 vertex + 0.2 face centroid (world); nearest stage corner point
        pa = 0.8 * A["co_w"][A["vi"]] + 0.2 * A["cen"][A["face_of_loop"]]
        pr = 0.8 * R["co_w"][R["vi"]] + 0.2 * R["cen"][R["face_of_loop"]]
        kd = kdtree.KDTree(len(pr))
        for i, p in enumerate(pr):
            kd.insert(p, i)
        kd.balance()
        idx = np.full(A["nl"], -1, dtype=np.int64)
        for i, p in enumerate(pa):
            _c, j, d = kd.find(p)
            if j is not None and d <= CORNER_MATCH_TOL:
                idx[i] = j
        un = int((idx < 0).sum())
        rec["match"] = ("position (corner point = 0.8 vertex + 0.2 face centroid, nearest stage corner within "
                        f"{CORNER_MATCH_TOL * 1000:g} mm)")
    m = idx >= 0
    dots = np.clip((A["n"][m] * R["n"][idx[m]]).sum(1), -1.0, 1.0)
    ang = np.degrees(np.arccos(dots))
    rec["unmatched_corners"] = un
    rec["n_compared"] = int(m.sum())
    if len(ang):
        p99 = float(np.percentile(ang, 99))
        rec.update({"angle_deg_p99": rnd(p99, 4), "angle_deg_mean": rnd(ang.mean(), 4),
                    "angle_deg_max": rnd(ang.max(), 4), "n_over_1deg": int((ang > 1.0).sum())})
        ok = p99 <= NORMAL_P99_TOL and un == 0 and m.sum() == R["nl"]
    else:
        ok = False
    return rec, bool(ok)


# ---------------------------------------------------------------- G8.5
def c_g85(ctx):
    arm, note = find_arm()
    ad = arm.animation_data
    act = ad.action if ad else None
    acts = [a.name for a in bpy.data.actions]
    if act is None:
        raise Blocked(f"reimported armature {arm.name} has no action (actions: {acts})")
    missing = [n for n in ctx.names if n not in arm.pose.bones]
    fcs = fcurves_of(act)
    times = [k.co.x for fc in fcs for k in fc.keyframe_points]
    if not times:
        raise Blocked(f"action {act.name} has no keys")
    t0, t1 = min(times), max(times)
    non_int = sum(1 for t in times if abs(t - round(t)) > 1e-4)
    nkeys = [len(fc.keyframe_points) for fc in fcs]
    g0, g1 = int(round(t0)), int(round(t1))
    n_fbx = g1 - g0 + 1
    n_stage = len(ctx.stage_frames)
    sc = bpy.context.scene
    info = {"armature": arm.name, "armature_note": note, "action": act.name, "actions_in_file": acts,
            "key_time_range": [rnd(t0, 4), rnd(t1, 4)], "n_frames_fbx": n_fbx, "stage": ctx.stage_info,
            "frame_offset_fbx_minus_stage": g0 - ctx.stage_frames[0],
            "frame_offset_note": "importer anim_offset 1.0 shifts keys; corrected by aligning range starts",
            "keys_not_on_integer_frame": non_int,
            "n_fcurves": len(fcs), "keys_per_fcurve_[min,max]": [min(nkeys), max(nkeys)],
            "scene_fps_after_import": rnd(sc.render.fps / (sc.render.fps_base or 1.0), 4),
            "bones_missing": missing}
    if missing:
        return info, False
    n = min(n_fbx, n_stage)
    B = len(ctx.names)
    P = np.zeros((n, B))
    Rr = np.zeros((n, B))
    Tf = np.zeros((n, B, 3))
    for i in range(n):
        sc.frame_set(g0 + i)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = arm.evaluated_get(dg)
        mw = oe.matrix_world
        for j, name in enumerate(ctx.names):
            Mf = mw @ oe.pose.bones[name].matrix
            Tf[i, j] = tuple(Mf.translation)
            P[i, j], Rr[i, j] = world_err(Mf, ctx.stage_world[i][j])
    ip = np.unravel_index(int(P.argmax()), P.shape)
    ir = np.unravel_index(int(Rr.argmax()), Rr.shape)
    info.update({"n_frames_compared": n, "alignment": "fbx frame g0+i vs stage frame s0+i",
                 "pos_mm_max": mm(P.max()), "pos_worst_[bone,stage_frame]": [ctx.names[ip[1]],
                                                                              ctx.stage_frames[ip[0]]],
                 "rot_deg_max": rnd(Rr.max(), 4), "rot_worst_[bone,stage_frame]": [ctx.names[ir[1]],
                                                                                 ctx.stage_frames[ir[0]]],
                 "n_samples_over_tol": int(((P > POS_TOL) | (Rr > ROT_TOL)).sum()),
                 "per_bone_max_[pos_mm,rot_deg]": {nm: [mm(P[:, j].max()), rnd(Rr[:, j].max(), 4)]
                                                   for j, nm in enumerate(ctx.names)}})
    info.update(g85_over_tol_report(ctx, P, Rr, Tf))
    ok = n_fbx == n_stage and n > 0 and P.max() <= POS_TOL and Rr.max() <= ROT_TOL
    return info, bool(ok)


def g85_over_tol_report(ctx, P, Rr, Tf):
    """Report only (T34b): contiguous stage-frame ranges where any bone is over tolerance, mapped to the
    rigtest_manifest segments they overlap, and a per-bone table at the worst position frame."""
    segs = None
    try:
        with open(MANIFEST_JSON, "r", encoding="utf-8") as f:
            segs = [(s["name"], int(s["start"]), int(s["end"])) for s in json.load(f)["segments"]]
    except Exception:
        segs = None
    over = (P > POS_TOL) | (Rr > ROT_TOL)
    bad = over.any(axis=1)
    ranges = []
    i, n = 0, len(bad)
    while i < n:
        if not bad[i]:
            i += 1
            continue
        k = i
        while k + 1 < n and bad[k + 1]:
            k += 1
        sub_p, sub_r = P[i:k + 1], Rr[i:k + 1]
        ratio = np.maximum(sub_p / POS_TOL, sub_r / ROT_TOL)
        wb = ctx.names[int(np.unravel_index(int(ratio.argmax()), ratio.shape)[1])]
        f0, f1 = ctx.stage_frames[i], ctx.stage_frames[k]
        seg = ("manifest unreadable" if segs is None
               else [nm for nm, a, b in segs if a <= f1 and b >= f0])
        ranges.append([f0, f1, mm(sub_p.max()), rnd(sub_r.max(), 4), wb, seg])
        i = k + 1
    iw = int(np.unravel_index(int(P.argmax()), P.shape)[0])
    table = {}
    for j, nm in enumerate(ctx.names):
        ts = ctx.stage_world[iw][j].translation
        table[nm] = [mm(P[iw, j]), [rnd(x, 6) for x in ts], [rnd(x, 6) for x in Tf[iw, j]]]
    return {"over_tol_ranges_columns": ["stage_start", "stage_end", "max_mm", "max_deg", "worst_bone",
                                        "segment (manifest segments overlapping the range)"],
            "over_tol_ranges": ranges,
            "worst_frame_table": {"stage_frame": ctx.stage_frames[iw],
                                  "columns": ["pos_mm", "stage_world_loc", "fbx_world_loc"], "bones": table}}


# ---------------------------------------------------------------- G8.6
def kabsch(P, Q):
    cp, cq = P.mean(0), Q.mean(0)
    H = (P - cp).T @ (Q - cq)
    U, _S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, cq - R @ cp


def kd_of(P):
    kd = kdtree.KDTree(len(P))
    for i, p in enumerate(P):
        kd.insert(p, i)
    kd.balance()
    return kd


def sym_nn(X, W):
    """Symmetric nearest-vertex distances (X->W and W->X), concatenated."""
    kw, kx = kd_of(W), kd_of(X)
    return np.concatenate([np.array([kw.find(p)[2] for p in X]), np.array([kx.find(p)[2] for p in W])])


def c_g86(ctx):
    inv = inventory()
    obj = bpy.data.objects.get(CLUB_NAME)
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    note = None
    if obj is None or obj.type != "MESH":
        if len(meshes) != 1:
            raise Blocked(f"no mesh {CLUB_NAME!r} in goblin_club.fbx (meshes {[m.name for m in meshes]})")
        obj = meshes[0]
        note = f"club mesh object named {obj.name!r}, not {CLUB_NAME!r}"
    if SOCKET not in ctx.canon_mat:
        raise Blocked(f"{SOCKET} not in canonical_skeleton.json")
    me = obj.data
    tris = int(sum(p.loop_total - 2 for p in me.polygons))
    U = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", U)
    U = U.reshape(-1, 3)
    Mi = np.array(obj.matrix_world, dtype=np.float64)
    Uf = U @ Mi[:3, :3].T + Mi[:3, 3]            # vertices in the FBX file root frame (Blender axes)
    S = np.array(ctx.canon_mat[SOCKET], dtype=np.float64)   # socket rest world (GOB_export identity)
    X = Uf @ S[:3, :3].T + S[:3, 3]              # club placed at the socket rest, local 0
    ref = ctx.club_ref
    W = ref["W"]
    nn = sym_nn(X, W)
    if len(X) == len(W):
        d = np.linalg.norm(X - W, axis=1)
        method = "index (same vertex count)"
    else:
        d = nn
        method = "symmetric nearest vertex (vertex counts differ)"
    dev = float(d.max())
    # report: rigid fit of the file-frame vertices onto the reference (origin / +Y / roll)
    fit = {}
    if len(Uf) == len(W):
        R, t = kabsch(Uf, W)
        fit["method"] = "index Kabsch"
    else:
        R, t = S[:3, :3] / np.linalg.norm(S[:3, :3], axis=0), S[:3, 3].copy()
        kw = kd_of(W)
        for _it in range(50):
            nnx = np.array([kw.find(p)[1] for p in Uf @ R.T + t], dtype=np.int64)
            R2, t2 = kabsch(Uf, W[nnx])
            fin = np.abs(R2 - R).max() < 1e-10 and np.abs(t2 - t).max() < 1e-10
            R, t = R2, t2
            if fin:
                break
        fit["method"] = "ICP nearest-vertex (counts differ)"
    res = np.linalg.norm(Uf @ R.T + t - W, axis=1) if len(Uf) == len(W) else \
        np.array([kd_of(W).find(p)[2] for p in Uf @ R.T + t])
    grip = ctx.pivots["grip_r"]
    gco = np.array(grip["co"], dtype=np.float64)
    gax = np.array(grip["axis"], dtype=np.float64)
    gax /= np.linalg.norm(gax)
    S3 = S[:3, :3] / np.linalg.norm(S[:3, :3], axis=0)

    def ang(a, b):
        a = a / np.linalg.norm(a)
        b = b / np.linalg.norm(b)
        return rnd(math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b))))), 4)

    fit.update({"residual_mm_max": mm(res.max()), "fbx_origin_in_world": [rnd(x, 6) for x in t],
                "fbx_origin_to_grip_r_mm": mm(np.linalg.norm(t - gco)),
                "fbx_local_y_vs_grip_r_axis_deg": ang(R[:, 1], gax),
                "fbx_local_x_vs_socket_rest_x_deg (roll)": ang(R[:, 0], S3[:, 0]),
                "fit_rotation_vs_socket_rest_deg": rnd(goblib.quat_angle_deg(
                    Matrix(R.tolist()).to_quaternion(), Matrix(S3.tolist()).to_quaternion()), 4),
                "socket_rest_origin_to_grip_r_mm": mm(np.linalg.norm(S[:3, 3] - gco)),
                "socket_rest_y_vs_grip_r_axis_deg": ang(S3[:, 1], gax)})
    Mr = ref["M"]
    stage_club = None
    if ctx.stage_club_mw is not None:
        Sm = np.array(ctx.stage_club_mw, dtype=np.float64)
        stage_club = {"matrix_world_max_abs_diff_vs_work_ref": sci(np.abs(Sm - Mr).max())}
        if ctx.stage_club_W is not None:
            Ws = ctx.stage_club_W
            if len(Ws) == len(W):
                stage_club["verts_max_dev_mm_vs_work_ref"] = mm(np.linalg.norm(Ws - W, axis=1).max())
            if len(Ws) == len(X):
                stage_club["fbx_socket_placed_verts_max_dev_mm_vs_stage_rest"] = mm(
                    np.linalg.norm(X - Ws, axis=1).max())
    rec = {"club_object": obj.name, "club_note": note, "n_mesh_objects": len(meshes), "objects": inv,
           "triangles": tris, "work_club_triangles_report": ref["tris"],
           "n_verts_[fbx,ref]": [len(X), len(W)],
           "verts_max_dev_mm": mm(dev), "verts_mean_dev_mm": mm(d.mean()), "match": method,
           "verts_sym_nearest_max_dev_mm_report": mm(nn.max()),
           "fbx_club_matrix_world": mat_rows(obj.matrix_world),
           "socket_rest_matrix_world": mat_rows(S),
           "origin_axis_roll_report": fit,
           "work_ref": {"blend": rel(WORK_BLEND), "object": WORK_CLUB, "pose_position": ref["rig_pose_position"],
                        "parent": ref["parent"], "modifiers": ref["modifiers"], "matrix_world": mat_rows(Mr)},
           "stage_rest_club_report": stage_club}
    ok = tris < CLUB_TRI_LIMIT and dev <= CLUB_VERT_TOL
    return rec, bool(ok)


# ---------------------------------------------------------------- main
IDS = (
    ("G8.1", "goblin.fbx, goblin@rigtest.fbx, goblin_club.fbx exist; export_preset.json: axis_forward '-Z', "
             "axis_up 'Y', add_leaf_bones False, bake_anim_step 1.0, bake_anim_simplify_factor 0.0, "
             "bake_anim_force_startend_keying True (each found with one value); apply_unit_scale, "
             "apply_scale_options, bake_space_transform recorded; Blender->Unity axis 3x3 recorded",
     "settings read from the preset 'export_scene_fbx' section (nested dicts searched by key name; whole preset "
     "minus blender_reimport / fbx_files when the section is absent); Blender->Unity matrix = finite 3x3 under a "
     "path containing 'unity'; det / orthonormality / equality with Blender axis_conversion(-Z,Y) (and its "
     "X-flip) reported only"),
    ("G8.2", f"reimported goblin.fbx: objects exactly {ARM_NAME}(ARMATURE) + {MESH_NAME}(MESH), no other node; "
             f"both object scale and world scale = 1 +- {SCALE_TOL:g}",
     "empty factory scene + import_scene.fbx (manual orientation = preset axes, bake_space_transform = preset, "
     "automatic_bone_orientation False, global_scale 1.0); object matrix_world / matrix_basis reported; "
     "goblin@rigtest.fbx inventory reported only"),
    ("G8.3", f"goblin.fbx and goblin@rigtest.fbx: {N_BONES} bones, names and parents = canonical, no *_end "
             f"bone, rest (armature matrix_world @ bone.matrix_local) vs canonical rest_matrix <= "
             f"{REST_POS_TOL * 1000:g} mm and <= {REST_ROT_TOL:g} deg",
     "axis correction by the importer inverse mapping (preset axes as import axes); every bone set use_connect "
     "False in edit mode after import (importer_connected_bones; pre-disconnect values reported); position = "
     "|t1-t2|, rotation = 2 acos|q1.q2| of decompose(); bone length vs canonical reported only"),
    ("G8.4", f"custom normals: p99 angle(reimported goblin_mesh corner normals, stage_rest goblin_mesh corner "
             f"normals) <= {NORMAL_P99_TOL:g} deg, every stage corner matched",
     "mesh.corner_normals of the (unevaluated) mesh data transformed to world by inverse-transpose of "
     "matrix_world; corners matched by index when counts / corner vertex indices / face offsets are identical "
     f"and vertex positions agree <= {TOPO_POS_TOL * 1000:g} mm, else by nearest corner point "
     "(0.8 vertex + 0.2 face centroid)"),
    ("G8.5", f"reimported rigtest frame count = stage_rigtest GOB_export action frame count; every frame x "
             f"{N_BONES} bones world transform vs stage GOB_export <= {POS_TOL * 1000:g} mm and <= {ROT_TOL:g} deg",
     "frame count = round(max key time) - round(min key time) + 1 over all F-curves of the imported action; "
     "frames aligned by range start; every reimported bone use_connect False before the comparison "
     "(importer_connected_bones); world = matrix_world @ pose_bone.matrix (evaluated) per integer frame; "
     "rotation = 2 acos|q1.q2|"),
    ("G8.6", f"goblin_club.fbx: triangles < {CLUB_TRI_LIMIT}; mesh in the {SOCKET} rest frame: reimported club "
             f"vertices placed by the canonical {SOCKET} rest_matrix vs Blender rest club world vertices max <= "
             f"{CLUB_VERT_TOL * 1000:g} mm (covers origin = grip, +Y = handle axis, roll)",
     "X = rest_matrix(socket) @ matrix_world(imported club) @ mesh co; reference = gob_r05_rigtest.blend GOB_club "
     "world vertices with GOB_rig pose_position REST (in memory); matched by index when vertex counts are equal, "
     "else symmetric nearest vertex; triangles = sum(loop_total - 2); origin / +Y / roll from a rigid fit of the "
     "FBX vertices onto the reference vs pivots.json grip_r and the socket rest, and stage_rest goblin_club, "
     "reported only"),
    ("G8.7", f"goblin.fbx and every {CLIP_GLOB}: reimported {MESH_NAME} has shape key {MOUTH_KEY} (use_relative) "
             f"and exactly 4 materials named {list(MAT_NAMES)}; every clip: reimported {MOUTH_KEY} value x 100 vs "
             f"stage {STAGE_MESH} evaluated value x 100, every frame, |diff| <= {MOUTH_PCT_TOL:g}; >= 1 clip",
     "2026-09-26 HR2 (rig/data/hr2_contract.md). Same importer options as G8.2-G8.5; FBX value = evaluated "
     "key_blocks['mouth_open'].value of the reimported mesh (the DeformPercent curve / 100; no curve -> static "
     "value, a missing key reads 0 only when there is no curve), FBX frame g0+i vs stage frame s0+i (g0 = first "
     "armature key, anim_offset corrected), stage = rig/export/stage_<clip>.blend (rigtest: the G8.5 stage); "
     "material order vs the contract, faces per slot and the curve key counts reported only"),
)


def key_value(obj, dg, name=MOUTH_KEY):
    """Evaluated shape-key value `name` of mesh obj (None when absent)."""
    key = obj.data.shape_keys if obj is not None and obj.type == "MESH" else None
    if key is None or key.key_blocks.get(name) is None:
        return None
    try:
        return float(key.evaluated_get(dg).key_blocks[name].value)
    except Exception:
        return float(key.key_blocks[name].value)


def fbx_mesh():
    o = bpy.data.objects.get(MESH_NAME)
    if o is not None and o.type == "MESH":
        return o
    ms = [x for x in bpy.data.objects if x.type == "MESH"]
    return ms[0] if len(ms) == 1 else None


def stage_mouth(path):
    """Stage goblin_mesh evaluated mouth_open per frame of the GOB_export action range -> (frames, values, info)."""
    open_blend(path)
    exp = bpy.data.objects.get(STAGE_ARM)
    mesh = bpy.data.objects.get(STAGE_MESH)
    act = exp.animation_data.action if exp is not None and exp.animation_data else None
    if act is None:
        raise Blocked(f"{STAGE_ARM} action missing in {rel(path)}")
    if mesh is None or mesh.type != "MESH":
        raise Blocked(f"{STAGE_MESH} missing in {rel(path)}")
    f0, f1 = (int(round(x)) for x in act.frame_range)
    frames = list(range(f0, f1 + 1))
    sc = bpy.context.scene
    vals = []
    for f in frames:
        sc.frame_set(f)
        vals.append(key_value(mesh, bpy.context.evaluated_depsgraph_get()))
    key = mesh.data.shape_keys
    kad = key.animation_data if key is not None else None
    info = {"stage": rel(path), "frame_range": [f0, f1],
            "shape_keys": [k.name for k in key.key_blocks] if key is not None else None,
            "key_action": kad.action.name if kad is not None and kad.action is not None else None}
    return frames, vals, info


def fbx_contract(label):
    """Reimported goblin_mesh: shape key mouth_open (relative) + the 4 contract materials -> (record, ok)."""
    o = fbx_mesh()
    if o is None:
        return {"mesh": f"{MESH_NAME} missing (meshes: {[x.name for x in bpy.data.objects if x.type == 'MESH']})"}, False
    key = o.data.shape_keys
    names = [k.name for k in key.key_blocks] if key is not None else []
    kb = key.key_blocks.get(MOUTH_KEY) if key is not None else None
    mats = [s.material.name if s.material else None for s in o.material_slots]
    mi = np.empty(len(o.data.polygons), dtype=np.int64)
    o.data.polygons.foreach_get("material_index", mi)
    rec = {"mesh": o.name, "shape_keys": names, "use_relative": bool(key.use_relative) if key is not None else None,
           "mouth_relative_key": kb.relative_key.name if kb is not None and kb.relative_key else None,
           "materials": mats, "faces_per_material_slot": np.bincount(mi, minlength=len(mats)).tolist(),
           "materials_in_contract_order_report": mats == list(MAT_NAMES)}
    ok = (kb is not None and bool(key.use_relative) and len(mats) == len(MAT_NAMES)
          and sorted(m or "" for m in mats) == sorted(MAT_NAMES))
    return rec, bool(ok)


def fbx_mouth_curve(n_stage):
    """Reimported clip: mouth_open value per frame aligned by the armature action range start -> (values, info)."""
    arm, _note = find_arm()
    act = arm.animation_data.action if arm.animation_data else None
    if act is None:
        raise Blocked(f"reimported armature {arm.name} has no action")
    times = [k.co.x for fc in fcurves_of(act) for k in fc.keyframe_points]
    if not times:
        raise Blocked(f"action {act.name} has no keys")
    g0 = int(round(min(times)))
    o = fbx_mesh()
    key = o.data.shape_keys if o is not None else None
    kad = key.animation_data if key is not None else None
    kact = kad.action if kad is not None else None
    fcs = [fc for fc in fcurves_of(kact)] if kact is not None else []
    mfc = [fc for fc in fcs if MOUTH_KEY in fc.data_path]
    info = {"armature_key_start": g0, "key_action": kact.name if kact is not None else None,
            "key_action_fcurves": [fc.data_path for fc in fcs],
            "mouth_curve_keys": [len(fc.keyframe_points) for fc in mfc],
            "mouth_curve_present": bool(mfc)}
    sc = bpy.context.scene
    vals = []
    for i in range(n_stage):
        sc.frame_set(g0 + i)
        v = key_value(o, bpy.context.evaluated_depsgraph_get())
        vals.append(0.0 if v is None and not mfc else v)
    return vals, info


def c_g87(ctx, opts, ev):
    """G8.7: shape key + materials in goblin.fbx and every goblin@*.fbx; clip values vs stage (x100)."""
    out, ok = {}, True
    fresh_import(FBX_MAIN, opts)
    rec, o = fbx_contract(FBX_MAIN.name)
    out[FBX_MAIN.name] = rec
    ok = ok and o
    clips = sorted(goblib.EXPORT.glob(CLIP_GLOB))
    for p in clips:
        ev.add_input(p)
        clip = p.stem.split("@", 1)[1]
        stage = ctx.stage_rig if clip == ACTION else goblib.EXPORT / f"stage_{clip}.blend"
        entry = {"stage": rel(stage)}
        try:
            if not Path(stage).exists():
                raise Blocked(f"stage {rel(stage)} missing")
            ev.add_input(stage)
            frames, sv, sinfo = stage_mouth(stage)
            entry["stage_info"] = sinfo
            fresh_import(p, opts)
            rec, o = fbx_contract(p.name)
            entry.update(rec)
            fv, finfo = fbx_mouth_curve(len(frames))
            entry["fbx_curve"] = finfo
            if any(v is None for v in sv) or any(v is None for v in fv):
                entry["values"] = "mouth_open missing on the stage or the reimported mesh"
                o = False
            else:
                d = np.abs(np.array(fv) - np.array(sv)) * 100.0
                i = int(d.argmax()) if len(d) else 0
                entry.update({"n_frames": len(frames), "max_abs_diff_x100": rnd(d.max(), 6) if len(d) else None,
                              "worst_stage_frame": frames[i] if len(d) else None,
                              "stage_min_max": [rnd(min(sv), 6), rnd(max(sv), 6)],
                              "fbx_min_max": [rnd(min(fv), 6), rnd(max(fv), 6)],
                              "frames_stage_nonzero": [f for f, v in zip(frames, sv) if v != 0.0][:60]})
                o = o and len(d) > 0 and d.max() <= MOUTH_PCT_TOL
            entry["ok"] = bool(o)
        except Blocked as e:
            entry["blocked"] = str(e)
            entry["ok"] = False
            o = False
        out[p.name] = entry
        ok = ok and o
    out["clips_found"] = [p.name for p in clips]
    return out, bool(ok and clips)


class Ctx:
    def __init__(self):
        self.names = self.parent = self.canon_mat = self.length = None
        self.pivots = None
        self.stage_rest = STAGE_REST
        self.stage_rig = STAGE_RIG
        self.ref_mesh = None
        self.stage_club_mw = None
        self.stage_club_W = None
        self.stage_frames = self.stage_world = self.stage_info = None
        self.club_ref = None


def ensure_stages(ctx, ev):
    """Missing stage blends are built into TMP with s07b.build_export; returns build notes."""
    notes = {}
    need = [(STAGE_RIG, ACTION, "stage_rig"), (STAGE_REST, None, "stage_rest")]
    todo = [x for x in need if not x[0].exists()]
    for p, _a, _k in need:
        if p.exists():
            ev.add_input(p)
            notes[rel(p)] = "existing"
    if not todo:
        return notes
    if not S07B.exists():
        raise Blocked(f"stage blend(s) missing and {rel(S07B)} missing")
    s07b = importlib.import_module(S07B_NAME)
    TMP.mkdir(parents=True, exist_ok=True)
    for p, act, key in todo:
        out = TMP / p.name
        try:
            res = s07b.build_export(WORK_BLEND, act, out)
        except Exception:
            raise Blocked(f"build_export({act!r}) raised: {tb_tail()}")
        res = Path(res) if res else out
        if not res.exists():
            raise Blocked(f"build_export({act!r}) wrote no file")
        setattr(ctx, key, res)
        ev.add_input(res)
        notes[rel(p)] = f"missing -> built {rel(res)} (temporary)"
    return notes


def main():
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(__file__)
    ctx = Ctx()
    th = {i: t for i, t, _n in IDS}
    note = {i: n for i, _t, n in IDS}
    done = set()

    def put(cid, measured, ok):
        ev.criterion(cid, measured, th[cid], ok, note[cid])
        done.add(cid)

    def block(cids, reason):
        for cid in cids:
            if cid not in done:
                put(cid, f"blocked: {reason}", False)

    all_ids = [i for i, _t, _n in IDS]
    miss = []
    for p in (FBX_MAIN, FBX_RIG, FBX_CLUB, PRESET_JSON, CANON_JSON, PIVOTS_JSON, WORK_BLEND):
        if p.exists():
            ev.add_input(p)
        else:
            miss.append(rel(p))
    ev.add_stage_inputs("s08")
    _STATE["missing"] = miss

    try:
        preset = read_preset() if PRESET_JSON.exists() else None
        put("G8.1", *c_g81(preset))
        if miss:
            block(all_ids, "missing input: " + ", ".join(miss))
            return
        opts, opt_notes = import_options(preset)
        imp = {"options": opts, "notes": opt_notes,
               "axis_correction": "importer inverse mapping (use_manual_orientation, preset axes)"}
        ctx.names, ctx.parent, ctx.canon_mat, ctx.length = load_canon()
        with open(PIVOTS_JSON, "r", encoding="utf-8") as f:
            ctx.pivots = json.load(f)["pivots"]

        try:
            stage_notes = ensure_stages(ctx, ev)
        except Blocked as e:
            block(["G8.4", "G8.5"], str(e))
            stage_notes = None
        if stage_notes is not None:
            try:
                stage_rest_ref(ctx)
            except Blocked as e:
                block(["G8.4"], f"stage_rest: {e}")
            try:
                stage_rig_ref(ctx)
            except Blocked as e:
                block(["G8.5"], f"stage_rigtest: {e}")
        try:
            work_club_ref(ctx)
        except Blocked as e:
            block(["G8.6"], f"work club: {e}")

        # goblin.fbx: G8.2, G8.3 (main), G8.4
        g83 = {}
        ok83 = True
        try:
            fresh_import(FBX_MAIN, opts)
            m2, o2 = c_g82_main()
            put("G8.2", {"import": imp, **m2}, o2)
            g83[rel(FBX_MAIN)], o, _conn = skeleton_disconnected(ctx)
            ok83 = ok83 and o
            if "G8.4" not in done:
                m4, o4 = c_g84(ctx)
                m4["stage_refs"] = stage_notes
                put("G8.4", m4, o4)
        except Blocked as e:
            block(["G8.2", "G8.4"], str(e))
            g83[rel(FBX_MAIN)] = f"blocked: {e}"
            ok83 = False

        # goblin@rigtest.fbx: G8.3 (rigtest), G8.5
        try:
            fresh_import(FBX_RIG, opts)
            inv_rig = inventory()
            g83[rel(FBX_RIG)], o, conn_rig = skeleton_disconnected(ctx)
            ok83 = ok83 and o
            if "G8.5" not in done:
                m, o5 = c_g85(ctx)
                m["importer_connected_bones"] = conn_rig
                m["objects_report"] = inv_rig
                m["stage_refs"] = stage_notes
                put("G8.5", m, o5)
        except Blocked as e:
            block(["G8.5"], str(e))
            g83.setdefault(rel(FBX_RIG), f"blocked: {e}")
            ok83 = False
        put("G8.3", {"import": imp, **g83}, ok83 and len(g83) == 2)

        # goblin_club.fbx: G8.6
        if "G8.6" not in done:
            try:
                fresh_import(FBX_CLUB, opts)
                put("G8.6", *c_g86(ctx))
            except Blocked as e:
                block(["G8.6"], str(e))

        # HR2 blend shape + materials: G8.7
        try:
            put("G8.7", *c_g87(ctx, opts, ev))
        except Blocked as e:
            block(["G8.7"], str(e))
    finally:
        try:
            block(all_ids, "checker aborted before this criterion")
        finally:
            ev.write()
            if TMP.exists():
                shutil.rmtree(TMP, ignore_errors=True)


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
