"""check_p2_export - Player P2.6 bake / FBX / Unity checker (task T251; spec work/player/d-02-player-rig.md section 2
gate P2.6 and section 5 P2d plan; checker side).

Inputs (defaults; override after "--"):
  --work       work/player/rig/pl_r05_rigtest.blend        work rig PL_rig + action rigtest (--work-arm, --action)
  --stage-rest work/player/rig/export/stage_rest.blend     DEF-only armature Player (41 bones) + Player_mesh
  --stage-rig  work/player/rig/export/stage_rigtest.blend  same, baked action (--stage-arm, --stage-mesh)
  --fbx-rest   work/player/rig/export/Player.fbx
  --fbx-clip   work/player/rig/export/Player@rigtest.fbx
  --preset     work/player/rig/data/export_preset.json     player-owned (blender_reimport.import_scene_fbx options,
                                                           axis_mapping.blender_to_unity, export_scene_fbx)
  --canon      work/player/rig/data/canonical_skeleton.json
  --report     unity/AvatarCheck/player_report.json        contract of the T251 task (clips.<clip>.frames[].bones_world,
                                                           rest, bounds, spring, penetration); --clip rigtest
  --out        work/player/inspect/P2/check_p2_export.json
Methods (copied / reused, goblin files not imported or modified):
  goblin check_g7_bake.py   stage structure (DEF-only, 41 bones, unconnected, no constraints, rest vs canonical 1e-5,
                            rigtest stage rest vs rest stage 1e-6), work vs stage per-frame world matrices
  goblin check_g8_fbx.py    preset expectations (axis_forward -Z, axis_up Y, add_leaf_bones False, bake_anim_step 1,
                            simplify 0, force start/end keying), reimport with the preset options into an empty
                            factory scene, importer auto-connect undone, no *_end leaves
  check_p1.py (player)      imported: float64 rotation error rang(), dist(), tr(), sample_world(), assign_action(),
                            fresh_import(), find_arm(), disconnect_bones(), p13_import / p13_bones / p13_pose (Unity:
                            p_u vs M p_b; rest-relative dR = R R_rest^T, error = angle(dR_u, M dR_b M^T)); frame
                            alignment by range start (anim offset)
  goblin check_a3_clip.py / compare_g9.py   bounds criterion (0 frames outside); here measured independently
Spring metric (d-02 section 5): tip = tail of Skirt_*_02; sway = |spring tip - raw tip| (Unity world).  Cross-check:
  when a frame carries both a spring and a raw skirt block (skirt_spring / skirt_raw, or bones_world + skirt_raw), tail =
  p_u + dR_u (M R_rest_b (0, L, 0)), dR_u = R_u R_rest_u^T, L and R_rest_b from the rest stage.
Rows P2.6a..e as T251, P2.8a (T254: bindposes blocking <= 1e-4, local-rotation pitch margin report); every draft / report row: ok = True with note "calibration run: threshold set after measurement".
Nothing is saved: blends are opened read-only in memory; FBX files are imported into empty factory scenes.
Run:    blender --background --factory-startup --python check_p2_export.py [-- <overrides>]
Exit:   0 evidence written; 2 script error or missing input (evidence still written with blocked rows).
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
import check_p1 as p1  # noqa: E402  (player P1 checker: generic reimport / Unity helpers)
import check_p2_deform as p2d  # noqa: E402  (evidence / json helpers)

PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
GATE, CHECKER, TASK = "P2.6", "check_p2_export", "T251 + T254 (P2.8a)"
CALIB = p2d.CALIB
rel, jsonable, mm, rnd = p2d.rel, p2d.jsonable, p2d.mm, p2d.rnd

P = {"work": PLAYER_RIG / "pl_r05_rigtest.blend",
     "stage_rest": PLAYER_RIG / "export" / "stage_rest.blend",
     "stage_rig": PLAYER_RIG / "export" / "stage_rigtest.blend",
     "fbx_rest": PLAYER_RIG / "export" / "Player.fbx",
     "fbx_clip": PLAYER_RIG / "export" / "Player@rigtest.fbx",
     "preset": PLAYER_RIG / "data" / "export_preset.json",
     "canon": PLAYER_RIG / "data" / "canonical_skeleton.json",
     "report": REPO / "unity" / "AvatarCheck" / "player_report.json",
     "out": REPO / "work" / "player" / "inspect" / "P2" / "check_p2_export.json",
     "action": "rigtest", "clip": "rigtest", "work_arm": "PL_rig", "stage_arm": "Player", "stage_mesh": "Player_mesh"}
PATH_KEYS = ("work", "stage_rest", "stage_rig", "fbx_rest", "fbx_clip", "preset", "canon", "report", "out")

POS_TOL = 1e-5              # m (0.01 mm), d-01 section 6 / d-02 P2.6
ROT_TOL = 0.01              # deg
STAGE_REST_TOL = 1e-5       # goblin G7.2 stage rest vs canonical (max abs element)
SAME_REST_TOL = 1e-6        # goblin G7.3 rigtest stage rest vs rest stage
N_MATERIALS = 20            # d-02 section 5: Player_mesh with 20 material slots
M_SPEC = [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]   # d-02 section 5 axis mapping
EXPECT = (("axis_forward", "-Z"), ("axis_up", "Y"), ("add_leaf_bones", False), ("bake_anim_step", 1.0),
          ("bake_anim_simplify_factor", 0.0), ("bake_anim_force_startend_keying", True))   # goblin G8.1
BINDPOSE_TOL = 1e-4        # P2.8a (d-02 section 8)
PITCH_MARGIN_DRAFT = 30.0   # P2.8a draft (d-02 section 8, italic) -> calibration
NON_DEFORM = ("Root", "HeadEquipmentSocket", "WeaponSocket_L", "WeaponSocket_R", "BackSocket", "BackWeaponSocket")


class Blocked(Exception):
    pass


def tb_tail(n=3):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


# ---------------------------------------------------------------- context
class Ctx:
    def __init__(self):
        self.canon = self.preset = self.rep = None
        self.names = self.parent = None
        self.M = Matrix(M_SPEC)
        self.opts = None
        self.work = None          # {"frames", "W"}
        self.stage = {}           # "rest" / "rig": structure records
        self.stage_frames = self.stage_W = None
        self.stage_rest = None    # {name: 4x4 np} rest stage world rest
        self.stage_len = {}       # bone length (rest stage)
        self.local_euler = None   # P2.8a {bone: (euler XYZ deg, parent)} of the rest stage
        self.mesh_aabb = None     # per stage frame [(min_u, max_u)] in Unity space
        self.stage_info = {}


def load_canon(ctx):
    bones = ctx.canon.get("bones") if isinstance(ctx.canon, dict) else None
    if not isinstance(bones, list) or not bones:
        raise Blocked("canonical_skeleton.json has no 'bones' list")
    ctx.names = [b["name"] for b in bones]
    if set(ctx.names) == set(p1.NAMES):
        ctx.names = list(p1.NAMES)   # contract order: check_p1.p13_pose indexes stage_W by p1.NAMES
    ctx.parent = {b["name"]: b.get("parent") for b in bones}
    ctx.canon_rest = {b["name"]: np.array(b["rest_matrix"], dtype=np.float64) for b in bones if "rest_matrix" in b}


def pick_arm(name):
    a = bpy.data.objects.get(name)
    if a is not None and a.type == "ARMATURE":
        return a
    arms = [o for o in bpy.data.objects if o.type == "ARMATURE"]
    return arms[0] if len(arms) == 1 else None


def action_frames(act):
    f0, f1 = (int(round(x)) for x in act.frame_range)
    return list(range(f0, f1 + 1))


# ---------------------------------------------------------------- work
def work_phase(ctx):
    p1.open_blend(P["work"])
    arm = pick_arm(P["work_arm"])
    if arm is None:
        raise Blocked(f"work armature {P['work_arm']} not found in {rel(P['work'])}")
    act = bpy.data.actions.get(P["action"])
    if act is None:
        raise Blocked(f"action {P['action']!r} not in {rel(P['work'])} ({[a.name for a in bpy.data.actions]})")
    slot = p1.assign_action(arm, act)
    miss = [n for n in ctx.names if n not in arm.pose.bones]
    if miss:
        raise Blocked(f"work armature lacks {miss}")
    fr = action_frames(act)
    bpy.context.scene.tool_settings.use_keyframe_insert_auto = False
    ctx.work = {"frames": fr, "W": p1.sample_world(arm, fr, ctx.names), "armature": arm.name, "action": act.name,
                "slot": slot, "fps": p1.fps_of(bpy.context.scene)}


# ---------------------------------------------------------------- stages
def stage_struct(ctx, kind, path):
    p1.open_blend(path)
    arm = pick_arm(P["stage_arm"])
    if arm is None:
        raise Blocked(f"stage armature {P['stage_arm']} not found in {rel(path)}")
    ab = arm.data.bones
    names = [b.name for b in ab]
    mw = arm.matrix_world
    rest = {n: np.array(mw @ ab[n].matrix_local, dtype=np.float64) for n in names}
    rest_dev = {n: float(np.abs(rest[n] - ctx.canon_rest[n]).max()) for n in names if n in ctx.canon_rest}
    over = {n: float(f"{v:.3e}") for n, v in rest_dev.items() if v > STAGE_REST_TOL}
    par = {n: (ab[n].parent.name if ab[n].parent else None) for n in names}
    rec = {"blend": rel(path), "armature": arm.name, "n_bones": len(names),
           "only_canonical": sorted(set(ctx.names) - set(names)), "only_stage": sorted(set(names) - set(ctx.names)),
           "parent_mismatch": {n: {"canonical": ctx.parent[n], "stage": par[n]} for n in names
                               if n in ctx.parent and par[n] != ctx.parent[n]},
           "connected": sorted(b.name for b in ab if b.use_connect),
           "pose_constraints": sum(len(pb.constraints) for pb in arm.pose.bones),
           "deform_mismatch": sorted(n for n in names if n in ctx.parent and ab[n].use_deform != (n not in NON_DEFORM)),
           "rest_vs_canonical_max_abs": float(f"{max(rest_dev.values(), default=0.0):.3e}"),
           "rest_vs_canonical_over_tol": over,
           "armature_matrix_world_dev": float(f"{float(np.abs(np.array(mw) - np.eye(4)).max()):.3e}"),
           "objects": sorted(f"{o.name}({o.type})" for o in bpy.data.objects)}
    mo = bpy.data.objects.get(P["stage_mesh"])
    if mo is None or mo.type != "MESH":
        rec["mesh"] = f"missing {P['stage_mesh']}"
        mok = False
    else:
        mods = [(m.type, m.object.name if getattr(m, "object", None) else None) for m in mo.modifiers]
        rec["mesh"] = {"name": mo.name, "material_slots": len(mo.material_slots), "modifiers": mods,
                       "uv_layers": len(mo.data.uv_layers), "vertex_groups": len(mo.vertex_groups)}
        mok = len(mo.material_slots) == N_MATERIALS and ("ARMATURE", arm.name) in mods
    ok = (len(names) == len(ctx.names) and not rec["only_canonical"] and not rec["only_stage"]
          and not rec["parent_mismatch"] and not rec["connected"] and rec["pose_constraints"] == 0
          and not rec["deform_mismatch"] and not over and mok)
    if kind == "rest":
        ctx.stage_rest = rest
        ctx.stage_len = {n: float(ab[n].length) for n in names}
        ctx.local_euler = {}
        for n in names:
            b = ab[n]
            R = (b.parent.matrix_local.inverted() @ b.matrix_local) if b.parent else (mw @ b.matrix_local)
            R3 = R.to_3x3().normalized()
            e = R3.to_euler("XYZ")
            # branch-free pitch: Euler XYZ (R = Rz Ry Rx) has sin(Y) = -R[2][0]; margin = 90 - asin(|R[2][0]|)
            pitch = math.degrees(math.asin(min(1.0, abs(float(R3[2][0])))))
            ctx.local_euler[n] = ([math.degrees(a) for a in e], b.parent.name if b.parent else None, pitch)
    else:
        rs = ctx.stage_rest or {}
        d = max((float(np.abs(rest[n] - rs[n]).max()) for n in names if n in rs), default=None)
        rec["rest_vs_rest_stage_max_abs"] = float(f"{d:.3e}") if d is not None else "rest stage unavailable"
        ok = ok and d is not None and d <= SAME_REST_TOL
        ad = arm.animation_data
        act = ad.action if ad and ad.action else bpy.data.actions.get(P["action"])
        if act is None:
            raise Blocked(f"stage {rel(path)}: no action")
        assigned = None
        if not (ad and ad.action):
            assigned = p1.assign_action(arm, act)
        fr = action_frames(act)
        ctx.stage_frames = fr
        ctx.stage_W = p1.sample_world(arm, fr, ctx.names)
        ctx.stage_info = {"source": rel(path), "armature": arm.name, "action": act.name, "assigned_in_memory": assigned,
                          "frame_range": [fr[0], fr[-1]], "fps": p1.fps_of(bpy.context.scene)}
        rec["action"] = ctx.stage_info
        # mesh AABB per frame in Unity space (P2.6d)
        if mo is not None and mo.type == "MESH":
            M = np.array(ctx.M, dtype=np.float64)
            aabb = []
            sc = bpy.context.scene
            for f in fr:
                sc.frame_set(f)
                dg = bpy.context.evaluated_depsgraph_get()
                oe = mo.evaluated_get(dg)
                me = oe.to_mesh()
                try:
                    co = np.empty(len(me.vertices) * 3)
                    me.vertices.foreach_get("co", co)
                finally:
                    oe.to_mesh_clear()
                mwm = np.array(oe.matrix_world, dtype=np.float64)
                V = co.reshape(-1, 3) @ mwm[:3, :3].T + mwm[:3, 3]
                U = V @ M.T
                aabb.append((U.min(axis=0), U.max(axis=0)))
            ctx.mesh_aabb = aabb
    ctx.stage[kind] = rec
    return rec, ok


def frame_pairs(fa, fb):
    """align two frame lists by range start (anim offset corrected); -> (list of (ia, ib), offset fb0 - fa0)."""
    n = min(len(fa), len(fb))
    return [(i, i) for i in range(n)], fb[0] - fa[0]


def compare_WW(A, fa, B, fb, names):
    pairs, off = frame_pairs(fa, fb)
    Pm = np.zeros((len(pairs), len(names)))
    Rd = np.zeros((len(pairs), len(names)))
    for k, (i, j) in enumerate(pairs):
        for b in range(len(names)):
            Pm[k, b], Rd[k, b] = p1.pos_rot_err(A[i, b], B[j, b])
    keys = [[fa[i], names[b]] for i, _j in pairs for b in range(len(names))]
    per = {n: [mm(Pm[:, b].max()), rnd(Rd[:, b].max(), 5)] for b, n in enumerate(names)} if len(pairs) else {}
    return {"n_frames_a": len(fa), "n_frames_b": len(fb), "frame_count_equal": len(fa) == len(fb),
            "n_frames_compared": len(pairs), "frame_offset_b_minus_a": off,
            "pos_mm": p1.dist(Pm.ravel(), 1000.0, keys=keys), "rot_deg": p1.dist(Rd.ravel(), 1.0, 5, keys=keys),
            "per_bone_max_[pos_mm,rot_deg]": per}, (len(pairs) > 0 and float(Pm.max()) <= POS_TOL
                                                    and float(Rd.max()) <= ROT_TOL)


# ---------------------------------------------------------------- FBX
def preset_check(ctx):
    pr = ctx.preset
    exp = pr.get("export_scene_fbx") if isinstance(pr, dict) else None
    rec, ok = {}, isinstance(exp, dict)
    if isinstance(exp, dict):
        for k, v in EXPECT:
            got = exp.get(k)
            good = got == v if not isinstance(v, float) else isinstance(got, (int, float)) and abs(got - v) <= 1e-12
            rec[k] = {"expected": v, "found": got, "ok": good}
            ok = ok and good
    am = (pr.get("axis_mapping") or {}).get("blender_to_unity") if isinstance(pr, dict) else None
    amok = am is not None and float(np.abs(np.array(am, dtype=np.float64) - np.array(M_SPEC)).max()) <= 1e-9
    if amok:
        ctx.M = Matrix(am)
    ri = ((pr.get("blender_reimport") or {}).get("import_scene_fbx")) if isinstance(pr, dict) else None
    if isinstance(ri, dict) and ri:
        ctx.opts = dict(ri)
    return {"export_scene_fbx_expect": rec, "axis_mapping_blender_to_unity": am, "axis_mapping_equals_spec": amok,
            "reimport_options": ctx.opts, "files": sorted((pr.get("files") or {}).keys()) if isinstance(pr, dict) else None
            }, ok and amok and ctx.opts is not None


def fbx_struct(ctx, path, clip):
    p1.fresh_import(path, ctx.opts)
    arm = p1.find_arm()
    names = [b.name for b in arm.data.bones]
    par = {b.name: (b.parent.name if b.parent else None) for b in arm.data.bones}
    ends = sorted(n for n in names if n.lower().endswith("_end"))
    conn = p1.disconnect_bones(arm)
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    mrec = []
    for o in meshes:
        me = o.data
        mats = [s.material.name for s in o.material_slots if s.material]
        mrec.append({"name": o.name, "material_slots": len(o.material_slots), "unique_materials": len(set(mats)),
                     "uv_layers": len(me.uv_layers), "has_custom_normals": bool(getattr(me, "has_custom_normals", False)),
                     "verts": len(me.vertices), "armature_modifiers": [m.object.name if m.object else None
                                                                      for m in o.modifiers if m.type == "ARMATURE"]})
    rec = {"file": rel(path), "armature": arm.name, "n_bones": len(names),
           "only_canonical": sorted(set(ctx.names) - set(names)), "only_fbx": sorted(set(names) - set(ctx.names)),
           "parent_mismatch": {n: {"canonical": ctx.parent[n], "fbx": par[n]} for n in names
                               if n in ctx.parent and par[n] != ctx.parent[n]},
           "end_leaf_bones": ends, "importer_connected": conn, "meshes": mrec,
           "objects": sorted(f"{o.name}({o.type})" for o in bpy.data.objects),
           "actions": [a.name for a in bpy.data.actions]}
    mesh_ok = (len(meshes) == 1 and mrec[0]["material_slots"] == N_MATERIALS and mrec[0]["uv_layers"] > 0
               and mrec[0]["has_custom_normals"])
    ok = (len(names) == len(ctx.names) and not rec["only_canonical"] and not rec["only_fbx"]
          and not rec["parent_mismatch"] and not ends and conn["undone"] and mesh_ok)
    if ctx.stage_rest is not None:
        pe, re_ = {}, {}
        for n in ctx.names:
            if n in arm.data.bones:
                W = np.array(arm.matrix_world @ arm.data.bones[n].matrix_local, dtype=np.float64)
                pe[n], re_[n] = p1.pos_rot_err(W, ctx.stage_rest[n])
        rec["rest_vs_rest_stage"] = {"pos_mm": p1.dist(list(pe.values()), 1000.0, keys=list(pe)),
                                     "rot_deg": p1.dist(list(re_.values()), 1.0, 5, keys=list(re_))}
        if not clip:
            ok = ok and bool(pe) and max(pe.values()) <= POS_TOL and max(re_.values()) <= ROT_TOL
    return rec, ok, arm


def fbx_frames(ctx, arm):
    act = arm.animation_data.action if arm.animation_data else None
    if act is None:
        raise Blocked(f"reimported armature {arm.name} has no action")
    times = [k.co.x for fc in p1.fcurves_of(act) for k in fc.keyframe_points]
    if not times:
        raise Blocked(f"action {act.name} has no keys")
    g0, g1 = int(round(min(times))), int(round(max(times)))
    fr = list(range(g0, g1 + 1))
    F = p1.sample_world(arm, fr, ctx.names)
    rec, ok = compare_WW(F, fr, ctx.stage_W, ctx.stage_frames, ctx.names)
    rec.update({"action": act.name, "key_time_range": [rnd(min(times), 4), rnd(max(times), 4)],
                "keys_not_on_integer_frame": sum(1 for t in times if abs(t - round(t)) > 1e-4),
                "alignment": "fbx frame g0+i vs stage frame s0+i (1-frame anim offset corrected)"})
    return rec, ok


# ---------------------------------------------------------------- Unity
class UCtx:
    """attribute bag for check_p1.p13_pose (stage_frames, stage_W, stage_rest, stage_info, M)."""


def unity_ctx(ctx):
    u = UCtx()
    u.stage_frames, u.stage_W, u.stage_rest = ctx.stage_frames, ctx.stage_W, ctx.stage_rest
    u.stage_info, u.M = {"source": ctx.stage_info.get("source")}, ctx.M
    return u


def clip_block(ctx):
    clips = ctx.rep.get("clips")
    if not isinstance(clips, dict) or P["clip"] not in clips:
        raise Blocked(f"report has no clips.{P['clip']} (clips {sorted(clips) if isinstance(clips, dict) else clips})")
    return clips[P["clip"]]


def u_import(ctx):
    rep = ctx.rep
    imp, _ok_p1 = p1.p13_import(rep)
    # T255: compression is per clip (import.clips[*].compression); animationType at the top level
    ib = rep.get("import") if isinstance(rep.get("import"), dict) else {}
    at = str(ib.get("animationType", "")).replace(" ", "").lower()
    iclips = ib.get("clips") if isinstance(ib.get("clips"), dict) else {}
    comp = {k: (v.get("compression") if isinstance(v, dict) else None) for k, v in iclips.items()}
    ok_imp = at in ("generic", "2") and bool(comp) and all(str(c).replace(" ", "").lower() in ("off", "0")
                                                           for c in comp.values())
    imp["compression_per_clip"] = comp
    imp["rule_T255"] = "top-level animationType Generic; every import.clips[*].compression Off; >= 1 clip"
    bones, ok_b = p1.p13_bones(rep)
    er = rep.get("errors")
    bp = rep.get("bindposes_ok")
    rest = rep.get("rest")
    rest_ok = isinstance(rest, dict) and all(p1.tr(rest.get(n)) is not None for n in ctx.names)
    rec = {"import": imp, "bones": bones, "n_errors": len(er) if isinstance(er, list) else f"missing ({er!r})",
           "errors": (er or [])[:20] if isinstance(er, list) else None, "bindposes_ok": bp,
           "rest_block": {"present": isinstance(rest, dict), "n": len(rest) if isinstance(rest, dict) else 0,
                          "missing": [n for n in ctx.names if not isinstance(rest, dict) or p1.tr(rest.get(n)) is None][:20]},
           "unity_version": rep.get("unity_version"), "timing_s_report": rep.get("timing_s"),
           "renders_report": rep.get("renders"), "clips_in_report": sorted((rep.get("clips") or {}).keys())}
    ok = (ok_imp and ok_b and bones.get("n_report_bones") == len(ctx.names) and isinstance(er, list) and not er
          and bp is True and rest_ok)
    return rec, ok


def u_pose(ctx, key, names):
    cb = clip_block(ctx)
    pseudo = {"frames": cb.get("frames"), "rest": ctx.rep.get("rest")}
    return p1.p13_pose(unity_ctx(ctx), pseudo, key, names)


def u_ok(m):
    return (isinstance(m, dict) and m["pos_mm"].get("n", 0) > 0 and m["pos_mm"]["max"] <= POS_TOL * 1000
            and m["rot_deg"]["max"] <= ROT_TOL and not m["missing_bones"])


# ---------------------------------------------------------------- bounds
def c_bounds(ctx):
    b = ctx.rep.get("bounds")
    if not isinstance(b, dict):
        raise Blocked("report has no 'bounds' object")
    try:
        c = np.array(b["center"], dtype=np.float64)
        e = np.array(b["extents"], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        raise Blocked(f"bounds center / extents unreadable: {b}")
    lo, hi = c - e, c + e

    def outside(mn, mx):
        return float(max(0.0, *(lo - mn), *(mx - hi)))

    mesh = {"n_frames": 0}
    if ctx.mesh_aabb:
        o = [outside(mn, mx) for mn, mx in ctx.mesh_aabb]
        bad = [ctx.stage_frames[i] for i, v in enumerate(o) if v > 0]
        mesh = {"n_frames": len(o), "n_frames_outside": len(bad), "frames_outside_first": bad[:20],
                "max_outside_mm": mm(max(o)), "clip": P["action"],
                "union_aabb_u": [[rnd(x, 5) for x in np.min([a for a, _b in ctx.mesh_aabb], axis=0)],
                                 [rnd(x, 5) for x in np.max([b_ for _a, b_ in ctx.mesh_aabb], axis=0)]]}
    bones = {}
    for cn, cb in (ctx.rep.get("clips") or {}).items():
        n_out, n_fr, mx = 0, 0, 0.0
        for fr in cb.get("frames") or []:
            bw = fr.get("bones_world") if isinstance(fr, dict) else None
            if not isinstance(bw, dict):
                continue
            n_fr += 1
            pts = [p1.tr(v)[0] for v in bw.values() if p1.tr(v) is not None]
            if not pts:
                continue
            A = np.array([list(p) for p in pts])
            v = outside(A.min(axis=0), A.max(axis=0))
            n_out += int(v > 0)
            mx = max(mx, v)
        bones[cn] = {"n_frames": n_fr, "n_frames_bones_outside": n_out, "max_outside_mm": mm(mx)}
    ok = (b.get("applied") is True and mesh.get("n_frames", 0) > 0 and mesh.get("n_frames_outside", 1) == 0
          and all(v["n_frames_bones_outside"] == 0 for v in bones.values()) and bool(bones))
    return {"bounds": b, "stage_mesh_per_frame": mesh, "report_bones_per_clip": bones,
            "space_note": "center / extents read as Unity world (character root at the origin, root locked); stage "
                          "mesh vertices mapped by the preset axis mapping"}, ok


# ---------------------------------------------------------------- springs
def tails(ctx, blk, rest):
    """Skirt_*_02 tail world positions (Unity) from a bone block: p_u + dR_u (M R_rest_b (0, L, 0))."""
    out = {}
    M = np.array(ctx.M, dtype=np.float64)
    for n in ctx.names:
        if not (n.startswith("Skirt_") and n.endswith("_02")):
            continue
        t, tr_ = p1.tr((blk or {}).get(n)), p1.tr((rest or {}).get(n))
        if t is None or tr_ is None or n not in ctx.stage_rest:
            continue
        Rb = ctx.stage_rest[n][:3, :3]
        Rb = Rb / np.linalg.norm(Rb, axis=0)
        o_u = M @ (Rb @ np.array([0.0, ctx.stage_len.get(n, 0.0), 0.0]))
        Ru = np.array(t[1], dtype=np.float64)
        Rr = np.array(tr_[1], dtype=np.float64)
        out[n] = np.array(t[0], dtype=np.float64) + (Ru @ Rr.T) @ o_u
    return out


def c_springs(ctx):
    cb = clip_block(ctx)
    sp = cb.get("spring")
    if not isinstance(sp, dict):
        raise Blocked(f"clips.{P['clip']} has no 'spring' object")
    chains = [c for c in sp.get("chains") or [] if isinstance(c, dict)]
    rows = {}
    for c in chains:
        v = [float(x) for x in c.get("tip_sway_mm_per_frame") or [] if isinstance(x, (int, float))]
        rows[str(c.get("name"))] = {"max_sway_mm_report": c.get("max_sway_mm"), "per_frame_mm": p1.dist(v, 1.0, 3),
                                    "max_of_per_frame_equals_max": bool(v) and isinstance(c.get("max_sway_mm"), (int, float))
                                    and abs(max(v) - c["max_sway_mm"]) <= 1e-6, "n_frames": len(v)}
    # independent cross-check
    frames = cb.get("frames") or []
    rest = ctx.rep.get("rest")
    xc = "not enough data: frames carry no separate spring and raw skirt blocks"
    f0 = frames[0] if frames and isinstance(frames[0], dict) else {}
    spring_key = "skirt_spring" if "skirt_spring" in f0 else ("bones_world" if "skirt_raw" in f0 else None)
    if spring_key and "skirt_raw" in f0 and isinstance(rest, dict) and ctx.stage_rest:
        per = {}
        for fr in frames:
            ts, tr_ = tails(ctx, fr.get(spring_key), rest), tails(ctx, fr.get("skirt_raw"), rest)
            for n in ts:
                if n in tr_:
                    per.setdefault(n, []).append(float(np.linalg.norm(ts[n] - tr_[n])) * 1000.0)
        cmp_ = {}
        for c in chains:
            nm = str(c.get("name"))
            bone = next((b for b in (c.get("bones") or []) if str(b).endswith("_02")), f"{nm}_02")
            mine, theirs = per.get(bone), [float(x) for x in c.get("tip_sway_mm_per_frame") or []]
            if mine and theirs:
                n = min(len(mine), len(theirs))
                cmp_[nm] = {"bone": bone, "checker_max_mm": rnd(max(mine), 4), "report_max_mm": rnd(max(theirs), 4),
                            "max_abs_diff_mm": rnd(max(abs(a - b) for a, b in zip(mine[:n], theirs[:n])), 4),
                            "n_frames": n, "len_equal": len(mine) == len(theirs)}
            else:
                cmp_[nm] = {"bone": bone, "checker_frames": len(mine or []), "report_frames": len(theirs)}
        xc = {"spring_block": spring_key, "raw_block": "skirt_raw", "per_chain": cmp_,
              "rule": "tail = p_u + (R_u R_rest_u^T) M R_rest_b (0, L, 0); L, R_rest_b from the rest stage"}
    return {"params_report": sp.get("params"), "colliders_report": sp.get("colliders"), "n_chains": len(chains),
            "chains": rows, "crosscheck": xc}, True


def c_penetration(ctx):
    cb = clip_block(ctx)
    pen = cb.get("penetration")
    if not isinstance(pen, dict) or not isinstance(pen.get("per_frame"), list):
        raise Blocked(f"clips.{P['clip']} has no 'penetration.per_frame'")
    rows = [r for r in pen["per_frame"] if isinstance(r, dict)]
    on = [float(r.get("spring_on", 0)) for r in rows]
    off = [float(r.get("spring_off", 0)) for r in rows]
    dep = [float(r["max_depth_mm_on"]) for r in rows if isinstance(r.get("max_depth_mm_on"), (int, float))]
    fr = [r.get("f") for r in rows]
    return {"rule": pen.get("rule"), "n_frames": len(rows), "spring_on": p1.dist(on, 1.0, 2, keys=fr),
            "spring_off": p1.dist(off, 1.0, 2, keys=fr), "n_frames_on_gt_0": sum(1 for x in on if x > 0),
            "n_frames_off_gt_0": sum(1 for x in off if x > 0), "max_depth_mm_on": p1.dist(dep, 1.0, 3)}, True


# ---------------------------------------------------------------- P2.8a
def c_bindpose_margin(ctx):
    rep = ctx.rep
    bp = rep.get("bindposes_ok")
    sm = ((rep.get("extra") or {}).get("skinned_mesh") or {})
    mx = sm.get("bindpose_max_abs_diff_vs_rest", rep.get("bindpose_max_abs_diff"))
    bp_ok = bp is True and isinstance(mx, (int, float)) and mx <= BINDPOSE_TOL
    per = sm.get("bindpose_diff_per_bone") if isinstance(sm.get("bindpose_diff_per_bone"), dict) else {}
    over = {k: v for k, v in per.items() if isinstance(v, (int, float)) and v > BINDPOSE_TOL}
    margin = None
    if ctx.local_euler:
        rows = {n: {"euler_xyz_deg (Blender branch)": [rnd(a, 3) for a in e], "parent": par,
                    "abs_pitch_deg": rnd(pt, 3), "pitch_margin_deg": rnd(90.0 - pt, 3)}
                for n, (e, par, pt) in ctx.local_euler.items()}
        mins = sorted(rows.items(), key=lambda kv: kv[1]["pitch_margin_deg"])
        margin = {"min_deg": mins[0][1]["pitch_margin_deg"], "min_bone": mins[0][0],
                  "lowest_10": [[k, v["pitch_margin_deg"]] for k, v in mins[:10]],
                  "n_below_draft": sum(1 for _k, v in rows.items() if v["pitch_margin_deg"] < PITCH_MARGIN_DRAFT),
                  "draft_deg": PITCH_MARGIN_DRAFT, "per_bone": rows, "stage": ctx.stage.get("rest", {}).get("blend")
                  if isinstance(ctx.stage.get("rest"), dict) else None}
    return {"bindposes_ok": bp, "bindpose_max_abs_diff": mx, "bindpose_worst_bone": sm.get("bindpose_worst_bone"),
            "bones_over_tol": over, "tol": BINDPOSE_TOL, "pitch_margin": margin or "rest stage not read"}, bp_ok


# ---------------------------------------------------------------- rows
IDS = (
    ("P2.6a1", f"stages (rest + rigtest): DEF-only armature = canonical 41 names / parents / deform flags, 0 connected, "
               f"0 constraints, rest vs canonical <= {STAGE_REST_TOL:g}, rigtest stage rest vs rest stage <= "
               f"{SAME_REST_TOL:g}; mesh with {N_MATERIALS} material slots and an Armature modifier -> the stage armature",
     "goblin G7.2 / G7.3 method"),
    ("P2.6a2", f"bake: work DEF world (action rigtest in memory) vs rigtest stage, every frame <= {POS_TOL * 1000:g} mm "
               f"and <= {ROT_TOL:g} deg", "frames aligned by range start; rotation error float64 (check_p1.rang)"),
    ("P2.6b1", f"FBX: preset expectations (goblin G8.1) and axis mapping = d-02 section 5; Player.fbx and the clip reimported "
               "with the preset options: bone set / parents = canonical, no *_end, importer auto-connect undone, one mesh "
               f"with {N_MATERIALS} material slots, UVs and custom normals; Player.fbx rest vs rest stage <= "
               f"{POS_TOL * 1000:g} mm / {ROT_TOL:g} deg", "goblin G8.1 / G8.2 / G8.3 method"),
    ("P2.6b2", f"FBX clip reimport vs rigtest stage, every frame <= {POS_TOL * 1000:g} mm and <= {ROT_TOL:g} deg",
     "1-frame FBX anim offset corrected by aligning range starts"),
    ("P2.6c1", "Unity import: Generic, compression Off, 0 errors, 41 bones with canonical parents, bindposes_ok, rest "
               "block with every bone", "check_p1 p13_import / p13_bones; bindposes_ok blocking (d-02 section 8)"),
    ("P2.6c2", f"Unity non-skirt bones vs rigtest stage mapped by the preset axis mapping: position <= "
               f"{POS_TOL * 1000:g} mm, rest-relative rotation angle(dR_u, M dR_b M^T) <= {ROT_TOL:g} deg",
     "check_p1 p13_pose on clips.<clip>.frames bones_world; rest = report rest block (else first-frame relative, noted)"),
    ("P2.6c3", f"Unity skirt bones on the raw (spring-off) data: same thresholds when frames carry a separate skirt_raw "
               "block, else report only", "check_p1 p13_pose"),
    ("P2.6d", "bounds applied and every frame inside: rigtest stage mesh vertices (Unity space) and every report clip's "
              "bone positions, 0 frames outside", "measured independently (goblin A3.3c / G9.10 use the Unity count)"),
    ("P2.6e1", "(report) spring sway per chain: max and distribution; independent tip cross-check when possible", CALIB),
    ("P2.6e2", "(report) leg vs skirt penetration per frame, springs on vs off", CALIB),
    ("P2.8a", f"Unity bindposes_ok true and bindpose max element diff vs rest <= {BINDPOSE_TOL:g} (blocking); every DEF "
              f"bone's parent-relative rest rotation (Euler XYZ) pitch margin from +-90 deg reported (draft "
              f"{PITCH_MARGIN_DRAFT:g} deg, calibration)",
     "margin = 90 - asin(|R[2][0]|) (= 90 - |Y| of the Euler XYZ solution with |Y| <= 90) of parent^-1 @ child rest (Blender bone space = FBX local with primary Y / "
     "secondary X; a root bone uses its world rest). The Unity left-handed mirror keeps |Y|. bindpose max from "
     "report extra.skinned_mesh.bindpose_max_abs_diff_vs_rest (or bindpose_max_abs_diff); " + CALIB
     + " for the margin only"),
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
            print(f"[{GATE}/{CHECKER}] {c['id']} ok={c['ok']} measured={json.dumps(c['measured'])[:300]}")
        print(f"[{GATE}/{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


_STATE = {"missing": []}


def parse_args(argv):
    keys = {"--" + k.replace("_", "-"): k for k in P}
    i = 0
    while i < len(argv):
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: {' '.join(sorted(keys))} <value>")
        k = keys[argv[i]]
        P[k] = Path(argv[i + 1]) if k in PATH_KEYS else argv[i + 1]
        i += 2


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    ev.add_input(__file__)
    ev.add_input(p1.__file__)
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

    def run(cid, fn, *a):
        if cid in done:
            return
        try:
            put(cid, *fn(*a))
        except (Blocked, p1.Blocked) as e:
            block([cid], str(e))

    try:
        inputs = [k for k in PATH_KEYS if k != "out"]
        miss = [rel(P[k]) for k in inputs if not Path(P[k]).exists()]
        for k in inputs:
            ev.add_input(P[k])
        for q in sorted(HERE.glob("p2*.py")) + sorted(HERE.glob("unity_player*.ps1")):
            ev.add_input(q)
        _STATE["missing"] = miss
        ev.extra["names"] = {k: P[k] for k in ("action", "clip", "work_arm", "stage_arm", "stage_mesh")}
        if not Path(P["canon"]).exists():
            block(ids, f"missing input {rel(P['canon'])}")
            return
        ctx.canon = p2d.load_json(P["canon"])
        load_canon(ctx)
        ctx.preset = p2d.load_json(P["preset"]) if Path(P["preset"]).exists() else None
        ctx.rep = p2d.load_json(P["report"]) if Path(P["report"]).exists() else None
        # a: work + stages
        if Path(P["work"]).exists():
            try:
                work_phase(ctx)
            except (Blocked, p1.Blocked) as e:
                block(["P2.6a2"], f"work: {e}")
        else:
            block(["P2.6a2"], f"missing input {rel(P['work'])}")
        a1, a1_ok = {}, True
        for kind in ("rest", "rig"):
            path = P["stage_rest"] if kind == "rest" else P["stage_rig"]
            if not Path(path).exists():
                a1[kind], a1_ok = f"missing input {rel(path)}", False
                continue
            try:
                r, o = stage_struct(ctx, kind, path)
                a1[kind], a1_ok = r, a1_ok and o
            except (Blocked, p1.Blocked) as e:
                a1[kind], a1_ok = f"blocked: {e}", False
        put("P2.6a1", a1, a1_ok)
        if ctx.work is not None and ctx.stage_W is not None:
            r, o = compare_WW(ctx.work["W"], ctx.work["frames"], ctx.stage_W, ctx.stage_frames, ctx.names)
            r.update({"work": {k: v for k, v in ctx.work.items() if k != "W"}, "stage": ctx.stage_info})
            put("P2.6a2", r, o)
        else:
            block(["P2.6a2"], "work or rigtest stage not sampled")
        # b: FBX
        if ctx.preset is None:
            block(["P2.6b1", "P2.6b2"], f"missing input {rel(P['preset'])}")
        else:
            prec, pok = preset_check(ctx)
            b1, b1_ok = {"preset": prec}, pok
            clip_arm = None
            for key, path in (("rest", P["fbx_rest"]), ("clip", P["fbx_clip"])):
                if not Path(path).exists() or ctx.opts is None:
                    b1[key], b1_ok = (f"missing input {rel(path)}" if not Path(path).exists()
                                      else "no reimport options in the preset"), False
                    continue
                try:
                    r, o, arm = fbx_struct(ctx, path, key == "clip")
                    b1[key], b1_ok = r, b1_ok and o
                    if key == "clip":
                        clip_arm = arm
                        if ctx.stage_W is None:
                            block(["P2.6b2"], "rigtest stage not sampled")
                        else:
                            try:
                                put("P2.6b2", *fbx_frames(ctx, arm))
                            except (Blocked, p1.Blocked) as e:
                                block(["P2.6b2"], str(e))
                except (Blocked, p1.Blocked) as e:
                    b1[key], b1_ok = f"blocked: {e}", False
            put("P2.6b1", b1, b1_ok)
            if clip_arm is None:
                block(["P2.6b2"], f"clip FBX not imported ({rel(P['fbx_clip'])})")
        # c / d / e: Unity report
        if not isinstance(ctx.rep, dict):
            block([i for i in ids if i.startswith(("P2.6c", "P2.6d", "P2.6e", "P2.8a"))],
                  f"missing or unreadable input {rel(P['report'])}")
            return
        run("P2.6c1", u_import, ctx)
        if ctx.stage_W is None:
            block(["P2.6c2", "P2.6c3"], "rigtest stage not sampled")
        else:
            body = [n for n in ctx.names if not n.startswith("Skirt_")]
            skirt = [n for n in ctx.names if n.startswith("Skirt_")]
            try:
                m = u_pose(ctx, "bones_world", body)
                put("P2.6c2", m, u_ok(m))
            except (Blocked, p1.Blocked) as e:
                block(["P2.6c2"], str(e))
            try:
                f0 = (clip_block(ctx).get("frames") or [{}])[0]
                if isinstance(f0, dict) and "skirt_raw" in f0:
                    m = u_pose(ctx, "skirt_raw", skirt)
                    m["judged_on"] = "skirt_raw (spring-off) block"
                    put("P2.6c3", m, u_ok(m))
                else:
                    m = u_pose(ctx, "bones_world", skirt)
                    m["judged_on"] = "report only: frames carry no separate skirt_raw block"
                    put("P2.6c3", m, True)
            except (Blocked, p1.Blocked) as e:
                block(["P2.6c3"], str(e))
        run("P2.6d", c_bounds, ctx)
        run("P2.6e1", c_springs, ctx)
        run("P2.6e2", c_penetration, ctx)
        run("P2.8a", c_bindpose_margin, ctx)
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
