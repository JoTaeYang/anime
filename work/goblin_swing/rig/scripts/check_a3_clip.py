"""check_a3_clip - clip export checker (Gate A3 for attack_swing, I3 for idle, H3 for hit, D3 for death; design doc d-25
section 4 A3.1 .. A3.3; data contract data/a3_contract.md "Blender-side comparison").  Criterion ids =
<gate prefix>.1 / .2 / .3a .. .3f (.3f = 2026-09-26 HR2 blend shape mouth_open, rig/data/hr2_contract.md).

Arguments (after "--"):
  --action <name>          clip / action name (default attack_swing)
  --work <blend>           work blend, rig-relative or absolute (default: attack_swing -> gob_r07_swing_anim.blend,
                           idle -> gob_r08_idle.blend, hit -> gob_r08_hit.blend, death -> gob_r08_death.blend; required
                           otherwise)
  --gate-prefix A3|I3|H3|D3  evidence gate / criterion prefix (default by action: attack_swing A3, idle I3, hit H3,
                           death D3;
                           I3 otherwise)

Inputs (required; a missing one -> evidence with blocked criteria, "missing input" printed, exit 2):
  --work blend                            work rig GOB_rig (DEF 24) + action <action>
  rig/export/goblin@<action>.fbx          production clip FBX (s11_export_clip.py)
  unity/AvatarCheck/goblin_clip_report_<action>.json   Unity report (GoblinClipCheck.Run), Unity world coordinates
  rig/data/export_preset.json             blender_reimport.import_scene_fbx options, axis_mapping.blender_to_unity M
  rig/data/canonical_skeleton.json        24 bone names / parents / rest_matrix
Stage (reference for A3.1 / A3.2 / A3.3):
  rig/export/stage_<action>.blend         used when present; when missing it is built with
                                          s07b_export_rig.build_export(work blend, <action>, tmp) into
                                          rig/export/tmp_a3/ (temporary, deleted at the end)
Unity rest rotations (rest-relative comparison, compare_g9 method): the clip report 'rest' when it holds all 24 bones,
  else unity/AvatarCheck/goblin_report.json 'rest' (G9 report of the same goblin.fbx avatar; hashed when used).

Procedure:
  A. work blend: <action> assigned to GOB_rig in memory, every integer frame of the action range evaluated,
     24 DEF world matrices cached.
  B. stage blend: GOB_export evaluated at the same frames (world matrices, rest = matrix_world @ bone.matrix_local,
     goblin_club world).  A3.1 = work vs stage.
  C. FBX: empty factory scene, import_scene.fbx with the preset blender_reimport options, every bone use_connect
     False (G8 pitfall: importer auto-connect), frames aligned by range start (anim_offset shift corrected).
     A3.2 = FBX vs stage.
  D. Unity report: A3.3a bones / A3.3b club vs stage mapped by M (p_u = M p_b; rotation rest-relative
     dR = R R_rest^T, error = angle(dR_u, M dR_b M^T)), A3.3c bounds, A3.3d root motion, A3.3e importer,
     A3.3f blend shape: report blend_shapes + per-sample blend_shape {weight, curve, bake_world} vs the stage
     goblin_mesh evaluated mouth_open value x 100 every frame; on frames with stage value > 0 every reported
     BakeMesh mouth vertex vs the nearest stage goblin_mesh evaluated vertex mapped by M.
  Nothing is saved: blends / data / Unity files are never written.

Run:    bl.ps1 -Script check_a3_clip.py [-- --action idle]      (no blend)
Output: rig/inspect/<A3|I3|H3|D3>/check_a3_clip.json
Frame count expected = stage action frame range (A3.1 / A3.2 / A3.3a-d use it).
Exit:   0 when the evidence was written; 2 on a checker script error (run_main) or a missing input.
Units: distance measured in m, reported in mm; rotation error = degrees(2 acos |q1.q2|).
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import goblib  # noqa: E402,E702

import importlib  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import re  # noqa: E402
import shutil  # noqa: E402
import traceback  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Quaternion, Vector  # noqa: E402
from mathutils import kdtree as mkdtree  # noqa: E402

CHECKER = "check_a3_clip"
REPO = goblib.RIG.parent.parent.parent
G9_REPORT = REPO / "unity" / "AvatarCheck" / "goblin_report.json"
PRESET_JSON = goblib.DATA / "export_preset.json"
CANON_JSON = goblib.DATA / "canonical_skeleton.json"
S07B_NAME = "s07b_export_rig"
S07B = goblib.RIG / "scripts" / f"{S07B_NAME}.py"
TMP = goblib.EXPORT / "tmp_a3"
DEFAULT_WORK = {"attack_swing": "gob_r07_swing_anim.blend", "idle": "gob_r08_idle.blend",
                "hit": "gob_r08_hit.blend", "death": "gob_r08_death.blend",
                "roar": "gob_r08_roar.blend"}   # rig-relative
GATE_PREFIXES = ("A3", "I3", "H3", "D3", "Q3")
DEFAULT_GATE = {"attack_swing": "A3", "idle": "I3", "hit": "H3", "death": "D3", "roar": "Q3"}

# set by configure() from the command line (defaults = attack_swing / A3)
GATE = "A3"
ACTION = "attack_swing"
WORK_BLEND = goblib.RIG / DEFAULT_WORK["attack_swing"]
STAGE = goblib.EXPORT / "stage_attack_swing.blend"
FBX = goblib.EXPORT / "goblin@attack_swing.fbx"
REPORT = REPO / "unity" / "AvatarCheck" / "goblin_clip_report_attack_swing.json"


def configure(argv):
    """--action <name> (default attack_swing), --work <blend> (rig-relative or absolute; default by action),
    --gate-prefix A3|I3|H3|D3 (default DEFAULT_GATE by action, I3 otherwise).  Paths are derived from the action."""
    global GATE, ACTION, WORK_BLEND, STAGE, FBX, REPORT
    opts = {}
    i = 0
    while i < len(argv):
        k = argv[i]
        if k in ("--action", "--work", "--gate-prefix"):
            if i + 1 >= len(argv):
                raise ValueError(f"{k} needs a value")
            opts[k] = argv[i + 1]
            i += 2
            continue
        raise ValueError(f"unexpected argument {k!r} (usage: [--action <name>] [--work <blend>] "
                         f"[--gate-prefix {'|'.join(GATE_PREFIXES)}])")
    ACTION = opts.get("--action", "attack_swing")
    work = opts.get("--work", DEFAULT_WORK.get(ACTION))
    if work is None:
        raise ValueError(f"--work is required for action {ACTION!r} (defaults only for {sorted(DEFAULT_WORK)})")
    wp = Path(work)
    WORK_BLEND = wp if wp.is_absolute() else goblib.RIG / wp
    GATE = opts.get("--gate-prefix", DEFAULT_GATE.get(ACTION, "I3"))
    if GATE not in GATE_PREFIXES:
        raise ValueError(f"--gate-prefix must be one of {GATE_PREFIXES}, got {GATE!r}")
    STAGE = goblib.EXPORT / f"stage_{ACTION}.blend"
    FBX = goblib.EXPORT / f"goblin@{ACTION}.fbx"
    REPORT = REPO / "unity" / "AvatarCheck" / f"goblin_clip_report_{ACTION}.json"

WORK_ARM = "GOB_rig"
EXP_ARM = "GOB_export"
EXP_CLUB = "goblin_club"
EXP_MESH = "goblin_mesh"
MOUTH_KEY = "mouth_open"      # A3.3f (hr2_contract.md)
WEIGHT_TOL = 0.01             # A3.3f Unity weight vs stage value x 100
BAKE_TOL = 0.00001            # A3.3f BakeMesh mouth vertex vs Blender (m) = 0.01 mm
FBX_ARM = "goblin"
SOCKET = "weapon_socket_r"

N_BONES = 24
N_FRAMES = None      # set in stage_phase from the stage action frame range
FPS = 24.0
POS_TOL = 0.001      # m (A3.1 / A3.2 / A3.3a / A3.3b)
ROT_TOL = 1.0        # deg
ROOT_TOL = 1e-5      # A3.3d max abs component change vs the frame-1 sample
FRAME_MATCH = 1e-3

ANIM_TYPE = {0: "none", 1: "legacy", 2: "generic", 3: "human"}
AVATAR_SETUP = {0: "noavatar", 1: "createfromthismodel", 2: "copyfromother"}
COMPRESSION = {0: "off", 1: "keyframereduction", 2: "keyframereductionandcompression", 3: "optimal"}

def make_ids(p, nf):
    """Criterion (id, threshold, note) for gate prefix p; nf = stage action frame count (None before the stage is
    read -> 'N')."""
    N_FRAMES = nf if nf is not None else "N (stage action frame range)"  # noqa: N806 (text only)
    return (
    (f"{p}.1", f"{N_FRAMES} frames x {N_BONES} bones: work GOB_rig DEF world vs stage GOB_export world, position <= "
             f"{POS_TOL * 1000:g} mm, rotation <= {ROT_TOL:g} deg",
     f"world = matrix_world @ pose_bone.matrix (evaluated) per integer frame of the work action range; work blend "
     f"action {ACTION} assigned in memory; stage = {rel(STAGE)} (or a temporary "
     "build_export copy when missing); position = |t1-t2|, rotation = 2 acos|q1.q2| of decompose(); scale reported; "
     "frame count expected = stage action frame range"),
    (f"{p}.2", f"reimported {FBX.name}: {N_BONES} bones, names + parents = canonical, no *_end bone; "
             f"frame count = {N_FRAMES}; every frame x {N_BONES} bones world vs stage GOB_export <= "
             f"{POS_TOL * 1000:g} mm and <= {ROT_TOL:g} deg",
     "empty factory scene + import_scene.fbx with export_preset blender_reimport.import_scene_fbx options; every "
     "bone use_connect False in edit mode after import (importer_connected_bones reported); frame count = "
     "round(max key) - round(min key) + 1; FBX frame g0+i vs stage frame s0+i (anim_offset shift corrected); rest "
     "vs canonical reported only"),
    (f"{p}.3a", f"Unity report samples, every stage frame 1..{N_FRAMES} x {N_BONES} bones: pos <= {POS_TOL * 1000:g} mm, "
              f"rot <= {ROT_TOL:g} deg",
     "samples matched by frame; pos = |p_u - M p_b| (M = export_preset axis_mapping.blender_to_unity); rot = quat "
     "angle(dR_u, M dR_b M^T), dR = R R_rest^T (compare_g9); Blender rest = stage matrix_world @ bone.matrix_local; "
     "Unity rest = clip report 'rest' or goblin_report.json 'rest' (source reported); time_s vs (f - firstFrame)/24 "
     "reported"),
    (f"{p}.3b", f"club world at every sample: pos <= {POS_TOL * 1000:g} mm, rot <= {ROT_TOL:g} deg",
     "club pos vs M p_b(stage goblin_club); rot rest-relative with club rest: Blender = R_socket_rest_b @ C_b(first "
     "frame), Unity = report club_rest if present else R_socket_rest_u @ C_u(first sample), C = R_socket^T R_club; "
     "C_u / C_b constancy reported (compare_g9 G9.5 method)"),
    (f"{p}.3c", f"bounds: frames_checked = {N_FRAMES}, 0 frames outside the goblin.prefab localBounds",
     "report bounds: frames_checked, per_frame_outside entries with outside > 0, max_outside_m"),
    (f"{p}.3d", f"root motion: root_local and instance_root pos / rot equal to the frame-1 sample within {ROOT_TOL:g} "
              f"(max abs component) at every sample; clip hasRootCurves / hasMotionCurves / "
              f"hasGenericRootTransform reported",
     "per sample max |v(f) - v(frame 1)| over pos xyz and rot xyzw (quaternion sign aligned to frame 1)"),
    (f"{p}.3e", "importer: animationType Generic, avatarSetup CopyFromOther, sourceAvatar goblinAvatar "
              "(assetPath .../goblin.fbx), animationCompression Off",
     "report importer fields matched case-insensitively (enum ints mapped); resampleCurves, clip record and report "
     "errors reported"),
    (f"{p}.3f", f"blend shape {MOUTH_KEY} (2026-09-26 HR2): the Unity goblin_mesh has it; every stage frame "
              f"1..{N_FRAMES}: Unity weight and the blendShape.{MOUTH_KEY} curve value (absent -> 0) vs stage "
              f"{EXP_MESH} evaluated value x 100, |diff| <= {WEIGHT_TOL:g}; frames with stage value > 0: every "
              f"reported BakeMesh mouth vertex vs Blender <= {BAKE_TOL * 1000:g} mm (value 0 everywhere: weight "
              "check only)",
     "rig/data/hr2_contract.md thresholds (fixed from R0). Unity: report blend_shapes (index, mouth vertex indices = "
     "blend-shape delta > threshold, curve binding) and samples[].blend_shape {weight = SkinnedMeshRenderer "
     "GetBlendShapeWeight after SampleAnimation, curve = editor curve at t, bake_world = BakeMesh(useScale) mouth "
     "vertices in world when weight > 0}; Blender: stage goblin_mesh shape_keys.evaluated_get(depsgraph) value "
     "and the evaluated mesh world vertices (armature + shape key) at the same frame; vertex error = distance of "
     "each Unity vertex to the nearest M-mapped Blender vertex (Unity splits vertices at seams, so the match is "
     "by nearest position)"),
    )


SUFFIXES = (".1", ".2", ".3a", ".3b", ".3c", ".3d", ".3e", ".3f")

_STATE = {"missing": []}


# ---------------------------------------------------------------- helpers
class Blocked(Exception):
    """A criterion cannot be measured from the available data."""


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


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def open_blend(path):
    bpy.ops.wm.open_mainfile(filepath=str(path), load_ui=False)


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
    """In memory only.  Returns the slot identifier (or None)."""
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


def action_range(act):
    f0, f1 = act.frame_range
    return int(round(f0)), int(round(f1))


def fps_of(sc):
    return rnd(sc.render.fps / (sc.render.fps_base or 1.0), 4)


def decompose_np(M):
    _loc, q, sc = M.decompose()
    return np.array(M, dtype=np.float64), (q.w, q.x, q.y, q.z), tuple(sc)


def npm(A):
    return Matrix(np.asarray(A).tolist())


def rot3(A):
    """Rotation part of a 4x4 numpy world matrix (columns normalized)."""
    return npm(A).to_3x3().normalized()


def qu(xyzw):
    x, y, z, w = (float(v) for v in xyzw)
    return Quaternion((w, x, y, z)).normalized()


def rot_u(e):
    return qu(e["rot"]).to_matrix()


def ang(Ra, Rb):
    return goblib.quat_angle_deg(Ra.to_quaternion(), Rb.to_quaternion())


def stats(rows, scale=1.0, n=4):
    """rows = [(frame, bone, value)] -> max / p95 / mean / worst."""
    if not rows:
        return {"n": 0}
    v = np.array([r[2] for r in rows], dtype=np.float64)
    i = int(np.argmax(v))
    return {"n": len(rows), "max": round(float(v.max()) * scale, n), "p95": round(float(np.percentile(v, 95)) * scale, n),
            "mean": round(float(v.mean()) * scale, n), "worst": {"frame": rows[i][0], "bone": rows[i][1]}}


def find_sample(usamples, frame):
    best, bd = None, None
    for s in usamples:
        try:
            d = abs(float(s.get("frame", -1e9)) - frame)
        except (TypeError, ValueError):
            continue
        if bd is None or d < bd:
            best, bd = s, d
    return best if bd is not None and bd <= FRAME_MATCH else None


def load_canon(canon):
    bones = canon.get("bones") if isinstance(canon, dict) else None
    if not isinstance(bones, list) or not bones:
        raise Blocked("canonical_skeleton.json: no 'bones' list")
    names, parent, mat = [], {}, {}
    for b in bones:
        n = b["name"]
        names.append(n)
        parent[n] = b.get("parent") or None
        mat[n] = Matrix(b["rest_matrix"])
    return names, parent, mat


class Ctx:
    def __init__(self):
        self.names = self.parent = self.canon_mat = None
        self.frames = []
        self.W = None            # work {"M","q","s"} (F,B,...)
        self.work_info = {}
        self.stage_path = STAGE
        self.stage_note = None
        self.E = None            # stage {"M","q","s"}
        self.E_club = None       # (F,4,4)
        self.stage_rest = None   # {bone: 4x4 numpy} world rest
        self.stage_info = {}
        self.stage_mouth = None  # A3.3f stage goblin_mesh evaluated mouth_open per frame (None: no key)
        self.stage_mouth_verts = {}   # A3.3f frame -> (V,3) evaluated world vertices (frames with value > 0)


def key_value(obj, dg, name=MOUTH_KEY):
    """Evaluated shape-key value `name` of mesh obj (None when absent)."""
    key = obj.data.shape_keys if obj is not None and obj.type == "MESH" else None
    if key is None or key.key_blocks.get(name) is None:
        return None
    try:
        return float(key.evaluated_get(dg).key_blocks[name].value)
    except Exception:
        return float(key.key_blocks[name].value)


def eval_world_verts(obj, dg):
    oe = obj.evaluated_get(dg)
    me = oe.to_mesh()
    try:
        n = len(me.vertices)
        co = np.empty(n * 3, dtype=np.float64)
        me.vertices.foreach_get("co", co)
    finally:
        oe.to_mesh_clear()
    mw = np.array(oe.matrix_world, dtype=np.float64)
    return co.reshape(n, 3) @ mw[:3, :3].T + mw[:3, 3]


# ---------------------------------------------------------------- A: work blend
def work_phase(ctx):
    open_blend(WORK_BLEND)
    rig = bpy.data.objects.get(WORK_ARM)
    if rig is None or rig.type != "ARMATURE":
        raise Blocked(f"armature {WORK_ARM} missing in {rel(WORK_BLEND)}")
    act = bpy.data.actions.get(ACTION)
    if act is None:
        raise Blocked(f"action {ACTION} missing in {rel(WORK_BLEND)} (actions: {[a.name for a in bpy.data.actions]})")
    pre = rig.animation_data.action.name if rig.animation_data and rig.animation_data.action else None
    slot = assign_action(rig, act)
    missing = [n for n in ctx.names if n not in rig.pose.bones]
    if missing:
        raise Blocked(f"DEF bones missing in {WORK_ARM}: {missing}")
    f0, f1 = action_range(act)
    ctx.frames = list(range(f0, f1 + 1))
    sc = bpy.context.scene
    F, B = len(ctx.frames), len(ctx.names)
    W = {"M": np.zeros((F, B, 4, 4)), "q": np.zeros((F, B, 4)), "s": np.zeros((F, B, 3))}
    for i, f in enumerate(ctx.frames):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = rig.evaluated_get(dg)
        mw = oe.matrix_world
        for j, n in enumerate(ctx.names):
            W["M"][i, j], W["q"][i, j], W["s"][i, j] = decompose_np(mw @ oe.pose.bones[n].matrix)
    ctx.W = W
    ctx.work_info = {"action_before_assign": pre, "slot": slot, "frame_range": [f0, f1], "n_frames": F,
                     "use_frame_range": bool(getattr(act, "use_frame_range", False)), "fps": fps_of(sc),
                     "scene_frame_range": [sc.frame_start, sc.frame_end]}


# ---------------------------------------------------------------- B: stage
def ensure_stage(ctx, ev):
    if STAGE.exists():
        ctx.stage_path = STAGE
        ctx.stage_note = "existing"
        ev.add_input(STAGE)
        return
    if not S07B.exists():
        raise Blocked(f"{rel(STAGE)} missing and {rel(S07B)} missing")
    try:
        s07b = importlib.import_module(S07B_NAME)
    except Exception:
        raise Blocked(f"import {S07B_NAME} failed: {tb_tail()}")
    if TMP.exists():
        shutil.rmtree(TMP, ignore_errors=True)
    TMP.mkdir(parents=True, exist_ok=True)
    out = TMP / STAGE.name
    src_sha = goblib.sha256(WORK_BLEND)
    try:
        res = s07b.build_export(WORK_BLEND, ACTION, out)
    except Exception:
        raise Blocked(f"build_export({ACTION!r}) raised: {tb_tail()}")
    res = Path(res) if res else out
    if not res.exists():
        raise Blocked(f"build_export({ACTION!r}) wrote no file")
    ctx.stage_path = res
    ctx.stage_note = (f"{rel(STAGE)} missing -> built {rel(res)} with {S07B_NAME}.build_export (temporary); "
                      f"work blend sha256 unchanged: {goblib.sha256(WORK_BLEND) == src_sha}")
    ev.add_input(res)


def stage_phase(ctx):
    global N_FRAMES
    open_blend(ctx.stage_path)
    exp = bpy.data.objects.get(EXP_ARM)
    if exp is None or exp.type != "ARMATURE":
        raise Blocked(f"{EXP_ARM} missing in {rel(ctx.stage_path)}")
    ad = exp.animation_data
    act = ad.action if ad else None
    if act is None:
        raise Blocked(f"{EXP_ARM} has no action in {rel(ctx.stage_path)}")
    missing = [n for n in ctx.names if n not in exp.pose.bones]
    if missing:
        raise Blocked(f"{EXP_ARM} lacks bones {missing}")
    s0, s1 = action_range(act)
    N_FRAMES = s1 - s0 + 1
    club = bpy.data.objects.get(EXP_CLUB)
    smesh = bpy.data.objects.get(EXP_MESH)
    sc = bpy.context.scene
    F, B = len(ctx.frames), len(ctx.names)
    E = {"M": np.zeros((F, B, 4, 4)), "q": np.zeros((F, B, 4)), "s": np.zeros((F, B, 3))}
    E_club = np.zeros((F, 4, 4)) if club is not None else None
    mouth = []
    for i, f in enumerate(ctx.frames):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = exp.evaluated_get(dg)
        mw = oe.matrix_world
        for j, n in enumerate(ctx.names):
            E["M"][i, j], E["q"][i, j], E["s"][i, j] = decompose_np(mw @ oe.pose.bones[n].matrix)
        if club is not None:
            E_club[i] = np.array(club.evaluated_get(dg).matrix_world, dtype=np.float64)
        mv = key_value(smesh, dg)
        mouth.append(mv)
        if mv is not None and mv > 0.0:
            ctx.stage_mouth_verts[f] = eval_world_verts(smesh, dg)
    ctx.stage_mouth = None if any(v is None for v in mouth) else mouth
    mw0 = exp.matrix_world.copy()
    ctx.stage_rest = {n: np.array(mw0 @ exp.data.bones[n].matrix_local, dtype=np.float64) for n in ctx.names}
    ctx.E, ctx.E_club = E, E_club
    fcs = fcurves_of(act)
    ctx.stage_info = {"path": rel(ctx.stage_path), "note": ctx.stage_note, "action": act.name,
                      "frame_range": list(action_range(act)), "n_fcurves": len(fcs), "fps": fps_of(sc),
                      "n_bones": len(exp.data.bones),
                      "connected_bones": sorted(b.name for b in exp.data.bones if b.use_connect),
                      "club": (None if club is None else
                               {"parent": club.parent.name if club.parent else None, "parent_type": club.parent_type,
                                "parent_bone": club.parent_bone})}


# ---------------------------------------------------------------- A3.1
def c_a31(ctx):
    W, E = ctx.W, ctx.E
    pos = np.linalg.norm(W["M"][..., :3, 3] - E["M"][..., :3, 3], axis=-1)
    rot = np.degrees(2.0 * np.arccos(np.clip(np.abs((W["q"] * E["q"]).sum(-1)), 0.0, 1.0)))
    sc = np.abs(W["s"] - E["s"]).max(-1)
    F, B = pos.shape
    ip = np.unravel_index(int(pos.argmax()), pos.shape)
    ir = np.unravel_index(int(rot.argmax()), rot.shape)
    m = {"n_frames": F, "frames": [ctx.frames[0], ctx.frames[-1]], "n_bones": B,
         "pos_mm_max": mm(pos.max()), "pos_worst_[bone,frame]": [ctx.names[ip[1]], ctx.frames[ip[0]]],
         "rot_deg_max": rnd(rot.max(), 4), "rot_worst_[bone,frame]": [ctx.names[ir[1]], ctx.frames[ir[0]]],
         "n_samples_over_tol": int(((pos > POS_TOL) | (rot > ROT_TOL)).sum()),
         "scale_dev_work_vs_stage_max": sci(sc.max()),
         "per_bone_max_[pos_mm,rot_deg]": {n: [mm(pos[:, j].max()), rnd(rot[:, j].max(), 4)]
                                           for j, n in enumerate(ctx.names)},
         "work": ctx.work_info, "stage": ctx.stage_info}
    ok = F == N_FRAMES and B == N_BONES and pos.max() <= POS_TOL and rot.max() <= ROT_TOL
    return m, bool(ok)


# ---------------------------------------------------------------- A3.2
def import_options(preset):
    pr = (preset.get("blender_reimport") or {}).get("import_scene_fbx") if isinstance(preset, dict) else None
    if not isinstance(pr, dict) or not pr:
        raise Blocked("export_preset.json has no blender_reimport.import_scene_fbx options")
    return dict(pr)


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
    o = bpy.data.objects.get(FBX_ARM)
    if o is not None and o.type == "ARMATURE":
        return o, None
    arms = [x for x in bpy.data.objects if x.type == "ARMATURE"]
    if len(arms) == 1:
        return arms[0], f"armature object named {arms[0].name!r}, not {FBX_ARM!r}"
    raise Blocked(f"no armature {FBX_ARM!r} (armatures: {[a.name for a in arms]})")


def disconnect_bones(arm):
    """G8 pitfall: the FBX importer connects a child whose head lies on the parent's tail (location keys then
    ignored).  Every bone use_connect False in edit mode, back to object mode."""
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


def c_a32(ctx, opts):
    fresh_import(FBX, opts)
    arm, note = find_arm()
    objs = sorted(f"{o.name}({o.type})" for o in bpy.data.objects)
    bones = arm.data.bones
    bnames = [b.name for b in bones]
    ends = sorted(n for n in bnames if n.lower().endswith("_end"))
    only_c = sorted(set(ctx.names) - set(bnames))
    only_f = sorted(set(bnames) - set(ctx.names))
    par_mis = {}
    for n in ctx.names:
        b = bones.get(n)
        if b is None:
            continue
        p = b.parent.name if b.parent else None
        if p != ctx.parent[n]:
            par_mis[n] = {"canon": ctx.parent[n], "fbx": p}
    hier_ok = len(bnames) == N_BONES and not only_c and not only_f and not par_mis and not ends
    conn = disconnect_bones(arm)
    # rest vs canonical, report only
    rp, rr = {}, {}
    mw = arm.matrix_world
    for n in ctx.names:
        b = arm.data.bones.get(n)
        if b is None:
            continue
        Wm = mw @ b.matrix_local
        rp[n] = (Wm.translation - ctx.canon_mat[n].translation).length
        rr[n] = goblib.quat_angle_deg(Wm.decompose()[1], ctx.canon_mat[n].decompose()[1])
    rest_rep = ({"pos_mm_max": mm(max(rp.values())), "pos_worst": max(rp, key=rp.get),
                 "rot_deg_max": rnd(max(rr.values()), 4), "rot_worst": max(rr, key=rr.get)} if rp else None)
    m = {"import_options": opts, "objects": objs, "armature": arm.name, "armature_note": note,
         "n_bones": len(bnames), "only_canon": only_c, "only_fbx": only_f, "parent_mismatch": par_mis,
         "end_leaf_bones": ends, "hierarchy_ok": hier_ok, "importer_connected_bones": conn,
         "rest_vs_canonical_report": rest_rep, "actions_in_file": [a.name for a in bpy.data.actions]}
    ad = arm.animation_data
    act = ad.action if ad else None
    if act is None:
        m["frames"] = "blocked: reimported armature has no action"
        return m, False
    fcs = fcurves_of(act)
    times = [k.co.x for fc in fcs for k in fc.keyframe_points]
    if not times:
        m["frames"] = f"blocked: action {act.name} has no keys"
        return m, False
    t0, t1 = min(times), max(times)
    g0, g1 = int(round(t0)), int(round(t1))
    n_fbx = g1 - g0 + 1
    sc = bpy.context.scene
    m.update({"action": act.name, "key_time_range": [rnd(t0, 4), rnd(t1, 4)], "n_frames_fbx": n_fbx,
              "frame_offset_fbx_minus_stage": g0 - ctx.frames[0],
              "frame_offset_note": "importer anim_offset shifts keys; corrected by aligning range starts",
              "keys_not_on_integer_frame": sum(1 for t in times if abs(t - round(t)) > 1e-4),
              "n_fcurves": len(fcs), "scene_fps_after_import": fps_of(sc)})
    missing = [n for n in ctx.names if n not in arm.pose.bones]
    if missing:
        m["bones_missing"] = missing
        return m, False
    n = min(n_fbx, len(ctx.frames))
    B = len(ctx.names)
    P = np.zeros((n, B))
    R = np.zeros((n, B))
    for i in range(n):
        sc.frame_set(g0 + i)
        dg = bpy.context.evaluated_depsgraph_get()
        oe = arm.evaluated_get(dg)
        amw = oe.matrix_world
        for j, name in enumerate(ctx.names):
            Mf = amw @ oe.pose.bones[name].matrix
            S = npm(ctx.E["M"][i, j])
            P[i, j] = (Mf.translation - S.translation).length
            R[i, j] = goblib.quat_angle_deg(Mf.decompose()[1], S.decompose()[1])
    ip = np.unravel_index(int(P.argmax()), P.shape)
    ir = np.unravel_index(int(R.argmax()), R.shape)
    m.update({"n_frames_compared": n, "alignment": "fbx frame g0+i vs stage frame s0+i",
              "pos_mm_max": mm(P.max()), "pos_worst_[bone,stage_frame]": [ctx.names[ip[1]], ctx.frames[ip[0]]],
              "rot_deg_max": rnd(R.max(), 4), "rot_worst_[bone,stage_frame]": [ctx.names[ir[1]], ctx.frames[ir[0]]],
              "n_samples_over_tol": int(((P > POS_TOL) | (R > ROT_TOL)).sum()),
              "per_bone_max_[pos_mm,rot_deg]": {nm: [mm(P[:, j].max()), rnd(R[:, j].max(), 4)]
                                                for j, nm in enumerate(ctx.names)}})
    ok = (hier_ok and conn["undone"] and n_fbx == N_FRAMES and n_fbx == len(ctx.frames) and n > 0
          and P.max() <= POS_TOL and R.max() <= ROT_TOL)
    return m, bool(ok)


# ---------------------------------------------------------------- A3.3
def unity_rest(rep, names):
    r = rep.get("rest")
    if isinstance(r, dict) and all(isinstance(r.get(n), dict) and "rot" in r[n] for n in names):
        return r, "clip report 'rest'", None
    if G9_REPORT.exists():
        g9 = load(G9_REPORT)
        r = g9.get("rest")
        if isinstance(r, dict) and all(isinstance(r.get(n), dict) and "rot" in r[n] for n in names):
            return r, f"{rel(G9_REPORT)} 'rest' (clip report has no complete 'rest')", G9_REPORT
    raise Blocked("no Unity rest rotations: clip report 'rest' incomplete and goblin_report.json 'rest' unusable")


def c_a33a(ctx, rep, M, urest, rest_src):
    Mt = M.transposed()
    names = ctx.names
    Rb0 = {n: rot3(ctx.stage_rest[n]) for n in names}
    Ru0 = {n: rot_u(urest[n]) for n in names}
    usamples = rep.get("samples", []) or []
    clip = rep.get("clip") or {}
    first = clip.get("firstFrame", 1)
    try:
        first = float(first)
    except (TypeError, ValueError):
        first = 1.0
    pos_rows, rot_rows, missing_frames, missing_bones = [], [], [], set()
    per_bone = {n: [0.0, 0.0] for n in names}
    per_frame = {}
    tdiff = 0.0
    for i, f in enumerate(ctx.frames):
        us = find_sample(usamples, float(f))
        if us is None:
            missing_frames.append(f)
            continue
        if "time_s" in us:
            tdiff = max(tdiff, abs(float(us["time_s"]) - (f - first) / FPS))
        pf = per_frame.setdefault(f, [0.0, 0.0])
        for j, n in enumerate(names):
            ub = (us.get("bones") or {}).get(n)
            if not isinstance(ub, dict) or "pos" not in ub or "rot" not in ub:
                missing_bones.add(n)
                continue
            Eb = ctx.E["M"][i, j]
            pe = (Vector(ub["pos"]) - M @ Vector(Eb[:3, 3].tolist())).length
            dRb = M @ (rot3(Eb) @ Rb0[n].transposed()) @ Mt
            dRu = rot_u(ub) @ Ru0[n].transposed()
            re_ = ang(dRu, dRb)
            pos_rows.append((f, n, pe))
            rot_rows.append((f, n, re_))
            per_bone[n][0] = max(per_bone[n][0], pe)
            per_bone[n][1] = max(per_bone[n][1], re_)
            pf[0], pf[1] = max(pf[0], pe), max(pf[1], re_)
    m = {"n_stage_frames": len(ctx.frames), "n_report_samples": len(usamples),
         "n_matched": len(ctx.frames) - len(missing_frames), "missing_frames": missing_frames,
         "missing_bones": sorted(missing_bones), "unity_rest_source": rest_src,
         "clip_firstFrame_used": first, "time_s_max_diff_vs_(f-firstFrame)/24": sci(tdiff),
         "pos_mm": stats(pos_rows, 1000.0), "rot_deg": stats(rot_rows, 1.0),
         "per_bone_max[pos_mm, rot_deg]": {n: [mm(v[0]), rnd(v[1], 4)] for n, v in per_bone.items()},
         "per_frame_max[pos_mm, rot_deg]": {str(k): [mm(v[0]), rnd(v[1], 4)] for k, v in sorted(per_frame.items())}}
    pmax = max((r[2] for r in pos_rows), default=float("inf"))
    rmax = max((r[2] for r in rot_rows), default=float("inf"))
    ok = (len(ctx.frames) == N_FRAMES and not missing_frames and not missing_bones and bool(pos_rows)
          and pmax <= POS_TOL and rmax <= ROT_TOL)
    return m, bool(ok)


def unity_club_rest(rep):
    if isinstance(rep.get("club_rest"), dict) and "rot" in rep["club_rest"]:
        return rep["club_rest"], "report 'club_rest'"
    r = rep.get("rest") or {}
    for k in ("goblin_club", "club"):
        if isinstance(r.get(k), dict) and "rot" in r[k]:
            return r[k], f"report rest['{k}']"
    return None, None


def c_a33b(ctx, rep, M, urest):
    if ctx.E_club is None:
        raise Blocked(f"{EXP_CLUB} missing in the stage")
    Mt = M.transposed()
    js = ctx.names.index(SOCKET)
    usamples = rep.get("samples", []) or []
    pairs = []
    for i, f in enumerate(ctx.frames):
        us = find_sample(usamples, float(f))
        if us is None or not isinstance(us.get("club"), dict):
            continue
        pairs.append((i, f, us))
    if not pairs:
        raise Blocked("no report sample with a 'club' entry matched the stage frames")
    Rs_b0 = rot3(ctx.stage_rest[SOCKET])
    Cb = [rot3(ctx.E["M"][i, js]).transposed() @ rot3(ctx.E_club[i]) for i in range(len(ctx.frames))]
    Rc_b0 = Rs_b0 @ Cb[0]
    Rs_u0 = rot_u(urest[SOCKET])
    Cu = [rot_u(us["bones"][SOCKET]).transposed() @ rot_u(us["club"]) for _i, _f, us in pairs
          if isinstance((us.get("bones") or {}).get(SOCKET), dict)]
    cu_rest, src = unity_club_rest(rep)
    if cu_rest is not None:
        Rc_u0 = rot_u(cu_rest)
    else:
        if not Cu:
            raise Blocked(f"no Unity club rest and no {SOCKET} in samples to derive it")
        Rc_u0 = Rs_u0 @ Cu[0]
        src = "derived: R_socket_rest_u @ C_u(first matched sample)"
    pos_rows, rot_rows = [], []
    for i, f, us in pairs:
        pe = (Vector(us["club"]["pos"]) - M @ Vector(ctx.E_club[i][:3, 3].tolist())).length
        dRb = M @ (rot3(ctx.E_club[i]) @ Rc_b0.transposed()) @ Mt
        dRu = rot_u(us["club"]) @ Rc_u0.transposed()
        pos_rows.append((f, "club", pe))
        rot_rows.append((f, "club", ang(dRu, dRb)))
    I3 = Matrix.Identity(3)
    m = {"n_stage_frames": len(ctx.frames), "n_matched": len(pairs),
         "pos_mm": stats(pos_rows, 1000.0), "rot_deg": stats(rot_rows, 1.0),
         "blender_club_rest": "R_socket_rest_b @ C_b(first frame)", "unity_club_rest_source": src,
         "C_u(club local rot under socket, Unity)": {
             "max_angle_from_identity_deg": rnd(max((ang(c, I3) for c in Cu), default=float("nan")), 4),
             "max_spread_vs_first_deg": rnd(max((ang(c, Cu[0]) for c in Cu), default=float("nan")), 4)},
         "C_b(club rot rel. socket, Blender)": {
             "angle_from_identity_deg": rnd(ang(Cb[0], I3), 4),
             "max_spread_vs_first_deg": rnd(max(ang(c, Cb[0]) for c in Cb), 4)}}
    ok = (len(ctx.frames) == N_FRAMES and len(pairs) == len(ctx.frames)
          and max(r[2] for r in pos_rows) <= POS_TOL and max(r[2] for r in rot_rows) <= ROT_TOL)
    return m, bool(ok)


def c_a33c(rep):
    b = rep.get("bounds")
    if not isinstance(b, dict):
        raise Blocked("report has no 'bounds' object")
    pfo = b.get("per_frame_outside", []) or []
    viol = [x for x in pfo if float(x[1]) > 0.0]
    mx = max([float(b.get("max_outside_m", 0.0) or 0.0)] + [float(x[1]) for x in viol])
    fc = b.get("frames_checked")
    m = {"frames_checked": fc, "violation_frames": len(viol), "max_outside_mm": mm(mx),
         "violations": viol, "prefab": b.get("prefab"), "localBounds": b.get("localBounds")}
    ok = fc == N_FRAMES and len(viol) == 0 and mx <= 0.0
    return m, bool(ok)


def c_a33d(ctx, rep):
    usamples = rep.get("samples", []) or []
    clip = rep.get("clip") or {}
    curves = {k: clip.get(k) for k in ("hasRootCurves", "hasMotionCurves", "hasGenericRootTransform")}
    s0 = find_sample(usamples, float(ctx.frames[0])) if ctx.frames else None
    if s0 is None:
        raise Blocked(f"no report sample at frame {ctx.frames[0] if ctx.frames else '?'}")
    out, ok = {}, True
    for key in ("root_local", "instance_root"):
        e0 = s0.get(key)
        if not isinstance(e0, dict) or "pos" not in e0 or "rot" not in e0:
            out[key] = "missing in the frame-1 sample"
            ok = False
            continue
        p0 = np.array(e0["pos"], dtype=np.float64)
        q0 = np.array(e0["rot"], dtype=np.float64)
        dp_max, dq_max, wp, wq, lacking = 0.0, 0.0, None, None, []
        n = 0
        for f in ctx.frames:
            s = find_sample(usamples, float(f))
            e = s.get(key) if s is not None else None
            if not isinstance(e, dict) or "pos" not in e or "rot" not in e:
                lacking.append(f)
                continue
            n += 1
            dp = float(np.abs(np.array(e["pos"], dtype=np.float64) - p0).max())
            q = np.array(e["rot"], dtype=np.float64)
            dq = float(min(np.abs(q - q0).max(), np.abs(q + q0).max()))
            if wp is None or dp > dp_max:
                dp_max, wp = dp, f
            if wq is None or dq > dq_max:
                dq_max, wq = dq, f
        out[key] = {"frame1_pos": e0["pos"], "frame1_rot_xyzw": e0["rot"], "n_samples": n,
                    "frames_lacking": lacking, "pos_max_abs_change": sci(dp_max), "pos_worst_frame": wp,
                    "rot_max_abs_change": sci(dq_max), "rot_worst_frame": wq}
        ok = ok and not lacking and n == len(ctx.frames) and dp_max <= ROOT_TOL and dq_max <= ROOT_TOL
    out["clip_curves_report"] = curves
    return out, bool(ok and len(ctx.frames) == N_FRAMES)


def enum_norm(v, table):
    if isinstance(v, bool) or v is None:
        return str(v).lower()
    if isinstance(v, (int, float)):
        return table.get(int(v), str(v))
    s = str(v).strip()
    if re.fullmatch(r"-?\d+", s):
        return table.get(int(s), s)
    return re.sub(r"[\s_]", "", s).lower()


def c_a33e(rep):
    imp = rep.get("importer")
    if not isinstance(imp, dict):
        raise Blocked("report has no 'importer' object")
    low = {str(k).lower().replace("_", ""): k for k in imp}
    res, ok = {}, True
    for label, table, want in (("animationType", ANIM_TYPE, "generic"), ("avatarSetup", AVATAR_SETUP, "copyfromother"),
                               ("animationCompression", COMPRESSION, "off")):
        k = low.get(label.lower())
        if k is None:
            res[label] = {"found": False, "expected": want, "ok": False}
            ok = False
            continue
        got = enum_norm(imp[k], table)
        good = got == want
        res[label] = {"raw": imp[k], "value": got, "expected": want, "ok": good}
        ok = ok and good
    k = low.get("sourceavatar")
    sv = imp.get(k) if k is not None else None
    if isinstance(sv, dict):
        nm = str(sv.get("name", ""))
        ap = str(sv.get("assetPath", sv.get("path", ""))).replace("\\", "/")
        good = nm == "goblinAvatar" and Path(ap).name.lower() == "goblin.fbx"
        res["sourceAvatar"] = {"raw": sv, "expected": "name goblinAvatar, assetPath .../goblin.fbx", "ok": good}
    else:
        good = False
        res["sourceAvatar"] = {"raw": sv, "expected": "name goblinAvatar, assetPath .../goblin.fbx", "ok": False}
    ok = ok and good
    res["resampleCurves_report"] = imp.get("resampleCurves")
    res["clip_report"] = rep.get("clip")
    res["unity_version"] = rep.get("unity_version")
    res["unity_errors"] = rep.get("errors", [])
    return res, bool(ok)


# ---------------------------------------------------------------- A3.3f
def c_a33f(ctx, rep, M):
    bs = rep.get("blend_shapes")
    if not isinstance(bs, dict):
        raise Blocked("report has no 'blend_shapes' object (GoblinClipCheck HR2 section)")
    if ctx.stage_mouth is None:
        raise Blocked(f"stage {EXP_MESH} has no shape key {MOUTH_KEY}")
    Mn = np.array(M, dtype=np.float64)
    usamples = rep.get("samples", []) or []
    rows, lacking, no_bake = {}, [], []
    wmax = cmax = bmax = 0.0
    wworst = cworst = bworst = None
    n_bake = 0
    for i, f in enumerate(ctx.frames):
        sv = float(ctx.stage_mouth[i])
        s = find_sample(usamples, float(f))
        b = s.get("blend_shape") if isinstance(s, dict) else None
        if not isinstance(b, dict) or b.get("weight") is None:
            lacking.append(f)
            continue
        w = float(b["weight"])
        c = 0.0 if b.get("curve") is None else float(b["curve"])
        dw, dc = abs(w - 100.0 * sv), abs(c - 100.0 * sv)
        row = {"stage_x100": round(100.0 * sv, 6), "weight": round(w, 6), "curve": b.get("curve")}
        if wworst is None or dw > wmax:
            wmax, wworst = dw, f
        if cworst is None or dc > cmax:
            cmax, cworst = dc, f
        if sv > 0.0:
            pts = b.get("bake_world")
            V = ctx.stage_mouth_verts.get(f)
            if not isinstance(pts, list) or not pts or V is None:
                no_bake.append(f)
            else:
                P = np.array(pts, dtype=np.float64)
                Vu = V @ Mn.T
                kd = mkdtree.KDTree(len(Vu))
                for k, p in enumerate(Vu.tolist()):
                    kd.insert(p, k)
                kd.balance()
                d = np.array([kd.find(p.tolist())[2] for p in P])
                n_bake += 1
                row.update({"bake_n": int(len(P)), "bake_max_mm": mm(d.max()), "bake_p95_mm": mm(np.percentile(d, 95))})
                if bworst is None or d.max() > bmax:
                    bmax, bworst = float(d.max()), f
        rows[f] = row
    judged = [f for i, f in enumerate(ctx.frames) if float(ctx.stage_mouth[i]) > 0.0]
    m = {"unity_blend_shapes": {k: v for k, v in bs.items() if k != "mouth_vertex_indices"},
         "n_unity_mouth_vertices": len(bs.get("mouth_vertex_indices") or []),
         "n_frames": len(ctx.frames), "frames_lacking_blend_shape": lacking,
         "weight_max_abs_diff": round(wmax, 6), "weight_worst_frame": wworst,
         "curve_max_abs_diff": round(cmax, 6), "curve_worst_frame": cworst,
         "frames_stage_value_gt_0": judged, "bake_frames_checked": n_bake, "bake_frames_missing": no_bake,
         "bake_max_mm": mm(bmax) if n_bake else None, "bake_worst_frame": bworst,
         "stage_min_max_x100": [round(100.0 * min(ctx.stage_mouth), 6), round(100.0 * max(ctx.stage_mouth), 6)],
         "per_frame": rows}
    has = MOUTH_KEY in (bs.get("names") or []) and int(bs.get("mouth_open_index", -1)) >= 0
    ok = (has and len(ctx.frames) == N_FRAMES and not lacking and wmax <= WEIGHT_TOL and cmax <= WEIGHT_TOL
          and not no_bake and (bmax <= BAKE_TOL))
    return m, bool(ok)


# ---------------------------------------------------------------- main
def main():
    configure(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ALL_IDS = [GATE + s for s in SUFFIXES]  # noqa: N806
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(__file__)
    ctx = Ctx()
    done = set()

    def put(cid, measured, ok):
        cid = cid if cid.startswith(GATE) else GATE + cid
        ids = {i: (t, n) for i, t, n in make_ids(GATE, N_FRAMES)}
        ev.criterion(cid, measured, ids[cid][0], ok, ids[cid][1])
        done.add(cid)

    def block(cids, reason):
        for cid in cids:
            cid = cid if cid.startswith(GATE) else GATE + cid
            if cid not in done:
                put(cid, f"blocked: {reason}", False)

    miss = []
    for p in (WORK_BLEND, FBX, REPORT, PRESET_JSON, CANON_JSON):
        if p.exists():
            ev.add_input(p)
        else:
            miss.append(rel(p))
    ev.add_stage_inputs("s11")
    _STATE["missing"] = miss

    try:
        if miss:
            block(ALL_IDS, "missing input: " + ", ".join(miss))
            return
        preset, canon, rep = load(PRESET_JSON), load(CANON_JSON), load(REPORT)
        M = Matrix(preset["axis_mapping"]["blender_to_unity"])
        try:
            ctx.names, ctx.parent, ctx.canon_mat = load_canon(canon)
            work_phase(ctx)
            ensure_stage(ctx, ev)
            stage_phase(ctx)
        except Blocked as e:
            block(ALL_IDS, str(e))
            return
        put(".1", *c_a31(ctx))

        try:
            opts = import_options(preset)
            put(".2", *c_a32(ctx, opts))
        except Blocked as e:
            block([".2"], str(e))

        try:
            urest, rest_src, rest_file = unity_rest(rep, ctx.names)
            if rest_file is not None:
                ev.add_input(rest_file)
            put(".3a", *c_a33a(ctx, rep, M, urest, rest_src))
            put(".3b", *c_a33b(ctx, rep, M, urest))
        except Blocked as e:
            block([".3a", ".3b"], str(e))
        for cid, fn in ((".3c", lambda: c_a33c(rep)), (".3d", lambda: c_a33d(ctx, rep)),
                        (".3e", lambda: c_a33e(rep)), (".3f", lambda: c_a33f(ctx, rep, M))):
            try:
                put(cid, *fn())
            except Blocked as e:
                block([cid], str(e))
    finally:
        try:
            block(ALL_IDS, "checker aborted before this criterion")
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
