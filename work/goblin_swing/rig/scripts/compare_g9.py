"""compare_g9 - Gate G9 Unity vs Blender comparison (design doc d-23 sections 6 / 7, G9.1 .. G9.11; plan T39;
data contract data/g9_contract.md; G9.11 = 2026-09-26 HR2 blend shape + submeshes, rig/data/hr2_contract.md).

Inputs (all required; a missing one -> evidence with blocked criteria, "missing input" printed, exit 2):
  unity/AvatarCheck/goblin_report.json   Unity report (GoblinRigCheck.Run), Unity world coordinates
  rig/inspect/G9/blender_dump.json       Blender reference (s09_blender_ref.py), Blender world coordinates
  rig/data/export_preset.json            axis_mapping.blender_to_unity M (and blender_to_fbx G, report only)
  rig/data/canonical_skeleton.json       24 bone names / parents / rest_matrix (armature space)
  rig/data/compare_cams.json             render frames + cameras (s09)
  rig/inspect/G9/blender_<cam>_<frame>.png, unity_<cam>_<frame>.png   renders (G9.6 / G9.7)
Provenance (hashed when present): rig/export/goblin.fbx, goblin@rigtest.fbx, goblin_club.fbx, stage_rigtest.blend.

Mapping (contract): p_u = M p_b; R_u = M R_b M^T (det M = -1).  Rotations are compared rest-relative:
  dR = R_pose R_rest^T on each side, error = quaternion angle between dR_u and M dR_b M^T.
Run:    bl.ps1 -Script compare_g9.py            (no blend)   [-- --report <path>  debug override of the Unity report]
Output: rig/inspect/G9/compare_g9.json, rig/inspect/G9/side_by_side.png
Exit:   0 when the evidence was written; 2 on a script error (run_main) or a missing input.
Units: distance measured in m, reported in mm; rotation error = degrees(2 acos |q1.q2|).
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import goblib  # noqa: E402,E702

import json  # noqa: E402
import math  # noqa: E402
import re  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Quaternion, Vector  # noqa: E402

GATE = "G9"
CHECKER = "compare_g9"
REPO = goblib.RIG.parent.parent.parent
REPORT_DEFAULT = REPO / "unity" / "AvatarCheck" / "goblin_report.json"
G9DIR = goblib.INSPECT / "G9"
DUMP = G9DIR / "blender_dump.json"
PRESET = goblib.DATA / "export_preset.json"
CANON = goblib.DATA / "canonical_skeleton.json"
CAMS = goblib.DATA / "compare_cams.json"
SHEET = G9DIR / "side_by_side.png"
LIT_SHEET = G9DIR / "side_by_side_lit.png"
PROVENANCE = (goblib.EXPORT / "goblin.fbx", goblib.EXPORT / "goblin@rigtest.fbx", goblib.EXPORT / "goblin_club.fbx",
              goblib.EXPORT / "stage_rigtest.blend")

POS_TOL = 0.001          # m (G9.4 / G9.5 / G9.8)
ROT_TOL = 1.0            # deg
SCALE_TOL = 1e-4         # G9.2 lossyScale
DIR_TOL = 1.0            # deg (G9.3)
SIL_TOL = 3.0            # px at 1024 px height (G9.6)
BIND_TOL = 1e-4          # G9.9 max abs 4x4 element difference (translation in m)
FRAME_LAST = 583
FRAME_MATCH = 1e-3       # frame key match tolerance
HOLE_MAX_PX = 500       # G9.6: enclosed background holes smaller than this are filled in both masks
CLOSE_RADIUS = 2        # G9.6: morphological closing, disk radius (px), applied before the hole fill
BG_TOL = 0.1             # opaque render: max |rgb - background| <= BG_TOL counts as background
N_BONES = 24
CONTAINER_NAMES = ("goblin", "goblin_mesh")
MOUTH_SHAPE = "mouth_open"    # G9.11 (hr2_contract.md)
N_SUBMESH = 4                 # G9.11 submeshes / materials
MAT_NAMES = ("GOB_skin", "GOB_mouth_inner", "GOB_teeth", "GOB_tongue")   # G9.11 report
CLUB_NODE = "goblin_club"

ANIM_TYPE = {0: "none", 1: "legacy", 2: "generic", 3: "human"}
AVATAR_SETUP = {0: "noavatar", 1: "createfromthismodel", 2: "copyfromother"}
COMPRESSION = {0: "off", 1: "keyframereduction", 2: "keyframereductionandcompression", 3: "optimal"}

IDS = (
    ("G9.1", "goblin.fbx: Generic / CreateFromThisModel / root node 'root'; goblin@rigtest.fbx: Generic / "
             "CopyFromOther (goblin Avatar) / compression Off; goblin_club.fbx: None",
     "report importers[<file>] fields matched case-insensitively (animationType, avatarSetup, motionNodeName, "
     "sourceAvatar, animationCompression; enum ints mapped); root node ok when the last path segment is 'root'"),
    ("G9.2", "24 bones under goblin, names + parents = canonical, unexpected nodes 0, lossyScale 1 +-1e-4",
     "report hierarchy by name ('(Clone)' stripped); parent of root must be a goblin container; unexpected = "
     "report unexpected_nodes + own scan (names not in canonical / goblin / goblin_mesh, goblin_club subtree "
     "listed apart); max |lossyScale - 1| over every hierarchy entry"),
    ("G9.3", "rest root->head = Unity +Y <= 1 deg, hand_r->hand_l = Unity -X <= 1 deg",
     "report rest[bone].pos differences, angle to (0,1,0) / (-1,0,0); also M-mapped Blender rest vectors and "
     "rest position error vs M p_b (report only)"),
    ("G9.4", "24 bones at integer samples: pos <= 1 mm, rot <= 1 deg",
     "samples matched by frame; pos = |p_u - M p_b|; rot = quat angle(dR_u, M dR_b M^T), dR = R R_rest^T; "
     "max / p95 / worst (frame, bone)"),
    ("G9.5", "club world at integer samples: pos <= 1 mm, rot <= 1 deg",
     "club pos vs M p_b(club); rot rest-relative with club rest (Blender club_rest; Unity report club rest if "
     "present, else R_socket_rest_u @ C_u(first sample), C_u = R_socket_u^T R_club_u); C_u / C_b constancy reported"),
    ("G9.6", "silhouette contour max deviation <= 3 px (1024 px height), front + three_quarter, 3 frames; both masks "
             "closed (disk radius 2 px) and holes < 500 px filled before comparing",
     "mask = alpha > 0.5 (goblib.silhouette_mask) when the PNG has transparency, else pixels outside the "
     "border-connected background (|rgb - corner median| <= 0.1); goblib.contour_max_dev_px (symmetric Hausdorff "
     "of contour pixels); before the contour step, background components (4-connected) not touching the border "
     "and < 500 px are filled in both masks (holes_blender / holes_unity: filled / kept counts and sizes); "
     "order: closing (disk r = 2 px: dilation then erosion) -> hole fill -> contour"),
    ("G9.7", "[U] side-by-side sheets written: silhouette (Blender | Unity | overlay) and lit (Blender lit | Unity lit), "
             "per cam x frame",
     "rig/inspect/G9/side_by_side.png + side_by_side_lit.png (blender_lit_ / unity_lit_<cam>_<frame>.png, light in "
     "compare_cams lit_light, Unity extra.render_lit reported); ok = both sheets written and no lit PNG missing; "
     "quality = user"),
    ("G9.8", "Unity SampleAnimation at N + 0.25 / 0.5 / 0.75 vs linear reference (Blender integer frames N, N+1, "
             "parent-local lerp + slerp, world chain recomposed), all bones x all subsamples: position p95 <= 2 mm, "
             "rotation p95 <= 1 deg (quaternion angular distance), position max <= 20 mm, rotation max <= 15 deg; "
             "Blender Bezier reference report only (spec G9.8, fixed after measurement 2026-09-25)",
     "reference (b) = Blender integer frames f / f+1, per bone parent-local position lerp + rotation slerp, world "
     "chain recomposed; pos err = |p_u - M p_ref|, rot err = quat angle(dR_u, M dR_ref M^T) (rest-relative); "
     "p95 = numpy percentile 95 over all 72 x 24 errors; ok = all four checks. Report only: step_scaled_report "
     "(allowance 0.25 x Blender world step f -> f+1 + 1 mm / 1 deg, violations, worst margin = err / allowance), "
     "bezier_reference "
     "(G9.4-style vs Blender Bezier) and interp_diagnosis: Unity subsamples vs (a) Blender Bezier, (b) linear "
     "between Blender integer frames f / f+1 (local position lerp + local rotation slerp per bone, recomposed down "
     "the hierarchy), (c) step = Blender frame f; best_reference = smallest max pos error (rot max as tie-break); "
     "worst frames with segment / nearby events / f->f+1 motion"),
    ("G9.9", "bind matrices (inverse bindposes, mesh frame applied) = canonical rest (axis mapped) <= 1e-4",
     "W = F @ inv(bindpose), F = goblin_mesh world from the report hierarchy (else identity); target "
     "[M R_c M^T K | M head_c], K = one bone-axis convention rotation fitted over all bones (SVD); max abs element "
     "diff, pos mm, rot deg; K vs blender_to_fbx and identity reported"),
    ("G9.10", "0 bounds violation frames (all 583 frames checked)",
     "report bounds: frames_checked, per_frame_outside (frames with outside > 0), max_outside_m"),
    ("G9.11", f"goblin.fbx SkinnedMeshRenderer: blend shape '{MOUTH_SHAPE}' present; subMeshCount = {N_SUBMESH} and "
              f"{N_SUBMESH} materials",
     "2026-09-26 HR2 (rig/data/hr2_contract.md). report mesh_contract (GoblinRigCheck: goblin.fbx instance "
     "SkinnedMeshRenderer sharedMesh blend shapes and subMeshCount, sharedMaterials names); material names vs "
     "the contract names, blend-shape frame counts and max delta reported only"),
)
ALL_IDS = [i for i, _t, _n in IDS]

_STATE = {"missing": []}


class Blocked(Exception):
    """A criterion cannot be measured from the available data."""


# ---------------------------------------------------------------- helpers
def mm(x):
    x = float(x)
    return round(x * 1000.0, 4) if math.isfinite(x) else str(x)


def rnd(x, n=5):
    x = float(x)
    return round(x, n) if math.isfinite(x) else str(x)


def rel(p):
    return goblib._rel(p)


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def norm_name(s):
    return re.sub(r"\s*\(Clone\)$", "", str(s)).strip()


def norm_path(p):
    return "/".join(norm_name(x) for x in str(p).split("/"))


def qu(xyzw):
    x, y, z, w = (float(v) for v in xyzw)
    return Quaternion((w, x, y, z)).normalized()


def rot_u(e):
    return qu(e["rot"]).to_matrix()


def rot_b(e):
    if "rotm" in e:
        return Matrix(e["rotm"])
    return qu(e["rot"]).to_matrix()


def ang(Ra, Rb):
    return goblib.quat_angle_deg(Ra.to_quaternion(), Rb.to_quaternion())


def vang(a, b):
    a, b = Vector(a), Vector(b)
    if a.length < 1e-12 or b.length < 1e-12:
        return float("nan")
    return math.degrees(a.angle(b))


def mat_rows(Mx, n=6):
    return [[rnd(x, n) for x in row] for row in Mx]


def stats(rows, scale=1.0, n=4):
    """rows = [(frame, bone, value)] -> max / p95 / mean / worst."""
    if not rows:
        return {"n": 0}
    v = np.array([r[2] for r in rows], dtype=np.float64)
    i = int(np.argmax(v))
    return {"n": len(rows), "max": round(float(v.max()) * scale, n), "p95": round(float(np.percentile(v, 95)) * scale, n),
            "mean": round(float(v.mean()) * scale, n),
            "worst": {"frame": rows[i][0], "bone": rows[i][1]}}


def find_sample(usamples, frame):
    best, bd = None, None
    for s in usamples:
        d = abs(float(s.get("frame", -1e9)) - frame)
        if bd is None or d < bd:
            best, bd = s, d
    return best if bd is not None and bd <= FRAME_MATCH else None


# ---------------------------------------------------------------- G9.1
def flatten(d, pre=""):
    out = {}
    if isinstance(d, dict):
        for k, v in d.items():
            out.update(flatten(v, f"{pre}.{k}" if pre else str(k)))
    else:
        out[pre] = d
    return out


def field(flat, cands):
    cl = {c.lower().replace("_", "") for c in cands}
    for k, v in flat.items():
        if k.split(".")[-1].lower().replace("_", "") in cl:
            return k, v
    return None, None


def enum_norm(v, table):
    if isinstance(v, bool) or v is None:
        return str(v).lower()
    if isinstance(v, (int, float)):
        return table.get(int(v), str(v))
    s = str(v).strip()
    if re.fullmatch(r"-?\d+", s):
        return table.get(int(s), s)
    return re.sub(r"[\s_]", "", s).lower()


def importer_entry(importers, fname):
    for k, v in importers.items():
        if Path(str(k).replace("\\", "/")).name.lower() == fname.lower():
            return k, v
    return None, None


def c_g91(rep):
    imps = rep.get("importers")
    if not isinstance(imps, dict):
        raise Blocked("report has no 'importers' object")
    spec = {
        "goblin.fbx": [("animationType", ("animationType", "animation_type"), ANIM_TYPE, "generic"),
                       ("avatarSetup", ("avatarSetup", "avatar_setup"), AVATAR_SETUP, "createfromthismodel"),
                       ("motionNodeName", ("motionNodeName", "motion_node_name", "rootNode", "root_node"), None,
                        "root")],
        "goblin@rigtest.fbx": [("animationType", ("animationType", "animation_type"), ANIM_TYPE, "generic"),
                               ("avatarSetup", ("avatarSetup", "avatar_setup"), AVATAR_SETUP, "copyfromother"),
                               ("sourceAvatar", ("sourceAvatar", "source_avatar", "sourceAvatarName",
                                                 "sourceAvatarPath"), None, "goblin avatar"),
                               ("animationCompression", ("animationCompression", "animation_compression",
                                                         "compression"), COMPRESSION, "off")],
        "goblin_club.fbx": [("animationType", ("animationType", "animation_type"), ANIM_TYPE, "none")],
    }
    out, ok = {}, True
    for fname, items in spec.items():
        key, entry = importer_entry(imps, fname)
        if entry is None:
            out[fname] = "importer entry missing"
            ok = False
            continue
        flat = flatten(entry)
        res = {"report_key": key}
        for label, cands, table, want in items:
            k, v = field(flat, cands)
            if label == "sourceAvatar" and isinstance(entry, dict):
                dk = next((x for x in entry if x.lower().replace("_", "") in
                           {c.lower().replace("_", "") for c in cands}), None)
                if dk is not None and isinstance(entry[dk], dict):
                    sv = entry[dk]
                    nm = str(sv.get("name", ""))
                    ap = str(sv.get("assetPath", sv.get("path", ""))).replace("\\", "/")
                    good = nm == "goblinAvatar" and Path(ap).name.lower() == "goblin.fbx"
                    res[label] = {"field": dk, "raw": sv, "value": {"name": nm, "assetPath": ap},
                                  "expected": "name goblinAvatar, assetPath .../goblin.fbx", "ok": good}
                    ok = ok and good
                    continue
            if k is None:
                res[label] = {"found": False, "expected": want, "ok": False}
                ok = False
                continue
            if label == "motionNodeName":
                got = str(v).replace("\\", "/").strip().split("/")[-1].lower()
                good = got == "root"
            elif label == "sourceAvatar":
                s = str(v).lower()
                got = str(v)
                good = ("goblin" in s) and ("rigtest" not in s) and ("none" != s.strip()) and s.strip() != ""
            else:
                got = enum_norm(v, table)
                good = got == want
            res[label] = {"field": k, "raw": v, "value": got, "expected": want, "ok": good}
            ok = ok and good
        out[fname] = res
    out["unity_version"] = rep.get("unity_version")
    out["unity_errors"] = rep.get("errors", [])
    return out, ok


# ---------------------------------------------------------------- G9.2
def hierarchy_index(rep):
    h = rep.get("hierarchy")
    if not isinstance(h, list) or not h:
        raise Blocked("report has no 'hierarchy' list")
    ents = []
    for e in h:
        path = norm_path(e.get("path", e.get("name", "")))
        name = norm_name(e.get("name", path.split("/")[-1]))
        par = e.get("parent")
        par = norm_name(par) if par not in (None, "") else (path.split("/")[-2] if "/" in path else None)
        ents.append({"path": path, "name": name, "parent": par, "raw": e})
    return ents


def c_g92(rep, names, parent):
    ents = hierarchy_index(rep)
    by_name = {}
    for e in ents:
        by_name.setdefault(e["name"], []).append(e)
    missing, dup, pmis = [], [], []
    for n in names:
        lst = by_name.get(n, [])
        if not lst:
            missing.append(n)
            continue
        if len(lst) > 1:
            dup.append({n: [x["path"] for x in lst]})
        e = lst[0]
        want = parent[n]
        if want is None:
            good = e["parent"] in CONTAINER_NAMES[:1]
        else:
            good = e["parent"] == want
        if not good:
            pmis.append({"bone": n, "parent": e["parent"], "expected": want or "goblin", "path": e["path"]})
    root_path = by_name["root"][0]["path"] if by_name.get("root") else None
    under_goblin = bool(root_path) and "goblin" in root_path.split("/")[:-1]
    club_nodes, own_unexp = [], []
    for e in ents:
        if CLUB_NODE in e["path"].split("/"):
            club_nodes.append(e["path"])
        elif e["name"] not in names and e["name"] not in CONTAINER_NAMES:
            own_unexp.append(e["path"])
    rep_unexp = rep.get("unexpected_nodes", None)
    worst, worst_path = 0.0, None
    no_scale = []
    for e in ents:
        s = e["raw"].get("lossyScale")
        if s is None:
            no_scale.append(e["path"])
            continue
        d = max(abs(float(x) - 1.0) for x in s)
        if d > worst:
            worst, worst_path = d, e["path"]
    n_found = sum(1 for n in names if by_name.get(n))
    m = {"bones_found": n_found, "expected": N_BONES, "missing": missing, "duplicates": dup,
         "parent_mismatch": pmis, "root_path": root_path, "root_under_goblin": under_goblin,
         "report_unexpected_nodes": rep_unexp, "own_scan_unexpected": own_unexp, "club_subtree_nodes": club_nodes,
         "hierarchy_entries": len(ents), "lossyScale_max_dev": sci(worst), "lossyScale_worst_path": worst_path,
         "entries_without_lossyScale": no_scale}
    ok = (n_found == N_BONES and not missing and not pmis and not dup and under_goblin
          and isinstance(rep_unexp, list) and len(rep_unexp) == 0 and not own_unexp
          and worst <= SCALE_TOL and not no_scale)
    return m, ok


def sci(x):
    x = float(x)
    return float(f"{x:.3e}") if math.isfinite(x) else str(x)


# ---------------------------------------------------------------- G9.3
def c_g93(rep, dump, M):
    ur = rep.get("rest")
    if not isinstance(ur, dict):
        raise Blocked("report has no 'rest' object")
    for n in ("root", "head", "hand_r", "hand_l"):
        if n not in ur:
            raise Blocked(f"report rest has no '{n}'")
    br = dump["rest"]
    p = {n: Vector(ur[n]["pos"]) for n in ("root", "head", "hand_r", "hand_l")}
    v_up = p["head"] - p["root"]
    v_lr = p["hand_l"] - p["hand_r"]
    a_up = vang(v_up, (0, 1, 0))
    a_lr = vang(v_lr, (-1, 0, 0))
    bp = {n: M @ Vector(br[n]["pos"]) for n in ("root", "head", "hand_r", "hand_l")}
    bv_up, bv_lr = bp["head"] - bp["root"], bp["hand_l"] - bp["hand_r"]
    rest_err = []
    for n in dump["bones"]:
        if n in ur:
            rest_err.append((None, n, (Vector(ur[n]["pos"]) - M @ Vector(br[n]["pos"])).length))
    conv = {}
    for n in dump["bones"]:
        if n in ur and "rot" in ur[n]:
            conv[n] = (M @ rot_b(br[n]) @ M.transposed()).transposed() @ rot_u(ur[n])
    conv_m = {}
    if conv:
        k0 = conv.get("root", next(iter(conv.values())))
        conv_m = {"K_root_rows": mat_rows(k0, 4),
                  "K_root_angle_deg": rnd(math.degrees(k0.to_quaternion().angle), 4),
                  "max_angle_K_bone_vs_K_root_deg": rnd(max(ang(k, k0) for k in conv.values()), 4)}
    m = {"root_to_head_unity": [rnd(x, 6) for x in v_up], "angle_to_+Y_deg": rnd(a_up, 4),
         "hand_r_to_hand_l_unity": [rnd(x, 6) for x in v_lr], "angle_to_-X_deg": rnd(a_lr, 4),
         "blender_mapped_root_to_head": [rnd(x, 6) for x in bv_up],
         "blender_mapped_hand_r_to_hand_l": [rnd(x, 6) for x in bv_lr],
         "angle_unity_vs_mapped_up_deg": rnd(vang(v_up, bv_up), 4),
         "angle_unity_vs_mapped_lr_deg": rnd(vang(v_lr, bv_lr), 4),
         "rest_pos_err_vs_M_p_b_mm": stats(rest_err, 1000.0),
         "rest_rotation_convention(K = (M R_b M^T)^T R_u)": conv_m,
         "axis_mapping_blender_to_unity": [list(r) for r in M]}
    ok = a_up <= DIR_TOL and a_lr <= DIR_TOL
    return m, ok


# ---------------------------------------------------------------- G9.4 / G9.8
def compare_poses(bsamples, usamples, brest, urest, names, M, fracs=False):
    Mt = M.transposed()
    for n in names:
        if n not in urest:
            raise Blocked(f"report rest has no '{n}'")
    Rb0 = {n: rot_b(brest[n]) for n in names}
    Ru0 = {n: rot_u(urest[n]) for n in names}
    pos_rows, rot_rows = [], []
    per_bone = {n: [0.0, 0.0] for n in names}
    per_frame = {}
    per_frac = {}
    missing_frames, missing_bones = [], set()
    tdiff = 0.0
    for bs in bsamples:
        f = float(bs["frame"])
        us = find_sample(usamples, f)
        if us is None:
            missing_frames.append(f)
            continue
        if "time_s" in us:
            tdiff = max(tdiff, abs(float(us["time_s"]) - (f - 1.0) / 24.0))
        fk = round(f, 3)
        pf = per_frame.setdefault(fk, [0.0, 0.0])
        frac = round(f - math.floor(f), 3)
        pq = per_frac.setdefault(frac, [0.0, 0.0])
        for n in names:
            ub = us.get("bones", {}).get(n)
            if ub is None:
                missing_bones.add(n)
                continue
            bb = bs["bones"][n]
            pe = (Vector(ub["pos"]) - M @ Vector(bb["pos"])).length
            dRb = M @ (rot_b(bb) @ Rb0[n].transposed()) @ Mt
            dRu = rot_u(ub) @ Ru0[n].transposed()
            re_ = ang(dRu, dRb)
            pos_rows.append((fk, n, pe))
            rot_rows.append((fk, n, re_))
            per_bone[n][0] = max(per_bone[n][0], pe)
            per_bone[n][1] = max(per_bone[n][1], re_)
            pf[0], pf[1] = max(pf[0], pe), max(pf[1], re_)
            pq[0], pq[1] = max(pq[0], pe), max(pq[1], re_)
    m = {"n_blender": len(bsamples), "n_matched": len(bsamples) - len(missing_frames),
         "missing_frames": missing_frames, "missing_bones": sorted(missing_bones),
         "time_s_max_diff_vs_(f-1)/24": sci(tdiff),
         "pos_mm": stats(pos_rows, 1000.0), "rot_deg": stats(rot_rows, 1.0),
         "per_bone_max[pos_mm, rot_deg]": {n: [mm(v[0]), rnd(v[1], 4)] for n, v in per_bone.items()},
         "per_frame_max[pos_mm, rot_deg]": {str(k): [mm(v[0]), rnd(v[1], 4)] for k, v in sorted(per_frame.items())}}
    if fracs:
        m["per_fraction_max[pos_mm, rot_deg]"] = {str(k): [mm(v[0]), rnd(v[1], 4)] for k, v in sorted(per_frac.items())}
    pmax = max((r[2] for r in pos_rows), default=float("inf"))
    rmax = max((r[2] for r in rot_rows), default=float("inf"))
    ok = (not missing_frames and not missing_bones and bool(pos_rows) and pmax <= POS_TOL and rmax <= ROT_TOL)
    return m, ok


# ---------------------------------------------------------------- G9.8 interpolation diagnosis
MANIFEST = goblib.DATA / "rigtest_manifest.json"
DIAG_WORST_N = 6


def w4(e):
    W = rot_b(e).to_4x4()
    W.translation = Vector(e["pos"])
    return W


def entry_of(W):
    R = W.to_3x3().normalized()
    return {"pos": list(W.translation), "rotm": [list(r) for r in R]}


def linear_local(A, B, frac, names, parent):
    """Per bone: local (parent-relative) position lerp + rotation slerp between integer frames, recomposed."""
    out = {}
    for n in names:
        Wa, Wb = w4(A[n]), w4(B[n])
        p = parent[n]
        if p:
            La, Lb = w4(A[p]).inverted() @ Wa, w4(B[p]).inverted() @ Wb
        else:
            La, Lb = Wa, Wb
        q = La.to_quaternion().slerp(Lb.to_quaternion(), frac)
        L = q.to_matrix().to_4x4()
        L.translation = La.translation.lerp(Lb.translation, frac)
        out[n] = (w4(out[p]) @ L) if p else L
        out[n] = entry_of(out[n])
    return out


G98_K = 0.25            # step_scaled_report (report only): allowance fraction of the f -> f+1 step
G98_LIMITS = {"pos_p95_mm": 2.0, "rot_p95_deg": 1.0, "pos_max_mm": 20.0, "rot_max_deg": 15.0}   # G9.8 verdict
G98_NOTE = ("main diagnosis of f371 (2026-09-25): frames 366-371 the left-hand IK target (weapon space) is 1.03-1.73 x "
            "the arm length away, so the arm is fully straight (elbow 0.3 deg); at 372 the target comes back to 0.89 x "
            "and the elbow goes to 54 deg, a geometrically correct IK response: a steep transition, not a rig defect.")


def c_g98(dump, rep, names, parent, M):
    nxt = dump.get("next_frames")
    if not nxt:
        raise Blocked("blender_dump has no 'next_frames' (rerun s09_blender_ref)")
    urest = rep.get("rest", {})
    for n in names:
        if n not in urest:
            raise Blocked(f"report rest has no '{n}'")
    states = {int(round(float(s["frame"]))): s["bones"] for s in list(dump["samples"]) + list(nxt)}
    usamples = rep.get("subsamples", [])
    Mt = M.transposed()
    Rb0 = {n: rot_b(dump["rest"][n]) for n in names}
    Ru0 = {n: rot_u(urest[n]) for n in names}
    rows, viol, missing_frames, missing_bones = [], [], [], set()
    per_bone = {n: [0.0, 0.0] for n in names}
    for bs in dump["subsamples"]:
        F = float(bs["frame"])
        f0 = int(math.floor(F + 1e-9))
        frac = F - f0
        if f0 not in states or f0 + 1 not in states:
            raise Blocked(f"integer frames {f0} / {f0 + 1} missing in blender_dump")
        us = find_sample(usamples, F)
        if us is None:
            missing_frames.append(F)
            continue
        A, B = states[f0], states[f0 + 1]
        ref = linear_local(A, B, frac, names, parent)
        fk = round(F, 3)
        for n in names:
            ub = us.get("bones", {}).get(n)
            if ub is None:
                missing_bones.add(n)
                continue
            pe = (Vector(ub["pos"]) - M @ Vector(ref[n]["pos"])).length
            dRr = M @ (rot_b(ref[n]) @ Rb0[n].transposed()) @ Mt
            dRu = rot_u(ub) @ Ru0[n].transposed()
            re_ = ang(dRu, dRr)
            step_p = (M @ Vector(B[n]["pos"]) - M @ Vector(A[n]["pos"])).length
            step_r = ang(M @ rot_b(A[n]) @ Mt, M @ rot_b(B[n]) @ Mt)
            ap = G98_K * step_p + POS_TOL
            ar = G98_K * step_r + ROT_TOL
            mp, mr = pe / ap, re_ / ar
            rows.append((fk, n, pe, re_, ap, ar, mp, mr))
            per_bone[n][0] = max(per_bone[n][0], mp)
            per_bone[n][1] = max(per_bone[n][1], mr)
            if pe > ap or re_ > ar:
                viol.append({"frame": fk, "bone": n, "pos_err_mm": mm(pe), "pos_allow_mm": mm(ap),
                             "rot_err_deg": rnd(re_, 4), "rot_allow_deg": rnd(ar, 4)})
    if not rows:
        raise Blocked("no Unity subsample matched the Blender subsamples")

    def worst(i, j, ai, unit_mm):
        r = max(rows, key=lambda x: x[j])
        f = mm if unit_mm else (lambda v: rnd(v, 4))
        return {"margin": rnd(r[j], 4), "frame": r[0], "bone": r[1], "err": f(r[i]), "allowance": f(r[ai])}

    pos_rows = [(r[0], r[1], r[2]) for r in rows]
    rot_rows = [(r[0], r[1], r[3]) for r in rows]
    pv = np.array([r[2] for r in rows], dtype=np.float64)
    rv = np.array([r[3] for r in rows], dtype=np.float64)
    vals = {"pos_p95_mm": float(np.percentile(pv, 95)) * 1000.0, "rot_p95_deg": float(np.percentile(rv, 95)),
            "pos_max_mm": float(pv.max()) * 1000.0, "rot_max_deg": float(rv.max())}
    checks = {k: {"value": rnd(v, 4), "limit": G98_LIMITS[k], "ok": v <= G98_LIMITS[k]} for k, v in vals.items()}
    m = {"reference": "b_linear_local_lerp_slerp", "n_blender": len(dump["subsamples"]),
         "n_matched": len(dump["subsamples"]) - len(missing_frames), "missing_frames": missing_frames,
         "missing_bones": sorted(missing_bones), "checked": len(rows),
         "checks": checks,
         "pos_err_mm": stats(pos_rows, 1000.0), "rot_err_deg": stats(rot_rows, 1.0),
         "step_scaled_report": {
             "note": "report only (earlier draft criterion, not used for ok)",
             "allowance": f"pos {G98_K} x |M(p(f+1) - p(f))| + {POS_TOL * 1000:g} mm; "
                          f"rot {G98_K} x angle(q(f), q(f+1)) + {ROT_TOL:g} deg",
             "violations_count": len(viol), "violations": viol,
             "worst_margin_pos(err/allow)": worst(2, 6, 4, True),
             "worst_margin_rot(err/allow)": worst(3, 7, 5, False),
             "per_bone_max_margin[pos, rot]": {n: [rnd(v[0], 4), rnd(v[1], 4)] for n, v in per_bone.items()}},
         "note_f371": G98_NOTE}
    ok = not missing_frames and not missing_bones and all(c["ok"] for c in checks.values())
    return m, ok


def interp_diagnosis(dump, rep, names, parent, M):
    nxt = dump.get("next_frames")
    if not nxt:
        raise Blocked("blender_dump has no 'next_frames' (rerun s09_blender_ref)")
    states = {int(round(float(s["frame"]))): s["bones"] for s in list(dump["samples"]) + list(nxt)}
    ref_b, ref_c = [], []
    for bs in dump["subsamples"]:
        F = float(bs["frame"])
        f0 = int(math.floor(F + 1e-9))
        frac = F - f0
        if f0 not in states or f0 + 1 not in states:
            raise Blocked(f"integer frames {f0} / {f0 + 1} missing in blender_dump")
        ref_b.append({"frame": F, "bones": linear_local(states[f0], states[f0 + 1], frac, names, parent)})
        ref_c.append({"frame": F, "bones": states[f0]})
    refs = {"a_blender_bezier": dump["subsamples"], "b_linear_local_lerp_slerp": ref_b, "c_step_prev_frame": ref_c}
    res, per_frame = {}, {}
    for k, lst in refs.items():
        m, _ok = compare_poses(lst, rep.get("subsamples", []), dump["rest"], rep.get("rest", {}), names, M,
                               fracs=True)
        res[k] = {"pos_mm": {x: m["pos_mm"].get(x) for x in ("max", "p95", "mean", "worst")},
                  "rot_deg": {x: m["rot_deg"].get(x) for x in ("max", "p95", "mean", "worst")},
                  "per_fraction_max[pos_mm, rot_deg]": m.get("per_fraction_max[pos_mm, rot_deg]")}
        per_frame[k] = m["per_frame_max[pos_mm, rot_deg]"]
    ranked = sorted(res, key=lambda k: (res[k]["pos_mm"]["max"], res[k]["rot_deg"]["max"]))
    best_by = {f"{q}_{s}": min(res, key=lambda k: res[k][q][s]) for q in ("pos_mm", "rot_deg") for s in ("max", "p95")}

    man = load(MANIFEST)
    segs, evs = man["segments"], man.get("events", [])
    pa = per_frame["a_blender_bezier"]
    top = sorted(pa, key=lambda k: (-pa[k][0], -pa[k][1]))[:DIAG_WORST_N]
    top_rot = sorted(pa, key=lambda k: -pa[k][1])[:3]
    worst = []
    for fk in list(dict.fromkeys(top + top_rot)):
        F = float(fk)
        f0 = int(math.floor(F + 1e-9))
        seg = [s for s in segs if s["start"] <= f0 < s["end"]] or [s for s in segs if s["start"] <= f0 <= s["end"]]
        near = [f"{e['frame']} {e['kind']} {e.get('args', {})}" for e in evs if f0 - 3 <= e["frame"] <= f0 + 4]
        A, B = states[f0], states[f0 + 1]
        dp = max(((Vector(B[n]["pos"]) - Vector(A[n]["pos"])).length, n) for n in names)
        dr = max((ang(rot_b(A[n]), rot_b(B[n])), n) for n in names)
        worst.append({"frame": F,
                      "err_by_ref[pos_mm, rot_deg]": {k: per_frame[k].get(fk) for k in refs},
                      "segment": ({"name": seg[0]["name"], "start": seg[0]["start"], "end": seg[0]["end"],
                                   "covers": seg[0].get("covers")} if seg else None),
                      "events_within_-3..+4": near,
                      "blender_step_f_to_f+1": {"max_bone_move_mm": [mm(dp[0]), dp[1]],
                                                "max_bone_rot_deg": [rnd(dr[0], 3), dr[1]]}})
    return {"references": res, "best_reference": ranked[0], "ranking_by_pos_max": ranked,
            "best_by_metric": best_by, "worst_frames(ref a)": worst,
            "note": "report only; G9.8 ok is judged against (b) with the step-scaled allowance (c_g98)"}


# ---------------------------------------------------------------- G9.5
def unity_club_rest(rep):
    for k in ("club_rest",):
        if isinstance(rep.get(k), dict) and "rot" in rep[k]:
            return rep[k], f"report '{k}'"
    r = rep.get("rest", {})
    for k in ("goblin_club", "club"):
        if isinstance(r.get(k), dict) and "rot" in r[k]:
            return r[k], f"report rest['{k}']"
    return None, None


def c_g95(rep, dump, M):
    Mt = M.transposed()
    urest = rep.get("rest", {})
    if "weapon_socket_r" not in urest:
        raise Blocked("report rest has no 'weapon_socket_r'")
    Rs_u0 = rot_u(urest["weapon_socket_r"])
    Rc_b0 = rot_b(dump["club_rest"])
    usamples = rep.get("samples", [])
    pairs = []
    for bs in dump["samples"]:
        us = find_sample(usamples, float(bs["frame"]))
        if us is None or not isinstance(us.get("club"), dict):
            continue
        pairs.append((bs, us))
    if not pairs:
        raise Blocked("no report sample with a 'club' entry matched the Blender samples")
    cu_rest, src = unity_club_rest(rep)
    Cu = [rot_u(us["bones"]["weapon_socket_r"]).transposed() @ rot_u(us["club"]) for _bs, us in pairs
          if "weapon_socket_r" in us.get("bones", {})]
    Cb = [rot_b(bs["bones"]["weapon_socket_r"]).transposed() @ rot_b(bs["club"]) for bs, _us in pairs]
    if cu_rest is not None:
        Rc_u0 = rot_u(cu_rest)
    else:
        if not Cu:
            raise Blocked("no Unity club rest and no weapon_socket_r in samples to derive it")
        Rc_u0 = Rs_u0 @ Cu[0]
        src = "derived: R_socket_rest_u @ C_u(first matched sample)"
    pos_rows, rot_rows = [], []
    for bs, us in pairs:
        f = round(float(bs["frame"]), 3)
        pe = (Vector(us["club"]["pos"]) - M @ Vector(bs["club"]["pos"])).length
        dRb = M @ (rot_b(bs["club"]) @ Rc_b0.transposed()) @ Mt
        dRu = rot_u(us["club"]) @ Rc_u0.transposed()
        pos_rows.append((f, "club", pe))
        rot_rows.append((f, "club", ang(dRu, dRb)))
    I3 = Matrix.Identity(3)
    m = {"n_blender": len(dump["samples"]), "n_matched": len(pairs),
         "pos_mm": stats(pos_rows, 1000.0), "rot_deg": stats(rot_rows, 1.0),
         "unity_club_rest_source": src,
         "C_u(club local rot under socket, Unity)": {
             "max_angle_from_identity_deg": rnd(max((ang(c, I3) for c in Cu), default=float("nan")), 4),
             "max_spread_vs_first_deg": rnd(max((ang(c, Cu[0]) for c in Cu), default=float("nan")), 4)},
         "C_b(club rot rel. socket, Blender)": {
             "angle_from_identity_deg": rnd(ang(Cb[0], I3), 4),
             "max_spread_vs_first_deg": rnd(max(ang(c, Cb[0]) for c in Cb), 4)},
         "club_rest_pos_unity_vs_M_p_b_mm": (mm((Vector(cu_rest["pos"]) - M @ Vector(dump["club_rest"]["pos"])).length)
                                             if cu_rest is not None and "pos" in cu_rest else None)}
    ok = (len(pairs) == len(dump["samples"]) and max(r[2] for r in pos_rows) <= POS_TOL
          and max(r[2] for r in rot_rows) <= ROT_TOL)
    return m, ok


# ---------------------------------------------------------------- images
def load_rgba(png):
    img = bpy.data.images.load(str(Path(png).resolve()), check_existing=False)
    try:
        w, h = img.size
        ch = img.channels
        buf = np.empty(w * h * ch, dtype=np.float32)
        img.pixels.foreach_get(buf)
    finally:
        bpy.data.images.remove(img)
    px = buf.reshape(h, w, ch)[::-1]
    if ch == 4:
        return np.ascontiguousarray(px)
    out = np.ones((h, w, 4), dtype=np.float32)
    out[:, :, :min(ch, 3)] = px[:, :, :min(ch, 3)]
    if ch == 1:
        out[:, :, 1] = out[:, :, 2] = px[:, :, 0]
    return out


def dilate8(m):
    p = np.pad(m, 1, constant_values=False)
    o = m.copy()
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            o |= p[dy:dy + m.shape[0], dx:dx + m.shape[1]]
    return o


def mask_of(png, px):
    a = px[:, :, 3]
    if float(a.min()) < 0.5:
        return goblib.silhouette_mask(png), "alpha>0.5"
    rgb = px[:, :, :3]
    corners = np.stack([rgb[0, 0], rgb[0, -1], rgb[-1, 0], rgb[-1, -1]])
    bg = np.median(corners, axis=0)
    bgish = np.abs(rgb - bg).max(axis=2) <= BG_TOL
    reach = np.zeros_like(bgish)
    reach[0, :], reach[-1, :], reach[:, 0], reach[:, -1] = bgish[0, :], bgish[-1, :], bgish[:, 0], bgish[:, -1]
    for _ in range(20000):
        nxt = bgish & dilate8(reach)
        if np.array_equal(nxt, reach):
            break
        reach = nxt
    return ~reach, f"background-difference (bg={[round(float(x), 3) for x in bg]}, tol={BG_TOL}, holes filled)"


def close_disk(mask, r=CLOSE_RADIUS):
    """Binary closing with a disk (dy^2 + dx^2 <= r^2; r = 2 -> 13 px inside a 5 x 5 square).
    Dilation pads with background, erosion pads with foreground (the image border does not erode)."""
    m = np.asarray(mask, dtype=bool)
    offs = [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1) if dy * dy + dx * dx <= r * r]
    h, w = m.shape
    p = np.pad(m, r, constant_values=False)
    dil = np.zeros_like(m)
    for dy, dx in offs:
        dil |= p[r + dy:r + dy + h, r + dx:r + dx + w]
    p = np.pad(dil, r, constant_values=True)
    ero = np.ones_like(m)
    for dy, dx in offs:
        ero &= p[r + dy:r + dy + h, r + dx:r + dx + w]
    return ero


def fill_small_holes(mask, max_px=HOLE_MAX_PX):
    """Fill background components (4-connected) that do not touch the image border and have < max_px pixels.
    Returns (filled mask, {filled_count, filled_sizes, kept_count, kept_sizes})."""
    m = np.asarray(mask, dtype=bool)
    bg = ~m
    reach = np.zeros_like(bg)
    reach[0, :], reach[-1, :], reach[:, 0], reach[:, -1] = bg[0, :], bg[-1, :], bg[:, 0], bg[:, -1]
    for _ in range(20000):
        p = np.pad(reach, 1, constant_values=False)
        nxt = bg & (reach | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:])
        if np.array_equal(nxt, reach):
            break
        reach = nxt
    holes = bg & ~reach
    out = m.copy()
    filled, kept = [], []
    if holes.any():
        h, w = holes.shape
        seen = np.zeros_like(holes)
        for y0, x0 in np.argwhere(holes):
            if seen[y0, x0]:
                continue
            comp, stack = [], [(int(y0), int(x0))]
            seen[y0, x0] = True
            while stack:
                y, x = stack.pop()
                comp.append((y, x))
                for yy, xx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                    if 0 <= yy < h and 0 <= xx < w and holes[yy, xx] and not seen[yy, xx]:
                        seen[yy, xx] = True
                        stack.append((yy, xx))
            if len(comp) < max_px:
                ys, xs = zip(*comp)
                out[list(ys), list(xs)] = True
                filled.append(len(comp))
            else:
                kept.append(len(comp))
    return out, {"filled_count": len(filled), "filled_sizes": sorted(filled, reverse=True)[:30],
                 "kept_count": len(kept), "kept_sizes": sorted(kept, reverse=True)[:30]}


def resize_nearest(m, shape):
    h, w = shape
    yi = (np.arange(h) * m.shape[0] / h).astype(int)
    xi = (np.arange(w) * m.shape[1] / w).astype(int)
    return m[yi][:, xi]


def find_unity_png(rep, cam, frame, prefix="unity"):
    pat = re.compile(rf"^{prefix}_{re.escape(cam)}_0*{int(frame)}\.png$", re.IGNORECASE)
    cands = []
    for p in rep.get("renders", []) or []:
        p = str(p).replace("\\", "/")
        if not pat.match(Path(p).name):
            continue
        cands += [REPO / "work" / "goblin_swing" / p, REPO / p, goblib.RIG / p, Path(p)]
    cands.append(G9DIR / f"{prefix}_{cam}_{int(frame)}.png")
    for c in cands:
        try:
            if c.exists():
                return c
        except OSError:
            pass
    for c in G9DIR.glob(f"{prefix}_{cam}_*.png"):
        if pat.match(c.name):
            return c
    return None


def c_g96_g97(rep, cams, ev, miss):
    frames = [int(f) for f in cams["frames"]]
    views = list(cams["cameras"].keys())
    pairs, lost = [], []
    for v in views:
        for f in frames:
            bp = G9DIR / f"blender_{v}_{f}.png"
            up = find_unity_png(rep, v, f)
            if not bp.exists():
                lost.append(rel(bp))
            if up is None:
                lost.append(rel(G9DIR / f"unity_{v}_{f}.png"))
            if bp.exists() and up is not None:
                pairs.append((v, f, bp, up))
    miss.extend(lost)
    if lost:
        raise Blocked("render(s) missing: " + ", ".join(lost))
    rows, cells, worst = {}, [], 0.0
    for v, f, bp, up in pairs:
        ev.add_input(bp)
        ev.add_input(up)
        ev.render(bp)
        ev.render(up)
        bpx, upx = load_rgba(bp), load_rgba(up)
        mb, mb_how = mask_of(bp, bpx)
        mu, mu_how = mask_of(up, upx)
        note = None
        if mu.shape != mb.shape:
            note = f"unity {mu.shape} resized (nearest) to blender {mb.shape}"
            mu = resize_nearest(mu, mb.shape)
            upx = np.stack([resize_nearest(upx[:, :, c], mb.shape) for c in range(4)], axis=2)
        mb, mu = close_disk(mb), close_disk(mu)
        mb, hb = fill_small_holes(mb)
        mu, hu = fill_small_holes(mu)
        d = goblib.contour_max_dev_px(mb, mu)
        inter = float(np.logical_and(mb, mu).sum())
        union = float(np.logical_or(mb, mu).sum())
        cb = np.argwhere(mb).mean(axis=0) if mb.any() else np.array([np.nan, np.nan])
        cu = np.argwhere(mu).mean(axis=0) if mu.any() else np.array([np.nan, np.nan])
        rows[f"{v}_{f}"] = {"contour_max_dev_px": rnd(d, 3), "iou": rnd(inter / union if union else float("nan"), 5),
                            "px_blender": int(mb.sum()), "px_unity": int(mu.sum()),
                            "centroid_shift_px[dy,dx]": [rnd(x, 3) for x in (cu - cb)],
                            "shape": list(mb.shape), "mask_blender": mb_how, "mask_unity": mu_how,
                            "closing": {"applied": True, "disk_radius_px": CLOSE_RADIUS},
                            "holes_blender": hb, "holes_unity": hu,
                            "unity_png": rel(up), **({"note": note} if note else {})}
        worst = max(worst, d if math.isfinite(d) else float("inf"))
        cells.append((bpx, upx, mb, mu))
    m6 = {"max_px": rnd(worst, 3), "pairs": rows, "frames": frames, "cameras": views}
    ok6 = len(pairs) == len(frames) * len(views) and worst <= SIL_TOL
    if SHEET.exists():
        SHEET.unlink()
    write_sheet(cells)
    ev.render(SHEET)
    m7 = {"sheet": rel(SHEET), "layout": "rows = " + ", ".join(f"{v}_{f}" for v, f, _b, _u in pairs)
          + "; columns = Blender | Unity | overlay (red = Blender only, blue = Unity only, gray = both)",
          "exists": SHEET.exists()}
    lit = lit_sheet(rep, cams, ev, views, frames)
    m7["lit"] = lit
    return (m6, ok6), (m7, SHEET.exists() and lit["exists"] and not lit["missing"])


def lit_sheet(rep, cams, ev, views, frames):
    """Blender lit | Unity lit per cam x frame; a missing PNG becomes a gray cell and is listed."""
    rows, missing, order = [], [], []
    for v in views:
        for f in frames:
            bp = G9DIR / f"blender_lit_{v}_{f}.png"
            up = find_unity_png(rep, v, f, prefix="unity_lit")
            row = []
            for p, label in ((bp if bp.exists() else None, rel(bp)),
                             (up, rel(G9DIR / f"unity_lit_{v}_{f}.png"))):
                if p is None:
                    missing.append(label)
                    row.append(None)
                else:
                    ev.add_input(p)
                    ev.render(p)
                    row.append(load_rgba(p))
            rows.append(row)
            order.append(f"{v}_{f}")
    if LIT_SHEET.exists():
        LIT_SHEET.unlink()
    write_grid(rows, LIT_SHEET)
    ev.render(LIT_SHEET)
    return {"sheet": rel(LIT_SHEET), "exists": LIT_SHEET.exists(), "missing": missing,
            "layout": "rows = " + ", ".join(order) + "; columns = Blender lit | Unity lit (missing = gray cell)",
            "blender_light": cams.get("lit_light"), "unity_render_lit": (rep.get("extra") or {}).get("render_lit")}


def write_grid(rows, path, cell=512, gap=4):
    def down(px):
        h, w = px.shape[:2]
        s = max(1, h // cell)
        rgb = px[:h - h % s, :w - w % s, :3]
        a = px[:h - h % s, :w - w % s, 3:4]
        rgb = rgb * a + (1.0 - a)  # over white
        return rgb.reshape(rgb.shape[0] // s, s, rgb.shape[1] // s, s, 3).mean(axis=(1, 3))

    tiles = [[down(t) if t is not None else None for t in row] for row in rows]
    real = [t for row in tiles for t in row if t is not None]
    th = max((t.shape[0] for t in real), default=cell)
    tw = max((t.shape[1] for t in real), default=cell)
    ncol = max(len(r) for r in tiles)
    H = len(tiles) * th + (len(tiles) + 1) * gap
    W = ncol * tw + (ncol + 1) * gap
    sheet = np.full((H, W, 3), 0.25, dtype=np.float32)
    for r, row in enumerate(tiles):
        for c, t in enumerate(row):
            y = gap + r * (th + gap)
            x = gap + c * (tw + gap)
            if t is None:
                sheet[y:y + th, x:x + tw] = 0.5
            else:
                sheet[y:y + t.shape[0], x:x + t.shape[1]] = t
    rgba = np.concatenate([sheet, np.ones((H, W, 1), dtype=np.float32)], axis=2)[::-1]
    img = bpy.data.images.new("_g9_grid", W, H, alpha=False)
    try:
        img.pixels.foreach_set(np.ascontiguousarray(rgba).ravel())
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        img.filepath_raw = str(path)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)


def write_sheet(cells, cell=512, gap=4):
    def down(px):
        h, w = px.shape[:2]
        s = max(1, h // cell)
        rgb = px[:h - h % s, :w - w % s, :3]
        a = px[:h - h % s, :w - w % s, 3:4]
        rgb = rgb * a + (1.0 - a)  # over white
        return rgb.reshape(rgb.shape[0] // s, s, rgb.shape[1] // s, s, 3).mean(axis=(1, 3))

    tiles = []
    for bpx, upx, mb, mu in cells:
        ov = np.ones(mb.shape + (3,), dtype=np.float32)
        ov[mb & mu] = (0.6, 0.6, 0.6)
        ov[mb & ~mu] = (0.9, 0.1, 0.1)
        ov[mu & ~mb] = (0.1, 0.2, 0.9)
        ova = np.concatenate([ov, np.ones(mb.shape + (1,), dtype=np.float32)], axis=2)
        tiles.append([down(bpx), down(upx), down(ova)])
    th = max(t.shape[0] for row in tiles for t in row)
    tw = max(t.shape[1] for row in tiles for t in row)
    H = len(tiles) * th + (len(tiles) + 1) * gap
    W = 3 * tw + 4 * gap
    sheet = np.full((H, W, 3), 0.25, dtype=np.float32)
    for r, row in enumerate(tiles):
        for c, t in enumerate(row):
            y = gap + r * (th + gap)
            x = gap + c * (tw + gap)
            sheet[y:y + t.shape[0], x:x + t.shape[1]] = t
    rgba = np.concatenate([sheet, np.ones((H, W, 1), dtype=np.float32)], axis=2)[::-1]
    img = bpy.data.images.new("_g9_sheet", W, H, alpha=False)
    try:
        img.pixels.foreach_set(np.ascontiguousarray(rgba).ravel())
        SHEET.parent.mkdir(parents=True, exist_ok=True)
        img.filepath_raw = str(SHEET)
        img.file_format = "PNG"
        img.save()
    finally:
        bpy.data.images.remove(img)


# ---------------------------------------------------------------- G9.9
def trs_u(e):
    T = Matrix.Translation(Vector(e.get("localPosition", (0, 0, 0))))
    R = qu(e.get("localRotation", (0, 0, 0, 1))).to_matrix().to_4x4()
    s = e.get("localScale")
    S = Matrix.Diagonal(Vector(list(s) + [1.0])) if s is not None else Matrix.Identity(4)
    return T @ R @ S


def hierarchy_world(ents):
    by_path = {e["path"]: e for e in ents}
    memo = {}

    def lossy(path):
        e = by_path.get(path)
        return Vector(e["raw"].get("lossyScale", (1, 1, 1))) if e else Vector((1, 1, 1))

    def world(path):
        if path in memo:
            return memo[path]
        e = by_path[path]
        par = path.rsplit("/", 1)[0] if "/" in path else None
        raw = dict(e["raw"])
        if "localScale" not in raw:
            ls, lp = lossy(path), lossy(par) if par in by_path else Vector((1, 1, 1))
            raw["localScale"] = [ls[i] / lp[i] if abs(lp[i]) > 1e-12 else ls[i] for i in range(3)]
        W = trs_u(raw)
        if par in by_path:
            W = world(par) @ W
        memo[path] = W
        return W

    return {p: world(p) for p in by_path}


def bind_sets(rep):
    """Top-level 'bindposes*' (goblin.fbx) and extra 'bindposes*' (extra.bindposes_rigtest = goblin@rigtest.fbx)."""
    sets = {}
    srcs = [("", rep)]
    if isinstance(rep.get("extra"), dict):
        srcs.append(("extra.", rep["extra"]))
    for pre, d in srcs:
        for k, v in d.items():
            if not k.lower().startswith("bindposes") or not isinstance(v, dict) or not v:
                continue
            first = next(iter(v.values()))
            if isinstance(first, dict):
                for fk, fv in v.items():
                    sets[f"{pre}{k}[{fk}]"] = fv
            else:
                sets[f"{pre}{k}"] = v
    return sets


def fit_rotation(mats):
    S = np.zeros((3, 3))
    for Mx in mats:
        S += np.array(Mx)
    U, _s, Vt = np.linalg.svd(S)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return Matrix(R.tolist())


def c_g99(rep, dump, names, M, G):
    sets = bind_sets(rep)
    if not sets:
        raise Blocked("report has no 'bindposes'")
    Mt = M.transposed()
    canon = {n: Matrix(dump["canonical_bind"][n]) for n in names}
    F, F_src = Matrix.Identity(4), "identity (goblin_mesh not in report hierarchy)"
    try:
        ents = hierarchy_index(rep)
        W = hierarchy_world(ents)
        mp = [p for p in W if p.split("/")[-1] == "goblin_mesh"]
        if mp:
            F, F_src = W[mp[0]], f"report hierarchy '{mp[0]}' (composed local TRS)"
    except Blocked:
        pass
    urest = rep.get("rest", {})
    out, ok = {"mesh_frame_F": {"source": F_src, "rows": mat_rows(F)}}, True
    per_set = {}
    for label, bp in sets.items():
        missing = [n for n in names if n not in bp]
        Wb = {}
        for n in names:
            if n in bp:
                Wb[n] = F @ Matrix(bp[n]).inverted_safe()
        Kfit = fit_rotation([(M @ canon[n].to_3x3() @ Mt).transposed() @ Wb[n].to_3x3().normalized() for n in Wb])
        elem, pos, rot, per = [], [], [], {}
        rest_pos, rest_rot = [], []
        for n, Wn in Wb.items():
            Rt = M @ canon[n].to_3x3() @ Mt @ Kfit
            T = Rt.to_4x4()
            T.translation = M @ canon[n].translation
            e = max(abs(Wn[i][j] - T[i][j]) for i in range(4) for j in range(4))
            pe = (Wn.translation - T.translation).length
            re_ = ang(Wn.to_3x3().normalized(), Rt)
            elem.append((None, n, e))
            pos.append((None, n, pe))
            rot.append((None, n, re_))
            per[n] = [sci(e), mm(pe), rnd(re_, 5)]
            if n in urest:
                rest_pos.append((None, n, (Wn.translation - Vector(urest[n]["pos"])).length))
                rest_rot.append((None, n, ang(Wn.to_3x3().normalized(), rot_u(urest[n]))))
        scales = [max(abs(s - 1.0) for s in Wn.to_scale()) for Wn in Wb.values()]
        m = {"bones": len(Wb), "missing": missing,
             "max_abs_elem": stats(elem, 1.0, 8), "pos_mm": stats(pos, 1000.0, 5), "rot_deg": stats(rot, 1.0, 5),
             "K_fit_rows": mat_rows(Kfit, 5),
             "K_angle_vs_identity_deg": rnd(ang(Kfit, Matrix.Identity(3)), 4),
             "K_angle_vs_blender_to_fbx_deg": rnd(ang(Kfit, G), 4),
             "bind_world_scale_max_dev": sci(max(scales) if scales else float("nan")),
             "vs_unity_rest(report rest)": {"pos_mm": stats(rest_pos, 1000.0, 5), "rot_deg": stats(rest_rot, 1.0, 5)},
             "per_bone[max_abs_elem, pos_mm, rot_deg]": per}
        out[label] = m
        ok = ok and not missing and len(Wb) == N_BONES and max(r[2] for r in elem) <= BIND_TOL
        per_set[label] = {"file": "goblin@rigtest.fbx" if "rigtest" in label.lower() else "goblin.fbx",
                          "max_abs_elem": sci(max((r[2] for r in elem), default=float("nan"))),
                          "pos_mm_max": m["pos_mm"].get("max"), "rot_deg_max": m["rot_deg"].get("max"),
                          "bones": len(Wb), "ok": (not missing and len(Wb) == N_BONES
                                                   and max((r[2] for r in elem), default=float("inf")) <= BIND_TOL)}
    names_sets = list(sets.keys())
    files = {v["file"] for v in per_set.values()}
    for fn in ("goblin.fbx", "goblin@rigtest.fbx"):
        if fn not in files:
            per_set[fn] = "missing"
            ok = False
    out["per_set_max"] = per_set
    out["sets"] = names_sets
    out["note"] = ("spec asks goblin.fbx and goblin@rigtest.fbx; sets found: " + ", ".join(names_sets)
                   + "; the same mesh frame F (goblin.fbx instance hierarchy) is applied to both sets")
    return out, ok


# ---------------------------------------------------------------- G9.10
def c_g910(rep):
    b = rep.get("bounds")
    if not isinstance(b, dict):
        raise Blocked("report has no 'bounds' object")
    pfo = b.get("per_frame_outside", []) or []
    viol = [x for x in pfo if float(x[1]) > 0.0]
    mx = max([float(b.get("max_outside_m", 0.0) or 0.0)] + [float(x[1]) for x in viol])
    fc = b.get("frames_checked")
    m = {"frames_checked": fc, "violation_frames": len(viol), "max_outside_mm": mm(mx),
         "worst_frame": b.get("worst_frame"), "first_violations": viol[:20],
         "localBounds": b.get("localBounds"), "rootBone": b.get("rootBone")}
    ok = fc == FRAME_LAST and len(viol) == 0 and mx <= 0.0
    return m, ok


def c_g911(rep):
    mc = rep.get("mesh_contract")
    if not isinstance(mc, dict):
        raise Blocked("report has no 'mesh_contract' object (GoblinRigCheck HR2 section)")
    shapes = list(mc.get("blendShapes") or [])
    mats = list(mc.get("materials") or [])
    m = {"blendShapes": shapes, "blendShapeCount": mc.get("blendShapeCount"),
         "blendShapeFrameCounts": mc.get("blendShapeFrameCounts"), "mouth_open_max_delta_m": mc.get("mouth_open_max_delta_m"),
         "subMeshCount": mc.get("subMeshCount"), "materialCount": mc.get("materialCount"), "materials": mats,
         "materials_equal_contract_names_report": sorted(str(x) for x in mats) == sorted(MAT_NAMES),
         "materials_in_contract_order_report": mats == list(MAT_NAMES),
         "mesh": mc.get("mesh"), "vertexCount": mc.get("vertexCount"), "source": mc.get("source")}
    ok = (MOUTH_SHAPE in shapes and mc.get("subMeshCount") == N_SUBMESH and mc.get("materialCount") == N_SUBMESH
          and len(mats) == N_SUBMESH)
    return m, bool(ok)


# ---------------------------------------------------------------- main
def report_path():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if "--report" in argv:
        return Path(argv[argv.index("--report") + 1]).resolve()
    return REPORT_DEFAULT


def main():
    ev = goblib.Evidence(GATE, CHECKER)
    ev.add_input(__file__)
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

    report = report_path()
    miss = []
    for p in (report, DUMP, PRESET, CANON, CAMS):
        if p.exists():
            ev.add_input(p)
        else:
            miss.append(rel(p))
    ev.add_stage_inputs("s09")
    for p in PROVENANCE:
        if p.exists():
            ev.add_input(p)
    _STATE["missing"] = miss

    try:
        if miss:
            block(ALL_IDS, "missing input: " + ", ".join(miss))
            return
        rep, dump, preset, canon, cams = (load(p) for p in (report, DUMP, PRESET, CANON, CAMS))
        M = Matrix(preset["axis_mapping"]["blender_to_unity"])
        G = Matrix(preset["axis_mapping"]["blender_to_fbx"])
        names = [b["name"] for b in canon["bones"]]
        parent = {b["name"]: b["parent"] for b in canon["bones"]}
        if list(dump.get("bones", [])) != names:
            raise RuntimeError("blender_dump bones differ from canonical_skeleton.json order / names")

        steps = (
            ("G9.1", lambda: c_g91(rep)),
            ("G9.2", lambda: c_g92(rep, names, parent)),
            ("G9.3", lambda: c_g93(rep, dump, M)),
            ("G9.4", lambda: compare_poses(dump["samples"], rep.get("samples", []), dump["rest"],
                                           rep.get("rest", {}), names, M)),
            ("G9.5", lambda: c_g95(rep, dump, M)),
        )
        for cid, fn in steps:
            try:
                m, ok = fn()
                if cid == "G9.4":
                    m["clip"] = rep.get("clip")
                    m["fps"] = rep.get("fps")
                put(cid, m, ok)
            except Blocked as e:
                block([cid], str(e))
        try:
            (m6, ok6), (m7, ok7) = c_g96_g97(rep, cams, ev, miss)
            put("G9.6", m6, ok6)
            put("G9.7", m7, ok7)
        except Blocked as e:
            block(["G9.6", "G9.7"], str(e))
        try:
            m, ok = c_g98(dump, rep, names, parent, M)
            mb, _okb = compare_poses(dump["subsamples"], rep.get("subsamples", []), dump["rest"],
                                     rep.get("rest", {}), names, M, fracs=True)
            m["bezier_reference(report only)"] = mb
            try:
                m["interp_diagnosis"] = interp_diagnosis(dump, rep, names, parent, M)
            except Blocked as e:
                m["interp_diagnosis"] = f"blocked: {e}"
            put("G9.8", m, ok)
        except Blocked as e:
            block(["G9.8"], str(e))
        for cid, fn in (("G9.9", lambda: c_g99(rep, dump, names, M, G)), ("G9.10", lambda: c_g910(rep)),
                        ("G9.11", lambda: c_g911(rep))):
            try:
                put(cid, *fn())
            except Blocked as e:
                block([cid], str(e))
    finally:
        try:
            block(ALL_IDS, "checker aborted before this criterion")
        finally:
            ev.write()


if __name__ == "__main__":
    goblib.run_main(main)
    if _STATE["missing"]:
        print("missing input: " + ", ".join(_STATE["missing"]))
        sys.stdout.flush()
        sys.exit(2)
