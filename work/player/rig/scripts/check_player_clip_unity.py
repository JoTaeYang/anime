"""check_player_clip_unity - Player P3 per-clip FBX + Unity checker, U-rows (task T301; spec
work/player/d-03-player-anim-tools.md section 3 rows U1 .. U4; checker side).  Raw evidence only: main judges the gate.

Arguments (after "--"):
  --clip <name>   clip name (required); defaults below are derived from it
  --prefix <X>    criterion id prefix: ids "<X>.U1" .. ; default plain "U1" ..
  --work          work/player/rig/anim/pl_a_<clip>.blend          PL_rig + action <clip> (--action, --work-arm PL_rig)
  --stage         work/player/rig/export/stage_<clip>.blend        DEF-only `Player` (41 bones) + Player_mesh, baked
  --stage-rest    work/player/rig/export/stage_rest.blend
  --fbx           work/player/rig/export/Player@<clip>.fbx
  --report        unity/AvatarCheck/player_clip_report_<clip>.json  PlayerClipCheck (player_report.json schema +
                  import.clip_settings, root_motion {policy, root_path}; frames bones_world springs on, skirt_raw off)
  --rig-report    unity/AvatarCheck/player_report.json               PlayerRigCheck (rigtest) for U3
  --unity-assets  unity/AvatarCheck/Assets/Player                    Player@*.fbx present = the U3 clip set
  --clip-json     work/player/rig/data/clips/<clip>.json   --resolved work/player/rig/anim/<clip>_resolved.json
  --preset / --canon   data/export_preset.json, data/canonical_skeleton.json
  --out           work/player/inspect/clips/<clip>/check_clip_unity.json
Rows:
  U1 (blocking) bake: work DEF world vs stage every frame <= 0.01 mm / 0.01 deg; stage structure (check_p2_export
     stage_struct: DEF-only 41 canonical bones, 0 connected, 0 constraints, rest vs canonical / rest stage, 20 slots);
     FBX: preset expectations, reimport (preset options, importer auto-connect undone, no *_end, 1 mesh 20 slots) and
     every frame vs stage <= 0.01 mm / 0.01 deg (1-frame offset corrected) - check_p2_export functions imported
  U2 (blocking) Unity: Generic, compression Off, 0 errors, 41 bones / canonical parents, bindposes_ok, rest block
     (check_p2_export.u_import); non-skirt bones on bones_world and Skirt_* on skirt_raw vs stage, rest-relative
     <= 0.01 mm / 0.01 deg (check_p1.p13_pose); clip settings vs the root_motion policy: loopTime = clip loop,
     motionNodeName empty (locked / in_place_cycle) or ending in "Root" (root), root: lockRootPositionXZ false (other
     bake flags reported); report root_motion.policy = clip policy; root_path vs stage Root (root: <= 0.01 mm /
     0.01 deg rest-relative; locked / in_place_cycle: path displacement <= 0.01 mm)
  U3 (blocking) bounds applied; every frame of every Player@ clip present in Assets/Player inside the judged report's
     bounds: stage Player_mesh vertices in Root-local Unity space (pose delta of Root removed, axis mapping) and the
     bones_world of every report holding that clip (rigtest: player_report.json; clip reports), 0 frames outside;
     every present clip needs a stage; other reports' bounds compared (report)
  U4 (report) springs: sway per chain p95 / max (check_p2_export.c_springs incl. the tail cross-check) and poke on /
     off (check_p2_export.c_penetration) of the clip report
  U5 (report, T314; only when the clip JSON has "weapon") report weapon block {side, asset, points_local, frames [{f,
     tip, guard_a, guard_b, pommel}]} vs M (stage WeaponSocket_<side> world @ contract offset @ placeholder mesh points)
     per point per frame (max / p95 / worst); points_local vs M p_local (scale from the report, else 1; fitted scale
     reported); draft 0.1 mm; no weapon block / contract / weapons blend -> U5 blocked only.  Points: tip / pommel =
     mean of the vertices at max / min local Z, guard_a / guard_b = at max / min local Y.  T321: blocking <= 0.1 mm
  U6 (report, blocking candidate, T321; only when the clip JSON has "events" [{name, frame}]) Unity AnimationEvents
     from import.clip_settings.after[0].events or clips.<clip>.events [{functionName, time}]: names match, |time -
     (frame - first) / fps| <= 0.5 / fps; field absent -> U6 blocked only
Nothing is saved: blends are opened read-only in memory; FBX files are imported into empty factory scenes.
Run:    blender --background --factory-startup --python check_player_clip_unity.py -- --clip <name> [overrides]
Exit:   0 evidence written; 2 script error or missing input (evidence still written with blocked rows).
"""
import sys

sys.dont_write_bytecode = True   # no __pycache__ next to the scripts (set before the local imports)

import json  # noqa: E402
import math  # noqa: E402
import traceback  # noqa: E402
from pathlib import Path  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import check_p1 as p1  # noqa: E402
import check_p2_deform as p2d  # noqa: E402
import check_p2_export as p2e  # noqa: E402

PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
AC = REPO / "unity" / "AvatarCheck"
CHECKER, TASK = "check_player_clip_unity", "T301"
CALIB = p2d.CALIB
rel, jsonable, mm, rnd = p2d.rel, p2d.jsonable, p2d.mm, p2d.rnd

P = {"clip": None, "prefix": None, "work": None, "action": None, "stage": None,
     "stage_rest": PLAYER_RIG / "export" / "stage_rest.blend", "fbx": None, "report": None,
     "rig_report": AC / "player_report.json", "unity_assets": AC / "Assets" / "Player",
     "clip_json": None, "resolved": None, "preset": PLAYER_RIG / "data" / "export_preset.json",
     "canon": PLAYER_RIG / "data" / "canonical_skeleton.json", "work_arm": "PL_rig", "out": None,
     "weapon_contract": PLAYER_RIG / "data" / "weapon_socket_contract.json",   # T314 U5
     "weapons_dir": PLAYER_RIG / "weapons"}
PATH_KEYS = ("work", "stage", "stage_rest", "fbx", "report", "rig_report", "unity_assets", "clip_json", "resolved",
             "preset", "canon", "out", "weapon_contract", "weapons_dir")
POS_TOL, ROT_TOL = p2e.POS_TOL, p2e.ROT_TOL
BAKE_FLAGS = ("lockRootRotation", "lockRootHeightY", "lockRootPositionXZ", "keepOriginalOrientation",
              "keepOriginalPositionY", "keepOriginalPositionXZ", "loopPose", "heightFromFeet")


class Blocked(Exception):
    pass


BLK = (Blocked, p1.Blocked, p2e.Blocked, p2d.Blocked)


def tb_tail(n=3):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


def pick(res, clip, key, default=None):
    if isinstance(res, dict) and key in res:
        return res[key], "resolved"
    if isinstance(clip, dict) and key in clip:
        return clip[key], "clip_json"
    return default, "default"


def boolish(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str) and v.strip().lower() in ("true", "false", "1", "0"):
        return v.strip().lower() in ("true", "1")
    return None


# ---------------------------------------------------------------- U2 clip settings / root motion
def clip_settings(rep, clip):
    """-> (settings dict, source): import.clip_settings (dict, or list with name == clip, or {clip: {...}}),
    else import.files["Player@<clip>.fbx"] (motionNodeName + clipSettings entry of the clip)."""
    imp = rep.get("import") if isinstance(rep.get("import"), dict) else {}
    cs = imp.get("clip_settings")
    if isinstance(cs, dict) and isinstance(cs.get("after"), list):
        # T302: PlayerClipCheck format {before: [..], changed, resaved, after: [..]}; judged on after[0]
        aft = [x for x in cs["after"] if isinstance(x, dict)]
        if aft:
            out = dict(aft[0])
            out["_report (before / changed / resaved / policy_mapping)"] = {
                "before": cs.get("before"), "changed": cs.get("changed"), "resaved": cs.get("resaved"),
                "policy": cs.get("policy"), "json_loop": cs.get("json_loop"), "policy_mapping": cs.get("policy_mapping"),
                "n_after": len(aft)}
            return out, "import.clip_settings.after[0]"
    if isinstance(cs, list):
        cs = next((x for x in cs if isinstance(x, dict) and x.get("name") in (clip, None)), None)
    if isinstance(cs, dict) and clip in cs and isinstance(cs[clip], dict):
        cs = cs[clip]
    if isinstance(cs, dict):
        return cs, "import.clip_settings"
    fe = ((imp.get("files") or {}).get(f"Player@{clip}.fbx")) if isinstance(imp.get("files"), dict) else None
    if isinstance(fe, dict):
        ent = next((x for x in fe.get("clipSettings") or [] if isinstance(x, dict) and x.get("name") == clip), None)
        out = dict(ent or {})
        out["motionNodeName"] = fe.get("motionNodeName")
        return out, f"fallback import.files[Player@{clip}.fbx] (import.clip_settings absent)"
    return None, "absent"


def settings_vs_policy(cs, policy, loop):
    rows, ok = {}, True
    lt = boolish(cs.get("loopTime"))
    want_loop = True if policy == "in_place_cycle" else boolish(loop)
    rows["loopTime"] = {"found": cs.get("loopTime"), "expected": want_loop,
                        "ok": lt is not None and want_loop is not None and lt == want_loop}
    mn = cs.get("motionNodeName")
    mns = "" if mn is None else str(mn)
    if policy == "root":
        mok = mns.replace("\\", "/").split("/")[-1] == "Root"
        exp = "ends with 'Root'"
    else:
        mok = mns == ""
        exp = "empty"
    rows["motionNodeName"] = {"found": mn, "expected": exp, "ok": mok}
    for k in BAKE_FLAGS:
        if policy == "root" and k == "lockRootPositionXZ":
            v = boolish(cs.get(k))
            rows[k] = {"found": cs.get(k), "expected": False, "ok": v is False}
        else:
            rows[k] = {"found": cs.get(k), "expected": "report"}
    for r in rows.values():
        if "ok" in r:
            ok = ok and r["ok"]
    return rows, ok


def root_motion_block(rep, clip):
    rm = rep.get("root_motion")
    src = "root_motion"
    if not isinstance(rm, dict):
        rm = ((rep.get("clips") or {}).get(clip) or {}).get("root_motion")
        src = f"clips.{clip}.root_motion"
    return (rm, src) if isinstance(rm, dict) else (None, "absent")


def path_entries(rp):
    """root_path -> [(f, pos[3], rot xyzw[4])]; entries {f, root_world: [px, py, pz, qx, qy, qz, qw]} (T302:
    PlayerClipCheck format, Root bone after SampleAnimation, Unity world), {f, pos, rot} or [f, px, .., qw]."""
    out = []
    for e in rp if isinstance(rp, list) else []:
        try:
            if isinstance(e, dict) and isinstance(e.get("root_world"), (list, tuple)) and len(e["root_world"]) == 7:
                w = [float(x) for x in e["root_world"]]
                out.append((int(e["f"]), w[:3], w[3:]))
            elif isinstance(e, dict):
                out.append((int(e["f"]), [float(x) for x in e["pos"]], [float(x) for x in e["rot"]]))
            elif isinstance(e, (list, tuple)) and len(e) == 8:
                out.append((int(e[0]), [float(x) for x in e[1:4]], [float(x) for x in e[4:8]]))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def root_path_check(ctx, rm, policy):
    ent = path_entries(rm.get("root_path"))
    if not ent:
        return {"root_path": "absent or unparseable", "n": 0}, False
    M = np.array(ctx.M, dtype=np.float64)
    idx = {f: i for i, f in enumerate(ctx.stage_frames)}
    ri = ctx.names.index("Root")
    rest_u = p1.tr((ctx.rep.get("rest") or {}).get("Root"))
    Rrb = p1.rot3n(ctx.stage_rest["Root"])
    pe, re_, un = [], [], []
    for f, pos, rot in ent:
        i = idx.get(f)
        if i is None:
            un.append(f)
            continue
        Wb = ctx.stage_W[i, ri]
        pe.append(float(np.linalg.norm(np.array(pos) - M @ Wb[:3, 3])))
        Ru = np.array(p1.tr(pos + rot)[1], dtype=np.float64)
        if rest_u is not None:
            dRu = Ru @ np.array(rest_u[1], dtype=np.float64).T
            dRb = M @ (p1.rot3n(Wb) @ Rrb.T) @ M.T
            re_.append(p1.rang(dRu, dRb))
    disp = max(float(np.linalg.norm(np.array(p) - np.array(ent[0][1]))) for _f, p, _r in ent)
    rec = {"n": len(ent), "unmatched_frames": un[:20], "pos_vs_stage_mm": p1.dist(pe, 1000.0),
           "rot_vs_stage_deg_rest_relative": p1.dist(re_, 1.0, 5) if re_ else "no Unity rest for Root",
           "path_displacement_max_mm": mm(disp),
           "entry_key": "root_world" if isinstance((rm.get("root_path") or [{}])[0], dict) and "root_world" in rm["root_path"][0] else "pos / rot or 8-list"}
    if policy == "root":
        ok = bool(pe) and max(pe) <= POS_TOL and bool(re_) and max(re_) <= ROT_TOL and not un
    else:
        ok = disp <= POS_TOL
    return rec, ok


# ---------------------------------------------------------------- U3 bounds
def box_of(b, key):
    try:
        if key == "prefab":
            lb = ((b.get("prefab") or {}).get("localBounds_rootBone_space")) or {}
            c, e = np.array(lb["center"], dtype=np.float64), np.array(lb["extents"], dtype=np.float64)
        else:
            c, e = np.array(b["center"], dtype=np.float64), np.array(b["extents"], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        return None
    return c - e, c + e


def outside(lo, hi, mn, mx):
    return float(max(0.0, *(lo - mn), *(mx - hi)))


def stage_mesh_frames(path, clip, M):
    """per frame (root-local Unity AABB, world Unity AABB) of the stage Player_mesh; Root pose delta removed."""
    p1.open_blend(path)
    arm = p2e.pick_arm("Player")
    mo = bpy.data.objects.get("Player_mesh")
    if arm is None or mo is None:
        raise Blocked(f"{rel(path)}: Player / Player_mesh missing")
    ad = arm.animation_data
    act = ad.action if ad and ad.action else bpy.data.actions.get(clip)
    if act is None:
        raise Blocked(f"{rel(path)}: no action")
    if not (ad and ad.action):
        p1.assign_action(arm, act)
    frames = p2e.action_frames(act)
    mw = arm.matrix_world
    rest = np.array(mw @ arm.data.bones["Root"].matrix_local, dtype=np.float64)
    sc = bpy.context.scene
    out = []
    for f in frames:
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        ae = arm.evaluated_get(dg)
        D = np.array(mw @ ae.pose.bones["Root"].matrix, dtype=np.float64) @ np.linalg.inv(rest)
        Di = np.linalg.inv(D)
        V = p2d.eval_verts(mo)
        Vr = V @ Di[:3, :3].T + Di[:3, 3]
        Ur, Uw = Vr @ M.T, V @ M.T
        out.append((f, Ur.min(axis=0), Ur.max(axis=0), Uw.min(axis=0), Uw.max(axis=0)))
    return {"blend": rel(path), "action": act.name, "frames": [frames[0], frames[-1]], "n": len(frames)}, out


def report_bone_frames(cb):
    """per frame root-local Unity AABB of bones_world (and skirt_raw) points; Root delta removed."""
    out = []
    for fr in cb.get("frames") or []:
        if not isinstance(fr, dict):
            continue
        pts = []
        for key in ("bones_world", "skirt_raw"):
            blk = fr.get(key)
            if isinstance(blk, dict):
                pts += [p1.tr(v) for v in blk.values() if p1.tr(v) is not None]
        if not pts:
            continue
        rt = p1.tr((fr.get("bones_world") or {}).get("Root"))
        A = np.array([[float(c) for c in p[0]] for p in pts])
        if rt is not None:
            R = np.array(rt[1], dtype=np.float64)
            A = (A - np.array([float(c) for c in rt[0]])) @ R   # R^T (p - t): Root rest = identity at the origin
        out.append((fr.get("f"), A.min(axis=0), A.max(axis=0)))
    return out


def u3(ctx):
    b = ctx.rep.get("bounds")
    if not isinstance(b, dict):
        raise Blocked("clip report has no 'bounds' object")
    box = box_of(b, "prefab")
    box_src = "bounds.prefab.localBounds_rootBone_space"
    if box is None:
        box, box_src = box_of(b, "center"), "bounds.center / extents"
    if box is None:
        raise Blocked(f"bounds unreadable: {b}")
    lo, hi = box
    M = np.array(ctx.M, dtype=np.float64)
    present = sorted(p.name[len("Player@"):-len(".fbx")] for p in Path(P["unity_assets"]).glob("Player@*.fbx"))
    clips = sorted(set(present) | {P["clip"]})
    reports = {"rig_report": (P["rig_report"], p2d.load_json(P["rig_report"]) if Path(P["rig_report"]).exists() else None)}
    for c in clips:
        rp = AC / f"player_clip_report_{c}.json"
        if c == P["clip"]:
            reports[f"clip_report:{c}"] = (P["report"], ctx.rep)
        elif rp.exists():
            reports[f"clip_report:{c}"] = (rp, p2d.load_json(rp))
    per, ok = {}, b.get("applied") is True
    for c in clips:
        rec = {"in_unity_assets": c in present}
        st = PLAYER_RIG / "export" / f"stage_{c}.blend" if c != P["clip"] else Path(P["stage"])
        if st.exists():
            try:
                info, fr = stage_mesh_frames(st, c, M)
                o = [outside(lo, hi, a, b_) for _f, a, b_, _wa, _wb in fr]
                ow = [outside(lo, hi, wa, wb) for _f, _a, _b, wa, wb in fr]
                bad = [fr[i][0] for i, v in enumerate(o) if v > 0]
                rec["stage_mesh"] = {**info, "n_frames_outside_root_local": len(bad), "frames_outside_first": bad[:20],
                                     "max_outside_mm": mm(max(o)),
                                     "world_n_frames_outside (report)": sum(1 for v in ow if v > 0),
                                     "union_root_local_u": [[rnd(x, 5) for x in np.min([a for _f, a, _b, _c, _d in fr], axis=0)],
                                                            [rnd(x, 5) for x in np.max([b_ for _f, _a, b_, _c, _d in fr], axis=0)]]}
                ok = ok and not bad and bool(fr)
            except BLK as e:
                rec["stage_mesh"] = f"blocked: {e}"
                ok = False
        else:
            rec["stage_mesh"] = f"missing {rel(st)}"
            ok = False
        rb = {}
        for rk, (rpath, rj) in reports.items():
            cb = ((rj or {}).get("clips") or {}).get(c) if isinstance(rj, dict) else None
            if not isinstance(cb, dict):
                continue
            bf = report_bone_frames(cb)
            o = [outside(lo, hi, a, b_) for _f, a, b_ in bf]
            nb = sum(1 for v in o if v > 0)
            rb[rk] = {"report": rel(rpath), "n_frames": len(bf), "n_frames_bones_outside": nb,
                      "max_outside_mm": mm(max(o)) if o else None}
            ok = ok and nb == 0
        rec["report_bones"] = rb
        per[c] = rec
    cons = {}
    for rk, (rpath, rj) in reports.items():
        ob = (rj or {}).get("bounds") if isinstance(rj, dict) else None
        if not isinstance(ob, dict) or rj is ctx.rep:
            continue
        bx = box_of(ob, "prefab") or box_of(ob, "center")
        cons[rk] = {"report": rel(rpath), "max_abs_diff_vs_judged_m": float(f"{float(np.abs(np.concatenate([bx[0] - lo, bx[1] - hi])).max()):.3e}")
                    if bx is not None else "unreadable", "union_frames": ob.get("union_frames")}
    return {"judged_bounds": {"source": box_src, "min": [rnd(x, 6) for x in lo], "max": [rnd(x, 6) for x in hi],
                              "applied": b.get("applied"), "space": b.get("space"), "union_frames": b.get("union_frames"),
                              "union_rule": b.get("union_rule")},
            "clips_present_in_unity_assets": present, "clips_checked": clips, "per_clip": per,
            "other_reports_bounds_vs_judged (report)": cons,
            "space_note": "stage vertices: Root pose delta removed (world -> Root rest frame), then the preset axis mapping "
                          "(Unity Root rest = identity at the origin, rootBone = Root); report points: R_root^T (p - t_root)"}, ok


# ---------------------------------------------------------------- U5 weapon points (T314)
U5_DRAFT_MM = 0.1
U5_POINTS = ("tip", "guard_a", "guard_b", "pommel")
U5_TIE = 1e-6               # m: vertices within this of the extreme value are averaged
U5_POINT_DEF = {"tip": "mean of the weapon mesh vertices with max local Z (blade axis, grip -> tip)",
                "pommel": "mean of the vertices with min local Z (the far end below the grip)",
                "guard_a": "mean of the vertices with max local Y (guard axis, + end)",
                "guard_b": "mean of the vertices with min local Y (guard axis, - end)",
                "space": "weapon mesh data coordinates = the contract socket space (+Z blade, Y guard, X flat normal)",
                "tie": f"vertices within {U5_TIE:g} m of the extreme are averaged"}


def weapon_points(V):
    z, y = V[:, 2], V[:, 1]
    sel = {"tip": z >= z.max() - U5_TIE, "pommel": z <= z.min() + U5_TIE,
           "guard_a": y >= y.max() - U5_TIE, "guard_b": y <= y.min() + U5_TIE}
    return {k: V[m].mean(axis=0) for k, m in sel.items()}, {k: int(m.sum()) for k, m in sel.items()}


def vec3(v):
    if isinstance(v, dict):
        v = [v.get("x"), v.get("y"), v.get("z")]
    try:
        a = np.array([float(x) for x in v], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    return a if a.shape == (3,) else None


def export_compensation(con):
    """T314 addendum: contract unity.export_compensation -> (3x3 C, space "unity" | "blender", source) or (None, None,
    reason).  Accepted: a 3x3 / 4x4 matrix (the object itself, or keys matrix / matrix_3x3 / rotation_matrix), or
    quaternion_wxyz [4] / quaternion_xyzw [4], or euler_deg [3] with euler_order (default XYZ, mathutils intrinsic);
    space = the object's "space" key (default "unity": C acts on the weapon's Unity local = M p_blender)."""
    from mathutils import Euler, Quaternion
    ec = ((con or {}).get("unity") or {}).get("export_compensation") if isinstance(con, dict) else None
    if ec is None:
        return None, None, "contract unity.export_compensation missing"

    def mat(v):
        a = np.array(v, dtype=np.float64)
        return a[:3, :3] if a.shape in ((3, 3), (4, 4)) else None
    try:
        space = str(ec.get("space", "unity")).lower() if isinstance(ec, dict) else "unity"
        if isinstance(ec, list):
            C, src = mat(ec), "unity.export_compensation (matrix)"
        else:
            C, src = None, None
            for k in ("matrix", "matrix_3x3", "rotation_matrix"):
                if isinstance(ec.get(k), list):
                    C, src = mat(ec[k]), f"unity.export_compensation.{k}"
                    break
            if C is None and isinstance(ec.get("quaternion_wxyz"), list):
                C, src = np.array(Quaternion([float(x) for x in ec["quaternion_wxyz"]]).normalized().to_matrix()), \
                    "unity.export_compensation.quaternion_wxyz"
            if C is None and isinstance(ec.get("quaternion_xyzw"), list):
                x, y, z, w = (float(v) for v in ec["quaternion_xyzw"])
                C, src = np.array(Quaternion((w, x, y, z)).normalized().to_matrix()), \
                    "unity.export_compensation.quaternion_xyzw"
            if C is None and isinstance(ec.get("euler_deg"), list):
                order = str(ec.get("euler_order", "XYZ")).upper()
                C, src = np.array(Euler([math.radians(float(a)) for a in ec["euler_deg"]], order).to_matrix()), \
                    f"unity.export_compensation.euler_deg ({order})"
    except (TypeError, ValueError, KeyError) as e:
        return None, None, f"contract unity.export_compensation unreadable: {e}"
    if C is None or space not in ("unity", "blender"):
        return None, None, f"contract unity.export_compensation unreadable: {json.dumps(ec)[:200]}"
    return C, space, src


def u5(ctx, weapon):
    """U5: weapon convention through the pipeline (d-04 section 5 SI.U5)."""
    import check_player_clip as cpc   # contract lookup / offset / weapon mesh append (T311 functions)
    wb = ctx.rep.get("weapon")
    if not isinstance(wb, dict):
        wb = ((ctx.rep.get("clips") or {}).get(P["clip"]) or {}).get("weapon")
    if not isinstance(wb, dict):
        raise Blocked("clip report has no 'weapon' block")
    cpath = Path(P["weapon_contract"])
    if not cpath.exists():
        raise Blocked(f"missing input {rel(cpath)}")
    con = p2d.load_json(cpath)
    side = str(wb.get("side") or next(iter(weapon))).strip().upper()[:1]
    wname = str(weapon.get(side, weapon.get(side.lower(), next(iter(weapon.values())))))
    ent, key = cpc.contract_entry(con, side, wname)
    if ent is None:
        raise Blocked(f"{rel(cpath)}: no entry for side {side} / {wname}")
    T, tsrc = cpc.contract_offset(ent)
    bp = REPO / ent["file"] if isinstance(ent.get("file"), str) else Path(P["weapons_dir"]) / f"{wname}.blend"
    if not bp.exists():
        raise Blocked(f"missing input {rel(bp)}")
    try:
        V, _F, _bl, minfo = cpc.weapon_mesh(bp, str(ent.get("object") or ent.get("mesh") or cpc.C8_OBJECT_DEFAULT))
    except cpc.Blocked as e:
        raise Blocked(str(e))
    pts, npts = weapon_points(V)
    M = np.array(ctx.M, dtype=np.float64)
    socket = f"WeaponSocket_{side}"
    if ctx.stage_W is not None:
        SW, sfr, ssrc = ctx.stage_W, ctx.stage_frames, "stage (baked)"
    elif ctx.work is not None:
        SW, sfr, ssrc = ctx.work["W"], ctx.work["frames"], "work rig (stage not sampled)"
    else:
        raise Blocked("neither the stage nor the work rig was sampled")
    si = ctx.names.index(socket)
    # points_local vs the Blender mesh
    pl = wb.get("points_local") if isinstance(wb.get("points_local"), dict) else {}
    scale_rep = next((wb[k] for k in ("points_local_scale", "fbx_scale", "scale") if isinstance(wb.get(k), (int, float))),
                     None)
    s_used = float(scale_rep) if scale_rep is not None else 1.0
    C, c_space, c_src = export_compensation(con)
    plr, num, den = {}, 0.0, 0.0
    for k in U5_POINTS:
        if C is None:   # T314 addendum: no unity.export_compensation -> sub-check skipped (reported)
            break
        u = vec3(pl.get(k)) if k in pl else None
        b = C @ (M @ pts[k]) if c_space == "unity" else M @ (C @ pts[k])
        if u is None:
            plr[k] = {"report": pl.get(k), "blender_mapped_m": [rnd(x, 6) for x in b], "error_mm": None}
            continue
        num, den = num + float(np.linalg.norm(u)), den + float(np.linalg.norm(b))
        plr[k] = {"report": [rnd(x, 6) for x in u], "blender_mapped_m": [rnd(x, 6) for x in b],
                  "error_mm_scale_used": mm(np.linalg.norm(u - s_used * b))}
    s_fit = num / den if den > 0 else None
    for k in plr:
        u = vec3(pl.get(k)) if k in pl else None
        if u is not None and s_fit:
            b = C @ (M @ pts[k]) if c_space == "unity" else M @ (C @ pts[k])
            plr[k]["error_mm_scale_fitted"] = mm(np.linalg.norm(u - s_fit * b))
    pl_errs = [v["error_mm_scale_used"] for v in plr.values() if v.get("error_mm_scale_used") is not None]
    # per frame
    fr = [x for x in wb.get("frames") or [] if isinstance(x, dict) and isinstance(x.get("f"), (int, float))]
    if not fr:
        raise Blocked("weapon block has no frames")
    fs = [int(x["f"]) for x in fr]
    off = 0 if all(sfr[0] <= f <= sfr[-1] for f in fs) else sfr[0] - min(fs)
    idx = {f: i for i, f in enumerate(sfr)}
    per = {k: [] for k in U5_POINTS}
    keys = {k: [] for k in U5_POINTS}
    swap = []
    unmatched, missing = [], set()
    for x in fr:
        i = idx.get(int(x["f"]) + off)
        if i is None:
            unmatched.append(x["f"])
            continue
        Wm = SW[i, si] @ T
        exp = {k: M @ (Wm[:3, :3] @ pts[k] + Wm[:3, 3]) for k in U5_POINTS}
        for k in U5_POINTS:
            u = vec3(x.get(k))
            if u is None:
                missing.add(k)
                continue
            per[k].append(float(np.linalg.norm(u - exp[k])))
            keys[k].append(int(x["f"]))
        ga, gb = vec3(x.get("guard_a")), vec3(x.get("guard_b"))
        if ga is not None and gb is not None:
            swap.append(max(float(np.linalg.norm(ga - exp["guard_b"])), float(np.linalg.norm(gb - exp["guard_a"]))))
    rows = {k: p1.dist(v, 1000.0, 4, keys=keys[k]) for k, v in per.items()}
    allv = [v for k in per for v in per[k]]
    mx = max(allv) * 1000.0 if allv else None
    return {"weapon_block": {"side": wb.get("side"), "asset": wb.get("asset"), "n_frames": len(fr),
                             "keys": sorted(wb.keys())},
            "blender": {"socket": socket, "socket_source": ssrc, "contract": rel(cpath), "contract_key": key,
                        "offset_source": tsrc, "offset_matrix": [[rnd(v, 6) for v in r] for r in T], "mesh": minfo,
                        "mesh_blend": rel(bp), "points_local_blender_m": {k: [rnd(v, 6) for v in p] for k, p in pts.items()},
                        "n_vertices_averaged": npts},
            "point_definitions": U5_POINT_DEF,
            "per_point_mm": rows, "max_mm_all_points": rnd(mx, 4) if mx is not None else None,
            "meets_threshold": mx is not None and mx <= U5_DRAFT_MM and not unmatched and not missing,
            "draft_mm": U5_DRAFT_MM, "guard_names_swapped_max_mm (report)": mm(max(swap)) if swap else None,
            "unmatched_report_frames": unmatched[:20], "points_missing_in_frames": sorted(missing),
            "report_f_to_stage_offset": off,
            "points_local": {"export_compensation": {"source": c_src, "space": c_space,
                                                     "matrix": [[rnd(v, 6) for v in r] for r in C] if C is not None
                                                     else None,
                                                     "sub_check": "applied" if C is not None else
                                                     "skipped: contract unity.export_compensation missing / unreadable"},
                             "expected_rule": "unity space: scale * C (M p_local_b); blender space: scale * M (C p_local_b)",
                             "per_point": plr, "scale_used": s_used,
                             "scale_source": "report" if scale_rep is not None else "1 (no scale field in the report)",
                             "scale_fitted (report)": rnd(s_fit, 6) if s_fit else None,
                             "max_error_mm_scale_used": max(pl_errs) if pl_errs else None},
            "rule": "expected Unity world = M (socket_world_b @ contract offset @ p_local_b); M = preset "
                    "axis_mapping.blender_to_unity; socket_world_b = stage DEF WeaponSocket_x world per frame; "
                    "points_local expected = scale M p_local_b"}, \
        (mx is not None and mx <= U5_DRAFT_MM and not unmatched and not missing)


# ---------------------------------------------------------------- U6 AnimationEvents (T321)
U6_NAMES = ("hit_start", "hit_end")
U5_NOTE = ("T321: blocking at <= 0.1 mm; basis: Sword_Idle measured max 0.0005 mm (200x margin), d-04 section 6, "
           "d-05 section 2 SA1.U5")
U6_NOTE = ("T333 blocking (d-05 section 8): |time - (frame - first) / fps| <= 0.5 frame; basis: 0.0 measured; the half "
           "frame is the sampling resolution")


def u6(ctx, events, fps, first):
    """Unity AnimationEvents vs the clip JSON events: import.clip_settings.after[0].events or clips.<clip>.events,
    [{functionName, time}]; |time - (frame - first) / fps| <= 0.5 / fps; names match."""
    imp = ctx.rep.get("import") if isinstance(ctx.rep.get("import"), dict) else {}
    cs = imp.get("clip_settings")
    ue, src = None, None
    if isinstance(cs, dict) and isinstance(cs.get("after"), list) and cs["after"] and isinstance(cs["after"][0], dict) \
            and isinstance(cs["after"][0].get("events"), list):
        ue, src = cs["after"][0]["events"], "import.clip_settings.after[0].events"
    cb = ((ctx.rep.get("clips") or {}).get(P["clip"]) or {})
    if ue is None and isinstance(cb.get("events"), list):
        ue, src = cb["events"], f"clips.{P['clip']}.events"
    if ue is None:
        raise Blocked("clip report has no events (import.clip_settings.after[0].events / clips.<clip>.events)")
    tol = 0.5 / float(fps)
    je = [(str(e.get("name")), int(e.get("frame"))) for e in events or [] if isinstance(e, dict)
          and isinstance(e.get("frame"), (int, float))]
    ur = [(str(e.get("functionName")), float(e.get("time"))) for e in ue if isinstance(e, dict)
          and isinstance(e.get("time"), (int, float))]
    names_ok = sorted(n for n, _f in je) == sorted(n for n, _t in ur)
    rows, used, ok = [], set(), names_ok and len(ur) == len(ue)
    for n, f in je:
        exp = (f - first) / float(fps)
        cand = [(abs(t - exp), i) for i, (m, t) in enumerate(ur) if m == n and i not in used]
        if not cand:
            rows.append({"name": n, "frame": f, "expected_time_s": rnd(exp, 6), "unity": "missing"})
            ok = False
            continue
        d, i = min(cand)
        used.add(i)
        rows.append({"name": n, "frame": f, "expected_time_s": rnd(exp, 6), "unity_time_s": rnd(ur[i][1], 6),
                     "abs_diff_s": rnd(d, 6), "abs_diff_frames": rnd(d * float(fps), 4), "within_half_frame": d <= tol})
        ok = ok and d <= tol
    extra = [ur[i] for i in range(len(ur)) if i not in used]
    return {"source": src, "unity_events": ue, "clip_events": events, "fps": fps, "first_frame": first,
            "tol_s": rnd(tol, 6), "names_match": names_ok, "per_event": rows, "unity_events_unmatched": extra,
            "blocking_candidate_ok": ok and not extra,
            "rule": "expected time = (frame - first) / fps; names = functionName; each clip event matched to the "
                    "nearest unused Unity event of the same name"}, ok and not extra


# ---------------------------------------------------------------- evidence
IDS = (
    ("U1", "blocking", f"stage structure ok; work DEF vs stage and FBX reimport vs stage, every frame <= "
                       f"{POS_TOL * 1000:g} mm / {ROT_TOL:g} deg; preset expectations; FBX structure"),
    ("U2", "blocking", f"Unity Generic, compression Off, 0 errors, bindposes_ok, rest block; bones rest-relative <= "
                       f"{POS_TOL * 1000:g} mm / {ROT_TOL:g} deg (skirt on skirt_raw); clip settings match the "
                       "root_motion policy; report root_motion policy / path match"),
    ("U3", "blocking", "bounds applied; every frame of every Player@ clip in Assets/Player inside (stage mesh root-local "
                       "+ report bones), 0 outside; every present clip has a stage"),
    ("U4", "report", "springs: sway per chain p95 / max, poke springs on / off"),
    ("U5", "blocking", f"(only when the clip JSON has 'weapon') Unity weapon points (tip, guard_a, guard_b, pommel) "
                       f"vs Blender socket world x contract offset x placeholder mesh points, mapped by the preset "
                       f"axes: every point every frame <= {U5_DRAFT_MM:g} mm, no unmatched frame / missing point; "
                       f"points_local sub-check reported"),
    ("U6", "blocking", "(only when the clip JSON has 'events') Unity AnimationEvents: names = clip events, |time - "
                       "(frame - first) / fps| <= 0.5 / fps; ok = blocking_candidate_ok (d-05 section 8)"),
)


class Evidence(p2d.Evidence):
    def write(self):
        out = Path(P["out"])
        out.parent.mkdir(parents=True, exist_ok=True)
        order = {cid(i): k for k, (i, _s, _t) in enumerate(IDS)}
        self.criteria.sort(key=lambda c: order.get(c["id"], 99))
        doc = {"gate": P["prefix"] or P["clip"], "checker": CHECKER, "task": TASK, "clip": P["clip"],
               "blender": bpy.app.version_string, **jsonable(self.extra), "inputs": self.inputs,
               "criteria": self.criteria}
        with open(out, "w", encoding="utf-8", newline="\n") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)
        for c in self.criteria:
            print(f"[{CHECKER}] {c['id']} ({c['status']}) ok={c['ok']} measured={json.dumps(c['measured'])[:300]}")
        print(f"[{CHECKER}] wrote {rel(out)}")
        sys.stdout.flush()


def cid(i):
    return f"{P['prefix']}.{i}" if P["prefix"] else i


_STATE = {"missing": []}


def parse_args(argv):
    keys = {"--" + k.replace("_", "-"): k for k in P}
    i = 0
    while i < len(argv):
        if argv[i] not in keys or i + 1 >= len(argv):
            raise ValueError(f"bad argument {argv[i]!r}; usage: --clip <name> {' '.join(sorted(keys))} <value>")
        k = keys[argv[i]]
        P[k] = Path(argv[i + 1]) if k in PATH_KEYS else argv[i + 1]
        i += 2
    if not P["clip"]:
        raise ValueError("--clip <name> is required")
    c = P["clip"]
    P["action"] = P["action"] or c
    P["work"] = P["work"] or PLAYER_RIG / "anim" / f"pl_a_{c}.blend"
    P["stage"] = P["stage"] or PLAYER_RIG / "export" / f"stage_{c}.blend"
    P["fbx"] = P["fbx"] or PLAYER_RIG / "export" / f"Player@{c}.fbx"
    P["report"] = P["report"] or AC / f"player_clip_report_{c}.json"
    P["clip_json"] = P["clip_json"] or PLAYER_RIG / "data" / "clips" / f"{c}.json"
    P["resolved"] = P["resolved"] or PLAYER_RIG / "anim" / f"{c}_resolved.json"
    P["out"] = P["out"] or REPO / "work" / "player" / "inspect" / "clips" / c / "check_clip_unity.json"
    # check_p2_export functions read their module-level P
    p2e.P.update({"work": P["work"], "action": P["action"], "clip": c, "work_arm": P["work_arm"],
                  "stage_arm": "Player", "stage_mesh": "Player_mesh", "stage_rest": P["stage_rest"],
                  "stage_rig": P["stage"], "fbx_clip": P["fbx"], "report": P["report"], "preset": P["preset"],
                  "canon": P["canon"]})


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    for m in (__file__, p1.__file__, p2d.__file__, p2e.__file__):
        ev.add_input(m)
    th = {i: t for i, _s, t in IDS}
    st = {i: s for i, s, _t in IDS}
    ids = [i for i, _s, _t in IDS]
    done = set()

    def put(i, measured, ok, note=""):
        ev.criterion(cid(i), measured, th[i], bool(ok) if st[i] == "blocking" else True,
                     note or (CALIB if st[i] == "report" else ""))
        ev.criteria[-1]["status"] = st[i]
        done.add(i)

    def block(cids, reason):
        for i in cids:
            if i not in done:
                ev.criterion(cid(i), f"blocked: {reason}", th[i], False, "")
                ev.criteria[-1]["status"] = st[i]
                done.add(i)

    try:
        required = ("work", "stage", "stage_rest", "fbx", "report", "clip_json", "preset", "canon")
        miss = [rel(P[k]) for k in required if not Path(P[k]).exists()]
        for k in required + ("rig_report", "resolved"):
            ev.add_input(P[k])
        for q in sorted(HERE.glob("p30*.py")) + sorted(HERE.glob("p31*.py")) + sorted(HERE.glob("p26*.py")) + \
                sorted(HERE.glob("p27*.py")):
            ev.add_input(q)
        others = sorted(AC.glob("player_clip_report_*.json"))
        for q in others:
            ev.add_input(q)
        _STATE["missing"] = miss
        ev.extra["names"] = {k: str(P[k]) for k in ("clip", "prefix", "action", "work_arm")}
        if not Path(P["canon"]).exists() or not Path(P["preset"]).exists():
            block(ids, "missing input " + ", ".join(miss))
            return
        ctx = p2e.Ctx()
        ctx.canon = p2d.load_json(P["canon"])
        p2e.load_canon(ctx)
        ctx.preset = p2d.load_json(P["preset"])
        clipj = p2d.load_json(P["clip_json"]) if Path(P["clip_json"]).exists() else None
        res = p2d.load_json(P["resolved"]) if Path(P["resolved"]).exists() else None
        rm, rm_src = pick(res, clipj, "root_motion", None)
        policy = rm.get("policy") if isinstance(rm, dict) else rm
        loop, loop_src = pick(res, clipj, "loop", None)
        ev.extra["clip_info"] = {"root_motion": policy, "root_motion_source": rm_src, "loop": loop,
                                 "loop_source": loop_src}
        weapon, wp_src = pick(res, clipj, "weapon", None)   # T314: U5 only when the clip has a weapon
        ev.extra["clip_info"].update({"weapon": weapon, "weapon_source": wp_src})
        if weapon is None:
            ids.remove("U5")
        evts, evts_src = pick(res, clipj, "events", None)   # T321: U6 only when the clip has events
        fps_c, _fs = pick(res, clipj, "fps", 30)
        fr_c, _frs = pick(res, clipj, "frame_range", None)
        ev.extra["clip_info"].update({"events": evts, "events_source": evts_src, "fps": fps_c, "frame_range": fr_c})
        if evts is None:
            ids.remove("U6")
        # U1
        u1, u1_ok = {}, True
        prec, pok = p2e.preset_check(ctx)
        u1["preset"], u1_ok = prec, pok
        try:
            if not Path(P["work"]).exists():
                raise Blocked(f"missing input {rel(P['work'])}")
            p2e.work_phase(ctx)
        except BLK as e:
            u1["work"], u1_ok = f"blocked: {e}", False
        for kind, path in (("rest", P["stage_rest"]), ("rig", P["stage"])):
            try:
                if not Path(path).exists():
                    raise Blocked(f"missing input {rel(path)}")
                r, o = p2e.stage_struct(ctx, kind, path)
                u1[f"stage_{kind}"] = r
                u1_ok = u1_ok and o
            except BLK as e:
                u1[f"stage_{kind}"], u1_ok = f"blocked: {e}", False
        if ctx.work is not None and ctx.stage_W is not None:
            r, o = p2e.compare_WW(ctx.work["W"], ctx.work["frames"], ctx.stage_W, ctx.stage_frames, ctx.names)
            r.update({"work": {k: v for k, v in ctx.work.items() if k != "W"}, "stage": ctx.stage_info})
            u1["bake_work_vs_stage"], u1_ok = r, u1_ok and o
        else:
            u1["bake_work_vs_stage"], u1_ok = "blocked: work or stage not sampled", False
        try:
            if not Path(P["fbx"]).exists() or ctx.opts is None:
                raise Blocked(f"missing input {rel(P['fbx'])}" if not Path(P["fbx"]).exists()
                              else "no reimport options in the preset")
            r, o, arm = p2e.fbx_struct(ctx, P["fbx"], True)
            u1["fbx_structure"], u1_ok = r, u1_ok and o
            if ctx.stage_W is None:
                raise Blocked("stage not sampled")
            r, o = p2e.fbx_frames(ctx, arm)
            u1["fbx_vs_stage"], u1_ok = r, u1_ok and o
        except BLK as e:
            u1.setdefault("fbx_structure", f"blocked: {e}")
            u1["fbx_vs_stage"], u1_ok = f"blocked: {e}", False
        put("U1", u1, u1_ok)
        # U2 / U3 / U4 need the clip report
        if not Path(P["report"]).exists():
            block([i for i in ("U2", "U3", "U4", "U5", "U6") if i in ids], f"missing input {rel(P['report'])}")
            return
        ctx.rep = p2d.load_json(P["report"])
        u2, u2_ok = {}, True
        try:
            r, o = p2e.u_import(ctx)
            u2["import"], u2_ok = r, o
        except BLK as e:
            u2["import"], u2_ok = f"blocked: {e}", False
        if ctx.stage_W is None:
            u2["bones"], u2_ok = "blocked: stage not sampled", False
        else:
            body = [n for n in ctx.names if not n.startswith("Skirt_")]
            skirt = [n for n in ctx.names if n.startswith("Skirt_")]
            for key, blk, names in (("bones_body", "bones_world", body), ("bones_skirt_raw", "skirt_raw", skirt)):
                try:
                    m = p2e.u_pose(ctx, blk, names)
                    m["ok"] = p2e.u_ok(m)
                    u2[key], u2_ok = m, u2_ok and m["ok"]
                except BLK as e:
                    u2[key], u2_ok = f"blocked: {e}", False
        cs, cs_src = clip_settings(ctx.rep, P["clip"])
        if cs is None:
            u2["clip_settings"], u2_ok = "absent (import.clip_settings)", False
        else:
            rows, o = settings_vs_policy(cs, policy, loop)
            u2["clip_settings"] = {"source": cs_src, "raw": cs, "policy": policy, "rows": rows, "ok": o}
            u2_ok = u2_ok and o
        rmb, rmb_src = root_motion_block(ctx.rep, P["clip"])
        if rmb is None:
            u2["root_motion"], u2_ok = "absent (root_motion {policy, root_path})", False
        else:
            pol_ok = rmb.get("policy") == policy
            rec = {"source": rmb_src, "policy_report": rmb.get("policy"), "policy_clip": policy, "policy_ok": pol_ok}
            if ctx.stage_W is not None:
                pr, pok_ = root_path_check(ctx, rmb, policy)
                rec["root_path"], rec["root_path_ok"] = pr, pok_
            else:
                pok_ = False
            u2["root_motion"], u2_ok = rec, u2_ok and pol_ok and pok_
        put("U2", u2, u2_ok)
        try:
            put("U3", *u3(ctx))
        except BLK as e:
            block(["U3"], str(e))
        except Exception:
            block(["U3"], "row error: " + tb_tail(4))
            _STATE.setdefault("row_errors", []).append("U3")
        u4 = {}
        for key, fn in (("springs", p2e.c_springs), ("penetration", p2e.c_penetration)):
            try:
                u4[key] = fn(ctx)[0]
            except BLK as e:
                u4[key] = f"blocked: {e}"
        sp = u4.get("springs")
        if isinstance(sp, dict):
            u4["sway_summary_mm"] = {k: {"p95": (v.get("per_frame_mm") or {}).get("p95"),
                                         "max": (v.get("per_frame_mm") or {}).get("max")}
                                     for k, v in (sp.get("chains") or {}).items()}
        put("U4", u4, True)
        if weapon is not None:
            try:
                put("U5", *u5(ctx, weapon), note=U5_NOTE)
            except BLK as e:
                block(["U5"], str(e))
            except Exception:
                block(["U5"], "row error: " + tb_tail(4))
                _STATE.setdefault("row_errors", []).append("U5")
        if evts is not None:
            try:
                first = int(fr_c[0]) if isinstance(fr_c, (list, tuple)) and fr_c else \
                    (ctx.stage_frames[0] if ctx.stage_frames else 1)
                put("U6", *u6(ctx, evts, fps_c, first), note=U6_NOTE)
            except BLK as e:
                block(["U6"], str(e))
            except Exception:
                block(["U6"], "row error: " + tb_tail(4))
                _STATE.setdefault("row_errors", []).append("U6")
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
    if _STATE["missing"] or _STATE.get("row_errors"):
        print("missing input: " + ", ".join(_STATE["missing"]) if _STATE["missing"] else
              f"row errors: {_STATE['row_errors']}")
        sys.stdout.flush()
        sys.exit(2)
