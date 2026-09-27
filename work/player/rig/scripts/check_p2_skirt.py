"""check_p2_skirt - Player P2.8b/c/d skirt-follows-thigh checker (task T254; spec work/player/d-02-player-rig.md
section 8 P2e plan; checker side).

Inputs (defaults; override after "--"):
  --cur       work/player/rig/pl_r05_rigtest.blend                          PL_rig + action rigtest, PL_mesh (after)
  --bak       work/player/rig/backup_pre_P2e_2026-09-27/pl_r05_rigtest.blend  the pre-follow rig (before)
  --man-cur   work/player/rig/data/rigtest_manifest.json                     segments (sections) of rigtest
  --man-bak   <backup dir>/data/rigtest_manifest.json   (default: the backup's own, else --man-cur)
  --parts / --loops   parts.json, retopo_loops.json (tunic rows, axis_xy, ceiling_z; same topology in both rigs)
  --follow    work/player/rig/data/skirt_follow.json   production manifest of the follow layer
  --bak-p2f   work/player/rig/backup_pre_P2f_2026-09-27/pl_r05_rigtest.blend  (T258: the P2e rig, before thinning)
  --man-p2f   <backup_pre_P2f>/data/rigtest_manifest.json (default: its own, else --man-cur)
  --retopo-cur / --retopo-bak   pl_r01_retopo.blend now / in backup_pre_P2f (P2.9a vertex-by-vertex displacement)
  --zone      work/player/rig/data/thigh_thin_zone.json (T258)
  --arm PL_rig  --mesh PL_mesh  --action rigtest  --prop skirt_follow
  --out       work/player/inspect/P2/check_p2_skirt.json
Everything is evaluated in Blender (springs off: the DEF mesh deformed by the rig, no spring bones).  Mesh and tail
metrics are taken in the Pelvis rest frame (the Pelvis world delta of the frame is removed first), so root / cog /
pelvis motion of the whole body does not move the skirt axis.
Rows:
  P2.8b  per rigtest section: leg-through-skirt with the check_p2_deform.poke_out rule (leg vertices with rest z below
         the skirt ceiling that lie radially outside the skirt wall where the wall exists at their height and azimuth)
         for the current rig and the backup: max poke vertex count and max poke depth per section, before / after,
         candidate flag after <= before (report)
  P2.8c  current rig (backup beside it): hem ring (tunic outer row hem_outer) edge-length ratio vs rest per frame (max
         stretch, min compression); skirt faces (tunic below the belt, ceiling excluded) x leg_l / leg_r intersecting
         polygon pairs (check_p2_deform.overlap_pairs); adjacent-panel crossing = sign flips of the cyclic azimuth order
         of the Skirt_*_02 tails (F, FL, L, BL, B, BR, R, FR) and of the hem ring vertices vs rest (report)
  P2.8d  current rig with PROPS skirt_follow = 0 (in memory; F-curves on the property muted in memory) vs the backup:
         every Skirt_* DEF bone's world pose delta (pose world @ rest world^-1) per frame, max abs element <= 1e-5
         (blocking; the delta is invariant to the c2 roll change of the rest)
  P2.8f  skirt_follow.json present and parseable; PROPS skirt_follow float with UI default 0.5 and range [0, 1]; every
         bone name in the manifest exists on PL_rig; >= 1 driver or constraint on PL_rig references skirt_follow
  P2.9a  (T258) thigh-thin zone invisible at rest: every zone vertex covered by the skirt wall (poke-out ray, min
         clearance); 0 displaced vertices outside the zone / below the hem vs the backup retopo (blocking)
  P2.9b  (T258) per-section poke-out, current vs the P2e rig (backup_pre_P2f): after <= before (blocking); f477 /
         f513 = 0 reported.  P2.8 rows keep backup_pre_P2e as their "before".
  T259: P2.9 is retired (P2f reverted 2026-09-27).  Without data/thigh_thin_zone.json no P2.9 row is written, the
         backup_pre_P2f rig is not sampled, its inputs are not hashed or required, and the evidence carries the info
         field "P2.9": "P2.9 retired (P2f reverted 2026-09-27)".
Methods reused (imported, unchanged): check_p2_deform.Topo / eval_verts / poke_out / overlap_pairs (T221 / T223).
Nothing is saved: blends are opened read-only in memory.
Run:    blender --background --factory-startup --python check_p2_skirt.py [-- <overrides>]
Exit:   0 evidence written; 2 script error or missing input (evidence still written with blocked rows).
"""
import json
import math
import re
import sys
import traceback
from pathlib import Path

import bpy
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True   # no __pycache__ next to the scripts
import check_p2_deform as p2d  # noqa: E402

PLAYER_RIG = HERE.parent
REPO = PLAYER_RIG.parent.parent.parent
GATE, CHECKER, TASK = "P2.8", "check_p2_skirt", "T254 + T258 (P2.9a/b)"
CALIB = p2d.CALIB
rel, jsonable, mm, rnd = p2d.rel, p2d.jsonable, p2d.mm, p2d.rnd
BACKUP = PLAYER_RIG / "backup_pre_P2e_2026-09-27"
BACKUP_P2F = PLAYER_RIG / "backup_pre_P2f_2026-09-27"   # T258: P2e rig, before the thigh-top thinning
P = {"cur": PLAYER_RIG / "pl_r05_rigtest.blend", "bak": BACKUP / "pl_r05_rigtest.blend",
     "man_cur": PLAYER_RIG / "data" / "rigtest_manifest.json", "man_bak": None,
     "bak_p2f": BACKUP_P2F / "pl_r05_rigtest.blend", "man_p2f": None,
     "retopo_cur": PLAYER_RIG / "pl_r01_retopo.blend", "retopo_bak": BACKUP_P2F / "pl_r01_retopo.blend",
     "zone": PLAYER_RIG / "data" / "thigh_thin_zone.json",
     "parts": PLAYER_RIG / "data" / "parts.json", "loops": PLAYER_RIG / "data" / "retopo_loops.json",
     "follow": PLAYER_RIG / "data" / "skirt_follow.json",
     "out": REPO / "work" / "player" / "inspect" / "P2" / "check_p2_skirt.json",
     "arm": "PL_rig", "mesh": "PL_mesh", "action": "rigtest", "prop": "skirt_follow"}
PATH_KEYS = ("cur", "bak", "man_cur", "man_bak", "parts", "loops", "follow", "out", "bak_p2f", "man_p2f",
             "retopo_cur", "retopo_bak", "zone")
DISP_TOL = 1e-7           # P2.9a: a vertex counts as displaced when |p20 now - backup| > this (m)
SKIRT_DIRS = ("F", "FL", "L", "BL", "B", "BR", "R", "FR")
DELTA_TOL = 1e-5          # P2.8d
CEIL_TOL = p2d.CEIL_TOL
FOLLOW_DEFAULT = 0.5


class Blocked(Exception):
    pass


def tb_tail(n=3):
    return " | ".join(traceback.format_exc().strip().splitlines()[-n:])


def parts_map(raw):
    return raw["part_id"] if isinstance(raw, dict) and isinstance(raw.get("part_id"), dict) else raw


def az_deg(p, ax):
    """azimuth from the front (-Y) toward character left (+X), degrees [0, 360) (p21 tunic_weights convention)."""
    return math.degrees(math.atan2(p[0] - ax[0], -(p[1] - ax[1]))) % 360.0


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def sections(man, frames):
    segs = [s for s in (man or {}).get("segments") or [] if isinstance(s, dict)]
    out = {}
    for s in segs:
        out[s["name"]] = [f for f in frames if int(s["start"]) <= f <= int(s["end"])]
    if not out:
        out["all"] = list(frames)
    return out


def props_holder(arm, key):
    pb = arm.pose.bones.get("PROPS")
    if pb is not None and key in pb.keys():
        return pb
    b = arm.data.bones.get("PROPS")
    if b is not None and key in b.keys():
        return b
    return None


# ---------------------------------------------------------------- sampling
class Geo:
    """topology-derived selections shared by both rigs (same mesh topology)."""

    def __init__(self, ctx, mo, V0):
        t = p2d.Topo(mo, ctx.parts)
        self.topo = t
        tun = (ctx.loops or {}).get("tunic") or {}
        self.axis = tuple(tun.get("axis_xy") or (0.0, 0.0))
        cz = tun.get("ceiling_z")
        self.ceiling_z = float(cz) if isinstance(cz, (int, float)) else None
        tp = t.polys_of_fams(("tunic", "skirt"))
        bp = t.polys_of_fams(("belt",))
        zb = float(V0[t.verts_of_polys(bp), 2].min()) if len(bp) else 0.408
        cen = t.poly_centroid(V0)
        skirt = tp[cen[tp, 2] < zb]
        ceil = set()
        if self.ceiling_z is not None:
            ceil = {int(q) for q in skirt.tolist() if np.all(np.abs(V0[t.poly_verts[int(q)], 2] - self.ceiling_z) <= CEIL_TOL)}
        self.walls = np.array(sorted(set(skirt.tolist()) - ceil), dtype=np.int64)
        self.wall_tris = t.tris_of_polys(self.walls)
        self.n_ceiling = len(ceil)
        self.belt_bottom = zb
        self.leg_polys = {x: t.polys_of_fams(("leg",), x) for x in ("l", "r")}
        self.leg_tris = {x: t.tris_of_polys(p) for x, p in self.leg_polys.items()}
        zc = self.ceiling_z if self.ceiling_z is not None else np.inf
        self.leg_verts = {x: t.verts_of_polys(p)[V0[t.verts_of_polys(p), 2] < zc] for x, p in self.leg_polys.items()}
        hem = next((r for r in tun.get("outer_rows") or [] if r.get("name") == "hem_outer"), None)
        self.hem = np.array(hem["verts"], dtype=np.int64) if hem else np.zeros(0, np.int64)
        if len(self.hem):
            H = V0[self.hem]
            self.hem_len0 = np.linalg.norm(np.roll(H, -1, axis=0) - H, axis=1)
            a = np.array([az_deg(p, self.axis) for p in H])
            self.hem_sign0 = np.sign([wrap(d) for d in (np.roll(a, -1) - a)])
        # poke_out needs ctx.topo / ctx.skirt_axis
        self.pctx = type("PokeCtx", (), {})()
        self.pctx.topo, self.pctx.skirt_axis = t, self.axis


def open_rig(path):
    bpy.ops.wm.open_mainfile(filepath=str(path), load_ui=False)
    arm = bpy.data.objects.get(P["arm"])
    mo = bpy.data.objects.get(P["mesh"])
    if arm is None or arm.type != "ARMATURE" or mo is None or mo.type != "MESH":
        raise Blocked(f"{rel(path)}: {P['arm']} / {P['mesh']} missing")
    act = bpy.data.actions.get(P["action"])
    if act is None:
        raise Blocked(f"{rel(path)}: action {P['action']!r} missing")
    import check_p1 as p1   # noqa: E402  (assign_action, fcurves_of)
    p1.assign_action(arm, act)
    bpy.context.scene.tool_settings.use_keyframe_insert_auto = False
    arm.data.pose_position = "POSE"
    f0, f1 = (int(round(x)) for x in act.frame_range)
    return arm, mo, act, list(range(f0, f1 + 1)), p1


def skirt_bones(arm):
    return [n for n in arm.data.bones.keys() if n.startswith("Skirt_")]


def sample(ctx, path, tag, follow_zero=False, mesh=True):
    arm, mo, act, frames, p1 = open_rig(path)
    info = {"blend": rel(path), "frames": [frames[0], frames[-1]], "n_frames": len(frames)}
    if follow_zero:
        h = props_holder(arm, P["prop"])
        muted = []
        for fc in p1.fcurves_of(act):
            if f'["{P["prop"]}"]' in (fc.data_path or ""):
                fc.mute = True
                muted.append(fc.data_path)
        ad = arm.animation_data
        drv = [fc.data_path for fc in (ad.drivers if ad else []) if fc.data_path.endswith(f'["{P["prop"]}"]')]
        if h is None:
            info["skirt_follow"] = f"PROPS {P['prop']} not found: compared as is"
        else:
            h[P["prop"]] = 0.0
            info["skirt_follow"] = {"set": 0.0, "muted_fcurves": muted, "drivers_on_prop": drv}
    V0 = None
    if mesh:
        me = mo.data
        co = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", co)
        M = np.array(mo.matrix_world, dtype=np.float64)
        V0 = co.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]
        if ctx.geo is None:
            ctx.geo = Geo(ctx, mo, V0)
        elif len(V0) != ctx.geo.topo.nv:
            raise Blocked(f"{rel(path)}: {len(V0)} verts, first rig {ctx.geo.topo.nv}")
    sk = skirt_bones(arm)
    mw = arm.matrix_world
    rest = {n: np.array(mw @ arm.data.bones[n].matrix_local, dtype=np.float64) for n in sk}
    rest_inv = {n: np.linalg.inv(m) for n, m in rest.items()}
    pel_rest = np.array(mw @ arm.data.bones["Pelvis"].matrix_local, dtype=np.float64) if "Pelvis" in arm.data.bones else None
    per = {"poke": {}, "hem": {}, "isect": {}, "tail_flip": {}, "hem_flip": {}, "delta": {}}
    g = ctx.geo
    sc = bpy.context.scene
    for f in frames:
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        ae = arm.evaluated_get(dg)
        W = {n: np.array(mw @ ae.pose.bones[n].matrix, dtype=np.float64) for n in sk}
        per["delta"][f] = {n: W[n] @ rest_inv[n] for n in sk}
        if not mesh:
            continue
        # every mesh / tail metric in the Pelvis rest frame (undo the Pelvis world delta: root / cog / pelvis
        # sections move the whole body, the skirt axis and the azimuths are defined at rest)
        Minv = np.eye(4)
        if pel_rest is not None:
            Minv = np.linalg.inv(np.array(mw @ ae.pose.bones["Pelvis"].matrix, dtype=np.float64) @ np.linalg.inv(pel_rest))
        V = p2d.eval_verts(mo) @ Minv[:3, :3].T + Minv[:3, 3]
        pk = {x: p2d.poke_out(g.pctx, V, g.leg_verts[x], g.walls) for x in ("l", "r")}
        per["poke"][f] = {"n": sum(v.get("n_poke_out", 0) for v in pk.values()),
                          "max_mm": max((v.get("max_poke_out_mm") or 0.0) for v in pk.values()),
                          "per_leg": {x: [v.get("n_poke_out", 0), v.get("max_poke_out_mm")] for x, v in pk.items()}}
        if len(g.hem):
            H = V[g.hem]
            r = np.linalg.norm(np.roll(H, -1, axis=0) - H, axis=1) / np.maximum(g.hem_len0, 1e-12)
            per["hem"][f] = (float(r.max()), float(r.min()))
            a = np.array([az_deg(p, g.axis) for p in H])
            sg = np.sign([wrap(d) for d in (np.roll(a, -1) - a)])
            per["hem_flip"][f] = int((sg != g.hem_sign0).sum())
        per["isect"][f] = {x: len(p2d.overlap_pairs(g.topo, V, g.leg_tris[x], g.wall_tris)) for x in ("l", "r")}
        tails = []
        for d in SKIRT_DIRS:
            n = f"Skirt_{d}_02"
            if n in ae.pose.bones:
                tw = np.array(mw @ ae.pose.bones[n].tail, dtype=np.float64)
                tails.append(az_deg(Minv[:3, :3] @ tw + Minv[:3, 3], g.axis))
        if len(tails) == len(SKIRT_DIRS):
            dif = [wrap(tails[(i + 1) % 8] - tails[i]) for i in range(8)]
            per["tail_flip"][f] = int(sum(1 for x in dif if x <= 0.0))
    info["skirt_bones"] = sk
    return info, frames, per


# ---------------------------------------------------------------- rows
def sec_stat(vals):
    return max(vals) if vals else None


def compare_poke(after, before, secs):
    """per section max poke verts / depth of `after` vs `before` (P2.8b / P2.9b)."""
    rows, cand_all = {}, True
    for name, fr in secs.items():
        a = [after["poke"][f] for f in fr if f in after["poke"]]
        b = [before["poke"][f] for f in fr if f in before["poke"]]
        ra = {"max_verts": sec_stat([x["n"] for x in a]), "max_mm": sec_stat([x["max_mm"] for x in a]),
              "frames_gt0": sum(1 for x in a if x["n"] > 0)}
        rb = {"max_verts": sec_stat([x["n"] for x in b]), "max_mm": sec_stat([x["max_mm"] for x in b]),
              "frames_gt0": sum(1 for x in b if x["n"] > 0)}
        worst = max(fr, key=lambda f: after["poke"].get(f, {}).get("n", 0)) if fr else None
        cand = (ra["max_verts"] is not None and rb["max_verts"] is not None and ra["max_verts"] <= rb["max_verts"]
                and ra["max_mm"] <= rb["max_mm"] + 1e-9)
        cand_all = cand_all and cand
        rows[name] = {"frames": [fr[0], fr[-1]] if fr else None, "after": ra, "before": rb,
                      "after_le_before": cand, "after_worst_frame": worst,
                      "after_worst_per_leg": after["poke"].get(worst, {}).get("per_leg") if worst else None}
    spot = {f: {"after": after["poke"].get(f), "before": before["poke"].get(f)} for f in (477, 513)}
    return rows, cand_all, spot


def p29b(ctx):
    rows, cand_all, spot = compare_poke(ctx.cur, ctx.p2f, ctx.sec_cur)
    zero = {f: (v["after"] or {}).get("n") == 0 for f, v in spot.items()}
    return {"sections": rows, "after_le_before_all_sections": cand_all, "spot_frames_after_before": spot,
            "target_zero_at_f477_f513 (report)": zero, "before_rig": ctx.p2f_info,
            "rule": "as P2.8b (check_p2_deform.poke_out, springs off, Pelvis rest frame); before = the P2e rig "
                    "(backup_pre_P2f)"}, cand_all


def covered(g, V, verts):
    """rest inside test of the poke-out rule (check_p2_deform.poke_out ray walk, copied): horizontal ray from the skirt
    axis (tunic.axis_xy, vertex z) through the vertex against the wall faces; covered = >= 1 wall hit beyond the vertex;
    clearance = first hit beyond - vertex radius (m); footprint = any hit."""
    from mathutils import Vector
    bvh = p2d.bvh_of(V, g.topo.tri_v[g.wall_tris])
    out = []
    for v in verts.tolist():
        q = V[v]
        rv = q[:2] - np.asarray(g.axis, dtype=np.float64)
        r = float(np.linalg.norm(rv))
        if r < 1e-9:
            out.append((v, False, None, False))
            continue
        d = Vector((rv[0] / r, rv[1] / r, 0.0))
        o, hits, travelled = Vector((g.axis[0], g.axis[1], q[2])), [], 0.0
        for _ in range(64):
            h = bvh.ray_cast(o, d, p2d.POKE_RAY_MAX - travelled)
            if h[0] is None:
                break
            travelled += h[3]
            hits.append(travelled)
            o = h[0] + d * p2d.RAY_EPS
            travelled += p2d.RAY_EPS
        beyond = [x for x in hits if x > r]
        out.append((v, bool(beyond), (beyond[0] - r) if beyond else None, bool(hits)))
    return out


def read_rest(path):
    bpy.ops.wm.open_mainfile(filepath=str(path), load_ui=False)
    mo = bpy.data.objects.get(P["mesh"])
    if mo is None or mo.type != "MESH":
        raise Blocked(f"{rel(path)}: {P['mesh']} missing")
    me = mo.data
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    M = np.array(mo.matrix_world, dtype=np.float64)
    return co.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]


def p29a(ctx):
    import check_p2_static as p2s   # noqa: E402  (same zone manifest reader as P2.1)
    ids, info = p2s.zone_manifest(P["zone"])
    if ids is None:
        raise Blocked(info)
    for k in ("retopo_cur", "retopo_bak"):
        if not Path(P[k]).exists():
            raise Blocked(f"missing input {rel(P[k])}")
    Vb = read_rest(P["retopo_bak"])
    Vc = read_rest(P["retopo_cur"])
    if len(Vb) != len(Vc):
        raise Blocked(f"vertex count differs: now {len(Vc)}, backup {len(Vb)}")
    g = ctx.geo
    zone = np.array(sorted(ids), dtype=np.int64)
    bad_ids = [int(i) for i in zone if i < 0 or i >= len(Vc)]
    zone = zone[(zone >= 0) & (zone < len(Vc))]
    d = np.linalg.norm(Vc - Vb, axis=1)
    disp = d > DISP_TOL
    inz = np.zeros(len(Vc), dtype=bool)
    inz[zone] = True
    out_zone = np.nonzero(disp & ~inz)[0]
    hem_z = float(Vc[g.hem, 2].min()) if len(g.hem) else None
    below = np.nonzero(disp & (Vc[:, 2] < hem_z))[0] if hem_z is not None else np.zeros(0, np.int64)
    cov = covered(g, Vc, zone)
    not_cov = [v for v, c, _cl, _fp in cov if not c]
    clear = [cl for _v, c, cl, _fp in cov if c and cl is not None]
    wall_ids = set(g.topo.verts_of_polys(g.walls).tolist())
    ok = (not bad_ids and not len(out_zone) and not len(below) and not not_cov and bool(clear) and min(clear) > 0.0)
    return {"zone": info, "n_zone_verts": int(len(zone)), "zone_ids_out_of_range": bad_ids[:20],
            "zone_verts_displaced": int((disp & inz).sum()), "zone_max_displacement_mm": mm(d[inz].max()) if inz.any() else None,
            "displaced_outside_zone": int(len(out_zone)), "displaced_outside_zone_first": out_zone[:20].tolist(),
            "displaced_outside_zone_max_mm": mm(d[out_zone].max()) if len(out_zone) else None,
            "hem_z_mm": mm(hem_z) if hem_z is not None else None, "displaced_below_hem": int(len(below)),
            "zone_verts_not_covered": not_cov[:20], "n_zone_verts_not_covered": len(not_cov),
            "min_clearance_to_wall_mm": mm(min(clear)) if clear else None,
            "zone_verts_that_are_wall_verts": sorted(set(zone.tolist()) & wall_ids)[:20],
            "retopo_now": rel(P["retopo_cur"]), "retopo_backup": rel(P["retopo_bak"]), "disp_tol_m": DISP_TOL,
            "rule": "displacement = |p20 output - backup| per vertex (same indices); covered = the poke-out ray from "
                    "the skirt axis through the vertex meets the skirt wall beyond it at rest; clearance = distance "
                    "to that first wall hit"}, ok


def p28b(ctx):
    rows, cand_all = {}, True
    for name, fr in ctx.sec_cur.items():
        a = [ctx.cur["poke"][f] for f in fr if f in ctx.cur["poke"]]
        b = [ctx.bak["poke"][f] for f in fr if f in ctx.bak["poke"]]
        ra = {"max_verts": sec_stat([x["n"] for x in a]), "max_mm": sec_stat([x["max_mm"] for x in a]),
              "frames_gt0": sum(1 for x in a if x["n"] > 0)}
        rb = {"max_verts": sec_stat([x["n"] for x in b]), "max_mm": sec_stat([x["max_mm"] for x in b]),
              "frames_gt0": sum(1 for x in b if x["n"] > 0)}
        worst = max(fr, key=lambda f: ctx.cur["poke"].get(f, {}).get("n", 0)) if fr else None
        cand = (ra["max_verts"] is not None and rb["max_verts"] is not None and ra["max_verts"] <= rb["max_verts"]
                and ra["max_mm"] <= rb["max_mm"] + 1e-9)
        cand_all = cand_all and cand
        rows[name] = {"frames": [fr[0], fr[-1]] if fr else None, "after": ra, "before": rb,
                      "after_le_before": cand, "after_worst_frame": worst,
                      "after_worst_per_leg": ctx.cur["poke"].get(worst, {}).get("per_leg") if worst else None}
    spot = {f: {"after": ctx.cur["poke"].get(f), "before": ctx.bak["poke"].get(f)} for f in (477, 513)}
    return {"sections": rows, "candidate_after_le_before_all_sections": cand_all, "spot_frames_after_before": spot,
            "rule": "check_p2_deform.poke_out on the evaluated PL_mesh (springs off), leg_l + leg_r vertices with rest z "
                    f"below tunic.ceiling_z; walls = tunic faces below the belt bottom without the {ctx.geo.n_ceiling} "
                    "ceiling faces; section max over frames"}, True


def p28c(ctx):
    def one(data, secs):
        rows = {}
        for name, fr in secs.items():
            h = [data["hem"][f] for f in fr if f in data["hem"]]
            rows[name] = {"hem_ratio_max": rnd(max(x[0] for x in h), 4) if h else None,
                          "hem_ratio_min": rnd(min(x[1] for x in h), 4) if h else None,
                          "isect_pairs_max": {x: sec_stat([data["isect"][f][x] for f in fr if f in data["isect"]])
                                              for x in ("l", "r")},
                          "tail_order_flips_max": sec_stat([data["tail_flip"][f] for f in fr if f in data["tail_flip"]]),
                          "hem_order_flips_max": sec_stat([data["hem_flip"][f] for f in fr if f in data["hem_flip"]])}
        return rows
    return {"current": one(ctx.cur, ctx.sec_cur), "backup_reference": one(ctx.bak, ctx.sec_bak),
            "hem_ring": {"row": "tunic.outer_rows hem_outer", "n_verts": int(len(ctx.geo.hem))},
            "rules": {"hem_ratio": "edge length / rest edge length over the hem_outer ring (cyclic)",
                      "isect": "polygon pairs of skirt wall faces x leg_x faces whose triangles intersect "
                               "(check_p2_deform.overlap_pairs)",
                      "tail_order": "cyclic azimuth differences of the Skirt_*_02 tails in F, FL, L, BL, B, BR, R, FR "
                                    "order about tunic.axis_xy; a difference <= 0 = a crossing",
                      "hem_order": "azimuth difference sign of consecutive hem ring vertices vs rest",
                      "frame": "all mesh / tail metrics in the Pelvis rest frame (Pelvis world delta removed per frame)"}}, True


def p28d(ctx):
    names = sorted(set(ctx.zero_info.get("skirt_bones", [])) & set(ctx.bak_info.get("skirt_bones", [])))
    common = sorted(set(ctx.zero["delta"]) & set(ctx.bak["delta"]))
    worst, at, per = 0.0, None, {n: 0.0 for n in names}
    for f in common:
        for n in names:
            d = float(np.abs(ctx.zero["delta"][f][n] - ctx.bak["delta"][f][n]).max())
            per[n] = max(per[n], d)
            if d > worst:
                worst, at = d, [f, n]
    ok = bool(names) and bool(common) and worst <= DELTA_TOL
    return {"max_abs": float(f"{worst:.3e}"), "at": at, "n_frames": len(common), "n_bones": len(names),
            "per_bone_max": {n: float(f"{v:.3e}") for n, v in per.items()}, "current_rig": ctx.zero_info,
            "backup_rig": {k: v for k, v in ctx.bak_info.items() if k != "skirt_bones"},
            "method": "pose world @ rest world^-1 per Skirt_* bone (roll-invariant), current with skirt_follow = 0"}, ok


def p28f(ctx):
    path = Path(P["follow"])
    if not path.exists():
        raise Blocked(f"missing input {rel(path)}")
    man = p2d.load_json(path)
    bpy.ops.wm.open_mainfile(filepath=str(P["cur"]), load_ui=False)
    arm = bpy.data.objects.get(P["arm"])
    if arm is None:
        raise Blocked(f"{P['arm']} missing in {rel(P['cur'])}")
    # T255: bone names only from the structured keys (panels[*] hinge / def_bone / sources, probes[*] bone /
    # copy_location_from, prop.bone); a trailing " head" / " tail" names a bone point and is stripped
    refs = []

    def add(where, v):
        vals = v if isinstance(v, list) else [v]
        for x in vals:
            if isinstance(x, str) and x.strip():
                refs.append((where, re.sub(r"\s+(head|tail)$", "", x.strip())))
    for k, pn in ((man.get("panels") or {}) if isinstance(man, dict) else {}).items():
        if isinstance(pn, dict):
            for key in ("hinge", "def_bone", "def", "sources"):
                if key in pn:
                    add(f"panels.{k}.{key}", pn[key])
    for k, pr in ((man.get("probes") or {}) if isinstance(man, dict) else {}).items():
        if isinstance(pr, dict):
            for key in ("bone", "target", "target_bone", "copy_location_from", "parent"):
                if key in pr:
                    add(f"probes.{k}.{key}", pr[key])
    if isinstance(man, dict) and isinstance(man.get("prop"), dict) and "bone" in man["prop"]:
        add("prop.bone", man["prop"]["bone"])
    bone_like = sorted({b for _w, b in refs})
    missing = [f"{w}: {b}" for w, b in refs if b not in arm.pose.bones]
    strs = [b for _w, b in refs] + [str((pn or {}).get("def_constraint", "")).split(" ")[0]
                                    for pn in ((man.get("panels") or {}) if isinstance(man, dict) else {}).values()
                                    if isinstance(pn, dict)]
    h = props_holder(arm, P["prop"])
    prop = None
    prop_ok = False
    if h is not None:
        v = h[P["prop"]]
        try:
            ui = h.id_properties_ui(P["prop"]).as_dict()
        except Exception:
            ui = {}
        prop = {"value": v, "type": type(v).__name__, "ui": {k: ui.get(k) for k in ("min", "max", "default")}}
        prop_ok = (isinstance(v, float) and ui.get("min") == 0.0 and ui.get("max") == 1.0
                   and isinstance(ui.get("default"), (int, float)) and abs(ui["default"] - FOLLOW_DEFAULT) <= 1e-9)
    key = f'["{P["prop"]}"]'
    ad = arm.animation_data
    drv = [fc.data_path for fc in (ad.drivers if ad else [])
           if any(key in (t.data_path or "") for var in fc.driver.variables for t in var.targets)]
    cons = []
    for pb in arm.pose.bones:
        for c in pb.constraints:
            if any(s == c.name for s in strs) or (pb.name.startswith(("MCH_skirt", "MCH_Skirt")) and c.type != "IK"):
                cons.append(f"{pb.name}/{c.type}:{c.name}")
    ok = prop_ok and not missing and bool(drv or cons)
    return {"file": rel(path), "top_level_keys": sorted(man) if isinstance(man, dict) else type(man).__name__,
            "bone_names_in_manifest": bone_like, "missing_bones": missing, "prop": prop, "prop_ok": prop_ok,
            "drivers_reading_prop": drv[:40], "n_drivers_reading_prop": len(drv), "constraints_found": cons[:40],
            "n_constraints_found": len(cons)}, ok


IDS = (
    ("P2.8b", "(report) per rigtest section, springs off: leg-through-skirt poke vertices and depth, current vs backup; "
              "candidate flag after <= before", CALIB),
    ("P2.8c", "(report) hem ring stretch / compression vs rest, skirt x leg intersections, adjacent-panel crossings",
     CALIB),
    ("P2.8d", f"skirt_follow = 0 reproduces the backup DEF skirt bones: world pose delta max abs element <= {DELTA_TOL:g}",
     "blocking (d-02 section 8)"),
    ("P2.8f", "skirt_follow.json present; PROPS skirt_follow float, UI default 0.5, range [0, 1]; manifest bone names on "
              "PL_rig; >= 1 driver / constraint reading or named by the manifest", ""),
    ("P2.9a", "thigh_thin_zone.json present; every zone vertex covered by the skirt wall at rest (poke-out rule ray, "
              f"clearance > 0, min reported); 0 vertices displaced (> {DISP_TOL:g} m) outside the zone and 0 below "
              "the hem, p20 output vs backup_pre_P2f pl_r01_retopo", "blocking (d-02 section 10)"),
    ("P2.9b", "per rigtest section, springs off: poke-out after (current) <= before (P2e rig, backup_pre_P2f) in verts "
              "and depth; f477 / f513 = 0 reported as the target", "blocking after <= before (d-02 section 10)"),
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


class Ctx:
    def __init__(self):
        self.parts = self.loops = None
        self.geo = None
        self.cur = self.bak = self.zero = None
        self.sec_cur = self.sec_bak = None
        self.bak_info = self.zero_info = {}
        self.p2f = None
        self.p2f_info = {}


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
    if P["man_bak"] is None:
        cand = Path(P["bak"]).parent / "data" / "rigtest_manifest.json"
        P["man_bak"] = cand if cand.exists() else P["man_cur"]
    if P["man_p2f"] is None:
        cand = Path(P["bak_p2f"]).parent / "data" / "rigtest_manifest.json"
        P["man_p2f"] = cand if cand.exists() else P["man_cur"]


def main():
    parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    ev = Evidence()
    ev.add_input(__file__)
    ev.add_input(p2d.__file__)
    ctx = Ctx()
    th = {i: t for i, t, _n in IDS}
    nt = {i: n for i, _t, n in IDS}
    ids = [i for i, _t, _n in IDS]
    # T259: P2.9 retired (P2f reverted 2026-09-27): its rows exist only when the zone manifest exists
    p29_on = Path(P["zone"]).exists()
    if not p29_on:
        ids = [i for i in ids if not i.startswith("P2.9")]
        ev.extra["P2.9"] = "P2.9 retired (P2f reverted 2026-09-27)"
    P29_KEYS = ("bak_p2f", "man_p2f", "retopo_cur", "retopo_bak", "zone")
    done = set()

    def put(cid, measured, ok):
        ev.criterion(cid, measured, th[cid], ok, nt[cid])
        done.add(cid)

    def block(cids, reason):
        for c in cids:
            if c not in done and c in ids:
                put(c, f"blocked: {reason}", False)

    try:
        inputs = [k for k in PATH_KEYS if k != "out" and (p29_on or k not in P29_KEYS)]
        miss = [rel(P[k]) for k in inputs if not Path(P[k]).exists()]
        for k in inputs:
            ev.add_input(P[k])
        for q in sorted(HERE.glob("p2*.py")):
            ev.add_input(q)
        _STATE["missing"] = miss
        ev.extra["names"] = {k: P[k] for k in ("arm", "mesh", "action", "prop")}
        if not Path(P["parts"]).exists() or not Path(P["loops"]).exists():
            block(ids, "missing parts.json / retopo_loops.json")
            return
        ctx.parts = parts_map(p2d.load_json(P["parts"]))
        ctx.loops = p2d.load_json(P["loops"])
        man_cur = p2d.load_json(P["man_cur"]) if Path(P["man_cur"]).exists() else None
        man_bak = p2d.load_json(P["man_bak"]) if Path(P["man_bak"]).exists() else man_cur
        rigs_ok = Path(P["cur"]).exists() and Path(P["bak"]).exists()
        if not rigs_ok:
            block(["P2.8b", "P2.8c", "P2.8d", "P2.9a", "P2.9b"], "missing input " + ", ".join(
                rel(P[k]) for k in ("cur", "bak") if not Path(P[k]).exists()))
        else:
            try:
                bak_info, fb, ctx.bak = sample(ctx, P["bak"], "bak")
                ctx.bak_info = bak_info
                ctx.sec_bak = sections(man_bak, fb)
                cur_info, fc, ctx.cur = sample(ctx, P["cur"], "cur")
                ctx.sec_cur = sections(man_cur, fc)
                ev.extra["rigs"] = {"current": {k: v for k, v in cur_info.items() if k != "skirt_bones"},
                                    "backup": {k: v for k, v in bak_info.items() if k != "skirt_bones"}}
                for cid, fn in (("P2.8b", p28b), ("P2.8c", p28c)):
                    try:
                        put(cid, *fn(ctx))
                    except Blocked as e:
                        block([cid], str(e))
                zi, _fz, ctx.zero = sample(ctx, P["cur"], "zero", follow_zero=True, mesh=False)
                ctx.zero_info = zi
                put("P2.8d", *p28d(ctx))
                # T258 P2.9b: current vs the P2e rig (backup_pre_P2f); T259: skipped when P2.9 is retired
                if not p29_on:
                    pass
                elif Path(P["bak_p2f"]).exists():
                    try:
                        pi, _fp, ctx.p2f = sample(ctx, P["bak_p2f"], "p2f")
                        ctx.p2f_info = {k: v for k, v in pi.items() if k != "skirt_bones"}
                        put("P2.9b", *p29b(ctx))
                    except (Blocked, p2d.Blocked) as e:
                        block(["P2.9b"], str(e))
                else:
                    block(["P2.9b"], f"missing input {rel(P['bak_p2f'])}")
                if p29_on:
                    try:
                        put("P2.9a", *p29a(ctx))
                    except (Blocked, p2d.Blocked) as e:
                        block(["P2.9a"], str(e))
            except (Blocked, p2d.Blocked) as e:
                block(["P2.8b", "P2.8c", "P2.8d", "P2.9a", "P2.9b"], str(e))
        try:
            put("P2.8f", *p28f(ctx))
        except Blocked as e:
            block(["P2.8f"], str(e))
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
