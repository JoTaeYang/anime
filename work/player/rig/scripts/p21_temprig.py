"""p21_temprig.py - T220 (P2.2 input): temporary rig on the P2a retopo for the deform test (spec d-02 §2 P2.2).

Input : work/player/rig/pl_r01_retopo.blend (PL_mesh, written by p20_retopo.py; opened as the main file, not saved over)
        work/player/rig/data/p1_manifest.json (P1 skeleton v0 bone table written by p10_crude_rig.py: names, parents,
            head / tail, roll hint, deform flags)
        work/player/rig/data/parts.json, work/player/rig/data/retopo_loops.json (p20)
        work/player/inspect/P0b/p0b_measure.json (skirt belt / hem heights, as p10)
Output: work/player/rig/pl_r01t_temprig.blend (save copy): PL_mesh + armature object `PL_temprig` (identity transform,
        41 bones = the P1 v0 layout, all unconnected), vertex groups = deform bone names, one Armature modifier.
        This rig is test-only (P2.3 builds the real skeleton).

CLI:  blender --background --factory-startup work/player/rig/pl_r01_retopo.blend --python p21_temprig.py

Weights (adapted from p10_crude_rig.py skin rules and goblin s03_temprig.py):
- rigid parts 100 % on one bone (p10 RIGID table): head, eyes -> Head; fists -> Hand_x; shoes, cuffs -> Foot_x;
  belt, pouch -> Pelvis; sleeves -> Arm_x; scarf, scarf_tail -> Spine_02.
- arm / leg tubes: Blender bone heat (ARMATURE_AUTO) on a temporary copy holding only that tube, with only the limb
  chain deforming (Arm/Forearm/Hand or Thigh/Calf/Foot, as p10); vertices without heat weight -> nearest chain
  bone segment 100 % (count reported).
- tunic outer wall: p10 tunic_weights (Spine_01 / Spine_02 above the belt top, Pelvis in the belt band, below the belt
  bottom azimuth blend between the two neighbouring skirt chains, height blend Skirt_*_01 -> Skirt_*_02, Pelvis
  fading out over the top 20 % of the skirt). Inner-wall rows take the weights of their paired outer vertex (same
  row, same column) so the double wall moves together; the ceiling ring uses the rule at its own position; the
  ceiling interior is Pelvis 100 %.
- every vertex: <= MAX_INF influences (largest kept), normalised (s03).
"""
import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
IN_BLEND = RIG / "pl_r01_retopo.blend"
OUT_BLEND = RIG / "pl_r01t_temprig.blend"
MANIFEST = RIG / "data" / "p1_manifest.json"
PARTS_JSON = RIG / "data" / "parts.json"
LOOPS_JSON = RIG / "data" / "retopo_loops.json"
P0B = REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json"
MESH = "PL_mesh"
ARM = "PL_temprig"
COLL = "PL"
MAX_INF = 4
EPS_W = 1e-4
SKIRT_AZ = [("F", 0), ("FL", 45), ("L", 90), ("BL", 135), ("B", 180), ("BR", 225), ("R", 270), ("FR", 315)]
RIGID = {"head": "Head", "eye_l": "Head", "eye_r": "Head", "fist_l": "Hand_L", "fist_r": "Hand_R",
         "shoe_l": "Foot_L", "shoe_r": "Foot_R", "cuff_l": "Foot_L", "cuff_r": "Foot_R",
         "belt": "Pelvis", "pouch": "Pelvis", "sleeve_l": "Arm_L", "sleeve_r": "Arm_R",
         "scarf": "Spine_02", "scarf_tail": "Spine_02"}
AUTO = {"arm_l": ["Arm_L", "Forearm_L", "Hand_L"], "arm_r": ["Arm_R", "Forearm_R", "Hand_R"],
        "leg_l": ["Thigh_L", "Calf_L", "Foot_L"], "leg_r": ["Thigh_R", "Calf_R", "Foot_R"]}


def log(msg):
    print(f"[p21] {msg}")
    sys.stdout.flush()


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab[None, :]), axis=1)


def smooth01(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def tunic_weights(v, p0b, heads):
    """p10_crude_rig.tunic_weights (unchanged rule; the bottom-cap centre term never fires on the open skirt)."""
    sk = p0b["skirt"]
    z_bb, z_bt = sk["belt_z"]
    z_hem = sk["hem_z_min"]
    cx, cy = sk["hem_ellipse"]["center_xy"]
    x, y, z = v
    if z >= z_bt:
        s = smooth01((z - (heads["Spine_02"][2] - 0.04)) / 0.08)
        w = {"Spine_01": 1.0 - s, "Spine_02": s}
        p = max(0.0, 1.0 - (z - z_bt) / 0.03)
        return {k: val * (1.0 - p) for k, val in w.items()} | {"Pelvis": p}
    if z >= z_bb:
        return {"Pelvis": 1.0}
    t = max(0.0, min(1.0, (z_bb - z) / (z_bb - z_hem)))
    w_pel = max(0.0, 1.0 - t / 0.2)
    r = math.hypot(x - cx, y - cy)
    w_pel = max(w_pel, 1.0 - r / 0.08)
    w02 = smooth01((t - 0.35) / 0.4)
    az = math.degrees(math.atan2(x - cx, -(y - cy))) % 360.0
    i0 = int(az // 45.0) % 8
    i1 = (i0 + 1) % 8
    f = (az - i0 * 45.0) / 45.0
    w = {"Pelvis": w_pel}
    for tag_i, wa in ((i0, 1.0 - f), (i1, f)):
        tag = SKIRT_AZ[tag_i][0]
        w[f"Skirt_{tag}_01"] = w.get(f"Skirt_{tag}_01", 0.0) + (1.0 - w_pel) * wa * (1.0 - w02)
        w[f"Skirt_{tag}_02"] = w.get(f"Skirt_{tag}_02", 0.0) + (1.0 - w_pel) * wa * w02
    return {k: val for k, val in w.items() if val > 1e-6}


def build_armature(bones):
    ad = bpy.data.armatures.new(ARM)
    rig = bpy.data.objects.new(ARM, ad)
    coll = bpy.data.collections.get(COLL) or bpy.context.scene.collection
    coll.objects.link(rig)
    rig.matrix_world = Matrix.Identity(4)
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    vl.objects.active = rig
    rig.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = ad.edit_bones
    for b in bones:
        eb = ebs.new(b["name"])
        eb.head, eb.tail = Vector(b["head"]), Vector(b["tail"])
        eb.align_roll(Vector(b["roll_hint_z"]))
        eb.use_deform = b["use_deform"]
    for b in bones:
        eb = ebs[b["name"]]
        eb.parent = ebs[b["parent"]] if b["parent"] else None
        eb.use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")
    for pb in rig.pose.bones:
        pb.rotation_mode = "QUATERNION"
    return rig


def heat_part(gm, vpart, pid, rig, chain):
    """ARMATURE_AUTO on a temporary copy of one part with only `chain` deforming -> {vertex index: {bone: w}}."""
    me = gm.data
    nv = len(me.vertices)
    tmp_me = me.copy()
    tmp_me.name = "_p21_tmp"
    tmp = bpy.data.objects.new("_p21_tmp", tmp_me)
    bpy.context.scene.collection.objects.link(tmp)
    oi = tmp_me.attributes.new("_orig_idx", "INT", "POINT")
    oi.data.foreach_set("value", np.arange(nv, dtype=np.int32))
    bm = bmesh.new()
    bm.from_mesh(tmp_me)
    bm.verts.ensure_lookup_table()
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if vpart[v.index] != pid], context="VERTS")
    bm.to_mesh(tmp_me)
    bm.free()
    tmp_me.update()
    orig = np.empty(len(tmp_me.vertices), dtype=np.int32)
    tmp_me.attributes["_orig_idx"].data.foreach_get("value", orig)
    keep = {b.name: b.use_deform for b in rig.data.bones}
    for b in rig.data.bones:
        b.use_deform = b.name in chain
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    tmp.select_set(True)
    rig.select_set(True)
    vl.objects.active = rig
    res = bpy.ops.object.parent_set(type="ARMATURE_AUTO")
    for b in rig.data.bones:
        b.use_deform = keep[b.name]
    gname = {g.index: g.name for g in tmp.vertex_groups}
    out = {}
    for v in tmp_me.vertices:
        out[int(orig[v.index])] = {gname[g.group]: g.weight for g in v.groups if gname[g.group] in chain}
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.meshes.remove(tmp_me)
    return out, res


def main():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    parts = json.loads(PARTS_JSON.read_text(encoding="utf-8"))["part_id"]
    loops = json.loads(LOOPS_JSON.read_text(encoding="utf-8"))
    p0b = json.loads(P0B.read_text(encoding="utf-8"))
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    gm = bpy.data.objects.get(MESH)
    if gm is None or gm.type != "MESH":
        raise RuntimeError(f"missing {MESH}")
    if not np.allclose(np.array(gm.matrix_world), np.eye(4)):
        raise RuntimeError(f"{MESH} transform is not identity")
    if len(gm.vertex_groups) or len(gm.modifiers) or bpy.data.objects.get(ARM) is not None:
        raise RuntimeError("input already has vertex groups / modifiers / armature")
    me = gm.data
    nv = len(me.vertices)
    co = np.empty(nv * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    pid_f = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid_f)
    vpart = np.full(nv, -1, dtype=np.int32)
    for poly in me.polygons:
        for vi in poly.vertices:
            if vpart[vi] not in (-1, pid_f[poly.index]):
                raise RuntimeError(f"vertex {vi} shared by two parts")
            vpart[vi] = pid_f[poly.index]
    if (vpart < 0).any():
        raise RuntimeError("loose vertices")

    bones = manifest["bones"]
    rig = build_armature(bones)
    deform = [b["name"] for b in bones if b["use_deform"]]
    bi = {n: i for i, n in enumerate(deform)}
    heads = {b["name"]: b["head"] for b in bones}
    log(f"ARMATURE {ARM}: {len(rig.data.bones)} bones from {MANIFEST.name} (P1 v0 layout), deform {len(deform)}, "
        f"connected {sum(1 for b in rig.data.bones if b.use_connect)}")

    W = np.zeros((nv, len(deform)))
    # rigid
    for pname, bname in RIGID.items():
        W[vpart == parts[pname], bi[bname]] = 1.0
    # tubes
    for pname, chain in AUTO.items():
        hw, res = heat_part(gm, vpart, parts[pname], rig, chain)
        idx = np.nonzero(vpart == parts[pname])[0]
        for i in idx:
            for bn, w in hw.get(int(i), {}).items():
                W[i, bi[bn]] = w
        zero = idx[W[idx].sum(axis=1) <= EPS_W]
        if len(zero):
            segs = [(np.array(rig.data.bones[n].head_local), np.array(rig.data.bones[n].tail_local)) for n in chain]
            dist = np.stack([seg_dist(co[zero], a, b) for a, b in segs], axis=1)
            near = dist.argmin(axis=1)
            W[zero] = 0.0
            for z_, k in zip(zero, near):
                W[z_, bi[chain[k]]] = 1.0
        log(f"HEAT {pname}: {sorted(res)} chain {chain}, verts {len(idx)}, heat-failed filled by nearest segment "
            f"{len(zero)}")
    # tunic
    t = loops["tunic"]
    tid = parts["tunic"]
    tidx = np.nonzero(vpart == tid)[0]
    pair = {}
    for r in t["inner_rows"]:
        if r["paired_outer_row"] is not None:
            outer = t["outer_rows"][r["paired_outer_row"]]["verts"]
            for vi, vo in zip(r["verts"], outer):
                pair[vi] = vo
    ceil = set(t["ceiling_interior"])
    for i in tidx:
        if int(i) in ceil:
            W[i, bi["Pelvis"]] = 1.0
            continue
        src = pair.get(int(i), int(i))
        for bn, w in tunic_weights(co[src], p0b, heads).items():
            W[i, bi[bn]] = w
    log(f"TUNIC verts {len(tidx)}: inner-wall verts paired to outer {len(pair)}, ceiling interior (Pelvis) {len(ceil)}")

    # limit + normalise
    before = int(((W > EPS_W).sum(axis=1) > MAX_INF).sum())
    order = np.argsort(-W, axis=1)
    mask = np.zeros_like(W, dtype=bool)
    np.put_along_axis(mask, order[:, :MAX_INF], True, axis=1)
    W = np.where(mask & (W > EPS_W), W, 0.0)
    s = W.sum(axis=1)
    if (s <= 0).any():
        raise RuntimeError(f"{int((s <= 0).sum())} vertices without weight")
    W = W / s[:, None]
    log(f"LIMIT verts with > {MAX_INF} influences before {before}; after max {int((W > 0).sum(axis=1).max())}, "
        f"|sum-1| max {float(np.abs(W.sum(axis=1) - 1).max()):.2e}")
    for n in deform:
        col = W[:, bi[n]]
        idx = np.nonzero(col > 0)[0]
        if not len(idx):
            continue
        vg = gm.vertex_groups.new(name=n)
        for i in idx:
            vg.add([int(i)], float(col[i]), "REPLACE")
    mod = gm.modifiers.new("Armature", "ARMATURE")
    mod.object = rig
    mod.use_vertex_groups = True
    mod.use_bone_envelopes = False

    # report
    inv = {v: k for k, v in parts.items()}
    for pv in sorted(inv):
        m = vpart == pv
        dom = np.bincount(W[m].argmax(axis=1), minlength=len(deform))
        cnt = (W[m] > 0).sum(axis=1)
        log(f"WEIGHTS {inv[pv]:10s} ({int(m.sum())} v): influences hist "
            f"{ {k: int((cnt == k).sum()) for k in range(1, MAX_INF + 1)} } dominant "
            + ", ".join(f"{deform[i]}:{int(dom[i])}" for i in np.argsort(-dom) if dom[i] > 0))
    sk = [n for n in deform if n.startswith("Skirt_")]
    log("SKIRT bones verts with weight > 0 / sum of weights: " + ", ".join(
        f"{n} {int((W[:, bi[n]] > 0).sum())}/{W[:, bi[n]].sum():.1f}" for n in sk))
    no_w = [n for n in deform if not (W[:, bi[n]] > 0).any()]
    log(f"DEFORM bones without weights: {no_w}")

    vl = bpy.context.view_layer
    for pb in rig.pose.bones:
        pb.rotation_quaternion = Quaternion()
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)
    vl.update()
    dg = bpy.context.evaluated_depsgraph_get()
    e = gm.evaluated_get(dg)
    em = e.to_mesh()
    ev = np.empty(len(em.vertices) * 3)
    em.vertices.foreach_get("co", ev)
    e.to_mesh_clear()
    log(f"REST deformation (evaluated vs original) max {np.linalg.norm(ev.reshape(-1, 3) - co, axis=1).max() * 1000:.6f} mm")

    for o in vl.objects:
        o.select_set(False)
    vl.objects.active = rig
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    log(f"OUTPUT {OUT_BLEND}; objects {sorted((o.name, o.type) for o in bpy.data.objects)}; "
        f"{MESH} modifiers {[(m.type, m.object.name) for m in gm.modifiers]}, vertex groups {len(gm.vertex_groups)}")
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
