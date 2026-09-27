"""p22_skeleton.py - T230 (P2.3): skeleton v1 of the player on the P2a retopo (spec d-02 §2 P2.3).

Input : work/player/rig/pl_r01_retopo.blend (PL_mesh; opened as the main file, not saved over)
        work/player/rig/data/p1_manifest.json (P1 skeleton v0: bone names, parents, deform flags, roll hints; p10)
        work/player/rig/data/retopo_loops.json, work/player/rig/data/parts.json (p20)
        work/player/inspect/P0b/p0b_measure.json (landmarks)
Output: work/player/rig/pl_r02_skeleton.blend (save copy): PL_mesh unchanged (no weights / modifier) + armature
            object `PL_rig` (identity transform) in collection `RIG`
        work/player/rig/data/canonical_skeleton.json (goblin s04 schema: meta + bones [name, parent, use_connect,
            head, tail, roll, use_deform, rest_matrix])

CLI:  blender --background --factory-startup work/player/rig/pl_r01_retopo.blend --python p22_skeleton.py

Skeleton v1 = the P1 v0 tree (41 bones, names / parents / deform flags / roll hints of p1_manifest.json), with
positions (copied / adapted from work/goblin_swing/rig/scripts/s04_skeleton.py: ring centre = mean PL_mesh co of
the ring verts; canonical JSON; all bones unconnected - playbook §3 / goblin G8 - and p10_crude_rig.py
skeleton_table for the torso, head, sockets and feet):
  shoulder / hip  = P0b landmarks (as p10);  elbow = centre(elbow_x_1), wrist = centre(wrist_x_1),
  knee = centre(knee_x_1), ankle = centre(ankle_x_1);  fist centre = P0b landmark (Hand tail, WeaponSocket head).
  Every left / right pair is symmetrised (|x|, y, z averaged over both sides, then mirrored; asymmetry reported).
  Pelvis / Spine_01 / Spine_02 / Neck / Head / Clavicle / Foot / HeadEquipmentSocket / WeaponSocket: p10 rules
  (head top = max z of the PL_mesh head part).  BackSocket / BackWeaponSocket: ray onto the PL_mesh tunic part at
  the back centre line (z 0.64 / 0.58, as p10 on the source tunic).
  Skirt_<az>_01 head = outer belt_bot ring vertex of the skirt column, Skirt_<az>_01 tail = Skirt_<az>_02 head =
  outer skirt_2 ring vertex (z .348, the middle ring), Skirt_<az>_02 tail = hem_outer ring vertex; FL/FR, L/R, BL/BR
  pairs symmetrised, F / B on x = 0.
  No preferred-bend offset is added (P1 v0 had none); the resulting chain bend angles are reported.
Roll: P1 convention kept - edit bone align_roll(roll hint), hint = -Y (front) for the spine, head, arms, legs,
  clavicles, hands and weapon sockets; +Z for Foot and the back sockets; the outward skirt azimuth for Skirt_*.
v1.1 (T253, spec d-02 §7 / §8 a, roll only): ROLL_FIX adds +90 deg to Skirt_L_01 / Skirt_L_02 and -90 deg to
  Skirt_R_01 / Skirt_R_02 after align_roll (mirror kept: R roll = -L roll). Their local rotation (vs parent) was 0.9 deg
  from the +-90 deg Euler XYZ pitch singularity and the Unity bind poses were off by 0.036 (T252 scratch roll
  experiment, candidate c2). Heads / tails / parents / other rolls unchanged; the JSON records the change in
  meta.version / meta.changes. PITCH_MARGIN log: per bone |90 - |local Euler XYZ Y|| (local = parent-relative rest).
"""
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Quaternion, Vector
from mathutils.bvhtree import BVHTree

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
IN_BLEND = RIG / "pl_r01_retopo.blend"
OUT_BLEND = RIG / "pl_r02_skeleton.blend"
OUT_JSON = RIG / "data" / "canonical_skeleton.json"
MANIFEST = RIG / "data" / "p1_manifest.json"
PARTS_JSON = RIG / "data" / "parts.json"
LOOPS_JSON = RIG / "data" / "retopo_loops.json"
P0B = REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json"
MESH = "PL_mesh"
ARM = "PL_rig"
COLL = "RIG"
SKIRT_AZ = [("F", 0), ("FL", 45), ("L", 90), ("BL", 135), ("B", 180), ("BR", 225), ("R", 270), ("FR", 315)]
BACK_Z = {"BackSocket": 0.64, "BackWeaponSocket": 0.58}
PAIRS = [("FL", "FR"), ("L", "R"), ("BL", "BR")]
VERSION = "1.1"
ROLL_FIX = {"Skirt_L_01": 90.0, "Skirt_L_02": 90.0, "Skirt_R_01": -90.0, "Skirt_R_02": -90.0}   # deg, added after align_roll
CHANGES = [
    {"version": "1.0", "task": "T230", "change": "skeleton v1 (P1 v0 tree, 41 bones, positions on the P2a retopo)"},
    {"version": "1.1", "task": "T253", "date": "2026-09-27", "change": "roll only: Skirt_L_01 / Skirt_L_02 +90 deg, "
     "Skirt_R_01 / Skirt_R_02 -90 deg vs v1.0 (spec d-02 §7 / §8 a, T252 roll experiment candidate c2); local rotation "
     "moved off the +-90 deg Euler XYZ pitch singularity (Unity bind poses). Heads, tails, parents, deform flags and all "
     "other rolls unchanged.", "bones": {k: v for k, v in ROLL_FIX.items()}},
]


def log(msg):
    print(f"[p22] {msg}")
    sys.stdout.flush()


def v3(v, nd=6):
    return [round(float(x), nd) + 0.0 for x in v]


def sym_pair(l, r):
    """(left, right) -> symmetric (left, right) and the mirror distance (mm)."""
    l, r = np.asarray(l, float), np.asarray(r, float)
    m = np.array(((abs(l[0]) + abs(r[0])) / 2.0, (l[1] + r[1]) / 2.0, (l[2] + r[2]) / 2.0))
    asym = float(np.linalg.norm(l - r * np.array((-1.0, 1.0, 1.0)))) * 1000.0
    return m, m * np.array((-1.0, 1.0, 1.0)), asym


def main():
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    parts = json.loads(PARTS_JSON.read_text(encoding="utf-8"))["part_id"]
    loops = json.loads(LOOPS_JSON.read_text(encoding="utf-8"))
    p0b = json.loads(P0B.read_text(encoding="utf-8"))
    lm = p0b["landmarks"]
    gm = bpy.data.objects[MESH]
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
    for p in me.polygons:
        vpart[list(p.vertices)] = pid_f[p.index]
    rings = loops["rings"]

    def centre(name):
        return co[np.array(rings[name]["verts"])].mean(axis=0)

    # ---------------------------------------------------------------- joint positions (symmetrised)
    raw = {}
    for x in ("l", "r"):
        raw[("shoulder", x)] = np.array(lm[f"shoulder_{x}"]["pos"])
        raw[("hip", x)] = np.array(lm[f"hip_{x}"]["pos"])
        raw[("fist", x)] = np.array(lm[f"fist_center_{x}"]["pos"])
        for j in ("elbow", "wrist", "knee", "ankle"):
            raw[(j, x)] = centre(f"{j}_{x}_1")
    J, asym = {}, {}
    for j in ("shoulder", "elbow", "wrist", "fist", "hip", "knee", "ankle"):
        J[(j, "L")], J[(j, "R")], asym[j] = sym_pair(raw[(j, "l")], raw[(j, "r")])
    for j in ("elbow", "wrist", "knee", "ankle"):
        lmk = lm[f"{j}_l"]["pos"]
        log(f"RING centre {j}_l_1 {v3(raw[(j, 'l')], 5)} / {j}_r_1 {v3(raw[(j, 'r')], 5)}; P0b landmark l "
            f"{v3(lmk, 5)} (|centre - landmark| {np.linalg.norm(raw[(j, 'l')] - np.array(lmk)) * 1000:.2f} mm)")
    log("ASYMMETRY before symmetrising (|L - mirror(R)|, mm): " + ", ".join(f"{k} {v:.2f}" for k, v in asym.items()))

    pel = np.array(lm["pelvis"]["pos"]); pel[0] = 0.0
    neck = np.array(lm["neck_base"]["pos"]); neck[0] = 0.0
    hbase = np.array(lm["head_base"]["pos"]); hbase[0] = 0.0
    hc = np.array(lm["head_center"]["pos"]); hc[0] = 0.0
    top = np.array((0.0, hc[1], float(co[vpart == parts["head"], 2].max())))
    sp1 = pel + (neck - pel) / 3.0
    sp2 = pel + (neck - pel) * 2.0 / 3.0

    # tunic-only BVH (back sockets)
    me.calc_loop_triangles()
    tv = np.array([t.vertices[:] for t in me.loop_triangles])
    tp = np.array([t.polygon_index for t in me.loop_triangles])
    tt = tv[pid_f[tp] == parts["tunic"]]
    bvh = BVHTree.FromPolygons(co.tolist(), tt.tolist(), all_triangles=True)

    def back_hit(z):
        o = Vector((0.0, 0.02 + 0.5, z))
        loc, _n, _i, _d = bvh.ray_cast(o, Vector((0.0, -1.0, 0.0)), 1.0)
        if loc is None:
            raise RuntimeError(f"back ray miss z {z}")
        return np.array(loc)

    # skirt from the ring vertices
    t = loops["tunic"]
    rowv = {r["name"]: r["verts"] for r in t["outer_rows"]}
    skirt_raw = {}
    for tag, az in SKIRT_AZ:
        k = t["columns"][tag]["column"]
        skirt_raw[tag] = [co[rowv["belt_bot"][k]], co[rowv["skirt_2"][k]], co[rowv["hem_outer"][k]]]
    skirt, skirt_asym = {}, {}
    for a_, b_ in PAIRS:
        res = [sym_pair(p, q) for p, q in zip(skirt_raw[a_], skirt_raw[b_])]
        skirt[a_] = [r[0] for r in res]
        skirt[b_] = [r[1] for r in res]
        skirt_asym[f"{a_}/{b_}"] = max(r[2] for r in res)
    for tag in ("F", "B"):
        skirt[tag] = [np.array((0.0, p[1], p[2])) for p in skirt_raw[tag]]
        skirt_asym[tag + " (x offset)"] = max(abs(p[0]) for p in skirt_raw[tag]) * 1000.0
    log("ASYMMETRY skirt ring points (mm): " + ", ".join(f"{k} {v:.2f}" for k, v in skirt_asym.items()))

    L = {"Root": ((0, 0, 0), (0, 0, 0.1)), "Pelvis": (pel, sp1), "Spine_01": (sp1, sp2), "Spine_02": (sp2, neck),
         "Neck": (neck, hbase), "Head": (hbase, hc), "HeadEquipmentSocket": (top, top + np.array((0, 0, 0.05)))}
    for s in ("L", "R"):
        sh, el, wr, fc = J[("shoulder", s)], J[("elbow", s)], J[("wrist", s)], J[("fist", s)]
        clav = np.array((0.03 if s == "L" else -0.03, sp2[1] + 0.003, neck[2] - 0.017))
        hd = (fc - wr) / np.linalg.norm(fc - wr)
        L[f"Clavicle_{s}"] = (clav, sh)
        L[f"Arm_{s}"] = (sh, el)
        L[f"Forearm_{s}"] = (el, wr)
        L[f"Hand_{s}"] = (wr, fc)
        L[f"WeaponSocket_{s}"] = (fc, fc + hd * 0.05)
        hp, kn, an = J[("hip", s)], J[("knee", s)], J[("ankle", s)]
        L[f"Thigh_{s}"] = (hp, kn)
        L[f"Calf_{s}"] = (kn, an)
        L[f"Foot_{s}"] = (an, np.array((an[0], an[1] - 0.12, 0.03)))
    for name, z in BACK_Z.items():
        h = back_hit(z)
        L[name] = (h, h + np.array((0.0, 0.05, 0.0)))
    hint = {}
    for tag, az in SKIRT_AZ:
        a, m, b = skirt[tag]
        L[f"Skirt_{tag}_01"] = (a, m)
        L[f"Skirt_{tag}_02"] = (m, b)
        d = Vector((math.sin(math.radians(az)), -math.cos(math.radians(az)), 0.0))
        hint[f"Skirt_{tag}_01"] = hint[f"Skirt_{tag}_02"] = d

    bones = manifest["bones"]
    names = [b["name"] for b in bones]
    if set(names) != set(L) or len(names) != 41:
        raise RuntimeError(f"bone set mismatch {sorted(set(names) ^ set(L))}")

    # ---------------------------------------------------------------- armature
    coll = bpy.data.collections.new(COLL)
    bpy.context.scene.collection.children.link(coll)
    ad = bpy.data.armatures.new(ARM)
    arm = bpy.data.objects.new(ARM, ad)
    coll.objects.link(arm)
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = ad.edit_bones
    for b in bones:
        eb = ebs.new(b["name"])
        eb.head, eb.tail = Vector(L[b["name"]][0]), Vector(L[b["name"]][1])
        eb.align_roll(hint.get(b["name"], Vector(b["roll_hint_z"])))
        if b["name"] in ROLL_FIX:
            eb.roll = eb.roll + math.radians(ROLL_FIX[b["name"]])
        eb.use_deform = b["use_deform"]
    for b in bones:
        eb = ebs[b["name"]]
        eb.parent = ebs[b["parent"]] if b["parent"] else None
        eb.use_connect = False
    rolls = {eb.name: float(eb.roll) for eb in ebs}
    bpy.ops.object.mode_set(mode="OBJECT")
    for pb in arm.pose.bones:
        pb.rotation_mode = "QUATERNION"
    vl.update()
    B = ad.bones
    log(f"ARMATURE {ARM}: {len(B)} bones, connected {sum(b.use_connect for b in B)}, deform "
        f"{sum(b.use_deform for b in B)}, matrix identity {np.allclose(np.array(arm.matrix_world), np.eye(4))}")
    p1 = {b["name"]: b for b in bones}
    for n in names:
        b = B[n]
        dh = (b.head_local - Vector(p1[n]["head"])).length * 1000.0
        dt = (b.tail_local - Vector(p1[n]["tail"])).length * 1000.0
        log(f"BONE {n:20s} parent {str(b.parent.name if b.parent else None):12s} head {v3(b.head_local, 5)} tail "
            f"{v3(b.tail_local, 5)} roll {math.degrees(rolls[n]):+9.4f} deg deform {b.use_deform} "
            f"(vs P1 v0: head {dh:.2f} mm, tail {dt:.2f} mm)")

    # ---------------------------------------------------------------- self checks
    mir = []
    for n in names:
        if n.endswith("_L") or "_L_" in n or (n.startswith("Skirt_") and n.split("_")[1] in ("FL", "L", "BL")):
            if n.startswith("Skirt_"):
                tag = n.split("_")[1]
                m = f"Skirt_{dict(PAIRS)[tag]}_{n.split('_')[2]}"
            else:
                m = n[:-2] + "_R"
            for a_, b_ in ((B[n].head_local, B[m].head_local), (B[n].tail_local, B[m].tail_local)):
                mir.append((Vector((-a_.x, a_.y, a_.z)) - b_).length * 1000.0)
    log(f"SYMMETRY max |mirror(L) - R| over paired bone heads / tails {max(mir):.6f} mm")
    rsym = max(abs(rolls[n] + rolls[f"Skirt_{dict(PAIRS)[n.split('_')[1]]}_{n.split('_')[2]}"])
               for n in names if n.startswith("Skirt_") and n.split("_")[1] in ("FL", "L", "BL"))
    log(f"ROLL_FIX {ROLL_FIX} (deg): rolls now " + ", ".join(f"{n} {math.degrees(rolls[n]):+.4f}" for n in ROLL_FIX)
        + f"; skirt roll mirror max |roll(L) + roll(R)| {math.degrees(rsym):.6f} deg")
    margins = []
    for n in names:
        b = B[n]
        rel = (b.parent.matrix_local.inverted() @ b.matrix_local) if b.parent else b.matrix_local
        e = rel.to_euler("XYZ")
        margins.append((90.0 - abs(math.degrees(e.y)), n, [round(math.degrees(a), 3) for a in e]))
    margins.sort()
    log("PITCH_MARGIN (deg, 90 - |local Euler XYZ Y|, parent-relative rest; smallest 8): "
        + "; ".join(f"{n} {m:.3f} {eu}" for m, n, eu in margins[:8]))

    def xaxis(n):
        return B[n].matrix_local.to_3x3().col[0].normalized()

    for key, chain in (("arm_L", ("Arm_L", "Forearm_L", "Hand_L")), ("arm_R", ("Arm_R", "Forearm_R", "Hand_R")),
                       ("leg_L", ("Thigh_L", "Calf_L")), ("leg_R", ("Thigh_R", "Calf_R"))):
        x0 = xaxis(chain[0])
        devs = [math.degrees(x0.angle(xaxis(b))) for b in chain]
        u = (B[chain[0]].tail_local - B[chain[0]].head_local).normalized()
        w = (B[chain[1]].tail_local - B[chain[1]].head_local).normalized()
        bend = math.degrees(u.angle(w))
        nrm = u.cross(w)
        nx = math.degrees(min(nrm.angle(x0), nrm.angle(-x0))) if nrm.length > 1e-9 else float("nan")
        log(f"CHAIN {key}: local X {v3(x0, 4)}, max X angle within chain {max(devs):.4f} deg; bend angle "
            f"{bend:.4f} deg, bend-plane normal vs X {nx:.3f} deg")
    for bn, tgt in (("Forearm_L", "Hand_L"), ("Forearm_R", "Hand_R"), ("Calf_L", "Foot_L"), ("Calf_R", "Foot_R")):
        h0 = arm.pose.bones[tgt].head.copy()
        arm.pose.bones[bn].rotation_quaternion = Quaternion((1, 0, 0), math.radians(30.0))
        vl.update()
        dh = arm.pose.bones[tgt].head - h0
        arm.pose.bones[bn].rotation_quaternion = Quaternion()
        vl.update()
        log(f"ROLLTEST {bn} local +X 30 deg -> {tgt} head d {v3(dh * 1000.0, 2)} mm (Y sign {'+' if dh.y > 0 else '-'})")

    # ---------------------------------------------------------------- canonical json
    out = {"meta": {"source_blend": IN_BLEND.name, "script": Path(__file__).name, "armature": ARM,
                    "units": "m, roll radians, rest_matrix = bone matrix_local row-major 4x4 (armature space)",
                    "inputs": ["p1_manifest.json", "retopo_loops.json", "parts.json", "p0b_measure.json"],
                    "tree": "P1 skeleton v0 (41 bones), all bones unconnected",
                    "roll_convention": "edit bone align_roll(hint): -Y for spine/head/arms/legs/clavicles/hands/"
                                       "weapon sockets, +Z for Foot and back sockets, outward azimuth for Skirt_*; "
                                       "v1.1: then + roll_fix_deg (Skirt_L_01/02 +90, Skirt_R_01/02 -90)",
                    "symmetrised": True, "asymmetry_mm": asym, "skirt_asymmetry_mm": skirt_asym,
                    "version": VERSION, "roll_fix_deg": ROLL_FIX, "changes": CHANGES},
           "bones": []}
    for b in bones:
        bb = B[b["name"]]
        out["bones"].append({"name": bb.name, "parent": bb.parent.name if bb.parent else None,
                             "use_connect": bool(bb.use_connect), "head": [float(x) for x in bb.head_local],
                             "tail": [float(x) for x in bb.tail_local], "roll": rolls[bb.name],
                             "use_deform": bool(bb.use_deform),
                             "rest_matrix": [[float(x) for x in row] for row in bb.matrix_local]})
    OUT_JSON.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    log(f"OUTPUT {OUT_JSON} bones {len(out['bones'])}")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    log(f"OUTPUT {OUT_BLEND} objects {sorted((o.name, o.type) for o in bpy.data.objects)}; {MESH} vgroups "
        f"{len(gm.vertex_groups)} modifiers {len(gm.modifiers)}")
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
