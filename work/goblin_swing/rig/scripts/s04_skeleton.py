"""s04_skeleton.py - T18: canonical DEF skeleton (G4.1-G4.9).

Input : rig/gob_r01_retopo.blend (GOB_mesh, GOB_club), rig/data/pivots.json, rig/data/parts.json,
        rig/data/retopo_loops.json
Output: rig/gob_r02_skeleton.blend (save copy; input not overwritten) with armature GOB_rig in collection RIG,
        rig/data/canonical_skeleton.json

Run:  bl.ps1 -Script s04_skeleton.py -Blend gob_r01_retopo.blend

Positions (left / right measured separately, no mirroring). Ring centre = mean GOB_mesh co of the ring verts.
  root (0,0,0)->(0,0,0.2) roll 0; pelvis/spine_01/spine_02/head from pivots (head tail z = max head-part z).
  shoulder_x: shoulder_root_x -> shoulder_x (pivots).
  upperarm_x: shoulder_x -> E,  E = centre(elbow_x_1) + perp(+Y, tube axis) * 4 mm.
  lowerarm_x: E -> W,  W = centre(wrist_x_1).  hand_x: W -> W + dir(lowerarm) * max hand-part projection.
  thigh_x: hip_x -> K,  K = centre(knee_x_1) + perp(-Y, tube axis) * 1.5 mm.  calf_x: K -> A = centre(ankle_x_1).
  foot_x: A -> (foot_tip.x, foot_tip.y, A.z).
  twist bones per G4.5; weapon_socket_r: grip_r -> grip_r + axis * 0.10, parent hand_r.
Roll (G4.4): one local X per arm / leg chain = bend-plane normal, signed so that +X rotation of lowerarm moves
the hand to -Y and of calf moves the foot to +Y.  weapon_socket_r uses the arm_r X.
No skinning / vertex groups / constraints / CTRL / MCH.  GOB_mesh and GOB_club are not touched.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Quaternion, Vector  # noqa: E402

OUT_BLEND = goblib.RIG / "gob_r02_skeleton.blend"
OUT_JSON = "canonical_skeleton.json"
ARM = "GOB_rig"
COLL = "RIG"
BEND_ELBOW = 0.004  # preferred bend offset (m), +Y
BEND_KNEE = 0.0015  # preferred bend offset (m), -Y
SOCKET_LEN = 0.10
ROOT_TAIL = (0.0, 0.0, 0.2)

# (name, parent, use_connect) in d-23 section 2 order (same as s03_temprig TREE)
TREE = [
    ("root", None, False),
    ("pelvis", "root", False),
    ("spine_01", "pelvis", True),
    ("spine_02", "spine_01", True),
    ("head", "spine_02", True),
    ("shoulder_l", "spine_02", False),
    ("upperarm_l", "shoulder_l", True),
    ("upperarm_twist_l", "upperarm_l", False),
    ("lowerarm_l", "upperarm_l", True),
    ("lowerarm_twist_l", "lowerarm_l", False),
    ("hand_l", "lowerarm_l", True),
    ("shoulder_r", "spine_02", False),
    ("upperarm_r", "shoulder_r", True),
    ("upperarm_twist_r", "upperarm_r", False),
    ("lowerarm_r", "upperarm_r", True),
    ("lowerarm_twist_r", "lowerarm_r", False),
    ("hand_r", "lowerarm_r", True),
    ("weapon_socket_r", "hand_r", False),
    ("thigh_l", "pelvis", False),
    ("calf_l", "thigh_l", True),
    ("foot_l", "calf_l", True),
    ("thigh_r", "pelvis", False),
    ("calf_r", "thigh_r", True),
    ("foot_r", "calf_r", True),
]
NON_DEFORM = {"root", "weapon_socket_r"}


def V(p):
    return Vector([float(x) for x in p])


def perp(v, axis):
    a = axis.normalized()
    w = v - a * v.dot(a)
    return w.normalized()


def roll_from_x(eb, xvec):
    """Set eb roll so that its local X axis = xvec projected perpendicular to the bone."""
    y = (eb.tail - eb.head).normalized()
    x = (xvec - y * xvec.dot(y)).normalized()
    eb.align_roll(x.cross(y))  # Z = X x Y


def main():
    piv = goblib.load_json("pivots.json")["pivots"]
    parts = goblib.load_json("parts.json")
    loops = goblib.load_json("retopo_loops.json")
    P = {k: V(v["co"]) for k, v in piv.items()}
    AX = {k: V(v["axis"]).normalized() for k, v in piv.items() if "axis" in v}

    gm = bpy.data.objects["GOB_mesh"]
    if loops.get("mesh", "GOB_mesh") != "GOB_mesh":
        raise RuntimeError(f"retopo_loops.json mesh={loops.get('mesh')} != GOB_mesh")
    if bpy.data.objects.get(ARM) is not None:
        raise RuntimeError(f"{ARM} already exists in the input")
    if bpy.data.collections.get(COLL) is not None:
        raise RuntimeError(f"collection {COLL} already exists in the input")
    if not np.allclose(np.array(gm.matrix_world), np.eye(4)):
        raise RuntimeError("GOB_mesh matrix_world is not identity")
    if gm.parent is not None or len(gm.vertex_groups) or len(gm.modifiers):
        raise RuntimeError("GOB_mesh already has parent / vertex groups / modifiers")
    me = gm.data
    nv = len(me.vertices)
    co = np.empty(nv * 3, dtype=np.float64)
    me.vertices.foreach_get("co", co)
    co = co.reshape(nv, 3)
    pid_f = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid_f)
    vpart = np.full(nv, -1, dtype=np.int32)
    for poly in me.polygons:
        p = pid_f[poly.index]
        for vi in poly.vertices:
            if vpart[vi] not in (-1, p):
                raise RuntimeError(f"vertex {vi} shared by parts {vpart[vi]} and {p}")
            vpart[vi] = p
    print(f"[s04] GOB_mesh verts={nv} per part: " + ", ".join(
        f"{n}={int((vpart == v).sum())}" for n, v in parts.items()))

    rings = loops["rings"]

    def ring_centre(name):
        idx = np.array(rings[name]["verts"], dtype=np.int64)
        return Vector(co[idx].mean(axis=0).tolist())

    C = {}
    for s in ("l", "r"):
        for j in ("elbow", "wrist", "knee", "ankle"):
            C[f"{j}_{s}"] = ring_centre(f"{j}_{s}_1")
    for k, c in C.items():
        print(f"[s04] RING centre {k}_1 = ({c.x:+.5f},{c.y:+.5f},{c.z:+.5f}) "
              f"n={len(rings[k[:-2] + '_' + k[-1] + '_1']['verts'])} |centre - pivot|={(c - P[k]).length * 1000:.3f} mm")

    # ---------------------------------------------------------------- bone layout
    L = {}  # name -> (head, tail)
    L["root"] = (Vector((0, 0, 0)), V(ROOT_TAIL))
    L["pelvis"] = (P["pelvis"], P["spine_01"])
    L["spine_01"] = (P["spine_01"], P["spine_02"])
    L["spine_02"] = (P["spine_02"], P["head"])
    head_top = float(co[vpart == parts["head"], 2].max())
    L["head"] = (P["head"], Vector((P["head"].x, P["head"].y, head_top)))
    chain_x = {}
    bend_info = {}
    for s in ("l", "r"):
        ax = AX[f"elbow_{s}"]
        elb = C[f"elbow_{s}"] + perp(Vector((0, 1, 0)), ax) * BEND_ELBOW
        sh, wr = P[f"shoulder_{s}"], C[f"wrist_{s}"]
        L[f"shoulder_{s}"] = (P[f"shoulder_root_{s}"], sh)
        L[f"upperarm_{s}"] = (sh, elb)
        L[f"lowerarm_{s}"] = (elb, wr)
        ld = (wr - elb).normalized()
        hv = co[vpart == parts[f"hand_{s}"]]
        hlen = float(((hv - np.array(wr)) @ np.array(ld)).max())
        L[f"hand_{s}"] = (wr, wr + ld * hlen)
        L[f"upperarm_twist_{s}"] = (sh, sh + (elb - sh) * 0.5)
        L[f"lowerarm_twist_{s}"] = (elb + (wr - elb) * 0.5, wr)
        n = (elb - sh).normalized().cross(ld).normalized()
        if n.cross(ld).dot(Vector((0, -1, 0))) < 0:  # +X rot moves lowerarm tail along X x Y = Z -> want -Y
            n = -n
        chain_x[f"arm_{s}"] = n
        bend_info[f"elbow_{s}"] = (sh, elb, wr, ax, C[f"elbow_{s}"])

        axl = AX[f"knee_{s}"]
        kne = C[f"knee_{s}"] + perp(Vector((0, -1, 0)), axl) * BEND_KNEE
        hp, an = P[f"hip_{s}"], C[f"ankle_{s}"]
        L[f"thigh_{s}"] = (hp, kne)
        L[f"calf_{s}"] = (kne, an)
        tip = P[f"foot_tip_{s}"]
        L[f"foot_{s}"] = (an, Vector((tip.x, tip.y, an.z)))
        cd = (an - kne).normalized()
        n = (kne - hp).normalized().cross(cd).normalized()
        if n.cross(cd).dot(Vector((0, 1, 0))) < 0:  # want +X rot of calf -> foot +Y
            n = -n
        chain_x[f"leg_{s}"] = n
        bend_info[f"knee_{s}"] = (hp, kne, an, axl, C[f"knee_{s}"])
    grip, gax = P["grip_r"], AX["grip_r"]
    L["weapon_socket_r"] = (grip, grip + gax * SOCKET_LEN)

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
    for name, _p, _c in TREE:
        eb = ebs.new(name)
        eb.head, eb.tail = L[name]
        eb.roll = 0.0
    for name, parent, conn in TREE:
        eb = ebs[name]
        if parent:
            eb.parent = ebs[parent]
            if conn:
                if (eb.head - ebs[parent].tail).length > 1e-6:
                    raise RuntimeError(f"{name}: head != parent tail for connected bone")
                eb.use_connect = True
        eb.use_deform = name not in NON_DEFORM
    for s in ("l", "r"):
        for b in (f"shoulder_{s}", f"upperarm_{s}", f"upperarm_twist_{s}", f"lowerarm_{s}",
                  f"lowerarm_twist_{s}", f"hand_{s}"):
            roll_from_x(ebs[b], chain_x[f"arm_{s}"])
        for b in (f"thigh_{s}", f"calf_{s}", f"foot_{s}"):
            roll_from_x(ebs[b], chain_x[f"leg_{s}"])
    roll_from_x(ebs["weapon_socket_r"], chain_x["arm_r"])
    ebs["root"].roll = 0.0
    rolls = {eb.name: float(eb.roll) for eb in ebs}
    bpy.ops.object.mode_set(mode="OBJECT")
    vl.update()

    bones = ad.bones
    print(f"[s04] ARMATURE {ARM}: {len(bones)} bones, deform={sum(b.use_deform for b in bones)}, "
          f"matrix_world identity={np.allclose(np.array(arm.matrix_world), np.eye(4))}, collection={COLL}")
    for name, _p, _c in TREE:
        b = bones[name]
        h, t = b.head_local, b.tail_local
        print(f"[s04] BONE {name:18s} parent={(b.parent.name if b.parent else '-'):12s} "
              f"head=({h.x:+.5f},{h.y:+.5f},{h.z:+.5f}) tail=({t.x:+.5f},{t.y:+.5f},{t.z:+.5f}) "
              f"roll={math.degrees(rolls[name]):+9.4f}deg deform={b.use_deform} connect={b.use_connect}")

    # ---------------------------------------------------------------- self checks
    def xaxis(n):
        return bones[n].matrix_local.to_3x3().col[0].normalized()

    for key, chain in (("arm_l", ("upperarm_l", "upperarm_twist_l", "lowerarm_l", "lowerarm_twist_l", "hand_l")),
                       ("arm_r", ("upperarm_r", "upperarm_twist_r", "lowerarm_r", "lowerarm_twist_r", "hand_r")),
                       ("leg_l", ("thigh_l", "calf_l", "foot_l")), ("leg_r", ("thigh_r", "calf_r", "foot_r"))):
        x0 = xaxis(chain[0])
        devs = [math.degrees(x0.angle(xaxis(b))) for b in chain]
        print(f"[s04] ROLL chain {key}: X({chain[0]})=({x0.x:+.4f},{x0.y:+.4f},{x0.z:+.4f}) "
              f"max angle to chain X = {max(devs):.4f} deg")

    for pb in arm.pose.bones:
        pb.rotation_mode = "QUATERNION"
        pb.rotation_quaternion = Quaternion()
    vl.update()
    for k, (a, m, b, ax, cen) in bend_info.items():
        s = k[-1]
        chain = ("arm_" if k.startswith("elbow") else "leg_") + s
        u, l = (m - a).normalized(), (b - m).normalized()
        ang = math.degrees(u.angle(l))
        d = m - cen
        off = d - ax * d.dot(ax)
        n = u.cross(l).normalized()
        nx = math.degrees(min(n.angle(chain_x[chain]), n.angle(-chain_x[chain])))
        bn, tgt = (f"lowerarm_{s}", f"hand_{s}") if chain.startswith("arm") else (f"calf_{s}", f"foot_{s}")
        t0 = arm.pose.bones[tgt].tail.copy()
        h0 = arm.pose.bones[tgt].head.copy()
        pb = arm.pose.bones[bn]
        pb.rotation_quaternion = Quaternion((1, 0, 0), math.radians(30.0))
        vl.update()
        dt = arm.pose.bones[tgt].tail - t0
        dh = arm.pose.bones[tgt].head - h0
        pb.rotation_quaternion = Quaternion()
        vl.update()
        print(f"[s04] SELFCHECK {chain}: bend angle={ang:.4f} deg; offset from {k}_1 ring centre perp to tube axis="
              f"{off.length * 1000:.4f} mm dir=({off.x / off.length:+.4f},{off.y / off.length:+.4f},"
              f"{off.z / off.length:+.4f}); bend-plane normal vs chain X={nx:.4f} deg; "
              f"{bn} +X30deg -> {tgt} head dY={dh.y * 1000:+.2f} mm tail dY={dt.y * 1000:+.2f} mm "
              f"sign head={'+' if dh.y > 0 else '-'} tail={'+' if dt.y > 0 else '-'}")
    for s in ("l", "r"):
        for tw, par in ((f"upperarm_twist_{s}", f"upperarm_{s}"), (f"lowerarm_twist_{s}", f"lowerarm_{s}")):
            qa = bones[tw].matrix_local.to_3x3().to_quaternion()
            qb = bones[par].matrix_local.to_3x3().to_quaternion()
            print(f"[s04] TWIST {tw}: len ratio={bones[tw].length / bones[par].length:.5f} "
                  f"orient diff={math.degrees(qa.rotation_difference(qb).angle):.5f} deg")
    ws = bones["weapon_socket_r"]
    wy = ws.matrix_local.to_3x3().col[1].normalized()
    print(f"[s04] SOCKET weapon_socket_r head-grip={(ws.head_local - grip).length * 1000:.4f} mm "
          f"Y vs grip axis={math.degrees(wy.angle(gax)):.4f} deg parent={ws.parent.name}")

    # ---------------------------------------------------------------- canonical json
    src = os.path.basename(bpy.data.filepath)
    out = {
        "meta": {
            "source_blend": src,
            "script": os.path.basename(__file__),
            "armature": ARM,
            "units": "m, roll radians, matrix_local row-major 4x4 (armature space)",
            "inputs": ["pivots.json", "parts.json", "retopo_loops.json"],
            "bend_elbow_m": BEND_ELBOW,
            "bend_knee_m": BEND_KNEE,
        },
        "bones": [],
    }
    for name, parent, conn in TREE:
        b = bones[name]
        out["bones"].append({
            "name": name,
            "parent": parent,
            "use_connect": bool(b.use_connect),
            "head": [float(x) for x in b.head_local],
            "tail": [float(x) for x in b.tail_local],
            "roll": rolls[name],
            "use_deform": bool(b.use_deform),
            "rest_matrix": [[float(x) for x in row] for row in b.matrix_local],
        })
    jp = goblib.save_json(OUT_JSON, out)
    print(f"[s04] OUTPUT {jp} bones={len(out['bones'])}")

    # ---------------------------------------------------------------- save
    print(f"[s04] GOB_mesh parent={gm.parent} vgroups={len(gm.vertex_groups)} modifiers={len(gm.modifiers)}")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s04] OUTPUT {OUT_BLEND} objects={sorted(o.name for o in bpy.data.objects)} "
          f"collections={sorted(c.name for c in bpy.data.collections)}")


goblib.run_main(main)
