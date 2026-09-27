"""s03_temprig.py - T13: temporary DEF rig for the G3 deform test (G3.1).

Input : rig/gob_r01_retopo.blend (GOB_mesh, GOB_club), rig/data/pivots.json, rig/data/parts.json
Output: rig/gob_r01t_temprig.blend (save copy; the input is not overwritten)

Run:  bl.ps1 -Script s03_temprig.py -Blend gob_r01_retopo.blend

Armature GOB_temprig: the 24 bones of d-23 section 2 (names / parents as specified), positions from
pivots.json via the pivot -> DEF table.  Preferred bend: elbow moved 4 mm toward +Y, knee 4 mm toward -Y,
perpendicular to the tube axis.  Roll (G4.4): one local X axis per arm / leg chain = bend-plane normal,
sign chosen so that +X rotation of lowerarm moves the hand to -Y and of calf moves the foot to +Y.
Twist bones per G4.5, weapon_socket_r per G4.6 (head = grip_r, local Y = grip axis).
root / weapon_socket_r: use_deform False.
This rig is test-only and is discarded at G4 (final DEF from retopo ring centres).

Skinning: body (part_id 0) = Blender bone-heat automatic weights (ARMATURE_AUTO, deform bones only) computed
on a temporary body-only copy; rigid parts = single bone 100 %.  <= 4 influences per vertex, normalized.
Vertices without heat weight are filled with the nearest deform bone (count reported).
GOB_club: bone parent weapon_socket_r, world transform kept.  Armature modifier on GOB_mesh.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bmesh  # noqa: E402
import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Quaternion, Vector  # noqa: E402

OUT_BLEND = goblib.RIG / "gob_r01t_temprig.blend"
ARM = "GOB_temprig"
COLL = "GOB"
BEND = 0.004  # preferred bend offset (m)
MAX_INF = 4
EPS_W = 1e-4
RIGID_BONE = {"head": "head", "hand_l": "hand_l", "hand_r": "hand_r", "shoe_l": "foot_l",
              "shoe_r": "foot_r", "belt": "spine_01"}
SOCKET_LEN = 0.10
ROOT_TAIL = (0.0, 0.0, 0.2)

# (name, parent, use_connect) in d-23 section 2 order
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


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab[None, :]), axis=1)


def main():
    piv = {k: v for k, v in goblib.load_json("pivots.json")["pivots"].items()}
    parts = goblib.load_json("parts.json")
    P = {k: V(v["co"]) for k, v in piv.items()}
    AX = {k: V(v["axis"]).normalized() for k, v in piv.items() if "axis" in v}

    gm = bpy.data.objects["GOB_mesh"]
    club = bpy.data.objects["GOB_club"]
    if bpy.data.objects.get(ARM) is not None:
        raise RuntimeError(f"{ARM} already exists in the input")
    me = gm.data
    nv = len(me.vertices)
    co = np.empty(nv * 3, dtype=np.float64)
    me.vertices.foreach_get("co", co)
    co = co.reshape(nv, 3)
    mw = np.array(gm.matrix_world)
    if not np.allclose(mw, np.eye(4)):
        raise RuntimeError("GOB_mesh matrix_world is not identity")
    pid_f = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid_f)
    vpart = np.full(nv, -1, dtype=np.int32)
    for poly in me.polygons:
        p = pid_f[poly.index]
        for vi in poly.vertices:
            if vpart[vi] not in (-1, p):
                raise RuntimeError(f"vertex {vi} shared by parts {vpart[vi]} and {p}")
            vpart[vi] = p
    if (vpart < 0).any():
        raise RuntimeError(f"{int((vpart < 0).sum())} loose vertices without part")
    print(f"[s03] GOB_mesh verts={nv} per part: " + ", ".join(
        f"{n}={int((vpart == v).sum())}" for n, v in parts.items())
          + f"; pre-existing vgroups={len(gm.vertex_groups)} modifiers={[m.type for m in gm.modifiers]}")
    if len(gm.vertex_groups) or len(gm.modifiers):
        raise RuntimeError("GOB_mesh already has vertex groups / modifiers")

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
        elb = P[f"elbow_{s}"] + perp(Vector((0, 1, 0)), ax) * BEND
        sh, wr = P[f"shoulder_{s}"], P[f"wrist_{s}"]
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
        bend_info[f"elbow_{s}"] = (sh, elb, wr, ax, P[f"elbow_{s}"])

        axl = AX[f"knee_{s}"]
        kne = P[f"knee_{s}"] + perp(Vector((0, -1, 0)), axl) * BEND
        hp, an = P[f"hip_{s}"], P[f"ankle_{s}"]
        L[f"thigh_{s}"] = (hp, kne)
        L[f"calf_{s}"] = (kne, an)
        tip = P[f"foot_tip_{s}"]
        L[f"foot_{s}"] = (an, Vector((tip.x, tip.y, tip.z)))
        cd = (an - kne).normalized()
        n = (kne - hp).normalized().cross(cd).normalized()
        if n.cross(cd).dot(Vector((0, 1, 0))) < 0:  # want +X rot of calf -> foot +Y
            n = -n
        chain_x[f"leg_{s}"] = n
        bend_info[f"knee_{s}"] = (hp, kne, an, axl, P[f"knee_{s}"])
    grip, gax = P["grip_r"], AX["grip_r"]
    L["weapon_socket_r"] = (grip, grip + gax * SOCKET_LEN)

    # ---------------------------------------------------------------- armature
    ad = bpy.data.armatures.new(ARM)
    arm = bpy.data.objects.new(ARM, ad)
    coll = bpy.data.collections.get(COLL) or bpy.context.scene.collection
    coll.objects.link(arm)
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = ad.edit_bones
    for name, parent, _c in TREE:
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
    bpy.ops.object.mode_set(mode="OBJECT")
    vl.update()

    bones = ad.bones
    print(f"[s03] ARMATURE {ARM}: {len(bones)} bones, deform={sum(b.use_deform for b in bones)}")
    rolls = {}
    bpy.ops.object.mode_set(mode="EDIT")
    for eb in ad.edit_bones:
        rolls[eb.name] = math.degrees(eb.roll)
    bpy.ops.object.mode_set(mode="OBJECT")
    for name, _p, _c in TREE:
        b = bones[name]
        h, t = b.head_local, b.tail_local
        print(f"[s03] BONE {name:18s} parent={(b.parent.name if b.parent else '-'):12s} "
              f"head=({h.x:+.4f},{h.y:+.4f},{h.z:+.4f}) tail=({t.x:+.4f},{t.y:+.4f},{t.z:+.4f}) "
              f"roll={rolls[name]:+8.3f} deform={b.use_deform} connect={b.use_connect}")

    def xaxis(n):
        return bones[n].matrix_local.to_3x3().col[0].normalized()

    for key, chain in (("arm_l", ("upperarm_l", "upperarm_twist_l", "lowerarm_l", "lowerarm_twist_l", "hand_l")),
                       ("arm_r", ("upperarm_r", "upperarm_twist_r", "lowerarm_r", "lowerarm_twist_r", "hand_r")),
                       ("leg_l", ("thigh_l", "calf_l")), ("leg_r", ("thigh_r", "calf_r"))):
        x0 = xaxis(chain[0])
        devs = [math.degrees(x0.angle(xaxis(b))) for b in chain]
        print(f"[s03] ROLL chain {key}: X({chain[0]})=({x0.x:+.4f},{x0.y:+.4f},{x0.z:+.4f}) "
              f"max angle to chain X = {max(devs):.4f} deg ({', '.join(f'{b}={d:.4f}' for b, d in zip(chain, devs))})")
    for s in ("l", "r"):
        for tw, par in ((f"upperarm_twist_{s}", f"upperarm_{s}"), (f"lowerarm_twist_{s}", f"lowerarm_{s}")):
            a = bones[tw].matrix_local.to_3x3()
            b = bones[par].matrix_local.to_3x3()
            ang = math.degrees(a.to_quaternion().rotation_difference(b.to_quaternion()).angle)
            print(f"[s03] TWIST {tw}: len={bones[tw].length:.4f} parent len={bones[par].length:.4f} "
                  f"ratio={bones[tw].length / bones[par].length:.4f} orient diff to {par}={ang:.5f} deg")
    ws = bones["weapon_socket_r"]
    wy = ws.matrix_local.to_3x3().col[1].normalized()
    print(f"[s03] SOCKET weapon_socket_r head-grip={(ws.head_local - grip).length * 1000:.4f} mm "
          f"Y-axis angle to grip axis={math.degrees(wy.angle(gax)):.4f} deg parent={ws.parent.name}")

    for k, (a, m, b, ax, orig) in bend_info.items():
        u, l = (m - a).normalized(), (b - m).normalized()
        ang = math.degrees(u.angle(l))
        d = m - orig
        off = d - ax * d.dot(ax)
        # distance of the bent joint from the tube axis line through orig
        n = u.cross(l).normalized()
        chain = "arm_" + k[-1] if k.startswith("elbow") else "leg_" + k[-1]
        nx = math.degrees(min(n.angle(chain_x[chain]), n.angle(-chain_x[chain])))
        print(f"[s03] BEND {k}: segment angle={ang:.4f} deg, joint offset from tube axis={off.length * 1000:.4f} mm "
              f"dir=({off.x / off.length:+.4f},{off.y / off.length:+.4f},{off.z / off.length:+.4f}) "
              f"sign Y={'+' if off.y > 0 else '-'}; bend-plane normal vs chain X={nx:.4f} deg")

    # ---------------------------------------------------------------- body heat weights
    body_idx = np.nonzero(vpart == parts["body"])[0]
    tmp_me = me.copy()
    tmp_me.name = "_s03_body_tmp"
    tmp = bpy.data.objects.new("_s03_body_tmp", tmp_me)
    coll.objects.link(tmp)
    oi = tmp_me.attributes.new("_orig_idx", "INT", "POINT")
    oi.data.foreach_set("value", np.arange(nv, dtype=np.int32))
    bm = bmesh.new()
    bm.from_mesh(tmp_me)
    bm.verts.ensure_lookup_table()
    keep = np.zeros(nv, dtype=bool)
    keep[body_idx] = True
    bmesh.ops.delete(bm, geom=[v for v in bm.verts if not keep[v.index]], context="VERTS")
    bm.to_mesh(tmp_me)
    bm.free()
    tmp_me.update()
    orig = np.empty(len(tmp_me.vertices), dtype=np.int32)
    tmp_me.attributes["_orig_idx"].data.foreach_get("value", orig)
    print(f"[s03] heat input: body copy verts={len(tmp_me.vertices)} faces={len(tmp_me.polygons)} "
          f"(body verts in GOB_mesh={len(body_idx)})")

    for o in vl.objects:
        o.select_set(False)
    tmp.select_set(True)
    arm.select_set(True)
    vl.objects.active = arm
    bpy.ops.object.parent_set(type="ARMATURE_AUTO")

    def_names = [n for n, _p, _c in TREE if n not in NON_DEFORM]
    bi = {n: i for i, n in enumerate(def_names)}
    W = np.zeros((nv, len(def_names)), dtype=np.float64)
    gname = {g.index: g.name for g in tmp.vertex_groups}
    stray = set()
    for v in tmp_me.vertices:
        for g in v.groups:
            n = gname[g.group]
            if n not in bi:
                stray.add(n)
                continue
            W[orig[v.index], bi[n]] = g.weight
    print(f"[s03] heat vertex groups on body copy: {len(tmp.vertex_groups)} "
          f"({sorted(gname.values())}); non-deform groups ignored={sorted(stray)}")
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.meshes.remove(tmp_me)

    # zero-weight body vertices -> nearest deform bone
    seg = [(np.array(bones[n].head_local), np.array(bones[n].tail_local)) for n in def_names]
    zero = body_idx[W[body_idx].sum(axis=1) <= EPS_W]
    if len(zero):
        dist = np.stack([seg_dist(co[zero], a, b) for a, b in seg], axis=1)
        near = dist.argmin(axis=1)
        W[zero] = 0.0
        W[zero, near] = 1.0
        print(f"[s03] heat-failed body vertices (sum<=1e-4) filled by nearest bone: {len(zero)} "
              f"-> bones {sorted({def_names[i] for i in near})}")
    else:
        print("[s03] heat-failed body vertices (sum<=1e-4) filled by nearest bone: 0")

    # rigid parts
    for pname, bname in RIGID_BONE.items():
        m = vpart == parts[pname]
        W[m] = 0.0
        W[m, bi[bname]] = 1.0

    # limit to MAX_INF, drop tiny, normalize
    before = int(((W > EPS_W).sum(axis=1) > MAX_INF).sum())
    order = np.argsort(-W, axis=1)
    mask = np.zeros_like(W, dtype=bool)
    np.put_along_axis(mask, order[:, :MAX_INF], True, axis=1)
    W = np.where(mask & (W > EPS_W), W, 0.0)
    s = W.sum(axis=1)
    if (s <= 0).any():
        raise RuntimeError(f"{int((s <= 0).sum())} vertices with zero weight after limit")
    W = W / s[:, None]
    print(f"[s03] limit: vertices with >{MAX_INF} influences before limit={before}; "
          f"after: max influences={int((W > 0).sum(axis=1).max())}, "
          f"|sum-1| max={float(np.abs(W.sum(axis=1) - 1).max()):.2e}")

    for n in def_names:
        vg = gm.vertex_groups.new(name=n)
        col = W[:, bi[n]]
        idx = np.nonzero(col > 0)[0]
        for i in idx:
            vg.add([int(i)], float(col[i]), "REPLACE")

    # summary per part
    for pname, pv in parts.items():
        m = vpart == pv
        cnt = (W[m] > 0).sum(axis=1)
        hist = {k: int((cnt == k).sum()) for k in range(1, MAX_INF + 1)}
        tot = W[m].sum(axis=0)
        top = np.argsort(-tot)[:5]
        dom = np.bincount(W[m].argmax(axis=1), minlength=len(def_names))
        print(f"[s03] WEIGHTS part {pname} ({int(m.sum())} v): nonzero-bone count hist={hist}; "
              f"max-influence bone (by summed weight)={def_names[top[0]]}; top5 summed="
              + ", ".join(f"{def_names[i]}:{tot[i]:.1f}" for i in top if tot[i] > 0)
              + "; dominant-bone vertex counts="
              + ", ".join(f"{def_names[i]}:{int(dom[i])}" for i in np.argsort(-dom) if dom[i] > 0))

    # ---------------------------------------------------------------- modifier, club parent
    mod = gm.modifiers.new("Armature", "ARMATURE")
    mod.object = arm
    mod.use_vertex_groups = True
    mod.use_bone_envelopes = False

    club_mw = club.matrix_world.copy()
    club.parent = arm
    club.parent_type = "BONE"
    club.parent_bone = "weapon_socket_r"
    vl.update()
    club.matrix_world = club_mw
    vl.update()
    cdev = float(np.abs(np.array(club.matrix_world) - np.array(club_mw)).max())
    print(f"[s03] GOB_club parent={club.parent.name}/{club.parent_type}/{club.parent_bone} "
          f"world matrix max abs diff after parenting={cdev:.3e}")

    # ---------------------------------------------------------------- rest deformation
    for pb in arm.pose.bones:
        pb.rotation_mode = "QUATERNION"
        pb.rotation_quaternion = Quaternion()
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)
    arm.data.pose_position = "POSE"
    vl.update()

    def eval_co():
        dg = bpy.context.evaluated_depsgraph_get()
        e = gm.evaluated_get(dg)
        em = e.to_mesh()
        a = np.empty(len(em.vertices) * 3)
        em.vertices.foreach_get("co", a)
        e.to_mesh_clear()
        return a.reshape(-1, 3)

    rest = eval_co()
    rdev = np.linalg.norm(rest - co, axis=1)
    print(f"[s03] REST deformation (evaluated vs original, pose identity): max={rdev.max() * 1000:.6f} mm "
          f"(vertex {int(rdev.argmax())})")

    # ---------------------------------------------------------------- roll self check
    for bn, tgt, pname in (("lowerarm_l", "hand_l", "hand_l"), ("lowerarm_r", "hand_r", "hand_r"),
                           ("calf_l", "foot_l", "shoe_l"), ("calf_r", "foot_r", "shoe_r")):
        h0 = goblib.bone_world(arm, tgt).translation.copy()
        t0 = (arm.matrix_world @ arm.pose.bones[tgt].tail).copy()
        m = vpart == parts[pname]
        c0 = rest[m].mean(axis=0)
        pb = arm.pose.bones[bn]
        pb.rotation_quaternion = Quaternion((1, 0, 0), math.radians(30.0))
        vl.update()
        h1 = goblib.bone_world(arm, tgt).translation.copy()
        t1 = (arm.matrix_world @ arm.pose.bones[tgt].tail).copy()
        c1 = eval_co()[m].mean(axis=0)
        pb.rotation_quaternion = Quaternion()
        vl.update()
        dh, dt, dc = h1 - h0, t1 - t0, c1 - c0
        print(f"[s03] ROLLTEST {bn} +X 30deg: {tgt} head d=({dh.x * 1000:+.2f},{dh.y * 1000:+.2f},{dh.z * 1000:+.2f}) mm "
              f"tail d=({dt.x * 1000:+.2f},{dt.y * 1000:+.2f},{dt.z * 1000:+.2f}) mm; {pname} centroid "
              f"d=({dc[0] * 1000:+.2f},{dc[1] * 1000:+.2f},{dc[2] * 1000:+.2f}) mm; "
              f"Y sign head={'+' if dh.y > 0 else '-'} tail={'+' if dt.y > 0 else '-'} "
              f"centroid={'+' if dc[1] > 0 else '-'}")
    rest2 = eval_co()
    print(f"[s03] REST after roll test reset: max={np.linalg.norm(rest2 - co, axis=1).max() * 1000:.6f} mm")

    # ---------------------------------------------------------------- save
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s03] OUTPUT {OUT_BLEND} objects={sorted(o.name for o in bpy.data.objects)} "
          f"meshes={sorted(m.name for m in bpy.data.meshes)}")


goblib.run_main(main)
