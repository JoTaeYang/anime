"""p10_crude_rig.py - T210 (P1.1): crude walking-skeleton rig of the Clay Explorer (spec d-01 §2-§3).

Input : C:/Users/whxod/Downloads/Meshy_AI_Clay_Explorer_0926122001_generate.fbx (read only; sha256 checked vs P0b)
        work/player/inspect/P0b/p0b_measure.json (landmarks, parts, skirt, colours; read only)
Output: work/player/rig/pl_p1_crude.blend  armature object `Player` (41 bones, all unconnected, identity object),
                                           mesh object `Player_mesh` (all 20 parts joined, one material per part,
                                           vertex groups = bone names, one Armature modifier), action `p1test`
        work/player/rig/data/p1_manifest.json  skeleton table, skin table, p1test segments, render frames, cameras
        work/player/inspect/P1/blender_<cam>_<frame>.png  Workbench renders (no springs) of the render frames

CLI:  blender --background --factory-startup --python p10_crude_rig.py

- Skeleton (§2): positions from the P0b landmarks, left/right symmetrised (|x|, y, z averaged over both sides).
- Skin (§3): rigid parts 100 % to one bone; arm / leg tubes by bone heat (ARMATURE_AUTO) restricted to their limb
  chain; tunic by height (Spine_01 / Spine_02 / Pelvis) and, below the belt, by azimuth + height to the skirt chains.
- p1test: 24 fps, frames 1..72, root locked (Root never moves); keys on rotation_quaternion (and Pelvis location).
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Quaternion, Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
SRC = Path(r"C:\Users\whxod\Downloads\Meshy_AI_Clay_Explorer_0926122001_generate.fbx")
P0B = REPO / "work" / "player" / "inspect" / "P0b" / "p0b_measure.json"
OUT_BLEND = RIG / "pl_p1_crude.blend"
MANIFEST = RIG / "data" / "p1_manifest.json"
INSPECT = REPO / "work" / "player" / "inspect" / "P1"

ARM = "Player"
MESH = "Player_mesh"
ACTION = "p1test"
FPS = 24
F0, F1 = 1, 72
RENDER_FRAMES = [1, 8, 21, 31, 48, 61]
SKIRT_AZ = [("F", 0), ("FL", 45), ("L", 90), ("BL", 135), ("B", 180), ("BR", 225), ("R", 270), ("FR", 315)]
BACK_Z = {"BackSocket": 0.64, "BackWeaponSocket": 0.58}   # upper back, on the tunic surface (design choice)
RENDER_RES = 1024
CAM_TARGET = (0.0, 0.0, 0.55)
CAM_DIST = 3.0
CAM_ORTHO = 1.3
CAMS = {"front": 0.0, "three_quarter": 45.0}   # azimuth from the character front (-Y) toward character left (+X)


def log(msg):
    print(f"[p10] {msg}")
    sys.stdout.flush()


def v3(v, nd=5):
    return [round(float(x), nd) + 0.0 for x in v]


# ---------------------------------------------------------------- source
def import_source(p0b):
    sha = hashlib.sha256(SRC.read_bytes()).hexdigest()
    log(f"SOURCE {SRC} sha256 {sha} (P0b {p0b['source_sha256']}, match {sha == p0b['source_sha256']})")
    if sha != p0b["source_sha256"]:
        raise RuntimeError("source sha256 differs from P0b")
    bpy.ops.wm.read_factory_settings(use_empty=True)
    res = bpy.ops.import_scene.fbx(filepath=str(SRC))
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    if res != {"FINISHED"} or len(meshes) != 1:
        raise RuntimeError(f"import {res}, meshes {[o.name for o in meshes]}")
    ob = meshes[0]
    mw = ob.matrix_world.copy()
    ob.data.transform(mw)
    ob.matrix_world = Matrix.Identity(4)
    for o in list(bpy.data.objects):
        if o != ob:
            bpy.data.objects.remove(o, do_unlink=True)
    log(f"IMPORT {ob.name}: {len(ob.data.vertices)} verts, {len(ob.data.polygons)} polys; matrix_world applied to data "
        f"(was loc {v3(mw.to_translation(), 7)}, rot {v3(mw.to_euler(), 7)}, scale {v3(mw.to_scale(), 7)})")
    return ob


def split_parts(ob, p0b):
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    vl.objects.active = ob
    ob.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.separate(type="LOOSE")
    bpy.ops.object.mode_set(mode="OBJECT")
    pieces = [o for o in bpy.data.objects if o.type == "MESH"]
    parts = p0b["parts"]
    out, used = {}, set()
    for o in pieces:
        me = o.data
        c = sum((v.co for v in me.vertices), Vector()) / len(me.vertices)
        tris = sum(len(p.vertices) - 2 for p in me.polygons)
        best = min(parts, key=lambda n: (Vector(parts[n]["centroid"]) - c).length)
        d = (Vector(parts[best]["centroid"]) - c).length
        if d > 0.01 or tris != parts[best]["tris"] or best in used:
            raise RuntimeError(f"piece {o.name}: nearest part {best} at {d:.4f} m, tris {tris} vs {parts[best]['tris']}")
        used.add(best)
        o.name = best
        me.name = best
        out[best] = o
    if set(out) != set(parts):
        raise RuntimeError(f"parts missing {sorted(set(parts) - set(out))}")
    log(f"PARTS {len(out)} loose pieces matched to P0b parts by vertex centroid (<= 1 cm) and tri count: "
        + ", ".join(f"{n} {parts[n]['tris']}" for n in sorted(out)))
    return out


# ---------------------------------------------------------------- skeleton table
def sym(lm, base):
    l, r = Vector(lm[base + "_l"]["pos"]), Vector(lm[base + "_r"]["pos"])
    return Vector(((abs(l.x) + abs(r.x)) / 2.0, (l.y + r.y) / 2.0, (l.z + r.z) / 2.0))


def mirror(v, side):
    return Vector((v.x if side == "L" else -v.x, v.y, v.z))


def ray_hit(ob, origin, direction):
    """Outer surface point of a mesh object (identity matrix): ray from 0.5 m outside toward `origin`."""
    direction = direction.normalized()
    ok, loc, nrm, idx = ob.ray_cast(origin + direction * 0.5, -direction)
    if not ok:
        raise RuntimeError(f"ray from {v3(origin)} dir {v3(direction)} missed {ob.name}")
    return loc


def skeleton_table(p0b, tunic):
    lm = p0b["landmarks"]
    sk = p0b["skirt"]
    shoulder, elbow, wrist, fist = sym(lm, "shoulder"), sym(lm, "elbow"), sym(lm, "wrist"), sym(lm, "fist_center")
    hip, knee, ankle = sym(lm, "hip"), sym(lm, "knee"), sym(lm, "ankle")
    pel = Vector(lm["pelvis"]["pos"])
    pel.x = 0.0
    neck = Vector(lm["neck_base"]["pos"])
    neck.x = 0.0
    hbase = Vector(lm["head_base"]["pos"])
    hbase.x = 0.0
    hc = Vector(lm["head_center"]["pos"])
    hc.x = 0.0
    top = Vector((0.0, hc.y, p0b["parts"]["head"]["bb_max"][2]))
    sp1 = pel.lerp(neck, 1.0 / 3.0)
    sp2 = pel.lerp(neck, 2.0 / 3.0)
    FRONT, UP = Vector((0.0, -1.0, 0.0)), Vector((0.0, 0.0, 1.0))
    B = []   # (name, parent, head, tail, z_hint, deform, source)

    def add(name, parent, head, tail, zh, deform, src):
        B.append((name, parent, Vector(head), Vector(tail), Vector(zh), deform, src))

    add("Root", None, (0, 0, 0), (0, 0, 0.1), FRONT, False, "origin (ground, root locked)")
    add("Pelvis", "Root", pel, sp1, FRONT, True, "P0b pelvis (x=0) -> 1/3 pelvis..neck_base")
    add("Spine_01", "Pelvis", sp1, sp2, FRONT, True, "1/3 -> 2/3 pelvis..neck_base")
    add("Spine_02", "Spine_01", sp2, neck, FRONT, True, "2/3 -> P0b neck_base")
    add("Neck", "Spine_02", neck, hbase, FRONT, True, "P0b neck_base -> head_base")
    add("Head", "Neck", hbase, hc, FRONT, True, "P0b head_base -> head_center")
    add("HeadEquipmentSocket", "Head", top, top + Vector((0, 0, 0.05)), FRONT, False, "head top (head bb max z)")
    for s in ("L", "R"):
        sh, el, wr, fc = mirror(shoulder, s), mirror(elbow, s), mirror(wrist, s), mirror(fist, s)
        clav = Vector((0.03 if s == "L" else -0.03, sp2.y + 0.003, neck.z - 0.017))
        hand_dir = (fc - wr).normalized()
        add(f"Clavicle_{s}", "Spine_02", clav, sh, FRONT, True, "spine top (x +-0.03, z neck_base-0.017) -> shoulder")
        add(f"Arm_{s}", f"Clavicle_{s}", sh, el, FRONT, True, "P0b shoulder -> elbow (sym)")
        add(f"Forearm_{s}", f"Arm_{s}", el, wr, FRONT, True, "P0b elbow -> wrist (sym)")
        add(f"Hand_{s}", f"Forearm_{s}", wr, fc, FRONT, True, "P0b wrist (inside fist) -> fist centre")
        add(f"WeaponSocket_{s}", f"Hand_{s}", fc, fc + hand_dir * 0.05, FRONT, False, "fist centre, along the hand")
    for name, z in BACK_Z.items():
        hit = ray_hit(tunic, Vector((0.0, 0.02, z)), Vector((0.0, 1.0, 0.0)))
        add(name, "Spine_02", hit, hit + Vector((0, 0.05, 0)), UP, False,
            f"tunic outer back surface at z {z} (centre line)")
    for s in ("L", "R"):
        hp, kn, an = mirror(hip, s), mirror(knee, s), mirror(ankle, s)
        add(f"Thigh_{s}", "Pelvis", hp, kn, FRONT, True, "P0b hip -> knee (sym)")
        add(f"Calf_{s}", f"Thigh_{s}", kn, an, FRONT, True, "P0b knee -> ankle (sym)")
        add(f"Foot_{s}", f"Calf_{s}", an, Vector((an.x, an.y - 0.12, 0.03)), UP, True,
            "P0b ankle -> 0.12 toward the toe at z 0.03")
    cx, cy = sk["hem_ellipse"]["center_xy"]
    z_top, z_hem = p0b["skirt"]["belt_z"][0], sk["hem_section_z"]
    for tag, az in SKIRT_AZ:
        d = Vector((math.sin(math.radians(az)), -math.cos(math.radians(az)), 0.0))
        a = ray_hit(tunic, Vector((cx, cy, z_top)), d)
        b = ray_hit(tunic, Vector((cx, cy, z_hem)), d)
        m = (a + b) / 2.0
        add(f"Skirt_{tag}_01", "Pelvis", a, m, d, True,
            f"tunic outer surface at belt bottom z {z_top} azimuth {az} -> midpoint")
        add(f"Skirt_{tag}_02", f"Skirt_{tag}_01", m, b, d, True, f"midpoint -> tunic outer surface at hem z {z_hem}")
    return B


def build_armature(table):
    ad = bpy.data.armatures.new(ARM)
    rig = bpy.data.objects.new(ARM, ad)
    bpy.context.scene.collection.objects.link(rig)
    rig.matrix_world = Matrix.Identity(4)
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    vl.objects.active = rig
    rig.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = ad.edit_bones
    for name, parent, head, tail, zh, deform, _ in table:
        eb = ebs.new(name)
        eb.head, eb.tail = head, tail
        eb.align_roll(zh)
        eb.use_deform = deform
    for name, parent, *_ in table:
        eb = ebs[name]
        eb.parent = ebs[parent] if parent else None
        eb.use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")
    for pb in rig.pose.bones:
        pb.rotation_mode = "QUATERNION"
    log(f"ARMATURE {rig.name}: {len(ad.bones)} bones, connected {sum(1 for b in ad.bones if b.use_connect)}, "
        f"deform {sum(1 for b in ad.bones if b.use_deform)}, object matrix identity {rig.matrix_world == Matrix.Identity(4)}")
    return rig


# ---------------------------------------------------------------- skin
RIGID = {"head": "Head", "eye_l": "Head", "eye_r": "Head", "fist_l": "Hand_L", "fist_r": "Hand_R",
         "shoe_l": "Foot_L", "shoe_r": "Foot_R", "cuff_l": "Foot_L", "cuff_r": "Foot_R",
         "belt": "Pelvis", "pouch": "Pelvis", "sleeve_l": "Arm_L", "sleeve_r": "Arm_R",
         "scarf": "Spine_02", "scarf_tail": "Spine_02"}
AUTO = {"arm_l": ["Arm_L", "Forearm_L", "Hand_L"], "arm_r": ["Arm_R", "Forearm_R", "Hand_R"],
        "leg_l": ["Thigh_L", "Calf_L", "Foot_L"], "leg_r": ["Thigh_R", "Calf_R", "Foot_R"]}


def seg_dist(p, a, b):
    ab = b - a
    t = max(0.0, min(1.0, (p - a).dot(ab) / ab.length_squared))
    return (p - (a + ab * t)).length


def skin_auto(ob, rig, bones):
    vl = bpy.context.view_layer
    keep = {b.name: b.use_deform for b in rig.data.bones}
    for b in rig.data.bones:
        b.use_deform = b.name in bones
    for o in vl.objects:
        o.select_set(False)
    ob.select_set(True)
    rig.select_set(True)
    vl.objects.active = rig
    res = bpy.ops.object.parent_set(type="ARMATURE_AUTO")
    for b in rig.data.bones:
        b.use_deform = keep[b.name]
    # unweighted vertices (heat failure) -> nearest chain bone segment, 100 %
    idx = {g.index: g.name for g in ob.vertex_groups}
    fixed = 0
    for v in ob.data.vertices:
        if sum(g.weight for g in v.groups if idx.get(g.group) in bones) <= 1e-6:
            near = min(bones, key=lambda n: seg_dist(v.co, rig.data.bones[n].head_local, rig.data.bones[n].tail_local))
            vg = ob.vertex_groups.get(near) or ob.vertex_groups.new(name=near)
            vg.add([v.index], 1.0, "REPLACE")
            fixed += 1
    stray = [g.name for g in ob.vertex_groups if g.name not in bones]
    for n in stray:
        ob.vertex_groups.remove(ob.vertex_groups[n])
    counts = {}
    for v in ob.data.vertices:
        top = max(v.groups, key=lambda g: g.weight)
        counts[ob.vertex_groups[top.group].name] = counts.get(ob.vertex_groups[top.group].name, 0) + 1
    return res, fixed, counts


def smooth01(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def tunic_weights(v, p0b, table):
    """Returns {bone: weight} for one tunic vertex (rest position)."""
    sk = p0b["skirt"]
    z_bb, z_bt = sk["belt_z"]
    z_hem = sk["hem_z_min"]
    head = {n: h for n, _, h, *_ in table}
    cx, cy = sk["hem_ellipse"]["center_xy"]
    x, y, z = v
    if z >= z_bt:
        s = smooth01((z - (head["Spine_02"].z - 0.04)) / 0.08)
        w = {"Spine_01": 1.0 - s, "Spine_02": s}
        p = max(0.0, 1.0 - (z - z_bt) / 0.03)
        return {k: val * (1.0 - p) for k, val in w.items()} | {"Pelvis": p}
    if z >= z_bb:
        return {"Pelvis": 1.0}
    t = max(0.0, min(1.0, (z_bb - z) / (z_bb - z_hem)))
    w_pel = max(0.0, 1.0 - t / 0.2)
    r = math.hypot(x - cx, y - cy)
    w_pel = max(w_pel, 1.0 - r / 0.08)          # bottom-cap centre (azimuth undefined) -> Pelvis
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


def skin(parts, rig, p0b, table):
    colors = p0b["colors"]
    rows = []
    for name, ob in sorted(parts.items()):
        mat = bpy.data.materials.new(f"M_{name}")
        c = colors[name]
        mat.diffuse_color = (c[0], c[1], c[2], 1.0)
        ob.data.materials.clear()
        ob.data.materials.append(mat)
        if name in RIGID:
            vg = ob.vertex_groups.new(name=RIGID[name])
            vg.add([v.index for v in ob.data.vertices], 1.0, "REPLACE")
            rows.append((name, f"{RIGID[name]} 100 %", "rigid", len(ob.data.vertices)))
        elif name in AUTO:
            res, fixed, counts = skin_auto(ob, rig, AUTO[name])
            rows.append((name, "auto: " + ", ".join(f"{k} {n}" for k, n in sorted(counts.items())) + " (dominant-bone vertex counts)",
                         f"bone heat {sorted(res)} restricted to {AUTO[name]}; {fixed} unweighted verts -> nearest segment",
                         len(ob.data.vertices)))
        elif name == "tunic":
            counts = {}
            for v in ob.data.vertices:
                for bn, w in tunic_weights(v.co, p0b, table).items():
                    vg = ob.vertex_groups.get(bn) or ob.vertex_groups.new(name=bn)
                    vg.add([v.index], w, "REPLACE")
                    counts[bn] = counts.get(bn, 0) + 1
            rows.append((name, "height/azimuth: " + ", ".join(f"{k} {n}" for k, n in sorted(counts.items()))
                         + " (verts with weight > 0)", "Spine_01/02 above belt top, Pelvis in belt band, "
                         "skirt chains below belt bottom (azimuth blend between neighbours, height blend 01->02, "
                         "Pelvis near the belt and at the cap centre)", len(ob.data.vertices)))
        else:
            raise RuntimeError(f"part {name} has no skin rule")
    # join into Player_mesh
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    objs = [parts[n] for n in sorted(parts)]
    for o in objs:
        o.select_set(True)
    vl.objects.active = parts["tunic"]
    bpy.ops.object.join()
    ob = parts["tunic"]
    ob.name = MESH
    ob.data.name = MESH
    if ob.parent is not None:
        mw = ob.matrix_world.copy()
        ob.parent = None
        ob.matrix_world = mw
    for m in list(ob.modifiers):
        ob.modifiers.remove(m)
    mod = ob.modifiers.new("Armature", "ARMATURE")
    mod.object = rig
    # normalise weights
    names = {g.index: g.name for g in ob.vertex_groups}
    unweighted = 0
    for v in ob.data.vertices:
        tot = sum(g.weight for g in v.groups)
        if tot <= 1e-6:
            unweighted += 1
            continue
        for g in v.groups:
            g.weight = g.weight / tot
    bad = [n for n in names.values() if n not in rig.data.bones or not rig.data.bones[n].use_deform]
    log(f"MESH {ob.name}: {len(ob.data.vertices)} verts, {len(ob.data.polygons)} polys, materials "
        f"{len(ob.data.materials)}, vertex groups {len(ob.vertex_groups)}, unweighted verts {unweighted}, groups "
        f"that are not deform bones {bad}, matrix identity {ob.matrix_world == Matrix.Identity(4)}, modifiers "
        f"{[(m.type, m.object.name) for m in ob.modifiers]}")
    for r in rows:
        log(f"SKIN {r[0]} ({r[3]} verts): {r[1]} | {r[2]}")
    return ob, rows


# ---------------------------------------------------------------- p1test
def rot(axis, deg):
    return Matrix.Rotation(math.radians(deg), 3, {"X": "X", "Y": "Y", "Z": "Z"}[axis])


def key_pose(ax_rot):
    """ax_rot: list of (axis, deg) applied in order (world / armature rest axes) -> 3x3."""
    m = Matrix.Identity(3)
    for a, d in ax_rot:
        m = rot(a, d) @ m
    return m


# p1test keys: frame -> {bone: [(axis, deg), ...]}, "Pelvis_loc": world offset (m)
def p1test_keys():
    K = {}

    def k(f, **kw):
        K[f] = kw

    # A: arms raise / lower (1-16)
    k(1)
    k(8, Arm_L=[("Y", -80)], Arm_R=[("Y", 80)], Forearm_L=[("Y", -20)], Forearm_R=[("Y", 20)])
    k(16)
    # B: hips to the P0b contact angles and beyond (16-36): flex 60 / extension 50 / abduction 40
    k(21, Thigh_L=[("X", -60)], Calf_L=[("X", 40)], Thigh_R=[("X", 50)])
    k(26, Thigh_R=[("X", -60)], Calf_R=[("X", 40)], Thigh_L=[("X", 50)])
    k(31, Thigh_L=[("Y", -40)], Thigh_R=[("Y", 40)])
    k(36)
    # C: rough walk step (36-56): contact L fwd / passing / contact R fwd / passing
    k(40, Thigh_L=[("X", -30)], Thigh_R=[("X", 20)], Calf_R=[("X", 10)], Arm_L=[("X", 20)], Arm_R=[("X", -20)],
      Pelvis_loc=(0, 0, -0.015))
    k(44, Thigh_R=[("X", -5)], Calf_R=[("X", 45)], Pelvis_loc=(0, 0, 0.01))
    k(48, Thigh_R=[("X", -30)], Thigh_L=[("X", 20)], Calf_L=[("X", 10)], Arm_R=[("X", 20)], Arm_L=[("X", -20)],
      Pelvis_loc=(0, 0, -0.015))
    k(52, Thigh_L=[("X", -5)], Calf_L=[("X", 45)], Pelvis_loc=(0, 0, 0.01))
    k(56)
    # D: fast turn (56-72): pelvis yaw +70 / -70 / 0, then hold so the springs settle
    k(59, Pelvis=[("Z", 70)])
    k(63, Pelvis=[("Z", -70)])
    k(66)
    k(72)
    return K


SEGMENTS = [
    {"name": "arms", "start": 1, "end": 16, "motion": "Arm_L/R raise 80 deg about world Y (A-pose -> ~34 deg above "
     "horizontal) + forearm 20 deg, peak frame 8, back to rest at 16"},
    {"name": "hips", "start": 16, "end": 36, "motion": "f21 Thigh_L flex 60 + Calf_L 40, Thigh_R extension 50; f26 "
     "mirrored; f31 abduction 40 both; rest at 36 (P0b contact: flex 40.5-41.3, ext 36, abd 24.8-25.8)"},
    {"name": "walk", "start": 36, "end": 56, "motion": "rough step: f40 contact L fwd 30 / R back 20, f44 passing R "
     "(knee 45), f48 contact R fwd, f52 passing L, arms counter-swing 20, pelvis bob -15/+10 mm; rest at 56"},
    {"name": "turn", "start": 56, "end": 72, "motion": "Pelvis yaw +70 at f59, -70 at f63, 0 at f66, hold 66-72 "
     "(spring settle)"},
]


def make_action(rig):
    scene = bpy.context.scene
    scene.render.fps = FPS
    scene.render.fps_base = 1.0
    scene.frame_start, scene.frame_end = F0, F1
    keys = p1test_keys()
    animated = sorted({b for kw in keys.values() for b in kw if b != "Pelvis_loc"} | {"Root", "Pelvis"})
    pbs = rig.pose.bones
    prev = {}
    for f in sorted(keys):
        kw = keys[f]
        for n in animated:
            pb = pbs[n]
            R = rig.data.bones[n].matrix_local.to_3x3()
            D = key_pose(kw.get(n, []))
            q = (R.inverted() @ D @ R).to_quaternion()
            if n in prev:
                q.make_compatible(prev[n])
            prev[n] = q.copy()
            pb.rotation_quaternion = q
            pb.location = (0.0, 0.0, 0.0)
            if n == "Pelvis":
                pb.location = R.inverted() @ Vector(kw.get("Pelvis_loc", (0.0, 0.0, 0.0)))
            pb.scale = (1.0, 1.0, 1.0)
            pb.keyframe_insert("rotation_quaternion", frame=f, group=n)
            pb.keyframe_insert("location", frame=f, group=n)
    act = rig.animation_data.action
    act.name = ACTION
    act.use_frame_range = True
    act.frame_start, act.frame_end = F0, F1
    log(f"ACTION {act.name}: slot {rig.animation_data.action_slot.identifier!r}, keyed bones {animated}, key frames "
        f"{sorted(keys)}, frame range {F0}..{F1} @ {FPS} fps, root locked (Root keyed identity)")
    # self-check: knee / fist world positions at the peak frames (direction of the authored motion)
    for f, what in ((8, "Hand_L"), (21, "Calf_L"), (21, "Calf_R"), (31, "Calf_L"), (59, "Thigh_L")):
        scene.frame_set(f)
        log(f"CHECK frame {f}: {what} head world {v3(rig.matrix_world @ pbs[what].head)} (rest "
            f"{v3(rig.data.bones[what].head_local)})")
    scene.frame_set(F0)
    return act


# ---------------------------------------------------------------- cameras / renders
def cam_def(name, az):
    d = Vector((math.sin(math.radians(az)), -math.cos(math.radians(az)), 0.0))
    tgt = Vector(CAM_TARGET)
    loc = tgt + d * CAM_DIST

    def to_unity(v):
        return [-v.x, v.z, -v.y]

    return {"blender": {"location": v3(loc, 6), "target": v3(tgt, 6), "ortho_scale": CAM_ORTHO},
            "unity": {"position": to_unity(loc), "forward": to_unity((tgt - loc).normalized()), "up": [0.0, 1.0, 0.0],
                      "orthographicSize": CAM_ORTHO / 2.0, "nearClipPlane": 0.01, "farClipPlane": 10.0},
            "azimuth_deg_from_front_toward_left": az}


def render(rig, cams):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "MATERIAL"
    scene.render.resolution_x = scene.render.resolution_y = RENDER_RES
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "Standard"
    scene.render.image_settings.file_format = "PNG"
    world = bpy.data.worlds.new("W")
    world.color = (1.0, 1.0, 1.0)
    scene.world = world
    cd = bpy.data.cameras.new("cam")
    cd.type = "ORTHO"
    cam = bpy.data.objects.new("cam", cd)
    scene.collection.objects.link(cam)
    scene.camera = cam
    INSPECT.mkdir(parents=True, exist_ok=True)
    out = []
    for name, c in cams.items():
        b = c["blender"]
        cd.ortho_scale = b["ortho_scale"]
        loc, tgt = Vector(b["location"]), Vector(b["target"])
        cam.location = loc
        cam.rotation_euler = (tgt - loc).to_track_quat("-Z", "Y").to_euler()
        for f in RENDER_FRAMES:
            scene.frame_set(f)
            p = INSPECT / f"blender_{name}_{f}.png"
            scene.render.filepath = str(p)
            bpy.ops.render.render(write_still=True)
            out.append(str(p.relative_to(REPO)).replace("\\", "/"))
    bpy.data.objects.remove(cam, do_unlink=True)
    bpy.data.cameras.remove(cd)
    scene.frame_set(F0)
    log(f"RENDERS {len(out)}: {out}")
    return out


# ---------------------------------------------------------------- main
def main():
    p0b = json.loads(P0B.read_text(encoding="utf-8"))
    src = import_source(p0b)
    parts = split_parts(src, p0b)
    table = skeleton_table(p0b, parts["tunic"])
    rig = build_armature(table)
    mesh, skin_rows = skin(parts, rig, p0b, table)
    act = make_action(rig)
    bpy.context.scene.frame_set(F0)
    for o in list(bpy.data.objects):
        if o not in (rig, mesh):
            bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)

    bones = []
    for name, parent, head, tail, zh, deform, src_txt in table:
        b = rig.data.bones[name]
        bones.append({"name": name, "parent": parent, "head": v3(b.head_local, 6), "tail": v3(b.tail_local, 6),
                      "roll_hint_z": v3(zh, 4), "use_deform": b.use_deform, "use_connect": b.use_connect,
                      "source": src_txt})
    cams = {n: cam_def(n, az) for n, az in CAMS.items()}
    manifest = {
        "task": "T210", "gate": "P1", "generated_by": "work/player/rig/scripts/p10_crude_rig.py",
        "source": str(SRC), "source_sha256": p0b["source_sha256"], "blend": "work/player/rig/pl_p1_crude.blend",
        "armature_object": ARM, "mesh_object": MESH, "action": ACTION, "fps": FPS, "frame_start": F0, "frame_end": F1,
        "segments": SEGMENTS, "render_frames": RENDER_FRAMES, "cameras": cams,
        "axis_mapping_blender_to_unity": "unity.x = -blender.x, unity.y = blender.z, unity.z = -blender.y",
        "bones": bones,
        "skirt_chains": [[f"Skirt_{t}_01", f"Skirt_{t}_02"] for t, _ in SKIRT_AZ],
        "materials": {f"M_{n}": {"part": n, "color": p0b["colors"][n]} for n in sorted(p0b["parts"])},
        "leg_parts": ["leg_l", "leg_r"], "skirt_part": "tunic",
        "skin": [{"part": r[0], "assignment": r[1], "method": r[2], "verts": r[3]} for r in skin_rows],
    }
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
    log(f"MANIFEST {MANIFEST}")
    for b in bones:
        log(f"BONE {b['name']:<20} parent {str(b['parent']):<12} head {b['head']} deform {b['use_deform']}")

    render(rig, cams)
    for o in list(bpy.data.objects):
        o.select_set(False)
    bpy.context.view_layer.objects.active = rig
    OUT_BLEND.parent.mkdir(parents=True, exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND))
    log(f"OUTPUT {OUT_BLEND}; objects {[(o.name, o.type) for o in bpy.data.objects]}; actions "
        f"{[(a.name, [s.identifier for s in a.slots]) for a in bpy.data.actions]}")
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
