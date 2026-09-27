"""GATE 02 - rig stress test on goblin_v03_controls.blend.
Run: blender -b goblin_v03_controls.blend --python step02b_gate.py
NEVER saves.  Poses are temporary; rest is restored and verified at the end.
"""
import bpy, json, math, os
from mathutils import Vector, Matrix, Quaternion
from mathutils.bvhtree import BVHTree

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step02b"
REPORT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step02b_gate.json"
os.makedirs(OUT, exist_ok=True)

sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
body = bpy.data.objects["GOB_body"]
PB = rig.pose.bones

DEF_BONES = ["ROOT", "COG", "HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
             "UPPERARM_L", "FOREARM_L", "HAND_L", "UPPERARM_R", "FOREARM_R",
             "HAND_R", "WEAPON_SOCKET", "WEAPON", "THIGH_L", "SHIN_L", "FOOT_L",
             "THIGH_R", "SHIN_R", "FOOT_R"]
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
LEN = {n: arm.bones[n].length for n in arm.bones.keys()}

R = {"tests": {}}


# ================================================================ pose helpers
def upd():
    rig.update_tag()
    bpy.context.view_layer.update()
    return rig.evaluated_get(bpy.context.evaluated_depsgraph_get())


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_axis_angle = (0, 0, 1, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0
        PB["FOOT_IK_" + S]["ik_stretch"] = 0.0


def set_world_loc(bone, world_delta):
    """pose-bone location that moves the bone by world_delta (parent at rest)."""
    M = REST[bone].to_3x3()
    PB[bone].location = M.inverted() @ Vector(world_delta)


def set_world_rot(bone, axis, deg, additive=False):
    """rotate the bone by `deg` about the world `axis` through its head."""
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ ql) if additive else ql
    else:
        e = ql.to_euler(pb.rotation_mode)
        pb.rotation_euler = e


def set_local_rot(bone, axis, deg):
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = q
    else:
        pb.rotation_euler = q.to_euler(pb.rotation_mode)


def bhead(ev, n):
    return ev.pose.bones[n].matrix.to_translation()


def btail(ev, n):
    m = ev.pose.bones[n].matrix
    return m @ Vector((0, LEN[n], 0))


def mat_dev(a, b):
    dp = (a.to_translation() - b.to_translation()).length
    q = a.to_quaternion().rotation_difference(b.to_quaternion())
    return dp, math.degrees(abs(q.angle))


def signed_angle(v1, v2, axis):
    v1 = v1 - axis * v1.dot(axis)
    v2 = v2 - axis * v2.dot(axis)
    if v1.length < 1e-9 or v2.length < 1e-9:
        return None
    v1 = v1.normalized(); v2 = v2.normalized()
    return math.degrees(math.atan2(v1.cross(v2).dot(axis), v1.dot(v2)))


def foot_error(ev):
    """world distance of each DEF foot head from its IK target head."""
    o = {}
    for S in ("L", "R"):
        o[S] = round((bhead(ev, "FOOT_" + S) - bhead(ev, "FOOT_IK_" + S)).length, 6)
    return o


def knee_dir(ev, S):
    """unit perpendicular offset of the knee from the hip->ankle line."""
    hip = bhead(ev, "THIGH_" + S)
    ank = bhead(ev, "FOOT_" + S)
    kn = bhead(ev, "SHIN_" + S)
    ax = (ank - hip)
    if ax.length < 1e-9:
        return None, 0.0
    ax.normalize()
    d = (kn - hip)
    d = d - ax * d.dot(ax)
    return ([round(c, 3) for c in d.normalized()] if d.length > 1e-9 else None,
            round(d.length, 5))


def elbow_angle(ev, S):
    """bend-plane azimuth (deg) of the elbow around the shoulder->wrist axis,
    measured from the world -Z reference projected on that plane."""
    sh = bhead(ev, "UPPERARM_" + S)
    wr = bhead(ev, "HAND_" + S)
    el = bhead(ev, "FOREARM_" + S)
    ax = (wr - sh)
    if ax.length < 1e-9:
        return None, 0.0
    ax.normalize()
    d = el - sh
    d = d - ax * d.dot(ax)
    ref = Vector((0, 0, -1))
    ref = ref - ax * ref.dot(ax)
    if ref.length < 1e-6:
        ref = Vector((0, -1, 0))
        ref = ref - ax * ref.dot(ax)
    a = signed_angle(ref, d, ax)
    return (round(a, 3) if a is not None else None, round(d.length, 5))


# ================================================================ head BVH
hv, hf = [], []
idx = {}
me = body.data
for p in me.polygons:
    vs = list(p.vertices)
    if all(me.vertices[i].co.z > 1.30 for i in vs):
        f = []
        for i in vs:
            if i not in idx:
                idx[i] = len(hv)
                hv.append(me.vertices[i].co.copy())
            f.append(idx[i])
        hf.append(tuple(f))
HEAD_BVH = BVHTree.FromPolygons(hv, hf, all_triangles=False, epsilon=0.0)
R["head_bvh"] = {"verts": len(hv), "faces": len(hf)}


def seg_clearance(a, b, n=25):
    """min distance from a segment to the head surface."""
    best = 1e9
    for i in range(n + 1):
        p = a.lerp(b, i / n)
        loc, nor, fi, dist = HEAD_BVH.find_nearest(p)
        if loc is not None and dist < best:
            best = dist
    return best


def arm_head_clearance(ev):
    segs = {
        "upperarm": (bhead(ev, "UPPERARM_R"), bhead(ev, "FOREARM_R")),
        "forearm": (bhead(ev, "FOREARM_R"), bhead(ev, "HAND_R")),
        "hand": (bhead(ev, "HAND_R"), btail(ev, "HAND_R")),
        "weapon": (bhead(ev, "WEAPON"), btail(ev, "WEAPON")),
    }
    return {k: round(seg_clearance(a, b), 4) for k, (a, b) in segs.items()}


# ================================================================ renderer
try:
    sc.render.engine = 'BLENDER_EEVEE_NEXT'
except TypeError:
    sc.render.engine = 'BLENDER_EEVEE'
sc.render.resolution_x = 900
sc.render.resolution_y = 900
sc.render.resolution_percentage = 100
sc.render.image_settings.file_format = 'PNG'
try:
    sc.eevee.taa_render_samples = 8
except Exception:
    pass
sc.world = bpy.data.worlds.new("tmp_world")
sc.world.use_nodes = True
_nt = sc.world.node_tree
_nt.nodes.clear()
_o = _nt.nodes.new("ShaderNodeOutputWorld")
_b = _nt.nodes.new("ShaderNodeBackground")
_b.inputs[0].default_value = (0.06, 0.06, 0.07, 1)
_nt.links.new(_b.outputs[0], _o.inputs[0])


def emis(name, rgb, strength=2.2):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    o = nt.nodes.new("ShaderNodeOutputMaterial")
    e = nt.nodes.new("ShaderNodeEmission")
    e.inputs[0].default_value = (rgb[0], rgb[1], rgb[2], 1)
    e.inputs[1].default_value = strength
    nt.links.new(e.outputs[0], o.inputs[0])
    return m


COL = {"spine": emis("c_spine", (1.0, 0.85, 0.10)),
       "L": emis("c_L", (0.15, 0.55, 1.0)),
       "R": emis("c_R", (1.0, 0.18, 0.18)),
       "weap": emis("c_weap", (0.15, 1.0, 0.35)),
       "root": emis("c_root", (1.0, 1.0, 1.0), 1.4),
       "joint": emis("c_joint", (1.0, 0.45, 1.0), 3.0),
       "ctrl": emis("c_ctrl", (0.2, 1.0, 1.0), 2.0)}

bmat = bpy.data.materials.new("tmp_body_xray")
bmat.use_nodes = True
nt = bmat.node_tree
nt.nodes.clear()
out = nt.nodes.new("ShaderNodeOutputMaterial")
mix = nt.nodes.new("ShaderNodeMixShader")
dif = nt.nodes.new("ShaderNodeBsdfDiffuse")
dif.inputs[0].default_value = (0.75, 0.75, 0.78, 1)
tra = nt.nodes.new("ShaderNodeBsdfTransparent")
fre = nt.nodes.new("ShaderNodeFresnel")
fre.inputs[0].default_value = 1.25
nt.links.new(fre.outputs[0], mix.inputs[0])
nt.links.new(tra.outputs[0], mix.inputs[1])
nt.links.new(dif.outputs[0], mix.inputs[2])
nt.links.new(mix.outputs[0], out.inputs[0])
if hasattr(bmat, "surface_render_method"):
    bmat.surface_render_method = 'BLENDED'
if hasattr(bmat, "show_transparent_back"):
    bmat.show_transparent_back = False
body.data.materials.clear()
body.data.materials.append(bmat)

TMP = bpy.data.collections.new("TMP_PROXY")
sc.collection.children.link(TMP)


def clear_proxy():
    for ob in list(TMP.objects):
        d = ob.data
        bpy.data.objects.remove(ob, do_unlink=True)
        if d and d.users == 0:
            bpy.data.meshes.remove(d)


def cylinder(name, a, b, r, mat):
    d = b - a
    L = d.length
    if L < 1e-6:
        return
    m = bpy.data.meshes.new(name)
    v, f = [], []
    N = 8
    for k in (0, 1):
        for i in range(N):
            ang = 2 * math.pi * i / N
            v.append((r * math.cos(ang), r * math.sin(ang), k * L))
    for i in range(N):
        j = (i + 1) % N
        f.append((i, j, N + j, N + i))
    f.append(tuple(range(N - 1, -1, -1)))
    f.append(tuple(range(N, 2 * N)))
    m.from_pydata(v, [], f)
    m.update()
    m.materials.append(mat)
    ob = bpy.data.objects.new(name, m)
    TMP.objects.link(ob)
    ob.matrix_world = Matrix.Translation(a) @ d.to_track_quat('Z', 'Y').to_matrix().to_4x4()


def ball(name, c, r, mat):
    m = bpy.data.meshes.new(name)
    v, f = [], []
    S, Rr = 8, 5
    for i in range(Rr + 1):
        th = math.pi * i / Rr
        for j in range(S):
            ph = 2 * math.pi * j / S
            v.append((r * math.sin(th) * math.cos(ph), r * math.sin(th) * math.sin(ph),
                      r * math.cos(th)))
    for i in range(Rr):
        for j in range(S):
            a = i * S + j
            b = i * S + (j + 1) % S
            f.append((a, b, b + S, a + S))
    m.from_pydata(v, [], f)
    m.update()
    m.materials.append(mat)
    ob = bpy.data.objects.new(name, m)
    TMP.objects.link(ob)
    ob.location = c


def grp(n):
    if n in ("WEAPON", "WEAPON_SOCKET"):
        return "weap"
    if n in ("ROOT", "COG"):
        return "root"
    if n.endswith("_L"):
        return "L"
    if n.endswith("_R"):
        return "R"
    return "spine"


def build_proxy(ev):
    clear_proxy()
    for n in DEF_BONES:
        a = bhead(ev, n)
        b = btail(ev, n)
        r = max(0.008, min(0.020, LEN[n] * 0.07))
        cylinder("BX_" + n, a, b, r, COL[grp(n)])
        ball("JT_" + n, a, 0.018, COL["joint"])
    for n in ("HAND_R", "HAND_L", "FOOT_R", "FOOT_L", "HEAD", "WEAPON"):
        ball("JTT_" + n, btail(ev, n), 0.018, COL["joint"])
    for n in ("FOOT_IK_L", "FOOT_IK_R", "HAND_IK_L", "HAND_IK_R",
              "KNEE_POLE_L", "KNEE_POLE_R", "ELBOW_POLE_L", "ELBOW_POLE_R"):
        ball("CT_" + n, bhead(ev, n), 0.026, COL["ctrl"])


cd = bpy.data.cameras.new("tmp_cam")
cd.type = 'ORTHO'
cam = bpy.data.objects.new("tmp_cam", cd)
sc.collection.objects.link(cam)
sc.camera = cam
lamp = bpy.data.lights.new("tmp_l", 'SUN')
lamp.energy = 3.0
lo = bpy.data.objects.new("tmp_l", lamp)
sc.collection.objects.link(lo)
lo.rotation_euler = (math.radians(55), 0, math.radians(30))

SHOTS = []


def shoot(name, target, az_deg, el_deg, scale):
    az, el = math.radians(az_deg), math.radians(el_deg)
    d = Vector((math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)))
    ctr = Vector(target)
    cam.location = ctr + d * 8.0
    cd.ortho_scale = scale
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, name + ".png")
    bpy.ops.render.render(write_still=True)
    SHOTS.append(sc.render.filepath)
    print("WROTE", sc.render.filepath)


C = (0.0, 0.0, 0.95)


def snap(ev, tag, views=("front", "side")):
    build_proxy(ev)
    for v in views:
        if v == "front":
            shoot(tag + "_front", C, 0, 0, 2.3)
        elif v == "side":
            shoot(tag + "_side", C, 90, 0, 2.3)
        elif v == "tq":
            shoot(tag + "_tq", C, 35, 10, 2.3)
        elif v == "legs":
            shoot(tag + "_legs", (0.0, 0.05, 0.28), 0, 0, 1.0)
        elif v == "legsside":
            shoot(tag + "_legsside", (0.0, 0.05, 0.28), 90, 0, 1.0)
        elif v == "club":
            shoot(tag + "_club", (-0.4, -0.15, 1.35), 90, 0, 1.6)


# ================================================================ 0. reopen check
def check_rest(tag):
    ev = upd()
    rows = [(n,) + mat_dev(ev.pose.bones[n].matrix, REST[n]) for n in DEF_BONES]
    return {"tag": tag,
            "max_pos_m": max(r[1] for r in rows),
            "max_rot_deg": max(r[2] for r in rows),
            "violations": [[n, round(p, 8), round(d, 6)] for n, p, d in rows
                           if p > 1e-4 or d > 0.01]}


clear_pose()
inv = [check_rest("reopen ik_fk=0")]
for S in ("L", "R"):
    PB["HAND_IK_" + S]["ik_fk"] = 1.0
inv.append(check_rest("reopen ik_fk=1"))
clear_pose()
R["reopen_invariance"] = inv

# sanity: does the ik_fk switch actually do anything?
clear_pose()
set_world_loc("HAND_IK_R", (0, 0, 0.20))
ev = upd()
d_fk = (bhead(ev, "HAND_R") - REST["HAND_R"].to_translation()).length
PB["HAND_IK_R"]["ik_fk"] = 1.0
ev = upd()
d_ik = (bhead(ev, "HAND_R") - REST["HAND_R"].to_translation()).length
R["ik_fk_switch_check"] = {"hand_move_at_ikfk0_m": round(d_fk, 5),
                           "hand_move_at_ikfk1_m": round(d_ik, 5)}
clear_pose()

ev = upd()
snap(ev, "g00_rest", ("front", "side"))

# ================================================================ T1 COG lateral
t1 = []
for dx in (-0.15, 0.15):
    clear_pose()
    set_world_loc("COG_CTRL", (dx, 0, 0))
    ev = upd()
    row = {"dx": dx, "foot_err_m": foot_error(ev)}
    for S in ("L", "R"):
        kd, kl = knee_dir(ev, S)
        row["knee_" + S] = {"dir": kd, "offset_m": kl}
    row["hips_world"] = [round(c, 4) for c in bhead(ev, "HIPS")]
    t1.append(row)
clear_pose()
set_world_loc("COG_CTRL", (0.15, 0, 0))
ev = upd()
snap(ev, "g01_cog_right", ("front", "legs"))
R["tests"]["T1_cog_lateral"] = t1

# ================================================================ T2 COG vertical
t2 = []
for dz in [0.05, 0.02, 0.01, 0.0, -0.02, -0.05, -0.08, -0.10, -0.12, -0.15,
           -0.18, -0.20, -0.22, -0.24, -0.245, -0.26]:
    clear_pose()
    set_world_loc("COG_CTRL", (0, 0, dz))
    ev = upd()
    fe = foot_error(ev)
    kL, oL = knee_dir(ev, "L")
    kR, oR = knee_dir(ev, "R")
    hipz = round(bhead(ev, "THIGH_L").z, 5)
    ank = bhead(ev, "FOOT_L")
    t2.append({"dz": dz, "foot_err_m": fe, "hip_z": hipz,
               "hip_ankle_dist": round((bhead(ev, "THIGH_L") - ank).length, 5),
               "knee_off_L": oL, "knee_off_R": oR,
               "knee_dir_L": kL})
clear_pose()
set_world_loc("COG_CTRL", (0, 0, -0.15))
ev = upd()
snap(ev, "g02_cog_down15", ("front", "legsside"))
clear_pose()
set_world_loc("COG_CTRL", (0, 0, -0.24))
ev = upd()
snap(ev, "g02_cog_down24", ("front", "legsside"))
R["tests"]["T2_cog_vertical"] = t2

# ================================================================ T3 chest
t3 = []
for label, axis, deg in [("twist+45", (0, 0, 1), 45), ("twist-45", (0, 0, 1), -45),
                         ("sidebend30", (0, 1, 0), 30),
                         ("forward30", (1, 0, 0), 30)]:
    clear_pose()
    set_world_rot("CHEST_CTRL", axis, deg)
    ev = upd()
    sp = ev.pose.bones["SPINE_01"].matrix.to_quaternion().rotation_difference(
        REST["SPINE_01"].to_quaternion())
    ch = ev.pose.bones["CHEST"].matrix.to_quaternion().rotation_difference(
        REST["CHEST"].to_quaternion())
    hd = ev.pose.bones["HEAD"].matrix.to_quaternion().rotation_difference(
        REST["HEAD"].to_quaternion())
    t3.append({"case": label, "cmd_deg": deg,
               "spine01_world_rot_deg": round(math.degrees(abs(sp.angle)), 3),
               "chest_world_rot_deg": round(math.degrees(abs(ch.angle)), 3),
               "head_world_rot_deg": round(math.degrees(abs(hd.angle)), 3),
               "head_tip": [round(c, 4) for c in btail(ev, "HEAD")],
               "club_tip": [round(c, 4) for c in btail(ev, "WEAPON")],
               "foot_err_m": foot_error(ev)})
clear_pose()
set_world_rot("CHEST_CTRL", (0, 0, 1), 45)
ev = upd()
snap(ev, "g03_chest_twist45", ("front", "tq"))
clear_pose()
set_world_rot("CHEST_CTRL", (1, 0, 0), 30)
ev = upd()
snap(ev, "g03_chest_fwd30", ("side",))
R["tests"]["T3_chest"] = t3

# ================================================================ T4 arms raise
clear_pose()
set_world_rot("UPPERARM_FK_L", (0, -1, 0), 90)
set_world_rot("UPPERARM_FK_R", (0, 1, 0), 90)
ev = upd()
t4 = {"both_90": {"handL": [round(c, 4) for c in bhead(ev, "HAND_L")],
                  "handR": [round(c, 4) for c in bhead(ev, "HAND_R")],
                  "clearance": arm_head_clearance(ev),
                  "club_tip": [round(c, 4) for c in btail(ev, "WEAPON")]}}
snap(ev, "g04_arms90", ("front", "side"))

sweep = []
for a in range(0, 181, 10):
    clear_pose()
    set_world_rot("UPPERARM_FK_R", (0, 1, 0), a)
    ev = upd()
    cl = arm_head_clearance(ev)
    sweep.append({"deg": a, "clearance": cl,
                  "min": round(min(cl.values()), 4),
                  "hand": [round(c, 3) for c in bhead(ev, "HAND_R")],
                  "club_tip": [round(c, 3) for c in btail(ev, "WEAPON")]})
t4["right_raise_sweep"] = sweep
clear_pose()
set_world_rot("UPPERARM_FK_R", (0, 1, 0), 140)
ev = upd()
snap(ev, "g04_armR_140", ("front", "side"))
R["tests"]["T4_arm_raise"] = t4

# ================================================================ T5 elbows / IK sweep
clear_pose()
for S in ("L", "R"):
    set_local_rot("FOREARM_FK_" + S, (1, 0, 0), 90)
ev = upd()
t5 = {"fk_elbow90": {}}
for S in ("L", "R"):
    ang, off = elbow_angle(ev, S)
    up = (bhead(ev, "FOREARM_" + S) - bhead(ev, "UPPERARM_" + S)).normalized()
    fo = (bhead(ev, "HAND_" + S) - bhead(ev, "FOREARM_" + S)).normalized()
    t5["fk_elbow90"][S] = {"elbow_flex_deg": round(180 - math.degrees(up.angle(fo)), 2),
                           "bendplane_deg": ang,
                           "wrist": [round(c, 4) for c in bhead(ev, "HAND_" + S)]}
snap(ev, "g05_fk_elbow90", ("front", "side"))

# IK sweep: right hand arcs from overhead-right to low-left across the body
clear_pose()
PB["HAND_IK_R"]["ik_fk"] = 1.0
sh = REST["UPPERARM_R"].to_translation()
wr0 = REST["HAND_R"].to_translation()
Rr = 0.52          # reach radius, inside max reach 0.6336
A = Vector((-0.42, -0.30, 1.85))    # overhead right
Bp = Vector((0.30, -0.42, 0.72))    # low left across the body
ik_sweep = []
prev = None
for i in range(20):
    t = i / 19.0
    p = A.lerp(Bp, t)
    d = p - sh
    if d.length > Rr:
        p = sh + d.normalized() * Rr
    clear_pose()
    PB["HAND_IK_R"]["ik_fk"] = 1.0
    set_world_loc("HAND_IK_R", p - wr0)
    ev = upd()
    ang, off = elbow_angle(ev, "R")
    reach_err = (bhead(ev, "HAND_R") - bhead(ev, "HAND_IK_R")).length
    row = {"i": i, "target": [round(c, 3) for c in p], "bendplane_deg": ang,
           "elbow_off_m": off, "reach_err_m": round(reach_err, 5),
           "elbow": [round(c, 3) for c in bhead(ev, "FOREARM_R")]}
    if prev is not None and ang is not None and prev is not None:
        d1 = ang - prev
        while d1 > 180:
            d1 -= 360
        while d1 < -180:
            d1 += 360
        row["delta_deg"] = round(d1, 3)
    prev = ang
    ik_sweep.append(row)
t5["ik_sweep"] = ik_sweep
t5["ik_sweep_max_jump_deg"] = round(max(abs(r.get("delta_deg", 0)) for r in ik_sweep), 3)
t5["ik_sweep_sign_changes"] = sum(
    1 for a, b in zip(ik_sweep, ik_sweep[1:])
    if a["bendplane_deg"] is not None and b["bendplane_deg"] is not None
    and (a["bendplane_deg"] > 0) != (b["bendplane_deg"] > 0))
clear_pose()
PB["HAND_IK_R"]["ik_fk"] = 1.0
set_world_loc("HAND_IK_R", (A - wr0))
ev = upd()
snap(ev, "g05_ik_overhead", ("front", "side"))
R["tests"]["T5_elbows"] = t5

# ================================================================ T6 hips yaw
t6 = []
for deg in (-30, 30):
    clear_pose()
    set_world_rot("HIPS_CTRL", (0, 0, 1), deg)
    set_world_loc("COG_CTRL", (0.05, 0, -0.04))
    ev = upd()
    kL, oL = knee_dir(ev, "L")
    kR, oR = knee_dir(ev, "R")
    t6.append({"yaw": deg, "foot_err_m": foot_error(ev),
               "knee_dir_L": kL, "knee_dir_R": kR,
               "knee_off_L": oL, "knee_off_R": oR,
               "hip_L": [round(c, 4) for c in bhead(ev, "THIGH_L")],
               "hip_R": [round(c, 4) for c in bhead(ev, "THIGH_R")]})
clear_pose()
set_world_rot("HIPS_CTRL", (0, 0, 1), 30)
set_world_loc("COG_CTRL", (0.05, 0, -0.04))
ev = upd()
snap(ev, "g06_hips_yaw30", ("front", "legs"))
R["tests"]["T6_hips_yaw"] = t6

# ================================================================ T7 foot step
clear_pose()
set_world_loc("FOOT_IK_R", (-0.15, 0, 0.10))
ev = upd()
kR, oR = knee_dir(ev, "R")
kL, oL = knee_dir(ev, "L")
t7 = {"foot_err_m": foot_error(ev), "knee_dir_R": kR, "knee_off_R": oR,
      "knee_dir_L": kL, "knee_off_L": oL,
      "knee_R_world": [round(c, 4) for c in bhead(ev, "SHIN_R")],
      "foot_R_world": [round(c, 4) for c in bhead(ev, "FOOT_R")]}
# small sweep of the step to look for knee flips
st = []
prevd = None
for i in range(13):
    f = i / 12.0
    clear_pose()
    set_world_loc("FOOT_IK_R", (-0.15 * f, 0, 0.10 * f))
    ev = upd()
    kd, ko = knee_dir(ev, "R")
    hip = bhead(ev, "THIGH_R")
    ank = bhead(ev, "FOOT_R")
    ax = (ank - hip).normalized()
    a = signed_angle(Vector((0, -1, 0)), bhead(ev, "SHIN_R") - hip, ax)
    row = {"f": round(f, 2), "knee_dir": kd, "knee_off": ko,
           "azimuth_deg": round(a, 2) if a is not None else None}
    if prevd is not None and a is not None:
        d1 = a - prevd
        while d1 > 180:
            d1 -= 360
        while d1 < -180:
            d1 += 360
        row["delta_deg"] = round(d1, 2)
    prevd = a
    st.append(row)
t7["sweep"] = st
t7["max_jump_deg"] = round(max(abs(r.get("delta_deg", 0)) for r in st), 2)
clear_pose()
set_world_loc("FOOT_IK_R", (-0.15, 0, 0.10))
ev = upd()
snap(ev, "g07_step_R", ("front", "legsside"))
R["tests"]["T7_foot_step"] = t7

# ================================================================ T8 head
t8 = []
for label, axis, deg in [("yaw+40", (0, 0, 1), 40), ("yaw-40", (0, 0, 1), -40),
                         ("pitch+30", (1, 0, 0), 30), ("pitch-30", (1, 0, 0), -30)]:
    clear_pose()
    set_world_rot("HEAD_CTRL", axis, deg)
    ev = upd()
    hq = ev.pose.bones["HEAD"].matrix.to_quaternion().rotation_difference(
        REST["HEAD"].to_quaternion())
    nq = ev.pose.bones["NECK"].matrix.to_quaternion().rotation_difference(
        REST["NECK"].to_quaternion())
    haxis = hq.axis
    t8.append({"case": label, "cmd_deg": deg,
               "head_world_rot_deg": round(math.degrees(abs(hq.angle)), 3),
               "head_rot_axis": [round(c, 3) for c in haxis],
               "neck_world_rot_deg": round(math.degrees(abs(nq.angle)), 3),
               "head_tip": [round(c, 4) for c in btail(ev, "HEAD")],
               "chest_moved_m": round((bhead(ev, "CHEST") -
                                       REST["CHEST"].to_translation()).length, 6)})
clear_pose()
set_world_rot("HEAD_CTRL", (0, 0, 1), 40)
ev = upd()
snap(ev, "g08_head_yaw40", ("front",))
R["tests"]["T8_head"] = t8

# ================================================================ T9 weapon
t9 = []
for label, ax in [("X", (1, 0, 0)), ("Y", (0, 1, 0)), ("Z", (0, 0, 1))]:
    for deg in (-20, 20):
        clear_pose()
        set_world_rot("WEAPON_CTRL", ax, deg)
        ev = upd()
        t9.append({"axis": label, "deg": deg,
                   "club_tip_move_m": round((btail(ev, "WEAPON") -
                                             REST["WEAPON"] @ Vector((0, LEN["WEAPON"], 0))
                                             ).length, 5),
                   "club_grip_move_m": round((bhead(ev, "WEAPON") -
                                              REST["WEAPON"].to_translation()).length, 6),
                   "hand_move_m": round((bhead(ev, "HAND_R") -
                                         REST["HAND_R"].to_translation()).length, 6),
                   "hand_rot_deg": round(math.degrees(abs(
                       ev.pose.bones["HAND_R"].matrix.to_quaternion().rotation_difference(
                           REST["HAND_R"].to_quaternion()).angle)), 5)})
clear_pose()
set_world_rot("WEAPON_CTRL", (0, 1, 0), 20)
ev = upd()
snap(ev, "g09_weapon_y20", ("front", "club"))
R["tests"]["T9_weapon"] = t9

# ================================================================ restore
clear_pose()
R["final_rest"] = check_rest("final restore")
resid = []
for pb in PB:
    if (pb.location.length > 1e-9 or (Vector(pb.scale) - Vector((1, 1, 1))).length > 1e-9
            or Vector(pb.rotation_euler).length > 1e-9
            or abs(pb.rotation_quaternion.w - 1) > 1e-9
            or Vector(pb.rotation_quaternion[1:]).length > 1e-9):
        resid.append(pb.name)
R["residual_posed_bones"] = resid
R["props"] = {n: dict(PB[n].items()) for n in ("HAND_IK_L", "HAND_IK_R",
                                              "FOOT_IK_L", "FOOT_IK_R")}
R["renders"] = SHOTS

with open(REPORT, "w") as f:
    json.dump(R, f, indent=1, default=str)
print("@@@GATE_DONE@@@", REPORT)
