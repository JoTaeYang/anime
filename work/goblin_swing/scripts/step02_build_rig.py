import bpy, json, math, hashlib
from mathutils import Vector
from mathutils.bvhtree import BVHTree

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\goblin_v02_armature.blend"
V01_HASH = "23242f69c0f857c3e55b8e48793fd2baedf573cf"

body = bpy.data.objects["GOB_body"]
me = body.data

# ---------------------------------------------------------------- volume COM
com = Vector((0, 0, 0))
vol = 0.0
vs = me.vertices
for poly in me.polygons:
    a, b, c = (vs[i].co for i in poly.vertices[:3])
    v = a.dot(b.cross(c)) / 6.0
    vol += v
    com += (a + b + c) * (v / 4.0)
com = com / vol if abs(vol) > 1e-12 else Vector((0, 0, 0))

# ------------------------------------------------------- club axis (medial fit)
CLUB_PTS = [Vector(p) for p in [
    (-0.646, -0.352, 0.752), (-0.674, -0.338, 1.116), (-0.688, -0.338, 1.256),
    (-0.702, -0.338, 1.312), (-0.730, -0.324, 1.620), (-0.744, -0.324, 1.648)]]
mz = sum(p.z for p in CLUB_PTS) / len(CLUB_PTS)
mx = sum(p.x for p in CLUB_PTS) / len(CLUB_PTS)
my = sum(p.y for p in CLUB_PTS) / len(CLUB_PTS)
szz = sum((p.z - mz) ** 2 for p in CLUB_PTS)
sx = sum((p.z - mz) * (p.x - mx) for p in CLUB_PTS) / szz
sy = sum((p.z - mz) * (p.y - my) for p in CLUB_PTS) / szz
CLUB_D = Vector((sx, sy, 1.0)).normalized()


def club_at(z):
    return Vector((mx + sx * (z - mz), my + sy * (z - mz), z))


GRIP = club_at(0.991)
CLUB_TOP_Z = 1.8539

# ------------------------------------------------------------------ bone table
# NOTE: the measured whole-mesh volume centroid is z~1.03 (chest height) because the
# cube head is ~1/3 of the body volume. The spec asks for COG in the "lower belly"
# region, so COG is placed at the top of the belt band instead; the measured COM is
# reported separately as `com`.
COG_Z = 0.80
B = {}
B["ROOT"] = ((0.0, 0.0, 0.0), (0.0, 0.35, 0.0), None, False, False)
B["COG"] = ((0.0, 0.088, COG_Z), (0.0, 0.088, COG_Z + 0.18), "ROOT", False, False)
B["HIPS"] = ((0.0, 0.088, 0.55), (0.0, 0.088, 0.75), "COG", False, True)
B["SPINE_01"] = ((0.0, 0.088, 0.75), (0.0, 0.086, 1.01), "HIPS", True, True)
B["CHEST"] = ((0.0, 0.086, 1.01), (0.0, 0.085, 1.27), "SPINE_01", True, True)
B["NECK"] = ((0.0, 0.085, 1.27), (0.0, 0.050, 1.32), "CHEST", True, True)
B["HEAD"] = ((0.0, 0.050, 1.32), (0.0, 0.005, 1.85), "NECK", True, True)

B["UPPERARM_L"] = ((0.320, 0.100, 1.270), (0.550, 0.152, 1.092), "CHEST", False, True)
B["FOREARM_L"] = ((0.550, 0.152, 1.092), (0.724, 0.080, 0.840), "UPPERARM_L", True, True)
B["HAND_L"] = ((0.724, 0.080, 0.840), (0.788, 0.048, 0.681), "FOREARM_L", True, True)

B["UPPERARM_R"] = ((-0.312, 0.118, 1.272), (-0.582, 0.016, 1.072), "CHEST", False, True)
B["FOREARM_R"] = ((-0.582, 0.016, 1.072), (-0.676, -0.234, 0.980), "UPPERARM_R", True, True)
B["HAND_R"] = ((-0.676, -0.234, 0.980), (-0.683, -0.408, 0.995), "FOREARM_R", True, True)

B["WEAPON_SOCKET"] = (tuple(GRIP), tuple(GRIP + CLUB_D * 0.12), "HAND_R", False, False)
_t = (1.820 - GRIP.z) / CLUB_D.z
B["WEAPON"] = (tuple(GRIP), tuple(GRIP + CLUB_D * _t), "WEAPON_SOCKET", False, True)

B["THIGH_L"] = ((0.278, 0.100, 0.405), (0.292, 0.092, 0.283), "HIPS", False, True)
B["SHIN_L"] = ((0.292, 0.092, 0.283), (0.305, 0.103, 0.160), "THIGH_L", True, True)
B["FOOT_L"] = ((0.305, 0.103, 0.160), (0.380, -0.080, 0.090), "SHIN_L", True, True)

B["THIGH_R"] = ((-0.278, 0.100, 0.405), (-0.293, 0.092, 0.283), "HIPS", False, True)
B["SHIN_R"] = ((-0.293, 0.092, 0.283), (-0.307, 0.103, 0.160), "THIGH_R", True, True)
B["FOOT_R"] = ((-0.307, 0.103, 0.160), (-0.382, -0.080, 0.090), "SHIN_R", True, True)

ORDER = ["ROOT", "COG", "HIPS", "SPINE_01", "CHEST", "NECK", "HEAD",
         "UPPERARM_L", "FOREARM_L", "HAND_L",
         "UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON_SOCKET", "WEAPON",
         "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R"]

# --------------------------------------------------------------- create rig
arm_data = bpy.data.armatures.new("GOB_rig_data")
rig = bpy.data.objects.new("GOB_rig", arm_data)
bpy.context.scene.collection.objects.link(rig)
rig.location = (0, 0, 0)
rig.rotation_mode = 'QUATERNION'
rig.rotation_quaternion = (1, 0, 0, 0)
rig.scale = (1, 1, 1)

for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
eb = arm_data.edit_bones
for n in ORDER:
    h, t, par, conn, dfm = B[n]
    b = eb.new(n)
    b.head = Vector(h)
    b.tail = Vector(t)
    b.use_deform = dfm
for n in ORDER:
    h, t, par, conn, dfm = B[n]
    if par:
        eb[n].parent = eb[par]
        eb[n].use_connect = conn


def set_local_x(bone, xdir):
    y = (bone.tail - bone.head).normalized()
    x = Vector(xdir)
    x = x - y * x.dot(y)
    x.normalize()
    bone.align_roll(x.cross(y))


WX = Vector((1, 0, 0))
# arm bend-plane normals from the actual bone vectors
def bend_normal(u_name, f_name):
    u = (eb[u_name].tail - eb[u_name].head).normalized()
    f = (eb[f_name].tail - eb[f_name].head).normalized()
    return u.cross(f).normalized()


N_R = bend_normal("UPPERARM_R", "FOREARM_R")
N_L = bend_normal("UPPERARM_L", "FOREARM_L")
for n in ORDER:
    if n in ("UPPERARM_R", "FOREARM_R", "HAND_R"):
        set_local_x(eb[n], N_R)
    elif n in ("UPPERARM_L", "FOREARM_L", "HAND_L"):
        set_local_x(eb[n], N_L)
    else:
        set_local_x(eb[n], WX)

axinfo = {}
for n in ORDER:
    b = eb[n]
    axinfo[n] = {"x_axis": [round(c, 4) for c in b.x_axis],
                 "y_axis": [round(c, 4) for c in b.y_axis],
                 "z_axis": [round(c, 4) for c in b.z_axis]}

bpy.ops.object.mode_set(mode='OBJECT')

# ------------------------------------------------------------- verification
bvh = BVHTree.FromObject(body, bpy.context.evaluated_depsgraph_get())
RAYS = [Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1)),
        Vector((0.577, 0.577, 0.577)), Vector((-0.577, 0.3, 0.75)),
        Vector((0.26, -0.8, 0.54))]


def inside_and_dist(p):
    votes = 0
    for d in RAYS:
        cur = Vector(p) + d * 1e-5
        n = 0
        for _ in range(80):
            hit = bvh.ray_cast(cur, d)
            if hit[0] is None:
                break
            n += 1
            cur = hit[0] + d * 1e-5
        if n % 2 == 1:
            votes += 1
    loc, nor, idx, dist = bvh.find_nearest(Vector(p))
    return (votes >= 4, votes, round(dist, 4) if loc else None)


rep = {"volume_m3": round(vol, 6), "com": [round(c, 4) for c in com],
       "club_dir": [round(c, 5) for c in CLUB_D],
       "grip": [round(c, 4) for c in GRIP],
       "bend_normal_R": [round(c, 4) for c in N_R],
       "bend_normal_L": [round(c, 4) for c in N_L]}

bones = []
for n in ORDER:
    b = rig.data.bones[n]
    bones.append({
        "name": n,
        "head": [round(c, 4) for c in b.head_local],
        "tail": [round(c, 4) for c in b.tail_local],
        "len": round(b.length, 4),
        "roll_deg": round(math.degrees(arm_data.bones[n].matrix_local.to_euler().z) if False else 0, 2),
        "parent": b.parent.name if b.parent else None,
        "connect": b.use_connect,
        "deform": b.use_deform,
        "axes": axinfo[n],
    })
rep["bones"] = bones

# rolls (need edit mode to read .roll)
bpy.ops.object.mode_set(mode='EDIT')
for d in rep["bones"]:
    d["roll_deg"] = round(math.degrees(arm_data.edit_bones[d["name"]].roll), 2)
bpy.ops.object.mode_set(mode='OBJECT')

# joint inside-mesh tests
LIMB = ["UPPERARM_L", "FOREARM_L", "HAND_L", "UPPERARM_R", "FOREARM_R", "HAND_R",
        "THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R", "WEAPON",
        "HIPS", "SPINE_01", "CHEST", "NECK", "HEAD", "WEAPON_SOCKET", "COG"]
jt = []
for n in LIMB:
    b = rig.data.bones[n]
    for which, p in (("head", b.head_local), ("tail", b.tail_local)):
        ins, votes, dist = inside_and_dist(p)
        jt.append({"bone": n, "pt": which, "p": [round(c, 4) for c in p],
                   "inside": ins, "votes": votes, "dist_surf": dist})
rep["joint_tests"] = jt

# --- how well do the bone chains follow the measured medial axis of each limb?
MEDIAL = {
    "armR": (["UPPERARM_R", "FOREARM_R"], [
        (-0.400, 0.104, 1.232), (-0.424, 0.104, 1.220), (-0.448, 0.092, 1.196),
        (-0.472, 0.092, 1.184), (-0.508, 0.080, 1.148), (-0.544, 0.056, 1.112),
        (-0.580, 0.020, 1.076), (-0.604, -0.016, 1.052), (-0.616, -0.040, 1.040),
        (-0.628, -0.064, 1.028), (-0.640, -0.100, 1.016), (-0.652, -0.136, 1.004),
        (-0.664, -0.184, 0.992)]),
    "armL": (["UPPERARM_L", "FOREARM_L"], [
        (0.424, 0.116, 1.212), (0.448, 0.128, 1.188), (0.496, 0.140, 1.152),
        (0.532, 0.152, 1.116), (0.568, 0.152, 1.068), (0.616, 0.140, 1.008),
        (0.628, 0.140, 0.984), (0.652, 0.128, 0.948), (0.664, 0.116, 0.924),
        (0.688, 0.104, 0.888), (0.700, 0.092, 0.864), (0.724, 0.080, 0.840)]),
    "legR": (["THIGH_R", "SHIN_R"], [
        (-0.310, 0.100, 0.140), (-0.300, 0.110, 0.200), (-0.290, 0.110, 0.280),
        (-0.290, 0.100, 0.300), (-0.280, 0.100, 0.390)]),
    "legL": (["THIGH_L", "SHIN_L"], [
        (0.310, 0.100, 0.140), (0.300, 0.110, 0.190), (0.290, 0.110, 0.280),
        (0.280, 0.100, 0.380)]),
    "club": (["WEAPON"], [
        (-0.646, -0.352, 0.752), (-0.674, -0.338, 1.116), (-0.688, -0.338, 1.256),
        (-0.702, -0.338, 1.312), (-0.730, -0.324, 1.620), (-0.744, -0.324, 1.648)]),
}


def seg_dist(p, a, b):
    ab = b - a
    t = max(0.0, min(1.0, (p - a).dot(ab) / ab.length_squared))
    return (p - (a + ab * t)).length


md = {}
for k, (bnames, pts) in MEDIAL.items():
    segs = [(Vector(rig.data.bones[n].head_local), Vector(rig.data.bones[n].tail_local))
            for n in bnames]
    ds = [min(seg_dist(Vector(p), a, b) for a, b in segs) for p in pts]
    md[k] = {"max_dev": round(max(ds), 4), "mean_dev": round(sum(ds) / len(ds), 4),
             "n": len(ds)}
rep["medial_deviation"] = md
rep["joints_outside"] = [j for j in jt if not j["inside"]]
rep["joints_tight"] = [j for j in jt if j["inside"] and j["dist_surf"] is not None and j["dist_surf"] < 0.005]

# structural checks
h = hashlib.sha1()
for v in me.vertices:
    h.update(b"%.6f|%.6f|%.6f;" % (v.co.x, v.co.y, v.co.z))
bb = [Vector(c) for c in body.bound_box]
rep["mesh_check"] = {
    "verts": len(me.vertices), "faces": len(me.polygons),
    "hash": h.hexdigest(), "hash_matches_v01": h.hexdigest() == V01_HASH,
    "bbox_min": [round(min(p[i] for p in bb), 4) for i in range(3)],
    "bbox_max": [round(max(p[i] for p in bb), 4) for i in range(3)],
    "modifiers": [m.type for m in body.modifiers],
    "vertex_groups": len(body.vertex_groups),
    "parent": body.parent.name if body.parent else None,
}
rep["rig_obj"] = {
    "name": rig.name, "data": arm_data.name,
    "loc": [round(c, 6) for c in rig.location],
    "quat": [round(c, 6) for c in rig.rotation_quaternion],
    "scale": [round(c, 6) for c in rig.scale],
    "identity": (all(abs(c) < 1e-9 for c in rig.location)
                 and abs(rig.rotation_quaternion[0] - 1) < 1e-9
                 and all(abs(c - 1) < 1e-9 for c in rig.scale)),
    "n_bones": len(rig.data.bones),
}
rep["zero_len_bones"] = [b["name"] for b in rep["bones"] if b["len"] < 1e-4]
hier_ok = all((rep["bones"][i]["parent"] == B[rep["bones"][i]["name"]][2]
               and rep["bones"][i]["connect"] == B[rep["bones"][i]["name"]][3]
               and rep["bones"][i]["deform"] == B[rep["bones"][i]["name"]][4])
              for i in range(len(rep["bones"])))
rep["hierarchy_matches_spec"] = hier_ok
rep["scene"] = {"fps": bpy.context.scene.render.fps,
                "start": bpy.context.scene.frame_start,
                "end": bpy.context.scene.frame_end}

print("@@@JSON_START@@@")
print(json.dumps(rep, indent=1))
print("@@@JSON_END@@@")

bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
