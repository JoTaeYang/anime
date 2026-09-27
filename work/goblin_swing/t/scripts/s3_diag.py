"""STEP03 diagnostic: colorize by dominant vertex group, render posed shoulders."""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Quaternion

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
OUT = os.path.join(T, "inspect", "step03", "diag")
os.makedirs(OUT, exist_ok=True)
sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
lo = bpy.data.objects["GOB_body_lo"]
hi = bpy.data.objects["GOB_body"]
club = bpy.data.objects["GOB_club"]
club.hide_viewport = club.hide_render = True
g = bpy.data.objects.get("REF_ground")
if g:
    g.hide_viewport = g.hide_render = True
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
me = lo.data
N = len(me.vertices)
P = np.empty(N * 3); me.vertices.foreach_get("co", P); P = P.reshape(N, 3)
X, Y, Z = P[:, 0], P[:, 1], P[:, 2]
aX = np.abs(X)
R = {}

# ---- egg width profile: max |x| of NON-arm-zone verts by z
prof = []
for loz in np.arange(0.40, 1.42, 0.02):
    m = (Z >= loz) & (Z < loz + 0.02)
    if m.sum() >= 4:
        prof.append([round(float(loz) + 0.01, 3), int(m.sum()),
                     round(float(np.percentile(aX[m], 99)), 4)])
R["max_absx_by_z_allverts"] = prof
# and for verts far from both arm axes
for S in ("L", "R"):
    sh = np.array(arm.bones["UPPERARM_" + S].head_local)
    tl = np.array(arm.bones["HAND_" + S].tail_local)
    u = tl - sh; u /= np.linalg.norm(u)
    v = P - sh; s = v @ u
    d = np.linalg.norm(v - s[:, None] * u, axis=1)
    far = d > 0.16
    pr = []
    for loz in np.arange(0.90, 1.35, 0.02):
        m = (Z >= loz) & (Z < loz + 0.02) & far & ((X > 0) if S == "L" else (X < 0))
        if m.sum() >= 3:
            pr.append([round(float(loz) + 0.01, 3), int(m.sum()), round(float(aX[m].max()), 4)])
    R["egg_maxabsx_by_z_" + S] = pr

# ---- interior geometry probe: verts lying INSIDE the surface near the shoulder
import bmesh
bm = bmesh.new(); bm.from_mesh(me); bm.faces.ensure_lookup_table()
nonman = sum(1 for e in bm.edges if not e.is_manifold)
R["nonmanifold_edges_lo"] = nonman
bm.free()

PAL = [("HIPS", (0.95, .35, .20)), ("SPINE_01", (.98, .70, .15)), ("CHEST", (.85, .95, .20)),
       ("NECK", (.35, .85, .30)), ("HEAD", (.15, .60, .95)),
       ("UPPERARM_L", (.20, .95, .85)), ("FOREARM_L", (.10, .55, .75)),
       ("FOREARM_TWIST_L", (.55, .80, 1.0)), ("HAND_L", (.55, .30, .95)),
       ("UPPERARM_R", (.95, .30, .75)), ("FOREARM_R", (.70, .10, .35)),
       ("FOREARM_TWIST_R", (1.0, .55, .30)), ("HAND_R", (1.0, .85, .85)),
       ("THIGH_L", (.55, .95, .55)), ("SHIN_L", (.20, .70, .35)), ("FOOT_L", (.05, .35, .15)),
       ("THIGH_R", (.95, .75, .55)), ("SHIN_R", (.80, .50, .15)), ("FOOT_R", (.40, .25, .05))]


def colorize(ob):
    m = ob.data
    m.materials.clear()
    idx = {}
    for n, rgb in PAL:
        mat = bpy.data.materials.get("L_" + n) or bpy.data.materials.new("L_" + n)
        mat.use_nodes = False
        mat.diffuse_color = (*rgb, 1.0)
        m.materials.append(mat)
        idx[n] = len(m.materials) - 1
    gi = {gg.index: gg.name for gg in ob.vertex_groups}
    vl = [0] * len(m.vertices)
    for v in m.vertices:
        best, bw = None, -1
        for gg in v.groups:
            if gg.weight > bw:
                bw, best = gg.weight, gi[gg.group]
        vl[v.index] = idx.get(best, 0)
    for p in m.polygons:
        c = {}
        for i in p.vertices:
            c[vl[i]] = c.get(vl[i], 0) + 1
        p.material_index = max(c.items(), key=lambda kv: kv[1])[0]
        p.use_smooth = True
    m.update()


colorize(lo)
sc.render.engine = 'BLENDER_WORKBENCH'
sh_ = sc.display.shading
sh_.light = 'FLAT'
sh_.color_type = 'MATERIAL'
sh_.show_cavity = True
sh_.cavity_type = 'BOTH'
sh_.show_object_outline = False
sc.display.render_aa = '8'
sc.render.image_settings.file_format = 'PNG'
sc.render.resolution_x = sc.render.resolution_y = 640
cd = bpy.data.cameras.new("dcam"); cd.type = 'ORTHO'
cam = bpy.data.objects.new("dcam", cd); sc.collection.objects.link(cam); sc.camera = cam


def aim(t, az, el, s):
    a, e = math.radians(az), math.radians(el)
    d = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)))
    ctr = Vector(t)
    cam.location = ctr + d * 9
    cd.ortho_scale = s
    cam.rotation_mode = 'XYZ'
    cam.rotation_euler = (ctr - cam.location).to_track_quat('-Z', 'Y').to_euler()


def shot(n, t, az, el, s):
    aim(t, az, el, s)
    sc.render.filepath = os.path.join(OUT, n + ".png")
    bpy.ops.render.render(write_still=True)


def clear():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0); pb.rotation_euler = (0, 0, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0


def upd():
    rig.update_tag(); lo.update_tag(); bpy.context.view_layer.update()
    return bpy.context.evaluated_depsgraph_get()


FKA = {"windup": {"UPPERARM_FK_R": (-52.15, 100.89, 32.7), "FOREARM_FK_R": (127.7, 0, 0),
                  "HAND_FK_R": (-0.49, 134.92, 1.29)},
       "impact": {"UPPERARM_FK_R": (41.27, -54.39, 117.86), "FOREARM_FK_R": (5.26, 0, 0),
                  "HAND_FK_R": (-31.03, -107.0, -3.88)}}

clear(); dg = upd()
shot("rest_shR", (-0.42, 0.12, 1.26), 0, 0, 0.85)
shot("rest_shR_top", (-0.42, 0.12, 1.26), -20, 55, 0.85)
shot("rest_armR", (-0.70, 0.15, 1.26), 0, 30, 1.20)

for k, d in FKA.items():
    clear()
    for b, e in d.items():
        PB[b].rotation_euler = [math.radians(v) for v in e]
    dg = upd()
    ev = rig.evaluated_get(dg)
    shoulder = rig.matrix_world @ ev.pose.bones["UPPERARM_R"].head
    elbow = rig.matrix_world @ ev.pose.bones["FOREARM_R"].head
    for az, el, tag in ((0, 0, "front"), (-40, -20, "low"), (-20, 55, "top"), (180, 0, "back")):
        shot("%s_shR_%s" % (k, tag), tuple(shoulder), az, el, 0.85)
    shot("%s_elbR" % k, tuple(elbow), 0, 25, 0.55)
    # twist of UPPERARM about its own axis
    q = (REST["UPPERARM_R"].inverted() @ ev.pose.bones["UPPERARM_R"].matrix).to_quaternion()
    e = q.to_euler('YXZ')
    R[k + "_UPPERARM_local_YXZ_deg"] = [round(math.degrees(v), 2) for v in e]
    R[k + "_UPPERARM_total_deg"] = round(math.degrees(abs(q.angle)), 2)
    qf = (REST["FOREARM_R"].inverted() @ ev.pose.bones["FOREARM_R"].matrix).to_quaternion()
    R[k + "_FOREARM_world_vs_rest_deg"] = round(math.degrees(abs(qf.angle)), 2)

clear()
print("@@@JSON_START@@@"); print(json.dumps(R, default=str)); print("@@@JSON_END@@@")
