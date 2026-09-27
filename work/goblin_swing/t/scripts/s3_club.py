"""Club/belly clearance remedy sweep for the STEP 02 impact arm pose."""
import bpy, math, json
import numpy as np
from mathutils import Vector, Quaternion
from mathutils.bvhtree import BVHTree

rig = bpy.data.objects["GOB_rig"]; arm = rig.data; PB = rig.pose.bones
ob = bpy.data.objects["GOB_body"]; club = bpy.data.objects["GOB_club"]
for c, v in ((bpy.data.collections["GOB_hi"], True), (bpy.data.collections["GOB_lo"], False)):
    c.hide_viewport = c.hide_render = not v
ob.hide_viewport = ob.hide_render = False
for lc in bpy.context.view_layer.layer_collection.children:
    if lc.name in ("GOB_hi", "GOB_lo"):
        lc.exclude = False; lc.hide_viewport = (lc.name == "GOB_lo")
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
me = ob.data; me.calc_loop_triangles()
TRI = [tuple(t.vertices) for t in me.loop_triangles]
cme = club.data; cme.calc_loop_triangles()
CTRI = [tuple(t.vertices) for t in cme.loop_triangles]
CP = np.empty(len(cme.vertices) * 3); cme.vertices.foreach_get("co", CP); CP = CP.reshape(-1, 3)


def clear():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0); pb.rotation_euler = (0, 0, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0


def upd():
    rig.update_tag(); ob.update_tag(); club.update_tag()
    bpy.context.view_layer.update()
    return bpy.context.evaluated_depsgraph_get()


def wrot(b, ax, dg_):
    M = REST[b].to_3x3(); q = Quaternion(Vector(ax).normalized(), math.radians(dg_))
    PB[b].rotation_euler = ((M.inverted() @ q.to_matrix() @ M).to_quaternion()
                            ).to_euler(PB[b].rotation_mode)


R = {}


def setpose(dz=0.0, dx=0.0, wx=0.0, elb=0.0, lean=0.0):
    clear()
    if lean:
        wrot("COG_CTRL", (1, 0, 0), lean)
    e = [41.27 + dx, -54.39, 117.86 + dz]
    PB["UPPERARM_FK_R"].rotation_euler = [math.radians(v) for v in e]
    PB["FOREARM_FK_R"].rotation_euler = [math.radians(5.26 + elb), 0, 0]
    PB["HAND_FK_R"].rotation_euler = [math.radians(v) for v in (-31.03, -107.0, -3.88)]
    if wx:
        PB["WEAPON_CTRL"].rotation_euler = [math.radians(wx), 0, 0]


def meas(tag):
    dg = upd()
    ev = ob.evaluated_get(dg); dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q)
    Q = Q.reshape(-1, 3); ev.to_mesh_clear()
    Q = (np.c_[Q, np.ones(len(Q))] @ np.array(ob.matrix_world).T)[:, :3]
    M = np.array(club.evaluated_get(dg).matrix_world)
    CW = CP @ M[:3, :3].T + M[:3, 3]
    bt = BVHTree.FromPolygons([Vector(q) for q in Q], TRI, all_triangles=True, epsilon=0.0)
    ct = BVHTree.FromPolygons([Vector(p) for p in CW], CTRI, all_triangles=True, epsilon=0.0)
    ov = ct.overlap(bt)
    deep, nin, mn = 0.0, 0, 1e9
    for i in range(0, len(CW), 3):
        p = Vector(CW[i]); cnt = 0; o = p.copy()
        for _ in range(8):
            loc, nor, fi, dd = bt.ray_cast(o, Vector((0, 0, 1)), 3.0)
            if loc is None:
                break
            cnt += 1; o = loc + Vector((0, 0, 1)) * 1e-4
        l2, _, _, d2 = bt.find_nearest(p, 1.0)
        if l2 is not None and d2 < mn:
            mn = d2
        if cnt % 2 == 1:
            nin += 1
            if l2 is not None and d2 > deep:
                deep = d2
    R[tag] = {"ovl": len(ov), "inside": nin, "pen_mm": round(deep * 1000, 1),
              "min_mm": round(mn * 1000, 1) if mn < 1e8 else None}
    print("###", tag, R[tag])


for v in (0, 15, 30, 45, 60):
    setpose(dz=-v); meas("armZ_minus%d" % v)
for v in (30, 60, 90):
    setpose(wx=v); meas("weaponCTRL_X%d" % v)
for v in (30, 50):
    setpose(elb=v); meas("elbow_plus%d" % v)
setpose(dz=-45, wx=60); meas("armZ45_wX60")
setpose(dz=-45, elb=30); meas("armZ45_elb30")
print("@@@J@@@"); print(json.dumps(R)); print("@@@E@@@")
