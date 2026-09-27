"""Is the leg 'crumple' real geometry or a shading artefact?  (read-only)"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector, Quaternion

sys.path.append(r"C:\Users\whxod\orca\anime\work\goblin_swing\scripts")
import step03_renderlib as RL

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
rig = bpy.data.objects["GOB_rig"]
body = bpy.data.objects["GOB_body"]
me = body.data
PB = rig.pose.bones
REST = {n: rig.data.bones[n].matrix_local.copy() for n in rig.data.bones.keys()}
N = len(me.vertices)
P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)
E = np.empty(len(me.edges) * 2, dtype=np.int32); me.edges.foreach_get("vertices", E)
E = E.reshape(-1, 2)
me.calc_loop_triangles()
TRI = np.array([t.vertices[:] for t in me.loop_triangles], dtype=np.int32)

R = {}
R["has_custom_normals"] = bool(me.has_custom_normals)
R["attributes"] = [a.name for a in me.attributes]
R["polygons_smooth"] = int(sum(1 for p in me.polygons if p.use_smooth))
R["n_polygons"] = len(me.polygons)
R["sharp_edges"] = int(sum(1 for e in me.edges if e.use_edge_sharp))


def deformed():
    rig.update_tag()
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    ev = body.evaluated_get(dg)
    dm = ev.to_mesh()
    Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q)
    ev.to_mesh_clear()
    return Q.reshape(-1, 3)


def wloc(bone, delta):
    PB[bone].location = REST[bone].to_3x3().inverted() @ Vector(delta)


def tri_normals(Q):
    a, b, c = Q[TRI[:, 0]], Q[TRI[:, 1]], Q[TRI[:, 2]]
    n = np.cross(b - a, c - a)
    L = np.linalg.norm(n, axis=1)
    L[L < 1e-12] = 1.0
    return n / L[:, None], L / 2.0


Z = P0[:, 2]
legL = (Z > 0.17) & (Z < 0.345) & (P0[:, 0] > 0.15)
tri_leg = np.nonzero(legL[TRI].all(axis=1))[0]
edge_leg = np.nonzero(legL[E].all(axis=1))[0]
R["leg_tris"] = int(len(tri_leg))
R["leg_edges"] = int(len(edge_leg))

N0, A0 = tri_normals(P0)
L0 = np.linalg.norm(P0[E[:, 0]] - P0[E[:, 1]], axis=1)

for tag, build in [("legL_fwd", lambda: wloc("FOOT_IK_L", (0, -0.12, 0.05))),
                   ("cog_down15", lambda: wloc("COG_CTRL", (0, 0, -0.15)))]:
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0); pb.rotation_euler = (0, 0, 0)
    build()
    Q = deformed()
    L1 = np.linalg.norm(Q[E[:, 0]] - Q[E[:, 1]], axis=1)
    strain = (L1[edge_leg] - L0[edge_leg]) / np.maximum(L0[edge_leg], 1e-9)
    N1, A1 = tri_normals(Q)
    # how much did each leg triangle rotate relative to the median leg rotation?
    med = N1[tri_leg].mean(0)
    ang0 = np.degrees(np.arccos(np.clip((N0[tri_leg] * N0[tri_leg][:, None][:, 0]).sum(1), -1, 1)))
    # normal-to-neighbour consistency is what the shading shows: compare the
    # angle each triangle normal turned, rest -> posed
    turn = np.degrees(np.arccos(np.clip((N0[tri_leg] * N1[tri_leg]).sum(1), -1, 1)))
    R[tag] = {
        "edge_strain_max_pct": round(float(np.abs(strain).max() * 100), 3),
        "edge_strain_p99_pct": round(float(np.percentile(np.abs(strain), 99) * 100), 3),
        "leg_area_change_pct": round(float((A1[tri_leg].sum() / A0[tri_leg].sum() - 1) * 100), 3),
        "normal_turn_deg": [round(float(turn.min()), 2), round(float(np.median(turn)), 2),
                            round(float(turn.max()), 2)],
        "normal_turn_spread_deg": round(float(turn.max() - turn.min()), 2),
    }

# render the same pose with cavity off / on to separate shading from geometry
for pb in PB:
    pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
    pb.rotation_quaternion = (1, 0, 0, 0); pb.rotation_euler = (0, 0, 0)
wloc("FOOT_IK_L", (0, -0.12, 0.05))
bpy.context.view_layer.update()
ev = rig.evaluated_get(bpy.context.evaluated_depsgraph_get())
knee = ev.pose.bones["SHIN_L"].matrix.to_translation()
OUT = os.path.join(ROOT, "inspect", "step03", "geomcheck")
cam, cd = RL.setup(res=1024, plain=True)
sh = bpy.context.scene.display.shading
RL.shoot(cam, cd, OUT, "cavity_on", knee, 345, 0, 0.42)
sh.show_cavity = False
RL.shoot(cam, cd, OUT, "cavity_off", knee, 345, 0, 0.42)
sh.show_cavity = True
# flat shading control
for p in me.polygons:
    p.use_smooth = False
me.update()
RL.shoot(cam, cd, OUT, "flat", knee, 345, 0, 0.42)
for p in me.polygons:
    p.use_smooth = True
me.update()
RL.shoot(cam, cd, OUT, "smooth", knee, 345, 0, 0.42)

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
