"""Probe v05: control rest matrices, projection sanity, eval speed."""
import bpy, json, math, os, sys, time
import numpy as np
from mathutils import Vector, Quaternion
from bpy_extras.object_utils import world_to_camera_view

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
cam = bpy.data.objects["REF_CAM"]
sc.camera = cam

out = {}
out["scene"] = {"fps": sc.render.fps, "start": sc.frame_start, "end": sc.frame_end,
                "res": [sc.render.resolution_x, sc.render.resolution_y]}
out["cam"] = {"loc": list(cam.location),
              "rot_deg": [math.degrees(v) for v in cam.rotation_euler],
              "type": cam.data.type, "lens": cam.data.lens,
              "ortho": cam.data.ortho_scale, "sensor_fit": cam.data.sensor_fit,
              "sensor_w": cam.data.sensor_width}

CTRLS = ["ROOT_CTRL", "COG_CTRL", "HIPS_CTRL", "CHEST_CTRL", "HEAD_CTRL",
         "UPPERARM_FK_L", "FOREARM_FK_L", "HAND_FK_L",
         "UPPERARM_FK_R", "FOREARM_FK_R", "HAND_FK_R",
         "FOOT_IK_L", "FOOT_IK_R", "WEAPON_CTRL", "KNEE_POLE_L", "KNEE_POLE_R"]
ci = {}
for n in CTRLS:
    b = arm.bones.get(n)
    if b is None:
        ci[n] = None
        continue
    ml = b.matrix_local
    ci[n] = {"rot_mode": PB[n].rotation_mode,
             "head": [round(v, 5) for v in b.head_local],
             "tail": [round(v, 5) for v in b.tail_local],
             "len": round(b.length, 5),
             "parent": b.parent.name if b.parent else None,
             # local axes in world/armature space
             "ax_x": [round(v, 4) for v in (ml.to_3x3() @ Vector((1, 0, 0)))],
             "ax_y": [round(v, 4) for v in (ml.to_3x3() @ Vector((0, 1, 0)))],
             "ax_z": [round(v, 4) for v in (ml.to_3x3() @ Vector((0, 0, 1)))]}
out["ctrls"] = ci
out["rig_matrix_world"] = [list(r) for r in rig.matrix_world]

# custom props
out["props"] = {}
for n in ("HAND_IK_L", "HAND_IK_R", "FOOT_IK_L", "FOOT_IK_R"):
    if n in PB:
        out["props"][n] = {k: PB[n][k] for k in PB[n].keys() if not k.startswith("_")}

# deform bone rest for arms
for n in ("UPPERARM_R", "FOREARM_R", "HAND_R", "WEAPON", "HEAD", "HIPS", "CHEST"):
    b = arm.bones.get(n)
    if b:
        out.setdefault("def", {})[n] = {
            "head": [round(v, 5) for v in b.head_local],
            "tail": [round(v, 5) for v in b.tail_local],
            "len": round(b.length, 5)}

# --- eval speed + rest landmark projection
mesh = bpy.data.objects["GOB_body_lo"]
mwm = np.array(mesh.matrix_world)
co = np.empty(len(mesh.data.vertices) * 3)
mesh.data.vertices.foreach_get("co", co)
V = (co.reshape(-1, 3) @ mwm[:3, :3].T) + mwm[:3, 3]
hb = arm.bones["HEAD"]
z0, hl, cx = hb.head_local.z, hb.length, hb.head_local.x
head = V[(V[:, 2] > z0 - 0.05 * hl) & (np.abs(V[:, 0] - cx) < 0.9 * hl)]
front = head[head[:, 1] < head[:, 1].min() + 0.015]
L = front[front[:, 0] > cx + 0.05 * hl]
R = front[front[:, 0] < cx - 0.05 * hl]
rest_pts = {"eye_L": ("HEAD", L.mean(0)), "eye_R": ("HEAD", R.mean(0)),
            "head_top": ("HEAD", V[V[:, 2] > V[:, 2].max() - 0.004].mean(0))}
zmin = V[:, 2].min()
ground = V[V[:, 2] < zmin + 0.005]
for tag, bone, sel in (("foot_L", "FOOT_L", ground[:, 0] > 0), ("foot_R", "FOOT_R", ground[:, 0] < 0)):
    P = ground[sel]
    rest_pts[tag] = (bone, np.array([(P[:, 0].min() + P[:, 0].max()) / 2,
                                     (P[:, 1].min() + P[:, 1].max()) / 2, zmin]))
aw_inv = rig.matrix_world.inverted()
localpts = {}
for k, (bn, p) in rest_pts.items():
    localpts[k] = [bn, list(arm.bones[bn].matrix_local.inverted() @ (aw_inv @ Vector(p)))]
out["rest_local_pts"] = localpts
out["n_verts_lo"] = len(mesh.data.vertices)

t0 = time.time()
for i in range(50):
    PB["COG_CTRL"].location = (0, 0, -0.001 * i)
    rig.update_tag()
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    ev = rig.evaluated_get(dg)
    _ = ev.pose.bones["WEAPON"].tail
out["eval_50_sec"] = round(time.time() - t0, 3)
PB["COG_CTRL"].location = (0, 0, 0)
rig.update_tag(); bpy.context.view_layer.update()

# rest projection of landmarks
dg = bpy.context.evaluated_depsgraph_get()
ev = rig.evaluated_get(dg)
W = ev.matrix_world
pts = {}
wb = ev.pose.bones["WEAPON"]
tail = W @ wb.tail
axis = (W @ wb.tail - W @ wb.head).normalized()
pts["weapon_tip"] = tail
pts["club_head_blobfit"] = tail - axis * 0.21
pts["hand_R"] = W @ ev.pose.bones["HAND_R"].head
pts["hand_L"] = W @ ((ev.pose.bones["HAND_L"].head + ev.pose.bones["HAND_L"].tail) / 2)
pts["hips"] = W @ ev.pose.bones["HIPS"].head
for k, (bn, off) in localpts.items():
    pts[k] = W @ (ev.pose.bones[bn].matrix @ Vector(off))
pts["head"] = (pts["eye_L"] + pts["eye_R"]) / 2
proj = {}
for k, v in pts.items():
    c = world_to_camera_view(sc, cam, v)
    proj[k] = [round(c.x * 448, 2), round((1 - c.y) * 576, 2)]
out["rest_proj_px"] = proj
out["rest_world"] = {k: [round(x, 4) for x in v] for k, v in pts.items()}

with open(os.path.join(ROOT, "inspect", "step05_probe.json"), "w") as f:
    json.dump(out, f, indent=1)
print("@@@PROBE_DONE@@@")
