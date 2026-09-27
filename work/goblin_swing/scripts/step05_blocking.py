"""STEP 05 - build goblin_v06_blocking.blend from goblin_v05_legfix.blend.

  blender -b goblin_v05_legfix.blend --python scripts/step05_blocking.py

Deterministic: every pose value comes from scripts/step05_poses.json (produced by
scripts/step05_solve.py).  Creates the action GOB_swing on GOB_rig, CONSTANT
interpolation everywhere, keys only on the blocking key frames, saves
goblin_v06_blocking.blend with the scene on frame 1.

Blender 5.1 notes: Action.fcurves no longer exists - the curves live in
action.layers[0].strips[0].channelbag(slot).fcurves, so interpolation is set by
walking the channelbags.  Bone.select is gone, so nothing here selects bones.
"""
import bpy, json, math, os, sys
from mathutils import Vector, Quaternion

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import step05_targets as T

POSES = json.load(open(os.path.join(ROOT, "scripts", "step05_poses.json")))
IDX = POSES["dof_layout"]
FOOTZ = POSES["footR_z"]

sc = bpy.context.scene
sc.render.fps = 24
sc.frame_start, sc.frame_end = 1, 39
sc.render.resolution_x, sc.render.resolution_y = 448, 576
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}

# rotation modes: keep exactly what the rig was built with.  COG/HIPS/CHEST/HEAD/
# FOOT_IK are XYZ Euler (clean, small angles, no gimbal risk in these ranges);
# the FK arm chains stay QUATERNION because they swing through >150 deg.
ROT_MODES = {n: PB[n].rotation_mode for n in
             ("COG_CTRL", "HIPS_CTRL", "CHEST_CTRL", "HEAD_CTRL", "FOOT_IK_L",
              "FOOT_IK_R", "UPPERARM_FK_L", "FOREARM_FK_L", "HAND_FK_L",
              "UPPERARM_FK_R", "FOREARM_FK_R", "HAND_FK_R")}


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_axis_angle = (0, 0, 1, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0
        PB["FOOT_IK_" + S]["leg_roll"] = 0.0


def set_wloc(bone, delta):
    PB[bone].location = REST[bone].to_3x3().inverted() @ Vector(delta)


def _q(rv_deg):
    v = Vector(rv_deg)
    return Quaternion(v.normalized(), math.radians(v.length)) if v.length > 1e-9 else Quaternion()


def set_wrot(bone, rv_deg):
    M = REST[bone].to_3x3()
    ql = (M.inverted() @ _q(rv_deg).to_matrix() @ M).to_quaternion()
    pb = PB[bone]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = ql
    else:
        pb.rotation_euler = ql.to_euler(pb.rotation_mode)


def set_lrot(bone, rv_deg):
    pb = PB[bone]
    q = _q(rv_deg)
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = q
    else:
        pb.rotation_euler = q.to_euler(pb.rotation_mode)


def apply(x, footR_z):
    clear_pose()
    a, b = IDX["cog_loc"]; set_wloc("COG_CTRL", [v / 100.0 for v in x[a:b]])
    for nm, bone in (("cog_rot", "COG_CTRL"), ("hips_rot", "HIPS_CTRL"),
                     ("chest_rot", "CHEST_CTRL"), ("head_rot", "HEAD_CTRL")):
        a, b = IDX[nm]; set_wrot(bone, x[a:b])
    a, b = IDX["footR_loc"]
    set_wloc("FOOT_IK_R", (x[a] / 100.0, x[a + 1] / 100.0, footR_z))
    for nm, bone in (("uaR", "UPPERARM_FK_R"), ("haR", "HAND_FK_R"), ("uaL", "UPPERARM_FK_L")):
        a, b = IDX[nm]; set_lrot(bone, x[a:b])
    for nm, bone in (("faR", "FOREARM_FK_R"), ("faL", "FOREARM_FK_L")):
        a, b = IDX[nm]; set_lrot(bone, (x[a], x[a + 1], 0.0))


# controls that get a key on EVERY key pose (self-contained blocking poses)
ANIMATED = [("COG_CTRL", ("location", "rot")),
            ("HIPS_CTRL", ("rot",)),
            ("CHEST_CTRL", ("rot",)),
            ("HEAD_CTRL", ("rot",)),
            ("UPPERARM_FK_R", ("rot",)), ("FOREARM_FK_R", ("rot",)), ("HAND_FK_R", ("rot",)),
            ("UPPERARM_FK_L", ("rot",)), ("FOREARM_FK_L", ("rot",)), ("HAND_FK_L", ("rot",)),
            ("FOOT_IK_R", ("location",))]


def rot_path(bone):
    return "rotation_quaternion" if PB[bone].rotation_mode == 'QUATERNION' else "rotation_euler"


# ------------------------------------------------------------------ build
if rig.animation_data:
    rig.animation_data_clear()
for a in list(bpy.data.actions):
    bpy.data.actions.remove(a)
clear_pose()

for f in T.KEYS:
    x = POSES["poses"][str(f)]
    # frame_set FIRST: it re-evaluates the (partially built) action and would
    # otherwise overwrite the pose we are about to key.
    sc.frame_set(f)
    apply(x, FOOTZ[str(f)])
    for bone, kinds in ANIMATED:
        pb = PB[bone]
        for k in kinds:
            pb.keyframe_insert(data_path="location" if k == "location" else rot_path(bone),
                               group=bone)
    # the planted left foot: keyed only at the two ends, and never moves
    if f in (1, 39):
        PB["FOOT_IK_L"].keyframe_insert(data_path="location", group="FOOT_IK_L")

act = rig.animation_data.action
act.name = "GOB_swing"
act.use_fake_user = True
try:
    act.use_frame_range = True
    act.frame_start, act.frame_end = 1, 39
except Exception:
    pass

# ------------------------------------------------------------------ CONSTANT
slot = rig.animation_data.action_slot
n_kf = 0
curves = []
for layer in act.layers:
    for strip in layer.strips:
        cb = strip.channelbag(slot)
        if cb is None:
            continue
        for fc in cb.fcurves:
            curves.append(fc)
            for kp in fc.keyframe_points:
                kp.interpolation = 'CONSTANT'
                kp.easing = 'AUTO'
                kp.handle_left_type = kp.handle_right_type = 'VECTOR'
                n_kf += 1
            fc.update()

sc.frame_set(1)
out = os.path.join(ROOT, "goblin_v06_blocking.blend")
bpy.ops.wm.save_as_mainfile(filepath=out)
print("@@@BLOCKING_DONE@@@ action=%s fcurves=%d keys=%d -> %s"
      % (act.name, len(curves), n_kf, out))
