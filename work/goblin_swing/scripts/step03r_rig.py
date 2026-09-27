"""STEP 03R - replace the folding 2-bone leg IK with a RIGID SLIDING leg.

Run: blender -b goblin_v04_skinned.blend --python step03r_rig.py [-- --out PATH]
Writes goblin_v05_legfix.blend.  Never touches v04.

Mechanism per side S:
  MCH_LEGTOP_S   edit-parent HIPS, head at the THIGH_S rest head (the hip socket).
                 ONE constraint: Limit Distance -> FOOT_IK_S (head), distance = L,
                 LIMITDIST_OUTSIDE.  So it rides the hips, but is pushed back out to
                 exactly L from the ankle whenever the hips come closer than L.
  MCH_LEGAIM_S   edit-parent ROOT_CTRL, head = ankle (FOOT_S rest head),
                 tail = hip socket, so its +Y IS the leg axis and its length is L.
                 Constraints:  Copy Location  <- FOOT_IK_S  (head -> ankle)
                               Stretch To     -> MCH_LEGTOP_S, rest_length = L,
                                                 volume NO_VOLUME, keep_axis SWING_Y
                               Locked Track   -> KNEE_POLE_S (lock Y, track Z),
                                                 influence driven by FOOT_IK_S["leg_roll"],
                                                 default 0
  THIGH_S        edit-parent changed HIPS -> MCH_LEGAIM_S.  No constraints.
                 inherit_scale FULL, so the Stretch-To scale (about the ANKLE, which is
                 MCH_LEGAIM_S's head) stretches the whole leg evenly along its axis.
  SHIN_S         IK constraint REMOVED.  Stays connect-parented to THIGH_S, so it keeps
                 exactly its rest orientation relative to THIGH_S: the knee never folds.
  FOOT_S         CR_footplant (Copy Rotation) replaced by Copy Transforms <- FOOT_IK_S,
                 and inherit_scale = NONE, so the shoe is exactly the control and never
                 picks up the leg stretch.

Removed: the IK_leg constraints, the ik_stretch custom property, and its 6 drivers.
Added:   leg_roll custom property on FOOT_IK_L/R (0 = automatic swing-only roll).
"""
import bpy, json, math, os, sys
import numpy as np
from mathutils import Vector

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = argv[argv.index("--out") + 1] if "--out" in argv else \
    os.path.join(ROOT, "goblin_v05_legfix.blend")
REPORT = os.path.join(ROOT, "inspect", "step03r_rig.json")

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
body = bpy.data.objects["GOB_body"]
me = body.data
PB = rig.pose.bones

DEF_BONES = [b.name for b in arm.bones if b.use_deform] + ["ROOT", "COG", "WEAPON_SOCKET"]
ALL_BONES = [b.name for b in arm.bones]
REF = {n: arm.bones[n].matrix_local.copy() for n in ALL_BONES}
N = len(me.vertices)
P0 = np.empty(N * 3); me.vertices.foreach_get("co", P0); P0 = P0.reshape(N, 3)

R = {}

# ---------------------------------------------------------------- geometry
LEG = {}
for S in ("L", "R"):
    A = REF["FOOT_" + S].to_translation()              # ankle
    H = REF["THIGH_" + S].to_translation()             # hip socket
    L = (H - A).length
    LEG[S] = {"ankle": [round(v, 6) for v in A], "hip_socket": [round(v, 6) for v in H],
              "L_m": L,
              "knee_kink_deg": round(math.degrees(
                  (REF["SHIN_" + S].to_translation() - H).angle(
                      A - REF["SHIN_" + S].to_translation())), 4)}
R["leg_geometry"] = LEG

# ================================================================ edit bones
for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
eb = arm.edit_bones

for S in ("L", "R"):
    A = Vector(REF["FOOT_" + S].to_translation())
    H = Vector(REF["THIGH_" + S].to_translation())

    top = eb.new("MCH_LEGTOP_" + S)
    top.head = H
    top.tail = H + Vector((0.0, 0.07, 0.0))
    top.use_deform = False
    top.use_connect = False
    top.parent = eb["HIPS"]

    aim = eb.new("MCH_LEGAIM_" + S)
    aim.head = A
    aim.tail = H
    aim.use_deform = False
    aim.use_connect = False
    aim.parent = eb["ROOT_CTRL"]
    # roll: +Z of the aim bone points at KNEE_POLE_S, so the (default-off) Locked
    # Track is the identity at rest.
    pole = eb["KNEE_POLE_" + S].head.copy()
    mid = (A + H) * 0.5
    zdir = pole - mid
    yv = (H - A).normalized()
    zdir = zdir - yv * zdir.dot(yv)
    aim.align_roll(zdir)

    eb["THIGH_" + S].parent = eb["MCH_LEGAIM_" + S]
    eb["THIGH_" + S].use_connect = False
    eb["FOOT_" + S].inherit_scale = 'NONE'

bpy.ops.object.mode_set(mode='OBJECT')

# bone collections
cMCH = arm.collections.get("MCH") or arm.collections.new("MCH")
for S in ("L", "R"):
    cMCH.assign(arm.bones["MCH_LEGTOP_" + S])
    cMCH.assign(arm.bones["MCH_LEGAIM_" + S])
    arm.bones["MCH_LEGTOP_" + S].color.palette = 'THEME08'
    arm.bones["MCH_LEGAIM_" + S].color.palette = 'THEME08'

# rest matrices must be unchanged for every pre-existing bone
mx = 0.0
for n in ALL_BONES:
    mx = max(mx, max(abs(arm.bones[n].matrix_local[i][j] - REF[n][i][j])
                     for i in range(4) for j in range(4)))
R["rest_matrix_local_drift"] = float(mx)

# ================================================================ drivers/props out
drv_removed = []
if rig.animation_data:
    for fc in list(rig.animation_data.drivers):
        if "ik_stretch" in fc.data_path or "IK_leg" in fc.data_path:
            drv_removed.append(fc.data_path)
            rig.animation_data.drivers.remove(fc)
R["drivers_removed"] = drv_removed

for S in ("L", "R"):
    if "ik_stretch" in PB["FOOT_IK_" + S].keys():
        del PB["FOOT_IK_" + S]["ik_stretch"]
    PB["THIGH_" + S].ik_stretch = 0.0
    PB["SHIN_" + S].ik_stretch = 0.0
    pb = PB["FOOT_IK_" + S]
    pb["leg_roll"] = 0.0
    pb.id_properties_ui("leg_roll").update(
        min=0.0, max=1.0, soft_min=0.0, soft_max=1.0,
        description="0 = leg roll is automatic (swing only); "
                    "1 = leg rolls so its +Z faces KNEE_POLE_%s" % S)

# ================================================================ constraints
def cadd(bone, ctype, name, **kw):
    c = PB[bone].constraints.new(ctype)
    c.name = name
    c.target = rig
    for k, v in kw.items():
        setattr(c, k, v)
    return c


built = {}
for S in ("L", "R"):
    L = LEG[S]["L_m"]

    # old leg IK out
    for c in list(PB["SHIN_" + S].constraints):
        if c.type == 'IK':
            PB["SHIN_" + S].constraints.remove(c)
    for c in list(PB["FOOT_" + S].constraints):
        PB["FOOT_" + S].constraints.remove(c)

    cadd("MCH_LEGTOP_" + S, 'LIMIT_DISTANCE', "LD_leg_min",
         subtarget="FOOT_IK_" + S, head_tail=0.0, distance=L,
         limit_mode='LIMITDIST_OUTSIDE', use_transform_limit=False,
         target_space='WORLD', owner_space='WORLD', influence=1.0)

    cadd("MCH_LEGAIM_" + S, 'COPY_LOCATION', "CL_ankle",
         subtarget="FOOT_IK_" + S, head_tail=0.0,
         target_space='WORLD', owner_space='WORLD', influence=1.0)
    cadd("MCH_LEGAIM_" + S, 'STRETCH_TO', "ST_leg",
         subtarget="MCH_LEGTOP_" + S, head_tail=0.0, rest_length=L,
         volume='NO_VOLUME', keep_axis='SWING_Y', bulge=1.0, influence=1.0)
    lt = cadd("MCH_LEGAIM_" + S, 'LOCKED_TRACK', "LT_roll",
              subtarget="KNEE_POLE_" + S, head_tail=0.0,
              track_axis='TRACK_Z', lock_axis='LOCK_Y', influence=0.0)

    cadd("FOOT_" + S, 'COPY_TRANSFORMS', "CT_footplant",
         subtarget="FOOT_IK_" + S, target_space='WORLD', owner_space='WORLD',
         influence=1.0)

    # leg_roll driver
    fc = lt.driver_add("influence")
    d = fc.driver
    d.type = 'SCRIPTED'
    for v in list(d.variables):
        d.variables.remove(v)
    v = d.variables.new(); v.name = "v"; v.type = 'SINGLE_PROP'
    v.targets[0].id = rig
    v.targets[0].data_path = 'pose.bones["FOOT_IK_%s"]["leg_roll"]' % S
    d.expression = "v"

    built[S] = {"L_m": round(L, 6)}

R["built"] = built
R["constraints"] = {pb.name: [{"type": c.type, "name": c.name,
                               "sub": getattr(c, "subtarget", None),
                               "infl": round(c.influence, 4)}
                              for c in pb.constraints]
                    for pb in PB if pb.constraints}
R["invalid_constraints"] = [(pb.name, c.name) for pb in PB for c in pb.constraints
                            if not c.is_valid]
R["drivers"] = [fc.data_path for fc in (rig.animation_data.drivers
                                        if rig.animation_data else [])]
R["parents"] = {n: (arm.bones[n].parent.name if arm.bones[n].parent else None)
                for n in ("THIGH_L", "SHIN_L", "FOOT_L", "MCH_LEGAIM_L", "MCH_LEGTOP_L")}
R["inherit_scale"] = {n: arm.bones[n].inherit_scale
                      for n in ("THIGH_L", "SHIN_L", "FOOT_L", "THIGH_R", "SHIN_R", "FOOT_R")}

# ================================================================ rest invariance
def upd():
    rig.update_tag()
    bpy.context.view_layer.update()
    return bpy.context.evaluated_depsgraph_get()


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_axis_angle = (0, 0, 1, 0)


clear_pose()
dg = upd()
ev = rig.evaluated_get(dg)
rows = []
for n in ALL_BONES:
    if n.startswith("MCH_LEG"):
        continue
    m = ev.pose.bones[n].matrix
    dp = (m.to_translation() - REF[n].to_translation()).length
    q = m.to_quaternion().rotation_difference(REF[n].to_quaternion())
    rows.append((n, dp, math.degrees(abs(q.angle)),
                 (Vector(m.to_scale()) - Vector((1, 1, 1))).length))
R["rest_max_pos_m"] = max(r[1] for r in rows)
R["rest_max_rot_deg"] = max(r[2] for r in rows)
R["rest_max_scale_dev"] = max(r[3] for r in rows)
R["rest_violations"] = [(n, round(p, 9), round(d, 6), round(s, 9)) for n, p, d, s in rows
                        if p > 1e-5 or d > 0.01 or s > 1e-5]

evb = body.evaluated_get(dg)
dm = evb.to_mesh()
Q = np.empty(len(dm.vertices) * 3); dm.vertices.foreach_get("co", Q); Q = Q.reshape(-1, 3)
evb.to_mesh_clear()
dev = np.linalg.norm(Q - P0, axis=1)
R["rest_mesh_max_dev_m"] = float(dev.max())
R["rest_mesh_verts_over_1e6"] = int((dev > 1e-6).sum())

# ---- quick behaviour probe: COG down sweep, leg rigidity + stretch
probe = []
for dz in (0.0, -0.05, -0.10, -0.15, -0.20, 0.03):
    clear_pose()
    Mh = REF["COG_CTRL"].to_3x3()
    PB["COG_CTRL"].location = Mh.inverted() @ Vector((0, 0, dz))
    dg = upd()
    ev = rig.evaluated_get(dg)
    row = {"cog_dz": dz}
    for S in ("L",):
        mt = ev.pose.bones["MCH_LEGAIM_" + S].matrix
        th = ev.pose.bones["THIGH_" + S].matrix
        sh = ev.pose.bones["SHIN_" + S].matrix
        ft = ev.pose.bones["FOOT_" + S].matrix
        ank = mt.to_translation()
        hip = ev.pose.bones["MCH_LEGTOP_" + S].matrix.to_translation()
        row["ankle_err_m"] = round((ank - Vector(LEG[S]["ankle"])).length, 9)
        row["foot_err_m"] = round((ft.to_translation() - Vector(LEG[S]["ankle"])).length, 9)
        row["d_hip_ankle_m"] = round((hip - ank).length, 6)
        row["stretch"] = round(mt.to_scale()[1], 6)
        # knee angle between thigh and shin direction
        kt = th.to_translation()
        kn = sh.to_translation()
        an = sh @ Vector((0, arm.bones["SHIN_" + S].length, 0))
        row["knee_deg"] = round(math.degrees((kn - kt).angle(an - kn)), 4)
        row["legtop_z"] = round(kt.z, 5)
    probe.append(row)
R["cog_probe"] = probe

clear_pose()
upd()
R["residual_posed_bones"] = [pb.name for pb in PB
                             if pb.location.length > 1e-9 or
                             (Vector(pb.scale) - Vector((1, 1, 1))).length > 1e-9]
R["has_action"] = bool(rig.animation_data and rig.animation_data.action)
R["n_actions"] = len(bpy.data.actions)

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(REPORT, "w") as f:
    json.dump(R, f, indent=1, default=str)

bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
