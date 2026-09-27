"""STEP 02B (new T-pose goblin): animation controls.
blender -b gobT_v02a_def.blend --python step02b_build_controls.py -- --out PATH
"""
import bpy, json, math, os, sys
from mathutils import Vector, Matrix

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = argv[argv.index("--out") + 1] if "--out" in argv else os.path.join(T, "gobT_v02_rig.blend")
REPORT = os.path.join(T, "inspect", "step02b_build.json")

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
sc = bpy.context.scene
PB = rig.pose.bones
club = bpy.data.objects["GOB_club"]
CM0 = club.matrix_world.copy()

DEF_BONES = [b.name for b in arm.bones]
REF = {n: arm.bones[n].matrix_local.copy() for n in DEF_BONES}
rep = {"step02_bones": DEF_BONES}


def upd():
    rig.update_tag()
    bpy.context.view_layer.update()
    return rig.evaluated_get(bpy.context.evaluated_depsgraph_get())


def mat_dev(a, b):
    return ((a.to_translation() - b.to_translation()).length,
            math.degrees(abs(a.to_quaternion().rotation_difference(b.to_quaternion()).angle)))


def signed_angle(v1, v2, axis):
    v1 = v1 - axis * v1.dot(axis)
    v2 = v2 - axis * v2.dot(axis)
    if v1.length < 1e-9 or v2.length < 1e-9:
        return 0.0
    v1.normalize(); v2.normalize()
    return math.atan2(v1.cross(v2).dot(axis), v1.dot(v2))


# ==================================================================== edit bones
for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
eb = arm.edit_bones
H = {n: eb[n].head.copy() for n in DEF_BONES}
TL = {n: eb[n].tail.copy() for n in DEF_BONES}
WX = Vector((1, 0, 0))


def set_local_x(b, xdir):
    y = (b.tail - b.head).normalized()
    x = Vector(xdir); x = x - y * x.dot(y)
    if x.length < 1e-6:
        raise RuntimeError("degenerate roll ref " + b.name)
    x.normalize()
    b.align_roll(x.cross(y))


def mk(name, head, tail, parent, roll_from=None, roll_x=None):
    b = eb.new(name)
    b.head = Vector(head); b.tail = Vector(tail)
    b.use_deform = False; b.use_connect = False
    if parent:
        b.parent = eb[parent]
    if roll_from is not None:
        b.roll = eb[roll_from].roll
    else:
        set_local_x(b, roll_x if roll_x is not None else WX)
    return b


mk("ROOT_CTRL", H["ROOT"], TL["ROOT"], None, roll_from="ROOT")
mk("COG_CTRL", H["COG"], TL["COG"], "ROOT_CTRL", roll_from="COG")
mk("HIPS_CTRL", H["HIPS"], TL["HIPS"], "COG_CTRL", roll_from="HIPS")
mk("CHEST_CTRL", H["SPINE_01"], TL["CHEST"], "HIPS_CTRL", roll_x=WX)
mk("MCH_TORSO_FOLLOW", H["CHEST"], TL["CHEST"], "ROOT_CTRL", roll_from="CHEST")
mk("HEAD_CTRL", H["NECK"], TL["HEAD"], "MCH_TORSO_FOLLOW", roll_x=WX)

for S in ("L", "R"):
    mk("UPPERARM_FK_" + S, H["UPPERARM_" + S], TL["UPPERARM_" + S],
       "MCH_TORSO_FOLLOW", roll_from="UPPERARM_" + S)
    mk("FOREARM_FK_" + S, H["FOREARM_" + S], TL["FOREARM_" + S],
       "UPPERARM_FK_" + S, roll_from="FOREARM_" + S)
    mk("HAND_FK_" + S, H["HAND_" + S], TL["HAND_" + S],
       "FOREARM_FK_" + S, roll_from="HAND_" + S)
    mk("HAND_IK_" + S, H["HAND_" + S], TL["HAND_" + S], "COG_CTRL", roll_from="HAND_" + S)
    sh, el, wr = H["UPPERARM_" + S], H["FOREARM_" + S], H["HAND_" + S]
    out = (el - (sh + wr) * 0.5)
    if out.length < 1e-6:
        out = Vector((0, 1, 0))
    out.normalize()
    ph = el + out * 0.35
    mk("ELBOW_POLE_" + S, ph, ph + out * 0.09, "COG_CTRL", roll_x=WX)

    mk("FOOT_IK_" + S, H["FOOT_" + S], TL["FOOT_" + S], "ROOT_CTRL", roll_from="FOOT_" + S)
    ph = H["SHIN_" + S] + Vector((0, -0.35, 0))
    mk("KNEE_POLE_" + S, ph, ph + Vector((0, -0.09, 0)), "ROOT_CTRL", roll_x=WX)

mk("WEAPON_CTRL", H["WEAPON"], TL["WEAPON"], "WEAPON_SOCKET", roll_from="WEAPON")

# --- rigid sliding leg machinery
LEG = {}
for S in ("L", "R"):
    A = H["FOOT_" + S].copy()          # ankle
    HP = H["THIGH_" + S].copy()        # hip socket
    L = (HP - A).length
    LEG[S] = L
    top = eb.new("MCH_LEGTOP_" + S)
    top.head = HP; top.tail = HP + Vector((0.0, 0.07, 0.0))
    top.use_deform = False; top.use_connect = False; top.parent = eb["HIPS"]
    aim = eb.new("MCH_LEGAIM_" + S)
    aim.head = A; aim.tail = HP
    aim.use_deform = False; aim.use_connect = False; aim.parent = eb["ROOT_CTRL"]
    pole = eb["KNEE_POLE_" + S].head.copy()
    mid = (A + HP) * 0.5
    yv = (HP - A).normalized()
    zd = pole - mid
    zd = zd - yv * zd.dot(yv)
    aim.align_roll(zd)
    eb["THIGH_" + S].parent = eb["MCH_LEGAIM_" + S]
    eb["THIGH_" + S].use_connect = False
    eb["FOOT_" + S].inherit_scale = 'NONE'

CTRL_BONES = ["ROOT_CTRL", "COG_CTRL", "HIPS_CTRL", "CHEST_CTRL", "HEAD_CTRL",
              "UPPERARM_FK_L", "FOREARM_FK_L", "HAND_FK_L",
              "UPPERARM_FK_R", "FOREARM_FK_R", "HAND_FK_R",
              "HAND_IK_L", "HAND_IK_R", "ELBOW_POLE_L", "ELBOW_POLE_R",
              "FOOT_IK_L", "FOOT_IK_R", "KNEE_POLE_L", "KNEE_POLE_R", "WEAPON_CTRL"]
MCH_BONES = ["MCH_TORSO_FOLLOW", "MCH_LEGTOP_L", "MCH_LEGTOP_R",
             "MCH_LEGAIM_L", "MCH_LEGAIM_R"]
bpy.ops.object.mode_set(mode='OBJECT')

drift = max(max(abs(arm.bones[n].matrix_local[i][j] - REF[n][i][j])
                for i in range(4) for j in range(4)) for n in DEF_BONES)
rep["rest_matrix_local_drift"] = float(drift)
rep["leg_L_m"] = {S: round(LEG[S], 6) for S in LEG}

# ==================================================================== collections
for c in list(arm.collections):
    arm.collections.remove(c)
cDEF = arm.collections.new("DEF"); cCTRL = arm.collections.new("CTRL")
cMCH = arm.collections.new("MCH")
for n in DEF_BONES:
    cDEF.assign(arm.bones[n])
for n in CTRL_BONES:
    cCTRL.assign(arm.bones[n])
for n in MCH_BONES:
    cMCH.assign(arm.bones[n])
cDEF.is_visible = False; cMCH.is_visible = False; cCTRL.is_visible = True

# ==================================================================== widgets
WCOL = bpy.data.collections.new("GOB_rig_widgets")
sc.collection.children.link(WCOL)
WCOL.hide_viewport = True; WCOL.hide_render = True


def _obj(name, v, e, f=None):
    me = bpy.data.meshes.new("WGT_" + name)
    me.from_pydata(v, e, f or [])
    me.update()
    ob = bpy.data.objects.new("WGT_" + name, me)
    WCOL.objects.link(ob)
    return ob


def w_ring(name, n=16, r=1.0, axis='Y'):
    v, e = [], []
    for i in range(n):
        a = 2 * math.pi * i / n
        c, s = r * math.cos(a), r * math.sin(a)
        v.append((c, 0.0, s) if axis == 'Y' else (c, s, 0.0))
        e.append((i, (i + 1) % n))
    return _obj(name, v, e)


def w_cube(name, r=1.0):
    v = [(x * r, y * r, z * r) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    e = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6), (3, 7),
         (4, 5), (4, 6), (5, 7), (6, 7)]
    return _obj(name, v, e)


def w_diamond(name, r=1.0):
    v = [(0, r, 0), (0, -r, 0), (r, 0, 0), (-r, 0, 0), (0, 0, r), (0, 0, -r)]
    e = [(0, 2), (0, 3), (0, 4), (0, 5), (1, 2), (1, 3), (1, 4), (1, 5),
         (2, 4), (4, 3), (3, 5), (5, 2)]
    return _obj(name, v, e)


def w_bar(name):
    r = 0.12
    v = [(-r, 0, -r), (r, 0, -r), (r, 0, r), (-r, 0, r),
         (-r, 1, -r), (r, 1, -r), (r, 1, r), (-r, 1, r)]
    e = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]
    return _obj(name, v, e)


W = {"ring": w_ring("ring"), "ring_flat": w_ring("ring_flat", axis='Z'),
     "cube": w_cube("cube"), "diamond": w_diamond("diamond"), "bar": w_bar("bar")}
SHAPE = {
    "ROOT_CTRL": ("ring_flat", 1.3, 'THEME09'), "COG_CTRL": ("ring_flat", 2.2, 'THEME04'),
    "HIPS_CTRL": ("ring_flat", 1.8, 'THEME03'), "CHEST_CTRL": ("ring_flat", 0.7, 'THEME03'),
    "HEAD_CTRL": ("ring_flat", 0.55, 'THEME11'),
    "FOOT_IK_L": ("cube", 0.5, 'THEME04'), "FOOT_IK_R": ("cube", 0.5, 'THEME01'),
    "HAND_IK_L": ("cube", 0.6, 'THEME04'), "HAND_IK_R": ("cube", 0.6, 'THEME01'),
    "KNEE_POLE_L": ("diamond", 1.0, 'THEME04'), "KNEE_POLE_R": ("diamond", 1.0, 'THEME01'),
    "ELBOW_POLE_L": ("diamond", 1.0, 'THEME04'), "ELBOW_POLE_R": ("diamond", 1.0, 'THEME01'),
    "UPPERARM_FK_L": ("ring", 0.45, 'THEME02'), "FOREARM_FK_L": ("ring", 0.40, 'THEME02'),
    "HAND_FK_L": ("ring", 0.45, 'THEME02'),
    "UPPERARM_FK_R": ("ring", 0.45, 'THEME06'), "FOREARM_FK_R": ("ring", 0.40, 'THEME06'),
    "HAND_FK_R": ("ring", 0.45, 'THEME06'), "WEAPON_CTRL": ("bar", 1.0, 'THEME07'),
}
for n, (shp, scl, col) in SHAPE.items():
    pb = PB[n]
    pb.custom_shape = W[shp]
    pb.custom_shape_scale_xyz = (scl, scl, scl)
    pb.color.palette = col
    arm.bones[n].color.palette = col
    arm.bones[n].show_wire = True
for n in MCH_BONES:
    arm.bones[n].color.palette = 'THEME08'

# ==================================================================== rotation modes
ROT = {}
for n in ("ROOT_CTRL", "COG_CTRL", "HIPS_CTRL", "CHEST_CTRL", "HEAD_CTRL", "WEAPON_CTRL",
          "FOOT_IK_L", "FOOT_IK_R", "HAND_IK_L", "HAND_IK_R",
          "KNEE_POLE_L", "KNEE_POLE_R", "ELBOW_POLE_L", "ELBOW_POLE_R"):
    ROT[n] = 'XYZ'
for S in ("L", "R"):
    ROT["UPPERARM_FK_" + S] = 'YXZ'    # middle axis = local X (fore/aft, small range)
    ROT["FOREARM_FK_" + S] = 'XZY'     # middle axis = local Z (~0); X = hinge, Y = pronation
    ROT["HAND_FK_" + S] = 'XYZ'
for n, m in ROT.items():
    PB[n].rotation_mode = m
for n in CTRL_BONES:
    PB[n].lock_scale = (True, True, True)
for n in ("KNEE_POLE_L", "KNEE_POLE_R", "ELBOW_POLE_L", "ELBOW_POLE_R"):
    PB[n].lock_rotation = (True, True, True)
    PB[n].lock_rotation_w = True
rep["rotation_modes"] = ROT

# ==================================================================== custom props
def add_prop(bone, key, default, mn, mx, desc):
    pb = PB[bone]
    pb[key] = default
    pb.id_properties_ui(key).update(min=mn, max=mx, soft_min=mn, soft_max=mx, description=desc)


for S in ("L", "R"):
    add_prop("HAND_IK_" + S, "ik_fk", 0.0, 0.0, 1.0,
             "1 = arm follows HAND_IK_%s, 0 = arm follows the FK chain" % S)
    add_prop("FOOT_IK_" + S, "leg_roll", 0.0, 0.0, 1.0,
             "0 = leg roll automatic (swing only); 1 = leg +Z faces KNEE_POLE_%s" % S)

# ==================================================================== constraints
def cadd(bone, ctype, name, **kw):
    c = PB[bone].constraints.new(ctype)
    c.name = name
    c.target = rig
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def copy_xf(b, sub, name="CT"):
    return cadd(b, 'COPY_TRANSFORMS', name, subtarget=sub,
                target_space='WORLD', owner_space='WORLD')


def crot_local(b, sub, infl, name="CR"):
    return cadd(b, 'COPY_ROTATION', name, subtarget=sub,
                target_space='LOCAL_OWNER_ORIENT', owner_space='LOCAL',
                mix_mode='REPLACE', influence=infl)


def crot_world(b, sub, infl, name="CR_W"):
    return cadd(b, 'COPY_ROTATION', name, subtarget=sub,
                target_space='WORLD', owner_space='WORLD',
                mix_mode='REPLACE', influence=infl)


SPINE_SHARE, NECK_SHARE = 0.45, 0.40
copy_xf("ROOT", "ROOT_CTRL")
copy_xf("COG", "COG_CTRL")
copy_xf("HIPS", "HIPS_CTRL")
crot_local("SPINE_01", "CHEST_CTRL", SPINE_SHARE, "CR_chest_share")
crot_local("CHEST", "CHEST_CTRL", 1.0 - SPINE_SHARE, "CR_chest_share")
copy_xf("MCH_TORSO_FOLLOW", "CHEST", "CT_chest")
crot_local("NECK", "HEAD_CTRL", NECK_SHARE, "CR_head_share")
crot_local("HEAD", "HEAD_CTRL", 1.0 - NECK_SHARE, "CR_head_share")
copy_xf("WEAPON", "WEAPON_CTRL", "CT_weapon")

ik_cons, hand_ik_cons, lt_cons = {}, {}, {}
for S in ("L", "R"):
    crot_local("UPPERARM_" + S, "UPPERARM_FK_" + S, 1.0, "CR_fk")
    crot_local("FOREARM_" + S, "FOREARM_FK_" + S, 1.0, "CR_fk")
    crot_local("HAND_" + S, "HAND_FK_" + S, 1.0, "CR_fk")
    ik_cons[S] = cadd("FOREARM_" + S, 'IK', "IK_arm", subtarget="HAND_IK_" + S,
                      pole_target=rig, pole_subtarget="ELBOW_POLE_" + S,
                      chain_count=2, use_tail=True, use_stretch=False,
                      influence=0.0, iterations=500)
    hand_ik_cons[S] = crot_world("HAND_" + S, "HAND_IK_" + S, 0.0, "CR_ik")
    PB["UPPERARM_" + S].ik_stretch = 0.0
    PB["FOREARM_" + S].ik_stretch = 0.0

    L = LEG[S]
    cadd("MCH_LEGTOP_" + S, 'LIMIT_DISTANCE', "LD_leg_min", subtarget="FOOT_IK_" + S,
         head_tail=0.0, distance=L, limit_mode='LIMITDIST_OUTSIDE',
         use_transform_limit=False, target_space='WORLD', owner_space='WORLD', influence=1.0)
    cadd("MCH_LEGAIM_" + S, 'COPY_LOCATION', "CL_ankle", subtarget="FOOT_IK_" + S,
         head_tail=0.0, target_space='WORLD', owner_space='WORLD', influence=1.0)
    cadd("MCH_LEGAIM_" + S, 'STRETCH_TO', "ST_leg", subtarget="MCH_LEGTOP_" + S,
         head_tail=0.0, rest_length=L, volume='NO_VOLUME', keep_axis='SWING_Y',
         bulge=1.0, influence=1.0)
    lt_cons[S] = cadd("MCH_LEGAIM_" + S, 'LOCKED_TRACK', "LT_roll",
                      subtarget="KNEE_POLE_" + S, head_tail=0.0,
                      track_axis='TRACK_Z', lock_axis='LOCK_Y', influence=0.0)
    cadd("FOOT_" + S, 'COPY_TRANSFORMS', "CT_footplant", subtarget="FOOT_IK_" + S,
         target_space='WORLD', owner_space='WORLD', influence=1.0)

# ==================================================================== pole angles
def solve_pole(ik_con, root_bone, joint_bone, target_pt):
    root = REF[root_bone].to_translation()
    rest_joint = REF[joint_bone].to_translation()
    axis = (Vector(target_pt) - root).normalized()

    def err(a):
        ik_con.pole_angle = a
        ev = upd()
        j = ev.pose.bones[joint_bone].matrix.to_translation()
        return signed_angle(j - root, rest_joint - root, axis)

    def wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    e0 = err(0.0); e1 = err(0.1)
    s = (e1 - e0) / 0.1
    if abs(s) < 1e-6:
        raise RuntimeError("degenerate pole solve " + joint_bone)
    a = wrap(-e0 / s)
    best = (abs(err(a)), a)
    hist = []
    for _ in range(12):
        e = err(a)
        hist.append(round(math.degrees(e), 6))
        if abs(e) < best[0]:
            best = (abs(e), a)
        if abs(e) < 1e-7:
            break
        a = wrap(a - e / s)
    a = best[1]
    return a, math.degrees(err(a)), hist


poles = {}
for S in ("L", "R"):
    ik_cons[S].influence = 1.0
    a, r, h = solve_pole(ik_cons[S], "UPPERARM_" + S, "FOREARM_" + S,
                         REF["HAND_" + S].to_translation())
    poles["arm_" + S] = {"pole_angle_deg": round(math.degrees(a), 4),
                         "residual_deg": round(r, 6), "iter_err_deg": h}
    ik_cons[S].influence = 0.0
rep["pole_angles"] = poles

# ==================================================================== drivers
def drive(owner, path, prop_bone, prop, expr="v"):
    fc = owner.driver_add(path)
    d = fc.driver
    d.type = 'SCRIPTED'
    for v in list(d.variables):
        d.variables.remove(v)
    v = d.variables.new(); v.name = "v"; v.type = 'SINGLE_PROP'
    v.targets[0].id = rig
    v.targets[0].data_path = 'pose.bones["%s"]["%s"]' % (prop_bone, prop)
    d.expression = expr
    return fc


for S in ("L", "R"):
    drive(ik_cons[S], "influence", "HAND_IK_" + S, "ik_fk")
    drive(hand_ik_cons[S], "influence", "HAND_IK_" + S, "ik_fk")
    drive(lt_cons[S], "influence", "FOOT_IK_" + S, "leg_roll")

# ==================================================================== visibility
for nm, hid in (("GOB_body", True), ("GOB_body_lo", False)):
    o = bpy.data.objects[nm]
    o.hide_render = hid
    o.hide_viewport = hid
    for c in o.users_collection:
        c.hide_render = hid
        c.hide_viewport = hid

# ==================================================================== checks
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


def check_rest(tag):
    ev = upd()
    rows = []
    for n in DEF_BONES:
        if n.startswith("MCH_LEG"):
            continue
        dp, dr = mat_dev(ev.pose.bones[n].matrix, REF[n])
        ds = (Vector(ev.pose.bones[n].matrix.to_scale()) - Vector((1, 1, 1))).length
        rows.append((n, dp, dr, ds))
    return {"tag": tag, "max_pos_m": max(r[1] for r in rows),
            "max_rot_deg": max(r[2] for r in rows), "max_scale_dev": max(r[3] for r in rows),
            "violations": [(n, round(p, 9), round(d, 6), round(s, 9)) for n, p, d, s in rows
                           if p > 1e-5 or d > 0.01 or s > 1e-5]}


inv = []
clear_pose(); inv.append(check_rest("ik_fk=0"))
for S in ("L", "R"):
    PB["HAND_IK_" + S]["ik_fk"] = 1.0
inv.append(check_rest("ik_fk=1"))
clear_pose(); inv.append(check_rest("ik_fk=0 restored"))
rep["rest_invariance"] = inv

ev = upd()
WB = rig.matrix_world @ ev.pose.bones["WEAPON"].matrix @ arm.bones["WEAPON"].matrix_local.inverted()
rep["club_world_dev_at_rest"] = float(max(abs(club.matrix_world[i][j] - CM0[i][j])
                                          for i in range(4) for j in range(4)))
rep["club_rest_offset"] = [[round(v, 8) for v in row] for row in
                           ((rig.matrix_world @ ev.pose.bones["WEAPON"].matrix).inverted() @ CM0)]
rep["invalid_constraints"] = [(pb.name, c.name) for pb in PB for c in pb.constraints
                              if not c.is_valid]
rep["constraints"] = {pb.name: [{"type": c.type, "name": c.name,
                                 "sub": getattr(c, "subtarget", None),
                                 "infl": round(c.influence, 4)} for c in pb.constraints]
                      for pb in PB if pb.constraints}
rep["parents"] = {n: (arm.bones[n].parent.name if arm.bones[n].parent else None)
                  for n in arm.bones.keys()}
rep["residual_posed_bones"] = [pb.name for pb in PB if pb.location.length > 1e-9 or
                               (Vector(pb.scale) - Vector((1, 1, 1))).length > 1e-9]
rep["has_action"] = bool(rig.animation_data and rig.animation_data.action)
rep["n_actions"] = len(bpy.data.actions)
rep["n_drivers"] = len(rig.animation_data.drivers) if rig.animation_data else 0
rep["ctrl_bones"] = CTRL_BONES
rep["mch_bones"] = MCH_BONES
rep["n_bones_total"] = len(arm.bones)
rep["spine_share"] = SPINE_SHARE
rep["neck_share"] = NECK_SHARE

print("@@@JSON_START@@@")
print(json.dumps(rep, indent=1, default=str))
print("@@@JSON_END@@@")
with open(REPORT, "w") as f:
    json.dump(rep, f, indent=1, default=str)
bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
