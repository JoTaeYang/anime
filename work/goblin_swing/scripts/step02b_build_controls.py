"""STEP 02B - add animation controls to the goblin deform rig.

Run:  blender -b goblin_v02_armature.blend --python step02b_build_controls.py
Writes goblin_v03_controls.blend.  Never writes to the v02 file.
"""
import bpy, json, math
from mathutils import Vector, Matrix

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\goblin_v03_controls.blend"
REPORT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step02b_build.json"

rig = bpy.data.objects["GOB_rig"]
arm = rig.data
sc = bpy.context.scene

DEF_BONES = [b.name for b in arm.bones]            # the 21 v02 bones
REF = {n: arm.bones[n].matrix_local.copy() for n in DEF_BONES}   # v02 world matrices

rep = {"def_bones": DEF_BONES}

# ============================================================ helpers
def set_local_x(b, xdir):
    y = (b.tail - b.head).normalized()
    x = Vector(xdir)
    x = x - y * x.dot(y)
    if x.length < 1e-6:
        raise RuntimeError("degenerate roll ref for " + b.name)
    x.normalize()
    b.align_roll(x.cross(y))


def upd():
    rig.update_tag()
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    return rig.evaluated_get(dg)


def mat_dev(a, b):
    """(metres, degrees) between two 4x4 world matrices."""
    dp = (a.to_translation() - b.to_translation()).length
    q = a.to_quaternion().rotation_difference(b.to_quaternion())
    return dp, math.degrees(abs(q.angle))


def signed_angle(v1, v2, axis):
    v1 = (v1 - axis * v1.dot(axis))
    v2 = (v2 - axis * v2.dot(axis))
    if v1.length < 1e-9 or v2.length < 1e-9:
        return 0.0
    v1.normalize(); v2.normalize()
    return math.atan2(v1.cross(v2).dot(axis), v1.dot(v2))


# ============================================================ edit bones
for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
eb = arm.edit_bones

H = {n: eb[n].head.copy() for n in DEF_BONES}
T = {n: eb[n].tail.copy() for n in DEF_BONES}
WX = Vector((1, 0, 0))


def mk(name, head, tail, parent, roll_from=None, roll_x=None):
    b = eb.new(name)
    b.head = Vector(head)
    b.tail = Vector(tail)
    b.use_deform = False
    b.use_connect = False
    if parent:
        b.parent = eb[parent]
    if roll_from is not None:
        b.roll = eb[roll_from].roll
    else:
        set_local_x(b, roll_x if roll_x is not None else WX)
    return b


# --- torso / root -------------------------------------------------------
mk("ROOT_CTRL",  H["ROOT"], T["ROOT"], None, roll_from="ROOT")
mk("COG_CTRL",   H["COG"],  T["COG"],  "ROOT_CTRL", roll_from="COG")
mk("HIPS_CTRL",  H["HIPS"], T["HIPS"], "COG_CTRL", roll_from="HIPS")
mk("CHEST_CTRL", H["SPINE_01"], T["CHEST"], "HIPS_CTRL", roll_x=WX)
# MCH bone that mirrors DEF CHEST in world space -> parent for head + FK arms
mk("MCH_TORSO_FOLLOW", H["CHEST"], T["CHEST"], "ROOT_CTRL", roll_from="CHEST")
mk("HEAD_CTRL",  H["NECK"], T["HEAD"], "MCH_TORSO_FOLLOW", roll_x=WX)

# --- arms ---------------------------------------------------------------
for S in ("L", "R"):
    mk("UPPERARM_FK_" + S, H["UPPERARM_" + S], T["UPPERARM_" + S],
       "MCH_TORSO_FOLLOW", roll_from="UPPERARM_" + S)
    mk("FOREARM_FK_" + S, H["FOREARM_" + S], T["FOREARM_" + S],
       "UPPERARM_FK_" + S, roll_from="FOREARM_" + S)
    mk("HAND_FK_" + S, H["HAND_" + S], T["HAND_" + S],
       "FOREARM_FK_" + S, roll_from="HAND_" + S)
    mk("HAND_IK_" + S, H["HAND_" + S], T["HAND_" + S],
       "COG_CTRL", roll_from="HAND_" + S)
    # elbow pole: out along (elbow - midpoint(shoulder,wrist))
    sh = H["UPPERARM_" + S]; el = H["FOREARM_" + S]; wr = H["HAND_" + S]
    out = (el - (sh + wr) * 0.5)
    out.normalize()
    ph = el + out * 0.35
    mk("ELBOW_POLE_" + S, ph, ph + out * 0.09, "COG_CTRL", roll_x=WX)

# --- legs ---------------------------------------------------------------
for S in ("L", "R"):
    mk("FOOT_IK_" + S, H["FOOT_" + S], T["FOOT_" + S], "ROOT_CTRL",
       roll_from="FOOT_" + S)
    kn = H["SHIN_" + S]
    ph = kn + Vector((0, -0.35, 0))
    mk("KNEE_POLE_" + S, ph, ph + Vector((0, -0.09, 0)), "ROOT_CTRL", roll_x=WX)

# --- weapon -------------------------------------------------------------
mk("WEAPON_CTRL", H["WEAPON"], T["WEAPON"], "WEAPON_SOCKET", roll_from="WEAPON")

CTRL_BONES = ["ROOT_CTRL", "COG_CTRL", "HIPS_CTRL", "CHEST_CTRL", "HEAD_CTRL",
              "UPPERARM_FK_L", "FOREARM_FK_L", "HAND_FK_L",
              "UPPERARM_FK_R", "FOREARM_FK_R", "HAND_FK_R",
              "HAND_IK_L", "HAND_IK_R", "ELBOW_POLE_L", "ELBOW_POLE_R",
              "FOOT_IK_L", "FOOT_IK_R", "KNEE_POLE_L", "KNEE_POLE_R",
              "WEAPON_CTRL"]
MCH_BONES = ["MCH_TORSO_FOLLOW"]

bpy.ops.object.mode_set(mode='OBJECT')

# ============================================================ bone collections
for c in list(arm.collections):
    arm.collections.remove(c)
cDEF = arm.collections.new("DEF")
cCTRL = arm.collections.new("CTRL")
cMCH = arm.collections.new("MCH")
for n in DEF_BONES:
    cDEF.assign(arm.bones[n])
for n in CTRL_BONES:
    cCTRL.assign(arm.bones[n])
for n in MCH_BONES:
    cMCH.assign(arm.bones[n])
cDEF.is_visible = False
cMCH.is_visible = False
cCTRL.is_visible = True

# ============================================================ widgets
WCOL = bpy.data.collections.new("GOB_rig_widgets")
sc.collection.children.link(WCOL)
WCOL.hide_viewport = True
WCOL.hide_render = True


def _obj(name, verts, edges, faces=None):
    me = bpy.data.meshes.new("WGT_" + name)
    me.from_pydata(verts, edges, faces or [])
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
    """a 4-sided prism along +Y, 0..1"""
    r = 0.12
    v = [(-r, 0, -r), (r, 0, -r), (r, 0, r), (-r, 0, r),
         (-r, 1, -r), (r, 1, -r), (r, 1, r), (-r, 1, r)]
    e = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]
    return _obj(name, v, e)


W = {"ring": w_ring("ring"), "ring_flat": w_ring("ring_flat", axis='Z'),
     "cube": w_cube("cube"), "diamond": w_diamond("diamond"), "bar": w_bar("bar")}

PB = rig.pose.bones
SHAPE = {
    "ROOT_CTRL":  ("ring_flat", 1.3, 'THEME09'),
    "COG_CTRL":   ("ring_flat", 2.2, 'THEME04'),
    "HIPS_CTRL":  ("ring_flat", 1.8, 'THEME03'),
    "CHEST_CTRL": ("ring_flat", 0.7, 'THEME03'),
    "HEAD_CTRL":  ("ring_flat", 0.55, 'THEME11'),
    "FOOT_IK_L":  ("cube", 0.5, 'THEME04'),
    "FOOT_IK_R":  ("cube", 0.5, 'THEME01'),
    "HAND_IK_L":  ("cube", 0.6, 'THEME04'),
    "HAND_IK_R":  ("cube", 0.6, 'THEME01'),
    "KNEE_POLE_L": ("diamond", 1.0, 'THEME04'),
    "KNEE_POLE_R": ("diamond", 1.0, 'THEME01'),
    "ELBOW_POLE_L": ("diamond", 1.0, 'THEME04'),
    "ELBOW_POLE_R": ("diamond", 1.0, 'THEME01'),
    "UPPERARM_FK_L": ("ring", 0.45, 'THEME02'),
    "FOREARM_FK_L": ("ring", 0.40, 'THEME02'),
    "HAND_FK_L":  ("ring", 0.45, 'THEME02'),
    "UPPERARM_FK_R": ("ring", 0.45, 'THEME06'),
    "FOREARM_FK_R": ("ring", 0.40, 'THEME06'),
    "HAND_FK_R":  ("ring", 0.45, 'THEME06'),
    "WEAPON_CTRL": ("bar", 1.0, 'THEME07'),
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

# rotation modes
for n in ("ROOT_CTRL", "COG_CTRL", "HIPS_CTRL", "CHEST_CTRL", "HEAD_CTRL",
          "WEAPON_CTRL", "FOOT_IK_L", "FOOT_IK_R", "HAND_IK_L", "HAND_IK_R",
          "KNEE_POLE_L", "KNEE_POLE_R", "ELBOW_POLE_L", "ELBOW_POLE_R"):
    PB[n].rotation_mode = 'XYZ'
for S in ("L", "R"):
    for p in ("UPPERARM_FK_", "FOREARM_FK_", "HAND_FK_"):
        PB[p + S].rotation_mode = 'QUATERNION'
# locks: poles never need rotation, nothing needs scale
for n in CTRL_BONES:
    PB[n].lock_scale = (True, True, True)
for n in ("KNEE_POLE_L", "KNEE_POLE_R", "ELBOW_POLE_L", "ELBOW_POLE_R"):
    PB[n].lock_rotation = (True, True, True)
    PB[n].lock_rotation_w = True

# ============================================================ custom props
def add_prop(bone, key, default, mn, mx, desc):
    pb = PB[bone]
    pb[key] = default
    ui = pb.id_properties_ui(key)
    ui.update(min=mn, max=mx, soft_min=mn, soft_max=mx, description=desc)


for S in ("L", "R"):
    add_prop("HAND_IK_" + S, "ik_fk", 0.0, 0.0, 1.0,
             "1 = arm follows HAND_IK_%s, 0 = arm follows the FK chain" % S)
    add_prop("FOOT_IK_" + S, "ik_stretch", 0.0, 0.0, 1.0,
             "1 = leg IK may stretch past its natural length")

# ============================================================ constraints
def cadd(bone, ctype, name, **kw):
    c = PB[bone].constraints.new(ctype)
    c.name = name
    c.target = rig
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def copy_xf(bone, sub, name="CT"):
    return cadd(bone, 'COPY_TRANSFORMS', name, subtarget=sub,
                target_space='WORLD', owner_space='WORLD')


def copy_rot_local(bone, sub, infl, name="CR"):
    return cadd(bone, 'COPY_ROTATION', name, subtarget=sub,
                target_space='LOCAL_OWNER_ORIENT', owner_space='LOCAL',
                mix_mode='REPLACE', influence=infl)


def copy_rot_world(bone, sub, infl, name="CR_W"):
    return cadd(bone, 'COPY_ROTATION', name, subtarget=sub,
                target_space='WORLD', owner_space='WORLD',
                mix_mode='REPLACE', influence=infl)


SPINE_SHARE = 0.45
NECK_SHARE = 0.40

copy_xf("ROOT", "ROOT_CTRL")
copy_xf("COG", "COG_CTRL")
copy_xf("HIPS", "HIPS_CTRL")
copy_rot_local("SPINE_01", "CHEST_CTRL", SPINE_SHARE, "CR_chest_share")
copy_rot_local("CHEST", "CHEST_CTRL", 1.0 - SPINE_SHARE, "CR_chest_share")
copy_xf("MCH_TORSO_FOLLOW", "CHEST", "CT_chest")
copy_rot_local("NECK", "HEAD_CTRL", NECK_SHARE, "CR_head_share")
copy_rot_local("HEAD", "HEAD_CTRL", 1.0 - NECK_SHARE, "CR_head_share")
copy_xf("WEAPON", "WEAPON_CTRL", "CT_weapon")

ik_cons = {}
hand_ik_cons = {}
for S in ("L", "R"):
    copy_rot_local("UPPERARM_" + S, "UPPERARM_FK_" + S, 1.0, "CR_fk")
    copy_rot_local("FOREARM_" + S, "FOREARM_FK_" + S, 1.0, "CR_fk")
    copy_rot_local("HAND_" + S, "HAND_FK_" + S, 1.0, "CR_fk")
    ik = cadd("FOREARM_" + S, 'IK', "IK_arm", subtarget="HAND_IK_" + S,
              pole_target=rig, pole_subtarget="ELBOW_POLE_" + S,
              chain_count=2, use_tail=True, use_stretch=False, influence=0.0,
              iterations=500)
    ik_cons[S] = ik
    hand_ik_cons[S] = copy_rot_world("HAND_" + S, "HAND_IK_" + S, 0.0, "CR_ik")
    # legs
    lik = cadd("SHIN_" + S, 'IK', "IK_leg", subtarget="FOOT_IK_" + S,
               pole_target=rig, pole_subtarget="KNEE_POLE_" + S,
               chain_count=2, use_tail=True, use_stretch=False, influence=1.0,
               iterations=500)
    ik_cons["leg" + S] = lik
    copy_rot_world("FOOT_" + S, "FOOT_IK_" + S, 1.0, "CR_footplant")
    PB["THIGH_" + S].ik_stretch = 0.0
    PB["SHIN_" + S].ik_stretch = 0.0

# ============================================================ pole angles (solved)
def solve_pole(ik_con, root_bone, joint_bone, target_pt):
    """Linear solve: measured joint azimuth error is pole_angle + const."""
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

    e0 = err(0.0)
    e1 = err(0.1)
    s = (e1 - e0) / 0.1
    if abs(s) < 1e-6:
        raise RuntimeError("degenerate pole solve for " + joint_bone)
    a = wrap(-e0 / s)
    hist = []
    best = (abs(err(a)), a)
    for _ in range(12):
        e = err(a)
        hist.append(round(math.degrees(e), 6))
        if abs(e) < abs(best[0]):
            best = (abs(e), a)
        if abs(e) < 1e-7:
            break
        a = wrap(a - e / s)
    a = best[1]
    resid = err(a)
    return a, math.degrees(resid), hist


poles = {}
for S in ("L", "R"):
    a, r, h = solve_pole(ik_cons["leg" + S], "THIGH_" + S, "SHIN_" + S,
                         REF["FOOT_" + S].to_translation())
    poles["leg_" + S] = {"pole_angle_deg": round(math.degrees(a), 4),
                         "residual_deg": round(r, 6), "iter_err_deg": h}
# arms: solve with ik influence temporarily at 1
for S in ("L", "R"):
    ik_cons[S].influence = 1.0
    a, r, h = solve_pole(ik_cons[S], "UPPERARM_" + S, "FOREARM_" + S,
                         REF["HAND_" + S].to_translation())
    poles["arm_" + S] = {"pole_angle_deg": round(math.degrees(a), 4),
                         "residual_deg": round(r, 6), "iter_err_deg": h}
    ik_cons[S].influence = 0.0
rep["pole_angles"] = poles

# ============================================================ drivers
def drive(owner, path, prop_bone, prop, expr="v", index=-1):
    fc = owner.driver_add(path) if index < 0 else owner.driver_add(path, index)
    d = fc.driver
    d.type = 'SCRIPTED'
    for v in list(d.variables):
        d.variables.remove(v)
    v = d.variables.new()
    v.name = "v"
    v.type = 'SINGLE_PROP'
    v.targets[0].id = rig
    v.targets[0].data_path = 'pose.bones["%s"]["%s"]' % (prop_bone, prop)
    d.expression = expr
    return fc


for S in ("L", "R"):
    drive(ik_cons[S], "influence", "HAND_IK_" + S, "ik_fk")
    drive(hand_ik_cons[S], "influence", "HAND_IK_" + S, "ik_fk")
    drive(ik_cons["leg" + S], "use_stretch", "FOOT_IK_" + S, "ik_stretch",
          expr="v > 0.5")
    drive(PB["THIGH_" + S], "ik_stretch", "FOOT_IK_" + S, "ik_stretch",
          expr="0.1*v")
    drive(PB["SHIN_" + S], "ik_stretch", "FOOT_IK_" + S, "ik_stretch",
          expr="0.1*v")

rep["has_action"] = bool(rig.animation_data and rig.animation_data.action)
rep["n_drivers"] = len(rig.animation_data.drivers) if rig.animation_data else 0

# ============================================================ rest invariance
def check_rest(tag):
    ev = upd()
    worst = ("", 0.0, 0.0)
    rows = []
    for n in DEF_BONES:
        m = ev.pose.bones[n].matrix
        dp, dr = mat_dev(m, REF[n])
        rows.append((n, dp, dr))
        if dp > worst[1] or dr > worst[2]:
            worst = (n, max(dp, worst[1]), max(dr, worst[2]))
    mx_p = max(r[1] for r in rows)
    mx_r = max(r[2] for r in rows)
    bad = [(n, round(p, 8), round(d, 6)) for n, p, d in rows
           if p > 1e-4 or d > 0.01]
    return {"tag": tag, "max_pos_m": mx_p, "max_rot_deg": mx_r,
            "worst_bone": max(rows, key=lambda r: r[1] + r[2] * 1e-3)[0],
            "violations": bad}


inv = []
for S in ("L", "R"):
    PB["HAND_IK_" + S]["ik_fk"] = 0.0
inv.append(check_rest("ik_fk=0"))
for S in ("L", "R"):
    PB["HAND_IK_" + S]["ik_fk"] = 1.0
inv.append(check_rest("ik_fk=1"))
for S in ("L", "R"):
    PB["HAND_IK_" + S]["ik_fk"] = 0.0
inv.append(check_rest("ik_fk=0 (restored)"))
rep["rest_invariance"] = inv

# constraint validity
bad_c = []
for pb in PB:
    for c in pb.constraints:
        if not c.is_valid:
            bad_c.append((pb.name, c.name))
rep["invalid_constraints"] = bad_c

# leg geometry numbers
leg = {}
for S in ("L", "R"):
    lt = arm.bones["THIGH_" + S].length
    ls = arm.bones["SHIN_" + S].length
    d = (REF["FOOT_" + S].to_translation() - REF["THIGH_" + S].to_translation()).length
    leg[S] = {"thigh": round(lt, 5), "shin": round(ls, 5), "sum": round(lt + ls, 5),
              "rest_hip_ankle": round(d, 5),
              "max_reach": round(lt + ls, 5), "min_reach": round(abs(lt - ls), 5),
              "rest_slack_m": round(lt + ls - d, 5),
              "geom_max_crouch_m": round(d - abs(lt - ls), 5)}
rep["leg_geometry"] = leg

# pose must be clean
res = []
for pb in PB:
    if (pb.location.length > 1e-9 or
            (pb.rotation_mode == 'QUATERNION' and
             (pb.rotation_quaternion - Matrix.Identity(4).to_quaternion()).magnitude > 1e-9) or
            (pb.rotation_mode != 'QUATERNION' and Vector(pb.rotation_euler).length > 1e-9) or
            (Vector(pb.scale) - Vector((1, 1, 1))).length > 1e-9):
        res.append(pb.name)
rep["residual_posed_bones"] = res

rep["ctrl_bones"] = CTRL_BONES
rep["mch_bones"] = MCH_BONES
rep["spine_share"] = SPINE_SHARE
rep["neck_share"] = NECK_SHARE
rep["n_bones_total"] = len(arm.bones)

print("@@@JSON_START@@@")
print(json.dumps(rep, indent=1, default=str))
print("@@@JSON_END@@@")
with open(REPORT, "w") as f:
    json.dump(rep, f, indent=1, default=str)

bpy.ops.wm.save_as_mainfile(filepath=OUT)
print("SAVED", OUT)
