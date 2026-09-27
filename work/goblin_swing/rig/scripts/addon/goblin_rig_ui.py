"""goblin_rig_ui.py - Goblin rig UI: IK/FK snap, space switch (world matrix kept), reset, N-panel (T28).

The same source is embedded in the rig blend as the text "goblin_rig_ui.py" (use_module = True, needs Auto Run)
and can be installed / imported as an add-on. Everything is read from the blend text "goblin_manifest.json" (a copy of
rig/data/ctrl_manifest.json); no external file is needed.

Operators
  goblin.snap_ikfk(chain: str, direction: 'TO_FK' | 'TO_IK')
      TO_FK: FK controls <- MCH IK result (manifest ikfk.<chain>.fk_to_mch_ik, chain order), prop = fk_value.
      TO_IK: legs first set roll_props to 0; IK control <- FK hand/foot world matrix (legs @ ik_ctrl_in_def_foot_rest);
             pole <- elbow/knee + pole_distance * d, d = FK upper bone X axis (perpendicular to the root -> tip
             axis) turned about that axis by the rest angle between the upper bone X axis and the pole, then turned
             about the root -> tip axis until the MCH IK elbow / knee matches the FK one (a few depsgraph updates);
             prop = ik_value.
  goblin.switch_space(prop: str, value: int)
      record the space control's world matrix -> set PROPS[prop] -> update_tag + depsgraph update -> restore the world
      matrix. requires (manifest spaces.<prop>.requires) not met -> the switch is ABORTED with a warning (CANCELLED),
      nothing changes.
  goblin.reset_rig()
      every CTRL_* pose bone to identity, every PROPS property to its UI default (HR2: mouth_open default 0).
  goblin.ready_pose()  (T28c, spec G6.11: work start pose)
      CTRL_torso local Y (= world +Z) location = -0.015 m; legs keep their mode and their feet: IK legs bend at the
      knee with the feet planted; FK legs (T28d) are snapped IK <- FK first (IK takes the FK foot, roll props 0), solved
      in IK while the torso goes down, then snapped FK <- IK (prop back to FK). Arms keep their mode and bend the
      elbow: FK -> CTRL_lowerarm_fk_x rotation X = +15 deg (hand toward -Y); IK -> snap FK <- IK, lowerarm FK X =
      +15 deg, snap IK <- FK (prop back to IK). HR2: PROPS mouth_open (if present) is set to 0 first.
  With auto keying on, the operators key the changed bones' transform channels and the changed PROPS properties.
"""
import json
import math

import bpy
from mathutils import Matrix

bl_info = {
    "name": "Goblin Rig UI",
    "author": "goblin_swing",
    "version": (1, 0, 0),
    "blender": (5, 1, 0),
    "location": "View3D > Sidebar > Goblin",
    "description": "IK/FK snap, space switch and reset for the goblin rig (GOB_rig)",
    "category": "Rigging",
}

RIG_NAME = "GOB_rig"
MANIFEST_TEXT = "goblin_manifest.json"
PROPS_BONE = "PROPS"
MOUTH_PROP = "mouth_open"  # HR2: drives the GOB_mesh shape key mouth_open


# ---------------------------------------------------------------- helpers
def load_manifest():
    t = bpy.data.texts.get(MANIFEST_TEXT)
    if t is None:
        raise RuntimeError(f"blend text {MANIFEST_TEXT} missing")
    return json.loads(t.as_string())


def find_rig(context):
    ob = getattr(context, "active_object", None)
    if ob is not None and ob.type == "ARMATURE" and ob.pose is not None and PROPS_BONE in ob.pose.bones:
        return ob
    ob = bpy.data.objects.get(RIG_NAME)
    if ob is not None and ob.type == "ARMATURE" and ob.pose is not None and PROPS_BONE in ob.pose.bones:
        return ob
    return None


def refresh(context, rig):
    rig.update_tag()
    context.view_layer.update()


def world(rig, name):
    return rig.matrix_world @ rig.pose.bones[name].matrix


def set_world(context, rig, name, m):
    rig.pose.bones[name].matrix = rig.matrix_world.inverted() @ m
    refresh(context, rig)


def autokey_on(context):
    ts = context.scene.tool_settings
    return bool(getattr(ts, "use_keyframe_insert_auto", False))


def key_bone(context, rig, name):
    if not autokey_on(context):
        return
    pb = rig.pose.bones[name]
    pb.keyframe_insert("location", group=name)
    if pb.rotation_mode == "QUATERNION":
        pb.keyframe_insert("rotation_quaternion", group=name)
    elif pb.rotation_mode == "AXIS_ANGLE":
        pb.keyframe_insert("rotation_axis_angle", group=name)
    else:
        pb.keyframe_insert("rotation_euler", group=name)
    pb.keyframe_insert("scale", group=name)


def set_prop(context, rig, name, value):
    props = rig.pose.bones[PROPS_BONE]
    props[name] = type(props[name])(value)
    refresh(context, rig)
    if autokey_on(context):
        props.keyframe_insert(f'["{name}"]', group=PROPS_BONE)


def prop_default(rig, name):
    return rig.pose.bones[PROPS_BONE].id_properties_ui(name).as_dict().get("default")


def requires_problems(rig, man, prop, value):
    """List of unmet manifest requires conditions for setting spaces[prop] to value index."""
    sp = man["spaces"][prop]
    vname = sp["values"][value]
    req = (sp.get("requires") or {}).get(vname)
    if not req:
        return []
    props = rig.pose.bones[PROPS_BONE]
    bad = []
    for k, v in req.items():
        if k == "not":
            for k2, v2 in v.items():
                if props[k2] == v2:
                    bad.append(f"{prop}={vname} needs {k2} != {v2} (now {props[k2]})")
        elif props[k] != v:
            bad.append(f"{prop}={vname} needs {k} = {v} (now {props[k]})")
    return bad


SNAP_POLE_ITER = 8  # pole correction steps in IK <- FK
SNAP_POLE_TOL = 1e-5  # rad


def chain_twist(rig, mw, a, root, fk_pair, ik_pair):
    """Signed angle (rad) about axis a from the FK chain to the IK chain: elbow / knee offsets from the root -> tip
    axis when bent, else the upper bones' X axes."""
    def perp(v):
        return v - a * v.dot(a)

    fo = perp(mw @ fk_pair[1].head - root)
    io = perp(mw @ ik_pair[1].head - root)
    if fo.length < 5e-4 or io.length < 5e-4:
        fo = perp((mw.to_3x3() @ fk_pair[0].matrix.to_3x3()).col[0])
        io = perp((mw.to_3x3() @ ik_pair[0].matrix.to_3x3()).col[0])
    return math.atan2(a.dot(fo.cross(io)), fo.dot(io))


# ---------------------------------------------------------------- core actions
def snap_ikfk(context, rig, man, chain, direction):
    e = man["ikfk"][chain]
    changed = []
    if direction == "TO_FK":
        for fk, mch in e["fk_to_mch_ik"].items():
            set_world(context, rig, fk, world(rig, mch))
            changed.append(fk)
        for n in changed:
            key_bone(context, rig, n)
        set_prop(context, rig, e["prop"], e["fk_value"])
        return changed
    # TO_IK
    for rp in e.get("roll_props", []):
        set_prop(context, rig, rp, 0.0)
    fk = e["fk"]
    target = world(rig, fk[2])
    if "ik_ctrl_in_def_foot_rest" in e:
        target = target @ Matrix(e["ik_ctrl_in_def_foot_rest"])
    set_world(context, rig, e["ik"], target)
    changed.append(e["ik"])
    mw = rig.matrix_world
    pbs = rig.pose.bones
    root = mw @ pbs[fk[0]].head
    mid = mw @ pbs[fk[1]].head
    tip = mw @ pbs[fk[2]].head
    # Blender's IK pole constraint fixes the chain's root bone X axis about the root -> target axis relative to the
    # pole (plus pole_angle). Keep the rest angle between the root bone X axis and the pole direction (both projected
    # perpendicular to the root -> tip axis) and apply it to the FK upper bone's current X axis.
    bones = rig.data.bones
    r0, t0 = bones[fk[0]].head_local, bones[fk[2]].head_local
    a0 = (t0 - r0).normalized()
    x0 = bones[fk[0]].matrix_local.to_3x3().col[0]
    x0 = x0 - a0 * x0.dot(a0)
    p0 = bones[e["pole"]].head_local - r0
    p0 = p0 - a0 * p0.dot(a0)
    phi = math.atan2(a0.dot(x0.cross(p0)), x0.dot(p0))
    a = (tip - root).normalized()
    x = (mw.to_3x3() @ pbs[fk[0]].matrix.to_3x3()).col[0]
    x = (x - a * x.dot(a)).normalized()
    d = (x * math.cos(phi) + a.cross(x) * math.sin(phi)).normalized()
    pm = world(rig, e["pole"])
    pm.translation = mid + d * e["pole_distance"]
    set_world(context, rig, e["pole"], pm)
    # the solver's pole convention is not exactly "root X axis": measure the remaining rotation of the IK chain about
    # the root -> tip axis against the FK chain and turn the pole by it (converges in a few steps)
    mch = e["mch_ik"]
    for _ in range(SNAP_POLE_ITER):
        delta = chain_twist(rig, mw, a, root, (pbs[fk[0]], pbs[fk[1]]), (pbs[mch[0]], pbs[mch[1]]))
        if abs(delta) < SNAP_POLE_TOL:
            break
        v = world(rig, e["pole"]).translation - root
        v = v * math.cos(-delta) + a.cross(v) * math.sin(-delta) + a * a.dot(v) * (1.0 - math.cos(-delta))
        pm = world(rig, e["pole"])
        pm.translation = root + v
        set_world(context, rig, e["pole"], pm)
    changed.append(e["pole"])
    for n in changed:
        key_bone(context, rig, n)
    set_prop(context, rig, e["prop"], e["ik_value"])
    return changed


def switch_space(context, rig, man, prop, value):
    sp = man["spaces"][prop]
    ctrl = sp["control"]
    m = world(rig, ctrl)
    set_prop(context, rig, prop, int(value))
    set_world(context, rig, ctrl, m)
    key_bone(context, rig, ctrl)
    return ctrl


def reset_rig(context, rig, man):
    names = [pb.name for pb in rig.pose.bones if pb.name.startswith("CTRL_")]
    for n in names:
        pb = rig.pose.bones[n]
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        pb.rotation_euler = (0.0, 0.0, 0.0)
        pb.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
        pb.scale = (1.0, 1.0, 1.0)
    props = rig.pose.bones[PROPS_BONE]
    for k in list(props.keys()):
        d = prop_default(rig, k)
        if d is not None and isinstance(props[k], (int, float)):
            props[k] = type(props[k])(d)
    refresh(context, rig)
    if autokey_on(context):
        for n in names:
            key_bone(context, rig, n)
        for k in list(props.keys()):
            if isinstance(props[k], (int, float)):
                props.keyframe_insert(f'["{k}"]', group=PROPS_BONE)
    return names


READY_TORSO_Y = -0.015  # m, CTRL_torso local Y (= world +Z)
READY_ELBOW_X_DEG = 15.0  # CTRL_lowerarm_fk_x rotation X (elbow flexion, hand toward -Y)


def ready_pose(context, rig, man):
    pbs = rig.pose.bones
    props = pbs[PROPS_BONE]
    if MOUTH_PROP in props.keys():  # HR2: the ready pose has the mouth closed
        set_prop(context, rig, MOUTH_PROP, 0.0)
    fk_legs = [c for c, e in man.get("ikfk", {}).items()
               if c.startswith("leg_") and float(props[e["prop"]]) < 0.5]
    for chain in fk_legs:  # FK leg: temporarily IK with the IK foot on the FK foot
        snap_ikfk(context, rig, man, chain, "TO_IK")
    torso = pbs["CTRL_torso"]
    torso.location[1] = READY_TORSO_Y
    refresh(context, rig)
    key_bone(context, rig, "CTRL_torso")
    for chain in fk_legs:  # FK controls follow the IK solution, prop back to FK
        snap_ikfk(context, rig, man, chain, "TO_FK")
    for chain, e in man.get("ikfk", {}).items():
        if not chain.startswith("arm_"):
            continue
        lower = e["fk"][1]
        ik_mode = float(props[e["prop"]]) >= 0.5
        if ik_mode:
            snap_ikfk(context, rig, man, chain, "TO_FK")
        pbs[lower].rotation_euler[0] = math.radians(READY_ELBOW_X_DEG)
        refresh(context, rig)
        key_bone(context, rig, lower)
        if ik_mode:
            snap_ikfk(context, rig, man, chain, "TO_IK")


# ---------------------------------------------------------------- operators
class GOBLIN_OT_snap_ikfk(bpy.types.Operator):
    """Snap IK <-> FK for one chain keeping the DEF result"""
    bl_idname = "goblin.snap_ikfk"
    bl_label = "Goblin Snap IK/FK"
    bl_options = {"REGISTER", "UNDO"}

    chain: bpy.props.StringProperty(name="Chain")
    direction: bpy.props.EnumProperty(name="Direction", items=[("TO_FK", "FK <- IK", "FK controls match the IK "
                                                                "result"), ("TO_IK", "IK <- FK", "IK controls match "
                                                                "the FK result")])

    @classmethod
    def poll(cls, context):
        return find_rig(context) is not None

    def execute(self, context):
        rig = find_rig(context)
        man = load_manifest()
        if self.chain not in man.get("ikfk", {}):
            self.report({"ERROR"}, f"unknown chain {self.chain}")
            return {"CANCELLED"}
        snap_ikfk(context, rig, man, self.chain, self.direction)
        return {"FINISHED"}


class GOBLIN_OT_switch_space(bpy.types.Operator):
    """Switch a control's space keeping its world transform"""
    bl_idname = "goblin.switch_space"
    bl_label = "Goblin Switch Space"
    bl_options = {"REGISTER", "UNDO"}

    prop: bpy.props.StringProperty(name="Property")
    value: bpy.props.IntProperty(name="Value", min=0)

    @classmethod
    def poll(cls, context):
        return find_rig(context) is not None

    def execute(self, context):
        rig = find_rig(context)
        man = load_manifest()
        sp = man.get("spaces", {}).get(self.prop)
        if sp is None or not 0 <= self.value < len(sp["values"]):
            self.report({"ERROR"}, f"unknown space {self.prop}={self.value}")
            return {"CANCELLED"}
        bad = requires_problems(rig, man, self.prop, self.value)
        if bad:
            self.report({"WARNING"}, "switch aborted: " + "; ".join(bad))
            return {"CANCELLED"}
        switch_space(context, rig, man, self.prop, self.value)
        return {"FINISHED"}


class GOBLIN_OT_reset_rig(bpy.types.Operator):
    """Reset every CTRL to identity and every PROPS property to its default"""
    bl_idname = "goblin.reset_rig"
    bl_label = "Goblin Reset Rig"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return find_rig(context) is not None

    def execute(self, context):
        reset_rig(context, find_rig(context), load_manifest())
        return {"FINISHED"}


# ---------------------------------------------------------------- panel
class GOBLIN_OT_ready_pose(bpy.types.Operator):
    """Work start pose: torso down 15 mm (IK knees bend), elbows bent 15 deg in the current arm mode"""
    bl_idname = "goblin.ready_pose"
    bl_label = "Goblin Ready Pose"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return find_rig(context) is not None

    def execute(self, context):
        ready_pose(context, find_rig(context), load_manifest())
        return {"FINISHED"}


class GOBLIN_PT_rig(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Goblin"
    bl_label = "Goblin Rig"

    @classmethod
    def poll(cls, context):
        return find_rig(context) is not None and bpy.data.texts.get(MANIFEST_TEXT) is not None

    def draw(self, context):
        layout = self.layout
        rig = find_rig(context)
        man = load_manifest()
        props = rig.pose.bones[PROPS_BONE]
        layout.operator("goblin.ready_pose", text="Ready pose", icon="ARMATURE_DATA")
        box = layout.box()
        box.label(text="IK / FK (0 = FK, 1 = IK)")
        for chain, e in man.get("ikfk", {}).items():
            col = box.column(align=True)
            col.prop(props, f'["{e["prop"]}"]', text=chain, slider=True)
            row = col.row(align=True)
            op = row.operator("goblin.snap_ikfk", text="FK <- IK")
            op.chain, op.direction = chain, "TO_FK"
            op = row.operator("goblin.snap_ikfk", text="IK <- FK")
            op.chain, op.direction = chain, "TO_IK"
        box = layout.box()
        box.label(text="Spaces (switch keeps the world transform)")
        for prop, sp in man.get("spaces", {}).items():
            cur = int(props[prop])
            col = box.column(align=True)
            col.label(text=f"{prop}: {sp['values'][cur] if 0 <= cur < len(sp['values']) else cur}")
            row = col.row(align=True)
            for k, v in enumerate(sp["values"]):
                op = row.operator("goblin.switch_space", text=v, depress=(k == cur))
                op.prop, op.value = prop, k
            for vname, req in (sp.get("requires") or {}).items():
                bad = requires_problems(rig, man, prop, sp["values"].index(vname))
                r = col.row()
                r.alert = bool(bad)
                r.label(text=f"{vname} requires {json.dumps(req)}" + (" (NOT MET)" if bad else ""), icon="INFO")
        clamps = man.get("clamps", {})
        if clamps:
            box = layout.box()
            box.label(text="Foot roll (deg)")
            for p in clamps:
                box.prop(props, f'["{p}"]', slider=True)
        if MOUTH_PROP in props.keys():  # HR2
            box = layout.box()
            box.label(text="Face")
            box.prop(props, f'["{MOUTH_PROP}"]', text="Mouth open", slider=True)
        layout.operator("goblin.reset_rig", text="Reset rig", icon="LOOP_BACK")


CLASSES = (GOBLIN_OT_snap_ikfk, GOBLIN_OT_switch_space, GOBLIN_OT_reset_rig, GOBLIN_OT_ready_pose, GOBLIN_PT_rig)


def register():
    """Idempotent: a class already registered under the same name (other copy of this module) is replaced."""
    for cls in CLASSES:
        old = getattr(bpy.types, cls.__name__, None)
        if old is not None:
            try:
                bpy.utils.unregister_class(old)
            except RuntimeError:
                pass
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(CLASSES):
        old = getattr(bpy.types, cls.__name__, None)
        if old is not None:
            try:
                bpy.utils.unregister_class(old)
            except RuntimeError:
                pass


# blend text (use_module) and direct import register at load; the add-on system's own register() call is harmless
register()
