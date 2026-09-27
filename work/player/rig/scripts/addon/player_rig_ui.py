"""player_rig_ui.py - Player rig UI: IK/FK snap, space switch (world matrix kept), reset, ready pose, N-panel (T240).

Copied / adapted from work/goblin_swing/rig/scripts/addon/goblin_rig_ui.py (T28 / T28c / T28d; goblin file unchanged):
namespace goblin.* -> player.*, rig GOB_rig -> PL_rig, manifest text goblin_manifest.json -> player_manifest.json, no
mouth property, ready pose values for the player (READY_TORSO_Y, READY_ELBOW_DEG with the manifest flexion sign).

The same source is embedded in the rig blend as the text "player_rig_ui.py" (use_module = True, needs Auto Run) and can
be installed / imported as an add-on. Everything is read from the blend text "player_manifest.json" (a copy of
work/player/rig/data/ctrl_manifest.json).

Operators (same arguments / behaviour as the goblin ones)
  player.snap_ikfk(chain: str, direction: 'TO_FK' | 'TO_IK')
      TO_FK: FK controls <- MCH IK result (manifest ikfk.<chain>.fk_to_mch_ik, chain order; legs: the DEF-equivalent
             helpers), prop = fk_value.
      TO_IK: legs first set roll_props to 0; IK control <- FK hand / foot world matrix (legs @ ik_ctrl_in_def_foot_rest);
             pole <- elbow / knee + pole_distance * d (rest angle between the upper bone X axis and the pole applied
             to the FK upper bone X axis), then turned about the root -> tip axis until the IK elbow (arms) matches
             the FK one, or (legs, T242: manifest ik_chain present) until the thigh local X axis of the DEF-equivalent
             helper (manifest mch_ik) matches the FK thigh X axis; prop = ik_value.
  player.switch_space(prop: str, value: int)
      record the space control's world matrix -> set PROPS[prop] -> update_tag + depsgraph update -> restore the world
      matrix. Unmet manifest spaces.<prop>.requires -> aborted with a warning (CANCELLED).
  player.reset_rig()
      every CTRL_* pose bone to identity, every PROPS property to its UI default.
  player.ready_pose()
      PROPS skirt_follow (T253) = its UI default (0.5). CTRL_torso local Y (= world +Z) = READY_TORSO_Y (-0.004 m; IK knees bend about 18 deg with the feet planted);
      FK legs are snapped IK <- FK first, solved in IK while the torso goes down, then snapped FK <- IK. Arms keep their
      mode and bend the elbow by READY_ELBOW_DEG (15 deg) times the manifest flexion sign on CTRL_lowerarm_fk_x
      rotation X; IK arms: snap FK <- IK, bend, snap IK <- FK.
  With auto keying on, the operators key the changed bones' transform channels and the changed PROPS properties.
"""
import json
import math

import bpy
from mathutils import Matrix

bl_info = {
    "name": "Player Rig UI",
    "author": "player",
    "version": (1, 0, 0),
    "blender": (5, 1, 0),
    "location": "View3D > Sidebar > Player",
    "description": "IK/FK snap, space switch, reset and ready pose for the player rig (PL_rig)",
    "category": "Rigging",
}

RIG_NAME = "PL_rig"
MANIFEST_TEXT = "player_manifest.json"
PROPS_BONE = "PROPS"
READY_TORSO_Y = -0.004     # m, CTRL_torso local Y (= world +Z)
READY_ELBOW_DEG = 15.0     # elbow flexion on CTRL_lowerarm_fk_x rotation X (times ikfk.<arm>.flexion_sign.sign)
FOLLOW_PROP = "skirt_follow"   # T253: skirt follows thigh (PROPS float 0..1, default 0.5)


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


SNAP_POLE_ITER = 8
SNAP_POLE_TOL = 1e-5


def chain_twist(rig, mw, a, root, fk_pair, ik_pair, use_x=False):
    """Signed angle (rad) about axis a from the FK chain to the IK chain: elbow / knee offsets from the root -> tip
    axis when bent, else (or always with use_x) the upper bones' X axes."""
    def perp(v):
        return v - a * v.dot(a)

    fo = perp(mw @ fk_pair[1].head - root)
    io = perp(mw @ ik_pair[1].head - root)
    if use_x or fo.length < 5e-4 or io.length < 5e-4:
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
    mch = e["mch_ik"]
    # T242: legs (manifest ik_chain = MCH chain with the preferred-bend offset; mch_ik = DEF-equivalent helpers) align
    # the swivel by the thigh local X axes: the DEF leg rest bend plane is 61 deg off the hinge plane, so the knee
    # offset of a nearly straight FK leg points the wrong way and turned the IK chain by ~60 deg about the leg axis
    use_x = "ik_chain" in e
    for _ in range(SNAP_POLE_ITER):
        delta = chain_twist(rig, mw, a, root, (pbs[fk[0]], pbs[fk[1]]), (pbs[mch[0]], pbs[mch[1]]), use_x)
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


def ready_pose(context, rig, man):
    pbs = rig.pose.bones
    props = pbs[PROPS_BONE]
    if FOLLOW_PROP in props.keys():
        set_prop(context, rig, FOLLOW_PROP, prop_default(rig, FOLLOW_PROP))
    fk_legs = [c for c, e in man.get("ikfk", {}).items()
               if c.startswith("leg_") and float(props[e["prop"]]) < 0.5]
    for chain in fk_legs:
        snap_ikfk(context, rig, man, chain, "TO_IK")
    torso = pbs["CTRL_torso"]
    torso.location[1] = READY_TORSO_Y
    refresh(context, rig)
    key_bone(context, rig, "CTRL_torso")
    for chain in fk_legs:
        snap_ikfk(context, rig, man, chain, "TO_FK")
    for chain, e in man.get("ikfk", {}).items():
        if not chain.startswith("arm_"):
            continue
        lower = e["fk"][1]
        sign = float((e.get("flexion_sign") or {}).get("sign", 1))
        ik_mode = float(props[e["prop"]]) >= 0.5
        if ik_mode:
            snap_ikfk(context, rig, man, chain, "TO_FK")
        pbs[lower].rotation_euler[0] = sign * math.radians(READY_ELBOW_DEG)
        refresh(context, rig)
        key_bone(context, rig, lower)
        if ik_mode:
            snap_ikfk(context, rig, man, chain, "TO_IK")


# ---------------------------------------------------------------- operators
class PLAYER_OT_snap_ikfk(bpy.types.Operator):
    """Snap IK <-> FK for one chain keeping the DEF result"""
    bl_idname = "player.snap_ikfk"
    bl_label = "Player Snap IK/FK"
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


class PLAYER_OT_switch_space(bpy.types.Operator):
    """Switch a control's space keeping its world transform"""
    bl_idname = "player.switch_space"
    bl_label = "Player Switch Space"
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


class PLAYER_OT_reset_rig(bpy.types.Operator):
    """Reset every CTRL to identity and every PROPS property to its default"""
    bl_idname = "player.reset_rig"
    bl_label = "Player Reset Rig"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return find_rig(context) is not None

    def execute(self, context):
        reset_rig(context, find_rig(context), load_manifest())
        return {"FINISHED"}


class PLAYER_OT_ready_pose(bpy.types.Operator):
    """Work start pose: torso down 4 mm (IK knees bend about 18 deg), elbows bent 15 deg in the current arm mode"""
    bl_idname = "player.ready_pose"
    bl_label = "Player Ready Pose"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return find_rig(context) is not None

    def execute(self, context):
        ready_pose(context, find_rig(context), load_manifest())
        return {"FINISHED"}


class PLAYER_PT_rig(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Player"
    bl_label = "Player Rig"

    @classmethod
    def poll(cls, context):
        return find_rig(context) is not None and bpy.data.texts.get(MANIFEST_TEXT) is not None

    def draw(self, context):
        layout = self.layout
        rig = find_rig(context)
        man = load_manifest()
        props = rig.pose.bones[PROPS_BONE]
        layout.operator("player.ready_pose", text="Ready pose", icon="ARMATURE_DATA")
        box = layout.box()
        box.label(text="IK / FK (0 = FK, 1 = IK)")
        for chain, e in man.get("ikfk", {}).items():
            col = box.column(align=True)
            col.prop(props, f'["{e["prop"]}"]', text=chain, slider=True)
            row = col.row(align=True)
            op = row.operator("player.snap_ikfk", text="FK <- IK")
            op.chain, op.direction = chain, "TO_FK"
            op = row.operator("player.snap_ikfk", text="IK <- FK")
            op.chain, op.direction = chain, "TO_IK"
        box = layout.box()
        box.label(text="Spaces (switch keeps the world transform)")
        for prop, sp in man.get("spaces", {}).items():
            cur = int(props[prop])
            col = box.column(align=True)
            col.label(text=f"{prop}: {sp['values'][cur] if 0 <= cur < len(sp['values']) else cur}")
            row = col.row(align=True)
            for k, v in enumerate(sp["values"]):
                op = row.operator("player.switch_space", text=v, depress=(k == cur))
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
        if FOLLOW_PROP in props.keys():
            box = layout.box()
            box.label(text="Skirt")
            box.prop(props, f'["{FOLLOW_PROP}"]', text="Follow thigh", slider=True)
        layout.operator("player.reset_rig", text="Reset rig", icon="LOOP_BACK")


CLASSES = (PLAYER_OT_snap_ikfk, PLAYER_OT_switch_space, PLAYER_OT_reset_rig, PLAYER_OT_ready_pose, PLAYER_PT_rig)


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


register()
