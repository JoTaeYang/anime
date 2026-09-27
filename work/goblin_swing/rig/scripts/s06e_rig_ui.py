"""s06e_rig_ui.py - T28: rig UI (operators + N-panel), custom shapes, colours, visibility -> gob_r04_ctrl.blend.

Input : rig/work/r04d.blend (s06a-d controls), rig/data/ctrl_manifest.json (read only), rig/scripts/addon/goblin_rig_ui.py
Output: rig/gob_r04_ctrl.blend (save copy, save_version 0)

Run:  bl.ps1 -Script s06e_rig_ui.py -Blend work/r04d.blend

- Blend texts: "goblin_manifest.json" (copy of data/ctrl_manifest.json), "goblin_rig_ui.py" (= addon/goblin_rig_ui.py,
  use_module True). The add-on file is imported and the text is executed as a module; both register the operators
  goblin.snap_ikfk / goblin.switch_space / goblin.reset_rig and the N-panel (View3D > Sidebar > Goblin).
- Custom shapes: wire meshes in the collection WGT (excluded from the view layer), use_custom_shape_bone_size False
  (absolute metres). FK / torso / root / head: circles around the bone axis sized from the GOB_mesh rest vertices in a
  slab across the bone (max radial distance + margin, so no shape sinks into the mesh); IK hand / foot: wire boxes
  fitted to the hand_x / shoe_x part in the control's frame + margin; poles: diamonds; weapon: arrow along the club
  axis with a ring outside the fist; PROPS: small square.
- Colours: bone and pose bone colour palette CUSTOM, L blue / R red / C yellow from manifest controls.<CTRL>.side
  (PROPS = C).
- Visibility: bone collections CTRL visible, MCH / DEF hidden; WGT excluded; GOB_mesh / GOB_club visible.
- Self-checks (seeded random poses): (a) IK/FK snap per chain, both starts, DEF world error; (b) space switch per
  ordered value pair, control world error, plus one requires violation; (c) reset_rig -> DEF vs canonical rest;
  (T28c) ready_pose with arms FK and IK: knee / elbow bend, foot reach, lowest shoe z, DEF scale.
"""
import importlib.util
import json
import math
import os
import random
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402

OUT_BLEND = goblib.RIG / "gob_r04_ctrl.blend"
ADDON = Path(os.path.dirname(os.path.abspath(__file__))) / "addon" / "goblin_rig_ui.py"
ARM = "GOB_rig"
MESH = "GOB_mesh"
CLUB = "GOB_club"
TEXT_MANIFEST = "goblin_manifest.json"
TEXT_ADDON = "goblin_rig_ui.py"
WGT_COLL = "WGT"
SEED = 1234
N_SNAP = 4  # random poses per chain and start mode
COLORS = {  # side -> (normal, select, active)
    "L": ((0.10, 0.35, 1.00), (0.45, 0.70, 1.00), (0.80, 0.90, 1.00)),
    "R": ((1.00, 0.15, 0.10), (1.00, 0.50, 0.40), (1.00, 0.85, 0.80)),
    "C": ((1.00, 0.85, 0.05), (1.00, 0.95, 0.45), (1.00, 1.00, 0.85)),
}
SLAB = 0.012  # m, half thickness of the vertex slab used to size a circle


# ---------------------------------------------------------------- widgets
def circle(n=32, r=1.0, y=0.0):
    v = [(r * math.cos(2 * math.pi * i / n), y, r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    return v, [(i, (i + 1) % n) for i in range(n)]


def widget_meshes():
    shapes = {}
    shapes["WGT_circle"] = circle()
    shapes["WGT_square"] = ([(-1, 0, -1), (1, 0, -1), (1, 0, 1), (-1, 0, 1)], [(0, 1), (1, 2), (2, 3), (3, 0)])
    bv = [(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    be = [(i, j) for i in range(8) for j in range(i + 1, 8) if sum(a != b for a, b in zip(bv[i], bv[j])) == 1]
    shapes["WGT_box"] = (bv, be)
    dv = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    de = [(i, j) for i in range(6) for j in range(i + 1, 6) if i // 2 != j // 2]
    shapes["WGT_diamond"] = (dv, de)
    rv, re_ = circle(16, 0.4, 0.0)
    av = rv + [(0, 0, 0), (0, 1, 0), (0.08, 0.85, 0), (-0.08, 0.85, 0), (0, 0.85, 0.08), (0, 0.85, -0.08)]
    b = len(rv)
    ae = re_ + [(b, b + 1), (b + 1, b + 2), (b + 1, b + 3), (b + 1, b + 4), (b + 1, b + 5)]
    shapes["WGT_arrow"] = (av, ae)
    return shapes


def build_widgets():
    coll = bpy.data.collections.get(WGT_COLL)
    if coll is None:
        coll = bpy.data.collections.new(WGT_COLL)
        bpy.context.scene.collection.children.link(coll)
    objs = {}
    for name, (v, e) in widget_meshes().items():
        me = bpy.data.meshes.new(name)
        me.from_pydata(v, e, [])
        me.update()
        ob = bpy.data.objects.new(name, me)
        coll.objects.link(ob)
        objs[name] = ob
    coll.hide_render = True
    lc = bpy.context.view_layer.layer_collection.children.get(WGT_COLL)
    if lc is not None:
        lc.exclude = True
    return coll, objs


# ---------------------------------------------------------------- shape sizing from the mesh
def mesh_rest(mesh_obj):
    import numpy as np
    me = mesh_obj.data
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    mw = np.array(mesh_obj.matrix_world)
    co = co.reshape(-1, 3) @ mw[:3, :3].T + mw[:3, 3]
    pid = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid)
    vpart = np.full(len(me.vertices), -1, dtype=np.int32)
    for p in me.polygons:
        for vi in p.vertices:
            vpart[vi] = pid[p.index]
    return co, vpart


def ring_radius(co, bone, s, rsearch, margin):
    import numpy as np
    h = np.array(bone.head_local)
    y = np.array((bone.tail_local - bone.head_local).normalized())
    d = co - h
    a = d @ y
    rad = np.linalg.norm(d - np.outer(a, y), axis=1)
    for slab in (SLAB, 2 * SLAB, 4 * SLAB, 8 * SLAB):  # low-poly tubes: widen the slab until it holds vertices
        m = (np.abs(a - s) < slab) & (rad < rsearch)
        if m.any():
            return float(rad[m].max()) + margin, int(m.sum()), slab
    return rsearch * 0.5, 0, None


def part_box(co, vpart, part_id, bone, margin):
    import numpy as np
    sel = co[vpart == part_id]
    mi = np.array(bone.matrix_local.inverted())
    loc = sel @ mi[:3, :3].T + mi[:3, 3]
    lo, hi = loc.min(axis=0), loc.max(axis=0)
    return (lo + hi) / 2, (hi - lo) / 2 + margin


def shape_plan(arm, man, co, vpart, parts):
    """control -> (widget, translation (bone local m), scale xyz, note)."""
    import numpy as np
    bones = arm.data.bones
    plan = {}
    shoes = np.concatenate([co[vpart == parts["shoe_l"]], co[vpart == parts["shoe_r"]]])
    r_root = float(np.linalg.norm(shoes[:, :2], axis=1).max()) + 0.08
    plan["CTRL_root"] = ("WGT_circle", (0, 0, 0), (r_root,) * 3, "ground ring: shoes max XY radius + 80 mm")
    ring = {  # control: (station as fraction of length or metres, rsearch, margin)
        "CTRL_torso": (0.20, 0.5, 0.06), "CTRL_pelvis": (0.05, 0.5, 0.02),
        "CTRL_spine_01": ("0.5L", 0.5, 0.02), "CTRL_chest": ("0.3L", 0.5, 0.02), "CTRL_head": (0.05, 0.5, 0.02),
    }
    for x in ("l", "r"):
        ring[f"CTRL_shoulder_{x}"] = ("0.8L", 0.12, 0.015)
        for b in ("upperarm", "lowerarm", "thigh", "calf"):
            ring[f"CTRL_{b}_fk_{x}"] = ("0.5L", 0.15, 0.015)
        ring[f"CTRL_hand_fk_{x}"] = ("0.5L", 0.20, 0.015)
        ring[f"CTRL_foot_fk_{x}"] = ("0.5L", 0.25, 0.015)
    for name, (st, rs, mg) in ring.items():
        b = bones[name]
        s = b.length * float(st[:-1]) if isinstance(st, str) else st
        r, n, slab = ring_radius(co, b, s, rs, mg)
        plan[name] = ("WGT_circle", (0, s, 0), (r,) * 3, f"ring at {s * 1000:.0f} mm, {n} verts within slab +-"
                                                          f"{(slab or 0) * 1000:.0f} mm, margin {mg * 1000:.0f} mm")
    for x in ("l", "r"):
        for ctrl, part in ((f"CTRL_hand_ik_{x}", f"hand_{x}"), (f"CTRL_foot_ik_{x}", f"shoe_{x}")):
            c, hsz = part_box(co, vpart, parts[part], bones[ctrl], 0.015)
            plan[ctrl] = ("WGT_box", tuple(float(v) for v in c), tuple(float(v) for v in hsz),
                          f"box around part {part} + 15 mm")
        for pole in (f"CTRL_elbow_pole_{x}", f"CTRL_knee_pole_{x}"):
            plan[pole] = ("WGT_diamond", (0, 0, 0), (0.03,) * 3, "diamond 30 mm")
    plan["CTRL_weapon"] = ("WGT_arrow", (0, 0, 0), (0.35,) * 3, "arrow 350 mm along the club axis, ring r 140 mm")
    plan["PROPS"] = ("WGT_square", (0, 0, 0), (0.05,) * 3, "square 100 mm")
    missing = [c for c in man["controls"] if c not in plan]
    if missing:
        raise RuntimeError(f"no shape for {missing}")
    return plan


def apply_shapes(arm, plan, objs):
    for name, (w, t, s, _) in plan.items():
        pb = arm.pose.bones[name]
        pb.custom_shape = objs[w]
        pb.use_custom_shape_bone_size = False
        pb.custom_shape_translation = t
        pb.custom_shape_scale_xyz = s
        pb.custom_shape_rotation_euler = (0, 0, 0)


def apply_colors(arm, man):
    sides = {n: c["side"] for n, c in man["controls"].items()}
    sides["PROPS"] = "C"
    for name, side in sides.items():
        nrm, sel, act = COLORS[side]
        for col in (arm.data.bones[name].color, arm.pose.bones[name].color):
            col.palette = "CUSTOM"
            col.custom.normal = nrm
            col.custom.select = sel
            col.custom.active = act
    return sides


def apply_visibility(arm):
    for c in arm.data.collections_all:
        c.is_visible = c.name == "CTRL"
    for n in (MESH, CLUB):
        ob = bpy.data.objects[n]
        ob.hide_viewport = False
        ob.hide_set(False)


# ---------------------------------------------------------------- self-check helpers
def update(arm):
    arm.update_tag()
    bpy.context.view_layer.update()


def set_prop(arm, name, value):
    props = arm.pose.bones["PROPS"]
    props[name] = type(props[name])(value)
    update(arm)


def wm(arm, name):
    return arm.matrix_world @ arm.pose.bones[name].matrix


def err(m1, m2):
    return (m1.translation - m2.translation).length, goblib.quat_angle_deg(m1.to_quaternion(), m2.to_quaternion())


def rand_pose(arm, man, name, rng, channels=("rot", "loc")):
    pb = arm.pose.bones[name]
    for sw in man["controls"][name]["sweep"]:
        if sw["channel"] not in channels:
            continue
        i = "XYZ".index(sw["axis"])
        v = rng.uniform(sw["min"], sw["max"])
        if sw["channel"] == "rot":
            pb.rotation_euler[i] = math.radians(v)
        else:
            pb.location[i] = v


def reset(arm):
    r = bpy.ops.goblin.reset_rig()
    update(arm)
    return r


def check_snap(arm, man):
    res = {}
    for chain, e in man["ikfk"].items():
        worst = {"fk_start_to_ik": [0, 0], "fk_start_back_to_fk": [0, 0], "ik_start_to_fk": [0, 0],
                 "ik_start_back_to_ik": [0, 0]}
        rets = set()
        for i in range(N_SNAP):
            for start in ("fk", "ik"):
                rng = random.Random(f"{SEED}-{chain}-{start}-{i}")
                reset(arm)
                if start == "fk":
                    set_prop(arm, e["prop"], e["fk_value"])
                    for c in e["fk"]:
                        rand_pose(arm, man, c, rng, ("rot",))
                else:
                    set_prop(arm, e["prop"], e["ik_value"])
                    rand_pose(arm, man, e["ik"], rng)
                    rand_pose(arm, man, e["pole"], rng)
                    for rp in e.get("roll_props", []):
                        lo, hi = man["clamps"][rp]
                        arm.pose.bones["PROPS"][rp] = rng.uniform(lo, hi)
                update(arm)
                rec = {d: wm(arm, d) for d in e["def"]}
                first, second = ("TO_IK", "TO_FK") if start == "fk" else ("TO_FK", "TO_IK")
                k1, k2 = (("fk_start_to_ik", "fk_start_back_to_fk") if start == "fk"
                          else ("ik_start_to_fk", "ik_start_back_to_ik"))
                for step, key in ((first, k1), (second, k2)):
                    rets.add(str(bpy.ops.goblin.snap_ikfk(chain=chain, direction=step)))
                    update(arm)
                    for d in e["def"]:
                        p, a = err(rec[d], wm(arm, d))
                        worst[key][0] = max(worst[key][0], p)
                        worst[key][1] = max(worst[key][1], a)
        res[chain] = (worst, rets)
    return res


def check_switch(arm, man):
    res = {}
    for prop, sp in man["spaces"].items():
        ctrl = sp["control"]
        worst, pairs, rets = [0, 0], 0, set()
        n = len(sp["values"])
        for a in range(n):
            for b in range(n):
                if a == b:
                    continue
                rng = random.Random(f"{SEED}-{prop}-{a}-{b}")
                reset(arm)
                set_prop(arm, prop, a)
                rand_pose(arm, man, ctrl, rng)
                update(arm)
                m0 = wm(arm, ctrl)
                rets.add(str(bpy.ops.goblin.switch_space(prop=prop, value=b)))
                update(arm)
                p, ang = err(m0, wm(arm, ctrl))
                now = arm.pose.bones["PROPS"][prop]
                if now != b:
                    rets.add(f"prop not switched ({now} != {b})")
                worst = [max(worst[0], p), max(worst[1], ang)]
                pairs += 1
        res[prop] = (worst, pairs, rets)
    # requires violation: weapon_space = hand_l with the left arm in IK
    reset(arm)
    set_prop(arm, "arm_ik_fk_l", 1.0)
    r = bpy.ops.goblin.switch_space(prop="weapon_space", value=1)
    viol = (str(r), arm.pose.bones["PROPS"]["weapon_space"])
    reset(arm)
    return res, viol


def def_vs_canon(arm, canon):
    update(arm)
    worst, wn = 0.0, None
    for b in canon["bones"]:
        m = arm.pose.bones[b["name"]].matrix
        r = b["rest_matrix"]
        e = max(abs(m[i][j] - r[i][j]) for i in range(4) for j in range(4))
        if e > worst:
            worst, wn = e, b["name"]
    return worst, wn


def check_reset(arm, man, canon):
    rng = random.Random(f"{SEED}-reset")
    props = arm.pose.bones["PROPS"]
    for c in man["controls"]:
        rand_pose(arm, man, c, rng)
    for k in list(props.keys()):
        ui = props.id_properties_ui(k).as_dict()
        if isinstance(props[k], int):
            props[k] = rng.randint(int(ui["min"]), int(ui["max"]))
        else:
            props[k] = rng.uniform(ui["min"], ui["max"])
    update(arm)
    before, _ = def_vs_canon(arm, canon)
    r = bpy.ops.goblin.reset_rig()
    after, wn = def_vs_canon(arm, canon)
    nonid = []
    for pb in arm.pose.bones:
        if pb.name.startswith("CTRL_"):
            if (pb.location.length > 1e-9 or Vector(pb.rotation_euler).length > 1e-9
                    or (pb.scale - Vector((1, 1, 1))).length > 1e-9):
                nonid.append(pb.name)
    nondef = [k for k in props.keys() if props[k] != props.id_properties_ui(k).as_dict().get("default")]
    return before, after, wn, nonid, nondef, str(r)


def check_ready(arm, mesh_obj, vpart, parts):
    """T28c / T28d / G6.11: goblin.ready_pose from reset, arms FK / IK x legs FK / IK. Foot metric: IK leg ->
    reach error (MCH_calf_ik tail - MCH_roll_foot head); FK leg -> DEF foot head horizontal displacement from before
    the call (and vertical, reported)."""
    import numpy as np
    pbs = arm.pose.bones

    def bend(a, b):
        return math.degrees((pbs[a].tail - pbs[a].head).angle(pbs[b].tail - pbs[b].head))

    out = {}
    modes = (("FK", 0.0), ("IK", 1.0))
    for (amode, av), (lmode, lv) in [(a, b) for a in modes for b in modes]:
        mode = f"arms {amode} / legs {lmode}"
        reset(arm)
        for x in ("l", "r"):
            set_prop(arm, f"arm_ik_fk_{x}", av)
            set_prop(arm, f"leg_ik_fk_{x}", lv)
        foot0 = {x: arm.matrix_world @ pbs[f"foot_{x}"].head for x in ("l", "r")}
        r = str(bpy.ops.goblin.ready_pose())
        update(arm)
        dg = bpy.context.evaluated_depsgraph_get()
        ev = mesh_obj.evaluated_get(dg)
        me = ev.to_mesh()
        co = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", co)
        ev.to_mesh_clear()
        mw = np.array(mesh_obj.matrix_world)
        z = co.reshape(-1, 3) @ mw[2, :3] + mw[2, 3]
        d = {"ret": r, "leg_mode": lmode, "props_after": {k: float(arm.pose.bones["PROPS"][k]) for k in (
            "arm_ik_fk_l", "arm_ik_fk_r", "leg_ik_fk_l", "leg_ik_fk_r")}}
        for x in ("l", "r"):
            d[f"knee_{x}"] = bend(f"thigh_{x}", f"calf_{x}")
            d[f"elbow_{x}"] = bend(f"upperarm_{x}", f"lowerarm_{x}")
            d[f"reach_{x}"] = (pbs[f"MCH_calf_ik_{x}"].tail - pbs[f"MCH_roll_foot_{x}"].head).length
            dv = (arm.matrix_world @ pbs[f"foot_{x}"].head) - foot0[x]
            d[f"foot_dxy_{x}"] = (dv.x ** 2 + dv.y ** 2) ** 0.5
            d[f"foot_dz_{x}"] = dv.z
            d[f"shoe_z_{x}"] = float(z[vpart == parts[f"shoe_{x}"]].min())
        sc = 0.0
        for b in arm.data.bones:
            if b.name.startswith(("CTRL_", "MCH_")) or b.name == "PROPS":
                continue
            sc = max(sc, max(abs(c - 1.0) for c in pbs[b.name].matrix.to_scale()))
        d["scale_dev"] = sc
        d["torso_loc_y"] = pbs["CTRL_torso"].location[1]
        out[mode] = d
    reset(arm)
    return out


# ---------------------------------------------------------------- main
def main():
    print(f"[s06e] Blender {bpy.app.version_string}")
    arm = bpy.data.objects[ARM]
    mesh_obj = bpy.data.objects[MESH]
    canon = goblib.load_json("canonical_skeleton.json")
    parts = goblib.load_json("parts.json")
    man_path = goblib.DATA / "ctrl_manifest.json"
    man_src = man_path.read_text(encoding="utf-8")
    man = json.loads(man_src)
    for name in (TEXT_MANIFEST, TEXT_ADDON):
        if bpy.data.texts.get(name) is not None:
            raise RuntimeError(f"text {name} already exists (expected input work/r04d.blend)")
    t = bpy.data.texts.new(TEXT_MANIFEST)
    t.write(man_src)
    addon_src = ADDON.read_text(encoding="utf-8")
    t = bpy.data.texts.new(TEXT_ADDON)
    t.write(addon_src)
    t.use_module = True
    print(f"[s06e] TEXTS {TEXT_MANIFEST} ({len(man_src)} chars, sha256 {goblib.sha256(man_path)[:12]} of "
          f"data/ctrl_manifest.json), {TEXT_ADDON} ({len(addon_src)} chars, use_module={t.use_module})")
    # add-on file import
    spec = importlib.util.spec_from_file_location("goblin_rig_ui", ADDON)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ops = [o for o in ("snap_ikfk", "switch_space", "reset_rig", "ready_pose") if hasattr(bpy.ops.goblin, o)]
    print(f"[s06e] ADDON import {ADDON.name}: ok, bl_info name '{mod.bl_info['name']}', operators registered "
          f"{ops}, panel registered {hasattr(bpy.types, 'GOBLIN_PT_rig')}")
    # blend text as module (the same source; replaces the registration)
    tmod = bpy.data.texts[TEXT_ADDON].as_module()
    ops = [o for o in ("snap_ikfk", "switch_space", "reset_rig", "ready_pose") if hasattr(bpy.ops.goblin, o)]
    print(f"[s06e] TEXT {TEXT_ADDON}.as_module(): ok ({tmod.__name__}), operators registered {ops}, panel "
          f"registered {hasattr(bpy.types, 'GOBLIN_PT_rig')}")
    # shapes / colours / visibility
    coll, objs = build_widgets()
    co, vpart = mesh_rest(mesh_obj)
    plan = shape_plan(arm, man, co, vpart, parts)
    apply_shapes(arm, plan, objs)
    sides = apply_colors(arm, man)
    apply_visibility(arm)
    for name, (w, tr, sc, note) in plan.items():
        print(f"[s06e] SHAPE {name}: {w}, side {sides[name]}, translation {tuple(round(v, 4) for v in tr)}, scale "
              f"{tuple(round(v, 4) for v in sc)} ({note})")
    print("[s06e] VISIBILITY bone collections: " + ", ".join(f"{c.name}={c.is_visible}"
                                                           for c in arm.data.collections_all)
          + f"; WGT excluded={bpy.context.view_layer.layer_collection.children[WGT_COLL].exclude}; "
          f"{MESH} visible={bpy.data.objects[MESH].visible_get()}, {CLUB} visible={bpy.data.objects[CLUB].visible_get()}")
    # self-checks
    bpy.context.view_layer.objects.active = arm
    arm.data.pose_position = "POSE"
    snap = check_snap(arm, man)
    for chain, (w, rets) in snap.items():
        print(f"[s06e] SELFCHECK (a) snap {chain} ({N_SNAP} seeded poses per start): " + ", ".join(
            f"{k} max {v[0] * 1000:.3f} mm / {v[1]:.3f} deg" for k, v in w.items()) + f"; operator returns {rets}")
    sw, viol = check_switch(arm, man)
    for prop, (w, pairs, rets) in sw.items():
        print(f"[s06e] SELFCHECK (b) switch {prop}: {pairs} ordered value pairs, control world max "
              f"{w[0] * 1000:.4f} mm / {w[1]:.4f} deg; operator returns {rets}")
    print(f"[s06e] SELFCHECK (b) requires violation (arm_ik_fk_l = 1, weapon_space -> hand_l): operator returned "
          f"{viol[0]}, weapon_space now {viol[1]} (policy: abort with warning)")
    before, after, wn, nonid, nondef, r = check_reset(arm, man, canon)
    print(f"[s06e] SELFCHECK (c) reset_rig: returned {r}; DEF vs canonical rest max abs diff before "
          f"{before:.3e}, after {after:.3e} (bone {wn}); CTRL not identity after: {nonid}; PROPS not default after: "
          f"{nondef}")
    rd = check_ready(arm, mesh_obj, vpart, parts)
    def foot_txt(d):
        if d["leg_mode"] == "IK":
            return f"foot reach l/r {d['reach_l'] * 1000:.3f}/{d['reach_r'] * 1000:.3f} mm"
        return (f"foot DEF XY displacement l/r {d['foot_dxy_l'] * 1000:.3f}/{d['foot_dxy_r'] * 1000:.3f} mm "
                f"(Z {d['foot_dz_l'] * 1000:+.3f}/{d['foot_dz_r'] * 1000:+.3f} mm)")

    print("[s06e] SELFCHECK ready_pose (target G6.11: knee 40-60 deg, elbow 10-30 deg, planted: IK leg reach <= 1 mm, "
          "FK leg foot XY displacement <= 1 mm, shoe z in [-1, +3] mm; DEF scale 1): " + "; ".join(
              f"{m}: returned {d['ret']}, props after {d['props_after']}, CTRL_torso loc Y {d['torso_loc_y']:+.3f} m, "
              f"knee l/r {d['knee_l']:.2f}/{d['knee_r']:.2f} deg, elbow l/r {d['elbow_l']:.2f}/{d['elbow_r']:.2f} "
              f"deg, {foot_txt(d)}, shoe z min l/r {d['shoe_z_l'] * 1000:+.3f}/{d['shoe_z_r'] * 1000:+.3f} mm, DEF "
              f"max |scale-1| {d['scale_dev']:.2e}"
              for m, d in rd.items()))
    reset(arm)
    g, gn = def_vs_canon(arm, canon)
    print(f"[s06e] G6.2 final state (reset_rig): DEF vs canonical rest max abs diff = {g:.3e} (bone {gn})")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s06e] OUTPUT {OUT_BLEND}")


if __name__ == "__main__":
    goblib.run_main(main)
