"""p24e_rig_ui.py - T240 (P2.5): rig UI (operators + N-panel), custom shapes, colours, visibility -> pl_r04_ctrl.blend.

Input : work/player/rig/work/pl_r04d.blend (p24a-d), work/player/rig/data/ctrl_manifest.json (read only),
        work/player/rig/scripts/addon/player_rig_ui.py
Output: work/player/rig/pl_r04_ctrl.blend (save copy), renders work/player/inspect/P2c/*.png

CLI:  blender --background --factory-startup work/player/rig/work/pl_r04d.blend --python p24e_rig_ui.py

Copied / adapted from work/goblin_swing/rig/scripts/s06e_rig_ui.py (T28; goblin files unchanged):
- Blend texts "player_manifest.json" (copy of data/ctrl_manifest.json) and "player_rig_ui.py" (= addon/player_rig_ui.py,
  use_module True). The add-on file is imported and the text is executed as a module; both register player.snap_ikfk,
  player.switch_space, player.reset_rig, player.ready_pose and the N-panel (View3D > Sidebar > Player).
- Custom shapes: wire meshes in the collection WGT (excluded from the view layer), absolute size: FK / torso / root /
  head rings sized from the PL_mesh rest vertices in a slab across the bone (max radial distance + margin); IK hand /
  foot: boxes around the fist_x / shoe_x part; poles: diamonds; weapons: arrows; PROPS: square.
- Colours: CUSTOM palette, L blue / R red / C yellow (manifest controls.<CTRL>.side).
- Visibility: bone collection CTRL visible, MCH / DEF hidden; WGT excluded; PL_mesh visible.
- Renders (Workbench; control shapes drawn as bevelled curves because bones do not render): control overview front /
  side at rest; IK/FK match poses (arm_l, leg_l: seeded FK pose -> render -> player.snap_ikfk TO_IK -> render, DEF
  world difference printed); ready pose front / side.
"""
import importlib.util
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Euler, Matrix, Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
IN_BLEND = RIG / "work" / "pl_r04d.blend"
OUT_BLEND = RIG / "pl_r04_ctrl.blend"
ADDON = HERE / "addon" / "player_rig_ui.py"
MANIFEST = RIG / "data" / "ctrl_manifest.json"
CANON = RIG / "data" / "canonical_skeleton.json"
PARTS_JSON = RIG / "data" / "parts.json"
INSPECT = REPO / "work" / "player" / "inspect" / "P2c"
ARM = "PL_rig"
MESH = "PL_mesh"
TEXT_MANIFEST = "player_manifest.json"
TEXT_ADDON = "player_rig_ui.py"
WGT_COLL = "WGT"
COLORS = {"L": ((0.10, 0.35, 1.00), (0.45, 0.70, 1.00), (0.80, 0.90, 1.00)),
          "R": ((1.00, 0.15, 0.10), (1.00, 0.50, 0.40), (1.00, 0.85, 0.80)),
          "C": ((1.00, 0.85, 0.05), (1.00, 0.95, 0.45), (1.00, 1.00, 0.85))}
SLAB = 0.012
OPS = ("snap_ikfk", "switch_space", "reset_rig", "ready_pose")


def log(msg):
    print(f"[p24e] {msg}")
    sys.stdout.flush()


# ---------------------------------------------------------------- widgets (s06e)
def circle(n=32, r=1.0, y=0.0):
    v = [(r * math.cos(2 * math.pi * i / n), y, r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    return v, [(i, (i + 1) % n) for i in range(n)]


def widget_meshes():
    shapes = {"WGT_circle": circle(),
              "WGT_square": ([(-1, 0, -1), (1, 0, -1), (1, 0, 1), (-1, 0, 1)], [(0, 1), (1, 2), (2, 3), (3, 0)])}
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
    return objs


def mesh_rest(mesh_obj):
    me = mesh_obj.data
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    pid = np.empty(len(me.polygons), dtype=np.int32)
    me.attributes["part_id"].data.foreach_get("value", pid)
    vpart = np.full(len(me.vertices), -1, dtype=np.int32)
    for p in me.polygons:
        vpart[list(p.vertices)] = pid[p.index]
    return co, vpart


def ring_radius(co, bone, s, rsearch, margin):
    h = np.array(bone.head_local)
    y = np.array((bone.tail_local - bone.head_local).normalized())
    d = co - h
    a = d @ y
    rad = np.linalg.norm(d - np.outer(a, y), axis=1)
    for slab in (SLAB, 2 * SLAB, 4 * SLAB, 8 * SLAB):
        m = (np.abs(a - s) < slab) & (rad < rsearch)
        if m.any():
            return float(rad[m].max()) + margin, int(m.sum()), slab
    return rsearch * 0.5, 0, None


def part_box(co, vpart, part_id, bone, margin):
    sel = co[vpart == part_id]
    mi = np.array(bone.matrix_local.inverted())
    loc = sel @ mi[:3, :3].T + mi[:3, 3]
    lo, hi = loc.min(axis=0), loc.max(axis=0)
    return (lo + hi) / 2, (hi - lo) / 2 + margin


def shape_plan(arm, man, co, vpart, parts):
    bones = arm.data.bones
    plan = {}
    shoes = np.concatenate([co[vpart == parts["shoe_l"]], co[vpart == parts["shoe_r"]]])
    r_root = float(np.linalg.norm(shoes[:, :2], axis=1).max()) + 0.08
    plan["CTRL_root"] = ("WGT_circle", (0, 0, 0), (r_root,) * 3, "ground ring: shoes max XY radius + 80 mm")
    ring = {"CTRL_torso": (0.02, 0.5, 0.05), "CTRL_pelvis": (0.03, 0.5, 0.02),
            "CTRL_spine_01": ("0.5L", 0.5, 0.02), "CTRL_chest": ("0.3L", 0.5, 0.02), "CTRL_head": ("0.5L", 0.5, 0.02)}
    for x in ("l", "r"):
        ring[f"CTRL_shoulder_{x}"] = ("0.8L", 0.12, 0.015)
        for b in ("upperarm", "lowerarm", "thigh", "calf"):
            ring[f"CTRL_{b}_fk_{x}"] = ("0.5L", 0.10, 0.015)
        ring[f"CTRL_hand_fk_{x}"] = ("0.5L", 0.12, 0.015)
        ring[f"CTRL_foot_fk_{x}"] = ("0.5L", 0.25, 0.015)
    for name, (st, rs, mg) in ring.items():
        b = bones[name]
        s = b.length * float(st[:-1]) if isinstance(st, str) else st
        r, n, slab = ring_radius(co, b, s, rs, mg)
        plan[name] = ("WGT_circle", (0, s, 0), (r,) * 3, f"ring at {s * 1000:.0f} mm, {n} verts, margin {mg * 1000:.0f} mm")
    for x in ("l", "r"):
        for ctrl, part in ((f"CTRL_hand_ik_{x}", f"fist_{x}"), (f"CTRL_foot_ik_{x}", f"shoe_{x}")):
            c, hsz = part_box(co, vpart, parts[part], bones[ctrl], 0.015)
            plan[ctrl] = ("WGT_box", tuple(float(v) for v in c), tuple(float(v) for v in hsz), f"box around {part} + 15 mm")
        for pole in (f"CTRL_elbow_pole_{x}", f"CTRL_knee_pole_{x}"):
            plan[pole] = ("WGT_diamond", (0, 0, 0), (0.03,) * 3, "diamond 30 mm")
        plan[f"CTRL_weapon_{x}"] = ("WGT_arrow", (0, 0, 0), (0.25,) * 3, "arrow 250 mm along the socket Y, ring r 100 mm")
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


# ---------------------------------------------------------------- renders
def view_basis(view):
    if view == "front":
        d, wup = Vector((0, 1, 0)), Vector((0, 0, 1))
    elif view == "side":
        d, wup = Vector((-1, 0, 0)), Vector((0, 0, 1))
    elif view == "three_quarter":
        az, el = math.radians(45.0), math.radians(15.0)
        pos = Vector((-math.cos(el) * math.sin(az), -math.cos(el) * math.cos(az), math.sin(el)))
        d, wup = -pos, Vector((0, 0, 1))
    else:
        raise ValueError(view)
    d = d.normalized()
    right = d.cross(wup).normalized()
    up = right.cross(d).normalized()
    return d, right, up


def ortho_render(objs, view, out_png, frame_bbox, res_h=1024, xray=0.0):
    out_png = Path(out_png).resolve()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    d, right, up = view_basis(view)
    mn, mx = Vector(frame_bbox[0]), Vector(frame_bbox[1])
    corners = [Vector((x, y, z)) for x in (mn.x, mx.x) for y in (mn.y, mx.y) for z in (mn.z, mx.z)]
    us = [q.dot(right) for q in corners]
    vs = [q.dot(up) for q in corners]
    ws = [q.dot(d) for q in corners]
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    w, h = max(u1 - u0, 1e-6), max(v1 - v0, 1e-6)
    depth = max(ws) - min(ws)
    center_plane = right * ((u0 + u1) / 2) + up * ((v0 + v1) / 2)
    dist = depth + max(w, h) + 1.0
    cam_loc = center_plane + d * (min(ws) - dist)
    sc = bpy.data.scenes.new("_p24e_render")
    cam_data = cam_obj = None
    try:
        for o in objs:
            sc.collection.objects.link(o)
        cam_data = bpy.data.cameras.new("_p24e_cam")
        cam_data.type = "ORTHO"
        cam_data.sensor_fit = "VERTICAL"
        cam_data.ortho_scale = h
        cam_data.clip_start = 0.001
        cam_data.clip_end = dist + depth + 10.0
        cam_obj = bpy.data.objects.new("_p24e_cam", cam_data)
        cam_obj.matrix_world = Matrix.Translation(cam_loc) @ Matrix((right, up, -d)).transposed().to_4x4()
        sc.collection.objects.link(cam_obj)
        sc.camera = cam_obj
        r = sc.render
        r.engine = "BLENDER_WORKBENCH"
        r.resolution_x = max(1, int(round(res_h * w / h)))
        r.resolution_y = res_h
        r.resolution_percentage = 100
        r.film_transparent = True
        r.use_file_extension = False
        r.image_settings.file_format = "PNG"
        r.image_settings.color_mode = "RGBA"
        r.filepath = str(out_png)
        sc.view_settings.view_transform = "Standard"
        sh = sc.display.shading
        sh.light = "STUDIO"
        sh.color_type = "MATERIAL"
        sh.show_xray = xray > 0
        if xray > 0:
            sh.xray_alpha = xray
        bpy.ops.render.render(write_still=True, scene=sc.name)
    finally:
        if cam_obj is not None:
            bpy.data.objects.remove(cam_obj, do_unlink=True)
        if cam_data is not None:
            bpy.data.cameras.remove(cam_data)
        bpy.data.scenes.remove(sc)
    return out_png


def shape_curves(arm, plan, objs, sides):
    """One curve object per side colour holding every control shape at its current pose (bevel 1.5 mm)."""
    mats = {}
    for side, (nrm, _s, _a) in COLORS.items():
        m = bpy.data.materials.new(f"_p24e_{side}")
        m.diffuse_color = (*nrm, 1.0)
        mats[side] = m
    out = []
    for side in COLORS:
        cu = bpy.data.curves.new(f"_p24e_shapes_{side}", "CURVE")
        cu.dimensions = "3D"
        cu.bevel_depth = 0.0015
        cu.materials.append(mats[side])
        for name, (w, t, s, _) in plan.items():
            if sides[name] != side:
                continue
            M = (arm.matrix_world @ arm.pose.bones[name].matrix @ Matrix.Translation(t)
                 @ Matrix.Diagonal((s[0], s[1], s[2], 1.0)))
            me = objs[w].data
            for e in me.edges:
                sp = cu.splines.new("POLY")
                sp.points.add(1)
                for k, vi in enumerate(e.vertices):
                    p = M @ me.vertices[vi].co
                    sp.points[k].co = (p.x, p.y, p.z, 1.0)
        ob = bpy.data.objects.new(f"_p24e_shapes_{side}", cu)
        out.append(ob)
    return out, mats


def drop(objs, mats=()):
    for o in objs:
        data = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if isinstance(data, bpy.types.Curve):
            bpy.data.curves.remove(data)
    for m in mats.values() if isinstance(mats, dict) else mats:
        bpy.data.materials.remove(m)


def update(arm):
    arm.update_tag()
    bpy.context.view_layer.update()


def def_world(arm, names):
    return {n: arm.matrix_world @ arm.pose.bones[n].matrix for n in names}


def world_err(a, b):
    p = max((a[n].translation - b[n].translation).length for n in a)
    r = max(math.degrees(a[n].to_quaternion().rotation_difference(b[n].to_quaternion()).angle) for n in a)
    return p, r


def main():
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    arm = bpy.data.objects[ARM]
    mesh_obj = bpy.data.objects[MESH]
    canon = json.loads(CANON.read_text(encoding="utf-8"))
    parts = json.loads(PARTS_JSON.read_text(encoding="utf-8"))["part_id"]
    man_src = MANIFEST.read_text(encoding="utf-8")
    man = json.loads(man_src)
    for name in (TEXT_MANIFEST, TEXT_ADDON):
        if bpy.data.texts.get(name) is not None:
            raise RuntimeError(f"text {name} already exists (expected input pl_r04d.blend)")
    t = bpy.data.texts.new(TEXT_MANIFEST)
    t.write(man_src)
    addon_src = ADDON.read_text(encoding="utf-8")
    t = bpy.data.texts.new(TEXT_ADDON)
    t.write(addon_src)
    t.use_module = True
    spec = importlib.util.spec_from_file_location("player_rig_ui", ADDON)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ops = [o for o in OPS if hasattr(bpy.ops.player, o)]
    log(f"ADDON import {ADDON.name}: bl_info '{mod.bl_info['name']}', operators {ops}, panel "
        f"{hasattr(bpy.types, 'PLAYER_PT_rig')}")
    tmod = bpy.data.texts[TEXT_ADDON].as_module()
    ops = [o for o in OPS if hasattr(bpy.ops.player, o)]
    log(f"TEXT {TEXT_ADDON}.as_module(): {tmod.__name__}, operators {ops}, panel {hasattr(bpy.types, 'PLAYER_PT_rig')}; "
        f"texts {sorted(t.name for t in bpy.data.texts)}")
    objs = build_widgets()
    co, vpart = mesh_rest(mesh_obj)
    plan = shape_plan(arm, man, co, vpart, parts)
    apply_shapes(arm, plan, objs)
    sides = apply_colors(arm, man)
    for c in arm.data.collections_all:
        c.is_visible = c.name == "CTRL"
    mesh_obj.hide_viewport = False
    for name, (w, tr, sc, note) in plan.items():
        log(f"SHAPE {name}: {w}, side {sides[name]}, translation {tuple(round(v, 4) for v in tr)}, scale "
            f"{tuple(round(v, 4) for v in sc)} ({note})")
    log("VISIBILITY " + ", ".join(f"{c.name}={c.is_visible}" for c in arm.data.collections_all)
        + f"; WGT excluded={bpy.context.view_layer.layer_collection.children[WGT_COLL].exclude}")
    bpy.context.view_layer.objects.active = arm
    arm.data.pose_position = "POSE"
    bpy.ops.player.reset_rig()
    update(arm)
    worst = max(max(abs(arm.pose.bones[b["name"]].matrix[i][j] - b["rest_matrix"][i][j]) for i in range(4)
                    for j in range(4)) for b in canon["bones"])
    log(f"G6.2 after player.reset_rig: DEF vs canonical rest max abs diff {worst:.3e}")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    log(f"OUTPUT {OUT_BLEND}; objects {sorted((o.name, o.type) for o in bpy.data.objects)}")

    # ---- renders (after the save; the saved file is at reset)
    out = []
    full = ((-0.55, -0.35, -0.02), (0.55, 0.35, 1.30))
    cs, mats = shape_curves(arm, plan, objs, sides)
    for view in ("front", "side"):
        out.append(ortho_render([mesh_obj] + cs, view, INSPECT / f"ctrl_overview_{view}.png", full, xray=0.45))
    drop(cs, mats)
    for chain, poses in (("arm_l", {"CTRL_upperarm_fk_l": (30, 20, 25), "CTRL_lowerarm_fk_l": (70, 0, 0),
                                    "CTRL_hand_fk_l": (20, 30, 0)}),
                         ("leg_l", {"CTRL_thigh_fk_l": (40, 10, 10), "CTRL_calf_fk_l": (-60, 0, 0),
                                    "CTRL_foot_fk_l": (15, 0, 5)})):
        e = man["ikfk"][chain]
        bpy.ops.player.reset_rig()
        arm.pose.bones["PROPS"][e["prop"]] = float(e["fk_value"])
        for c, (rx, ry, rz) in poses.items():
            arm.pose.bones[c].rotation_euler = Euler((math.radians(rx), math.radians(ry), math.radians(rz)))
        update(arm)
        before = def_world(arm, e["def"])
        box = ((-0.10, -0.40, -0.02), (0.60, 0.40, 0.85)) if chain == "arm_l" else ((-0.05, -0.45, -0.02),
                                                                                     (0.45, 0.35, 0.60))
        view = "front" if chain == "arm_l" else "side"
        out.append(ortho_render([mesh_obj], view, INSPECT / f"ikfk_{chain}_fk.png", box))
        bpy.ops.player.snap_ikfk(chain=chain, direction="TO_IK")
        update(arm)
        after = def_world(arm, e["def"])
        out.append(ortho_render([mesh_obj], view, INSPECT / f"ikfk_{chain}_ik.png", box))
        p, r = world_err(before, after)
        log(f"IKFK render pose {chain}: FK -> snap TO_IK, prop {arm.pose.bones['PROPS'][e['prop']]}; DEF world diff max "
            f"{p * 1000:.4f} mm / {r:.4f} deg")
    bpy.ops.player.reset_rig()
    bpy.ops.player.ready_pose()
    update(arm)
    cs, mats = shape_curves(arm, plan, objs, sides)
    for view in ("front", "side"):
        out.append(ortho_render([mesh_obj] + cs, view, INSPECT / f"ready_pose_{view}.png", full, xray=0.45))
    drop(cs, mats)
    bpy.ops.player.reset_rig()
    update(arm)
    log("RENDERS " + ", ".join(str(Path(q).relative_to(REPO)).replace("\\", "/") for q in out))
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
