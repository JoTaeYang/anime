"""p26_export_stage.py - T250 (P2.6): export stage = DEF-only armature `Player` baked from the work rig.

    build_export(src_blend: Path, action_name: str | None, out_blend: Path) -> Path

Input : src_blend (work rig blend, work/player/rig/pl_r05_rigtest.blend; opened with open_mainfile, never saved),
        work/player/rig/data/canonical_skeleton.json (read only)
Output: out_blend (save copy) holding only `Player` (armature) and `Player_mesh` in collection EXPORT

CLI:  blender --background --factory-startup work/player/rig/pl_r05_rigtest.blend --python p26_export_stage.py --
          --action rigtest --out export/stage_rigtest.blend
      ... -- --rest --out export/stage_rest.blend          (--out relative to work/player/rig/)

Copied / adapted from work/goblin_swing/rig/scripts/s07b_export_rig.py (T32; goblin files unchanged):
- `Player` (identity object): the 41 bones created only from canonical_skeleton.json (order, head, tail, roll, parent,
  use_deform); every bone use_connect False; matrix_local checked against rest_matrix.
- Bake: every Player pose bone (QUATERNION) gets Copy Transforms WORLD/WORLD <- PL_rig DEF bone of the same name;
  per frame frame_set(f), convert_space(pose_bone, pb.matrix, POSE -> LOCAL) with the constraints active, quaternions
  made compatible with the previous frame; then the constraints are removed and location / rotation_quaternion / scale
  are keyed every frame into a new slotted action ('rigtest' or 'rest', slot 'OBPlayer'). action_name None: 1 frame
  (frame 1) with the work action unassigned and the reset_rig state (every CTRL_* identity, PROPS at UI defaults).
- Stage mesh `Player_mesh` = copy of PL_mesh (mesh data copied: 20 material slots, part_id, custom normals, UVs, vertex
  groups), its Armature modifier -> Player. Everything else (PL_rig, WGT objects, texts, other actions, collections) is
  removed and orphans purged. The player has no weapon mesh and no shape keys (goblin club / HR2 parts dropped).
- Self-check (stdout): 41 bones, rest vs canonical, constraints left, bake range, work DEF vs Player world error on
  every baked frame (max mm / deg and the frame), object / action list.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
CANON = RIG / "data" / "canonical_skeleton.json"
WORK_RIG = "PL_rig"
WORK_MESH = "PL_mesh"
EXPORT = "Player"
MESH_OUT = "Player_mesh"
COLL = "EXPORT"
PROPS = "PROPS"
REST_TOL = 1e-5


def log(msg):
    print(f"[p26] {msg}")
    sys.stdout.flush()


def mdiff(a, b):
    q = a.to_quaternion().conjugated() @ b.to_quaternion()
    ang = 2.0 * math.atan2(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z), abs(q.w))
    return (a.translation - b.translation).length * 1000.0, math.degrees(ang)


def refresh(ob):
    ob.update_tag()
    bpy.context.view_layer.update()


def prepare_work(rig, action_name):
    scene = bpy.context.scene
    ad = rig.animation_data
    if action_name is not None:
        act = bpy.data.actions.get(action_name)
        if act is None:
            raise RuntimeError(f"action {action_name!r} not in {bpy.data.filepath}")
        if ad is None:
            ad = rig.animation_data_create()
        if ad.action != act:
            ad.action = act
        if ad.action_slot is None:
            slots = [s for s in act.slots if s.target_id_type == "OBJECT"]
            if len(slots) != 1:
                raise RuntimeError(f"action {action_name}: cannot pick an OBJECT slot")
            ad.action_slot = slots[0]
        f0, f1 = (int(round(v)) for v in act.frame_range)
        return f0, f1, f"work action {act.name!r} slot {ad.action_slot.identifier!r}, frames {f0}..{f1}"
    if ad is not None and ad.action is not None:
        ad.action = None
    pbs = rig.pose.bones
    for pb in pbs:
        if pb.name.startswith("CTRL_"):
            pb.location = (0.0, 0.0, 0.0)
            pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
            pb.rotation_euler = (0.0, 0.0, 0.0)
            pb.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
            pb.scale = (1.0, 1.0, 1.0)
    props = pbs[PROPS]
    for k in list(props.keys()):
        if isinstance(props[k], (int, float)):
            d = props.id_properties_ui(k).as_dict().get("default")
            if d is not None:
                props[k] = type(props[k])(d)
    scene.frame_set(1)
    refresh(rig)
    return 1, 1, "rest: work action unassigned, every CTRL_* identity, PROPS = UI defaults, frame 1"


def build_armature(canon, coll):
    ad = bpy.data.armatures.new(EXPORT)
    ex = bpy.data.objects.new(EXPORT, ad)
    coll.objects.link(ex)
    ex.matrix_world = Matrix.Identity(4)
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    vl.objects.active = ex
    ex.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = ad.edit_bones
    for b in canon["bones"]:
        eb = ebs.new(b["name"])
        eb.head = Vector(b["head"])
        eb.tail = Vector(b["tail"])
        eb.roll = float(b["roll"])
        eb.use_deform = bool(b["use_deform"])
    for b in canon["bones"]:
        eb = ebs[b["name"]]
        if b["parent"]:
            eb.parent = ebs[b["parent"]]
        eb.use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")
    for pb in ex.pose.bones:
        pb.rotation_mode = "QUATERNION"
    worst, wn = 0.0, None
    for b in canon["bones"]:
        m = ad.bones[b["name"]].matrix_local
        r = b["rest_matrix"]
        e = max(abs(m[i][j] - r[i][j]) for i in range(4) for j in range(4))
        if e > worst:
            worst, wn = e, b["name"]
    return ex, worst, wn


def bake(ex, rig, canon, f0, f1, act_name):
    scene = bpy.context.scene
    names = [b["name"] for b in canon["bones"]]
    pbs = ex.pose.bones
    for n in names:
        c = pbs[n].constraints.new("COPY_TRANSFORMS")
        c.name = "bake_copy"
        c.target = rig
        c.subtarget = n
        c.target_space = "WORLD"
        c.owner_space = "WORLD"
    frames = list(range(f0, f1 + 1))
    data = {n: [] for n in names}
    prev = {}
    for f in frames:
        scene.frame_set(f)
        for n in names:
            pb = pbs[n]
            m = ex.convert_space(pose_bone=pb, matrix=pb.matrix, from_space="POSE", to_space="LOCAL")
            loc, q, sc = m.decompose()
            if n in prev:
                q.make_compatible(prev[n])
            prev[n] = q.copy()
            data[n].append((loc.copy(), q.copy(), sc.copy()))
    for pb in pbs:
        for c in list(pb.constraints):
            pb.constraints.remove(c)
    n_con = sum(len(pb.constraints) for pb in pbs)
    act = bpy.data.actions.new(act_name + "__bake")
    slot = act.slots.new(id_type="OBJECT", name=EXPORT)
    ad = ex.animation_data_create()
    ad.action = act
    ad.action_slot = slot
    for i, f in enumerate(frames):
        for n in names:
            pb = pbs[n]
            loc, q, sc = data[n][i]
            pb.location = loc
            pb.rotation_quaternion = q
            pb.scale = sc
            pb.keyframe_insert("location", frame=f, group=n)
            pb.keyframe_insert("rotation_quaternion", frame=f, group=n)
            pb.keyframe_insert("scale", frame=f, group=n)
    act.use_frame_range = True
    act.frame_start, act.frame_end = f0, f1
    return act, slot, n_con, frames


def copy_mesh(ex, coll):
    src = bpy.data.objects[WORK_MESH]
    ob = src.copy()
    ob.data = src.data.copy()
    ob.name = MESH_OUT
    ob.data.name = MESH_OUT
    ob.animation_data_clear()
    coll.objects.link(ob)
    arms = [m for m in ob.modifiers if m.type == "ARMATURE"]
    if len(arms) != 1 or len(ob.modifiers) != 1:
        raise RuntimeError(f"{WORK_MESH}: expected one Armature modifier, got {[m.type for m in ob.modifiers]}")
    arms[0].object = ex
    if ob.parent is not None:
        raise RuntimeError(f"{WORK_MESH} has a parent {ob.parent.name}")
    return ob


def cleanup(keep_objs, coll, act):
    for ob in list(bpy.data.objects):
        if ob not in keep_objs:
            bpy.data.objects.remove(ob, do_unlink=True)
    for c in list(bpy.data.collections):
        if c != coll:
            bpy.data.collections.remove(c)
    for t in list(bpy.data.texts):
        bpy.data.texts.remove(t)
    for a in list(bpy.data.actions):
        if a != act:
            bpy.data.actions.remove(a)
    bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)


def build_export(src_blend, action_name, out_blend):
    src_blend, out_blend = Path(src_blend).resolve(), Path(out_blend).resolve()
    if out_blend == src_blend:
        raise RuntimeError("out_blend must differ from src_blend")
    bpy.ops.wm.open_mainfile(filepath=str(src_blend), load_ui=False)
    log(f"SOURCE {src_blend} (opened, never saved); action {action_name!r}")
    scene = bpy.context.scene
    canon = json.loads(CANON.read_text(encoding="utf-8"))
    names = [b["name"] for b in canon["bones"]]
    rig = bpy.data.objects[WORK_RIG]
    rig.data.pose_position = "POSE"
    f0, f1, method = prepare_work(rig, action_name)
    log(f"WORK STATE {method}")
    coll = bpy.data.collections.new(COLL)
    scene.collection.children.link(coll)
    ex, rest_err, rest_bone = build_armature(canon, coll)
    log(f"CHECK {EXPORT}: {len(ex.data.bones)} bones (canonical {len(names)}), names/order match "
        f"{[b.name for b in ex.data.bones] == names}, parents match "
        f"{all((ex.data.bones[b['name']].parent.name if ex.data.bones[b['name']].parent else None) == b['parent'] for b in canon['bones'])}, "
        f"use_deform False {[b.name for b in ex.data.bones if not b.use_deform]}; matrix_local vs canonical rest_matrix "
        f"max abs {rest_err:.3e} ({rest_bone}; target <= {REST_TOL:g}); connected bones "
        f"{sum(1 for b in ex.data.bones if b.use_connect)}; object matrix identity {ex.matrix_world == Matrix.Identity(4)}")
    act_name = action_name if action_name is not None else "rest"
    act, slot, n_con, frames = bake(ex, rig, canon, f0, f1, act_name)
    log(f"BAKE frames {frames[0]}..{frames[-1]} ({len(frames)}), constraints left on {EXPORT}: {n_con}; action slot "
        f"{slot.identifier!r}")
    mesh = copy_mesh(ex, coll)
    worst_p, worst_r, wp, wr = 0.0, 0.0, None, None
    for f in frames:
        scene.frame_set(f)
        for n in names:
            p, r = mdiff(rig.matrix_world @ rig.pose.bones[n].matrix, ex.matrix_world @ ex.pose.bones[n].matrix)
            if p > worst_p:
                worst_p, wp = p, (n, f)
            if r > worst_r:
                worst_r, wr = r, (n, f)
    log(f"CHECK every baked frame ({len(frames)}) x {len(names)} bones: work DEF vs {EXPORT} world max pos "
        f"{worst_p:.6f} mm {wp}, max rot {worst_r:.6f} deg {wr}")
    cleanup({ex, mesh}, coll, act)
    act.name = act_name
    scene.frame_start, scene.frame_end = frames[0], frames[-1]
    scene.frame_set(frames[0])
    log(f"MESH {mesh.name}: modifiers {[(m.type, m.object.name) for m in mesh.modifiers]}, vertex groups "
        f"{len(mesh.vertex_groups)}, materials {len(mesh.data.materials)} {[m.name for m in mesh.data.materials]}, "
        f"custom normals {mesh.data.has_custom_normals}, uv layers {[u.name for u in mesh.data.uv_layers]}")
    log("OBJECTS " + ", ".join(f"{o.name} ({o.type})" for o in bpy.data.objects)
        + f"; actions {[(a.name, [s.identifier for s in a.slots]) for a in bpy.data.actions]}; collections "
        f"{[c.name for c in bpy.data.collections]}; texts {len(bpy.data.texts)}")
    out_blend.parent.mkdir(parents=True, exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(out_blend), copy=True)
    log(f"OUTPUT {out_blend}")
    return out_blend


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="p26_export_stage.py")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--action")
    g.add_argument("--rest", action="store_true")
    ap.add_argument("--out", required=True, help="output blend, relative to work/player/rig/")
    a = ap.parse_args(argv)
    if not bpy.data.filepath:
        raise RuntimeError("no source blend open")
    build_export(Path(bpy.data.filepath), None if a.rest else a.action, RIG / a.out)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
