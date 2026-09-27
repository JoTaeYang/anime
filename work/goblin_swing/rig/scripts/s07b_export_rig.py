"""s07b_export_rig.py - T32 (G7.2-G7.4): export armature GOB_export from canonical_skeleton.json, baked from the work rig.

    build_export(src_blend: Path, action_name: str | None, out_blend: Path) -> Path

Input : src_blend (work rig blend, e.g. rig/gob_r05_rigtest.blend; opened with open_mainfile, never saved),
        rig/data/canonical_skeleton.json (read only)
Output: out_blend (save copy, save_version 0) holding only GOB_export, goblin_mesh, goblin_club (collection EXPORT)

CLI:  bl.ps1 -Script s07b_export_rig.py -Blend gob_r05_rigtest.blend -- --action rigtest --out export/stage_rigtest.blend
      bl.ps1 -Script s07b_export_rig.py -Blend gob_r05_rigtest.blend -- --rest --out export/stage_rest.blend
      (--out relative to rig/; the source is the blend given with -Blend)

- GOB_export (identity object): 24 bones created only from canonical_skeleton.json (TREE order: head, tail, roll,
  parent, use_deform); the work DEF rest is never read. matrix_local is checked against rest_matrix. Every bone is
  use_connect False (spec 6, T32b): canonical use_connect is work-rig information and is ignored for export, since
  a connected bone ignores its location keys.
- Bake: every GOB_export pose bone (rotation_mode QUATERNION) gets Copy Transforms WORLD/WORLD <- GOB_rig DEF bone of
  the same name. Visual keying as bpy_extras.anim_utils.bake_action does it: per frame frame_set(f), then
  Object.convert_space(pose_bone, pb.matrix, 'POSE' -> 'LOCAL') with the constraints active (parent = its visual pose);
  quaternions made compatible with the previous frame. Then all constraints are removed and location /
  rotation_quaternion / scale are keyed every frame into a new slotted action ('rigtest' or 'rest', slot 'OBGOB_export').
  action_name: the action's frame range (use_frame_range) on the work rig; None: 1 frame (frame 1) with the work rig's
  action unassigned and the reset_rig state (every CTRL_* identity, every PROPS property at its UI default).
- Stage: goblin_mesh = copy of GOB_mesh (mesh data copied, same vertex groups, Armature modifier -> GOB_export);
  goblin_club = copy of GOB_club, re-parented to GOB_export bone weapon_socket_r with the same parent type / bone /
  matrix_parent_inverse / matrix_basis (the socket rest equals canonical, so the world transform is kept). Everything
  else (GOB_rig, CTRL / MCH / WGT objects and collections, GOB_mesh, GOB_club, texts, other actions) is removed and
  orphan data purged.
- HR2 (rig/data/hr2_contract.md): goblin_mesh keeps GOB_mesh's shape keys (Basis / mouth_open, relative) and its
  material slots. The copied Key's driver (PROPS mouth_open on GOB_rig) is removed; instead, during the bake the
  evaluated value of every non-reference work shape key is sampled at every baked frame and keyed on goblin_mesh's
  Key (action '<action>_shapekeys', slot of id type KEY; 'rest_shapekeys' for --rest, frame 1 = value in the reset
  state, i.e. 0). A work mesh without shape keys gives no Key action.
- Self-checks (stdout): 24 bones, rest vs canonical, constraints, bake range, work DEF vs GOB_export world error on up
  to 5 sample frames, goblin_club vs GOB_club, object / armature / action list.
"""
import argparse
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

WORK_RIG = "GOB_rig"
WORK_MESH = "GOB_mesh"
WORK_CLUB = "GOB_club"
EXPORT = "GOB_export"
MESH_OUT = "goblin_mesh"
CLUB_OUT = "goblin_club"
COLL = "EXPORT"
PROPS = "PROPS"
SOCKET = "weapon_socket_r"
REST_TOL = 1e-5
N_SAMPLES = 5
SK_ACTION_SUFFIX = "_shapekeys"   # HR2: stage Key action name = <action> + this


def log(msg):
    print(f"[s07b] {msg}")
    sys.stdout.flush()


def mdiff(a, b):
    """(mm, deg); angle = 2 atan2(|v|, |w|) of the relative quaternion (float32-safe near 0)."""
    q = a.to_quaternion().conjugated() @ b.to_quaternion()
    ang = 2.0 * math.atan2(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z), abs(q.w))
    return (a.translation - b.translation).length * 1000.0, math.degrees(ang)


def refresh(ob):
    ob.update_tag()
    bpy.context.view_layer.update()


# ---------------------------------------------------------------- work rig state
def prepare_work(rig, action_name):
    """Returns (frame_start, frame_end, method text)."""
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
                raise RuntimeError(f"action {action_name}: cannot pick an OBJECT slot from {[s.identifier for s in act.slots]}")
            ad.action_slot = slots[0]
        f0, f1 = (int(round(v)) for v in act.frame_range)
        return f0, f1, (f"work action {act.name!r} slot {ad.action_slot.identifier!r}, frames {f0}..{f1} "
                        f"(action frame_range, use_frame_range={act.use_frame_range})")
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
    return 1, 1, ("rest: work rig action unassigned (animation_data.action = None, drivers kept), every CTRL_* "
                  "identity, every PROPS property = UI default (reset_rig state), frame 1")


# ---------------------------------------------------------------- export armature
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
        eb.use_connect = False  # T32b: canonical use_connect ignored for export
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
        if n not in rig.pose.bones:
            raise RuntimeError(f"work rig has no bone {n}")
        c = pbs[n].constraints.new("COPY_TRANSFORMS")
        c.name = "bake_copy"
        c.target = rig
        c.subtarget = n
        c.target_space = "WORLD"
        c.owner_space = "WORLD"
    frames = list(range(f0, f1 + 1))
    data = {n: [] for n in names}
    prev = {}
    wkey = bpy.data.objects[WORK_MESH].data.shape_keys   # HR2: sample the evaluated work shape-key values
    sk_names = [kb.name for kb in wkey.key_blocks if kb != wkey.reference_key] if wkey is not None else []
    sk_vals = {k: [] for k in sk_names}
    for f in frames:
        scene.frame_set(f)
        if sk_names:
            kev = wkey.evaluated_get(bpy.context.evaluated_depsgraph_get())
            for k in sk_names:
                sk_vals[k].append(float(kev.key_blocks[k].value))
        for n in names:
            pb = pbs[n]
            m = ex.convert_space(pose_bone=pb, matrix=pb.matrix, from_space="POSE", to_space="LOCAL")
            loc, q, sc = m.decompose()
            if n in prev:
                q.make_compatible(prev[n])
            prev[n] = q.copy()
            data[n].append((loc.copy(), q.copy(), sc.copy()))
    n_con = 0
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
    return act, slot, n_con, frames, sk_vals


def key_shape_keys(mesh, frames, sk_vals, act_name):
    """HR2: key the sampled work shape-key values on the stage mesh's Key at every baked frame (no driver)."""
    key = mesh.data.shape_keys
    if key is None or not sk_vals:
        return None
    kact = bpy.data.actions.new(act_name + SK_ACTION_SUFFIX)
    kslot = kact.slots.new(id_type="KEY", name=key.name)
    kad = key.animation_data_create()
    kad.action = kact
    kad.action_slot = kslot
    for k, vals in sk_vals.items():
        kb = key.key_blocks[k]
        for f, v in zip(frames, vals):
            kb.value = v
            kb.keyframe_insert("value", frame=f)
    kact.use_frame_range = True
    kact.frame_start, kact.frame_end = frames[0], frames[-1]
    return kact


# ---------------------------------------------------------------- stage objects
def copy_mesh(ex, coll):
    src = bpy.data.objects[WORK_MESH]
    ob = src.copy()
    ob.data = src.data.copy()
    ob.name = MESH_OUT
    ob.data.name = MESH_OUT
    ob.animation_data_clear()
    if ob.data.shape_keys is not None:   # HR2: the copied Key's driver targets the work rig; keys replace it
        ob.data.shape_keys.animation_data_clear()
    coll.objects.link(ob)
    arms = [m for m in ob.modifiers if m.type == "ARMATURE"]
    if len(arms) != 1:
        raise RuntimeError(f"{WORK_MESH}: expected one Armature modifier, got {[m.type for m in ob.modifiers]}")
    arms[0].object = ex
    if ob.parent is not None:
        raise RuntimeError(f"{WORK_MESH} has parent {ob.parent.name} (expected none)")
    return ob, [m.type for m in ob.modifiers], len(ob.vertex_groups)


def copy_club(ex, coll):
    src = bpy.data.objects[WORK_CLUB]
    if src.parent is None or src.parent_type != "BONE" or src.parent_bone != SOCKET:
        raise RuntimeError(f"{WORK_CLUB} parent {src.parent} / {src.parent_type} / {src.parent_bone} "
                           f"(expected {WORK_RIG} BONE {SOCKET})")
    pinv, basis = src.matrix_parent_inverse.copy(), src.matrix_basis.copy()
    ob = src.copy()
    ob.data = src.data.copy()
    ob.name = CLUB_OUT
    ob.data.name = CLUB_OUT
    ob.animation_data_clear()
    coll.objects.link(ob)
    ob.parent = ex
    ob.parent_type = "BONE"
    ob.parent_bone = SOCKET
    ob.matrix_parent_inverse = pinv
    ob.matrix_basis = basis
    return ob


def cleanup(keep_objs, coll, act):
    # HR2: the stage Key action is created after this cleanup (key_shape_keys)
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


# ---------------------------------------------------------------- main function
def build_export(src_blend: Path, action_name: str | None, out_blend: Path) -> Path:
    src_blend, out_blend = Path(src_blend).resolve(), Path(out_blend).resolve()
    if out_blend == src_blend:
        raise RuntimeError("out_blend must differ from src_blend")
    bpy.ops.wm.open_mainfile(filepath=str(src_blend), load_ui=False)
    log(f"SOURCE {src_blend} (opened, never saved); action {action_name!r}")
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    scene = bpy.context.scene
    canon = goblib.load_json("canonical_skeleton.json")
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
        f"max abs {rest_err:.3e} (bone {rest_bone}; target <= {REST_TOL:g}); connected bones "
        f"{sum(1 for b in ex.data.bones if b.use_connect)} (canonical use_connect True "
        f"{sum(1 for b in canon['bones'] if b['use_connect'])}, ignored); object matrix identity "
        f"{ex.matrix_world == Matrix.Identity(4)}")

    act_name = action_name if action_name is not None else "rest"
    act, slot, n_con, frames, sk_vals = bake(ex, rig, canon, f0, f1, act_name)
    log(f"BAKE frames {frames[0]}..{frames[-1]} ({len(frames)}), visual keying (convert_space POSE->LOCAL with "
        f"Copy Transforms WORLD/WORLD active), constraints left on {EXPORT}: {n_con}; action slot "
        f"{slot.identifier!r}, animation_data.action_slot {ex.animation_data.action_slot.identifier!r}")

    mesh, mods, n_vg = copy_mesh(ex, coll)
    club_src = bpy.data.objects[WORK_CLUB]
    club = copy_club(ex, coll)

    # sample comparison (work rig still present)
    k = min(N_SAMPLES, len(frames))
    samples = sorted({frames[round(i * (len(frames) - 1) / max(k - 1, 1))] for i in range(k)})
    for f in samples:
        scene.frame_set(f)
        worst_p, worst_r, wp, wr = 0.0, 0.0, None, None
        for n in names:
            p, r = mdiff(rig.matrix_world @ rig.pose.bones[n].matrix, ex.matrix_world @ ex.pose.bones[n].matrix)
            if p > worst_p:
                worst_p, wp = p, n
            if r > worst_r:
                worst_r, wr = r, n
        cp, cr = mdiff(club_src.matrix_world, club.matrix_world)
        log(f"CHECK frame {f}: work DEF vs {EXPORT} world, 24 bones: max pos {worst_p:.4f} mm ({wp}), max rot "
            f"{worst_r:.4f} deg ({wr}); {CLUB_OUT} vs {WORK_CLUB} world {cp:.4f} mm / {cr:.4f} deg")

    cleanup({ex, mesh, club}, coll, act)
    act.name = act_name  # the work action of the same name is gone now
    kact = key_shape_keys(mesh, frames, sk_vals, act_name)
    mkey = mesh.data.shape_keys
    log(f"SHAPE KEYS {mesh.name}: {[kb.name for kb in mkey.key_blocks] if mkey else []} (relative "
        f"{mkey.use_relative if mkey else None}), drivers on the stage Key "
        f"{len(mkey.animation_data.drivers) if mkey and mkey.animation_data else 0}; Key action "
        f"{kact.name if kact else None!r} slot {kact.slots[0].identifier if kact else None!r}; sampled work values per "
        f"key over {len(frames)} frames: " + ", ".join(f"{k} min {min(v):.4f} max {max(v):.4f} keyed {len(v)}"
                                                         for k, v in sk_vals.items())
        + f"; materials {[m.name if m else None for m in mesh.data.materials]}")
    scene.frame_start, scene.frame_end = frames[0], frames[-1]
    scene.frame_set(frames[0])
    log(f"MESH {mesh.name}: modifiers {mods}, Armature -> {mesh.modifiers[0].object.name}, vertex groups {n_vg}, "
        f"custom normals {mesh.data.has_custom_normals}; CLUB {club.name}: parent {club.parent.name} "
        f"{club.parent_type} {club.parent_bone}")
    log("OBJECTS " + ", ".join(f"{o.name} ({o.type}, collections {[c.name for c in o.users_collection]})"
                               for o in bpy.data.objects)
        + f"; armatures {[a.name for a in bpy.data.armatures]}; actions "
        f"{[(a.name, [s.identifier for s in a.slots]) for a in bpy.data.actions]}; collections "
        f"{[c.name for c in bpy.data.collections]}; texts {len(bpy.data.texts)}; constraints on {EXPORT} "
        f"{sum(len(pb.constraints) for pb in ex.pose.bones)}; object constraints "
        f"{sum(len(o.constraints) for o in bpy.data.objects)}")

    out_blend.parent.mkdir(parents=True, exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(out_blend), copy=True)
    log(f"OUTPUT {out_blend}")
    return out_blend


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="s07b_export_rig.py")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--action")
    g.add_argument("--rest", action="store_true")
    ap.add_argument("--out", required=True, help="output blend, relative to rig/")
    a = ap.parse_args(argv)
    if not bpy.data.filepath:
        raise RuntimeError("no source blend open (use bl.ps1 -Blend)")
    build_export(Path(bpy.data.filepath), None if a.rest else a.action, goblib.RIG / a.out)


if __name__ == "__main__":
    goblib.run_main(main)
