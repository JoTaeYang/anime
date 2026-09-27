"""p11_export.py - T210 (P1.1/P1.2): FBX export of the crude player rig with the goblin preset.

Input : work/player/rig/pl_p1_crude.blend (opened, never saved)
        work/goblin_swing/rig/data/export_preset.json (read only: export_scene_fbx kwargs, blender_reimport kwargs)
Output: work/player/rig/export/stage_rest.blend    armature `Player` (action unassigned, pose identity) + Player_mesh
        work/player/rig/export/stage_p1test.blend  same, action `p1test` active, scene renamed `p1test`
        work/player/rig/export/Player.fbx          from stage_rest, bake_anim False
        work/player/rig/export/Player@p1test.fbx   from stage_p1test, bake_anim True (AnimStack `p1test`)
        unity/AvatarCheck/Assets/Player/Player.fbx, Player@p1test.fbx (copies)

CLI:  blender --background --factory-startup --python p11_export.py

- The crude rig is already the export skeleton (every bone use_connect False, object matrices identity); the stages
  only differ in the active action.
- Self-check (stdout only): FBX header / node summary per file and one reimport per file with the preset's reimport
  settings (bone count, name / parent match, bones the importer connected, action frame range).
"""
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

import bpy
from mathutils import Matrix

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
REPO = RIG.parents[2]
SRC_BLEND = RIG / "pl_p1_crude.blend"
PRESET = REPO / "work" / "goblin_swing" / "rig" / "data" / "export_preset.json"
EXPORT = RIG / "export"
STAGE_REST = EXPORT / "stage_rest.blend"
STAGE_CLIP = EXPORT / "stage_p1test.blend"
FBX_BODY = EXPORT / "Player.fbx"
FBX_CLIP = EXPORT / "Player@p1test.fbx"
UNITY_DIR = REPO / "unity" / "AvatarCheck" / "Assets" / "Player"
ARM, MESH, CLIP = "Player", "Player_mesh", "p1test"


def log(msg):
    print(f"[p11] {msg}")
    sys.stdout.flush()


def v3(v, nd=6):
    return [round(float(x), nd) + 0.0 for x in v]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def open_src():
    bpy.ops.wm.open_mainfile(filepath=str(SRC_BLEND), load_ui=False)
    arm, mesh = bpy.data.objects[ARM], bpy.data.objects[MESH]
    return arm, mesh


def select_only(objs):
    vl = bpy.context.view_layer
    for o in vl.objects:
        o.select_set(False)
    for o in objs:
        o.select_set(True)
    vl.objects.active = objs[0]
    return sorted(o.name for o in bpy.context.selected_objects)


def export_fbx(path, objs, bake_anim, kw_preset):
    sel = select_only(objs)
    kw = dict(kw_preset)
    kw["object_types"] = set(kw["object_types"])
    kw["bake_anim"] = bake_anim
    res = bpy.ops.export_scene.fbx(filepath=str(path), **kw)
    if res != {"FINISHED"}:
        raise RuntimeError(f"export_scene.fbx {path.name} returned {res}")
    log(f"EXPORT {path.name}: selected {sel}, bake_anim {bake_anim}, scene {bpy.context.scene.name!r} frames "
        f"{bpy.context.scene.frame_start}..{bpy.context.scene.frame_end} -> {res}; {path.stat().st_size} bytes, "
        f"sha256 {sha(path)}")


# ---------------------------------------------------------------- FBX file summary (raw header / nodes)
def _child(e, id_):
    return next((c for c in e.elems if c.id == id_), None) if e is not None else None


def _p70(e):
    p = _child(e, b"Properties70")
    out = {}
    if p is not None:
        for c in p.elems:
            if c.id == b"P":
                out[c.props[0].decode()] = list(c.props[4:])
    return out


def _name(e):
    return e.props[1].split(b"\x00\x01")[0].decode()


def fbx_summary(path):
    from io_scene_fbx import parse_fbx
    root, ver = parse_fbx.parse(str(path))
    gs = _p70(_child(root, b"GlobalSettings"))
    hdr = {k: (gs[k][0] if k in gs and gs[k] else None) for k in
           ("UpAxis", "UpAxisSign", "FrontAxis", "FrontAxisSign", "CoordAxis", "CoordAxisSign",
            "UnitScaleFactor", "OriginalUnitScaleFactor", "TimeMode", "CustomFrameRate")}
    objs = _child(root, b"Objects")
    nodes, limbs, stacks, clusters = [], [], [], 0
    for e in objs.elems if objs is not None else []:
        if e.id == b"Model":
            kind = e.props[2].decode()
            if kind == "LimbNode":
                limbs.append(_name(e))
            else:
                p = _p70(e)
                nodes.append({"name": _name(e), "type": kind,
                              "lcl_translation": v3(p.get("Lcl Translation", [0, 0, 0])[:3]),
                              "lcl_rotation_deg": v3(p.get("Lcl Rotation", [0, 0, 0])[:3]),
                              "lcl_scaling": v3(p.get("Lcl Scaling", [1, 1, 1])[:3])})
        elif e.id == b"AnimationStack":
            stacks.append(_name(e))
        elif e.id == b"Deformer" and e.props[2] == b"Cluster":
            clusters += 1
    return {"fbx_version": ver, "header": hdr, "nodes": nodes, "limb_nodes": len(limbs),
            "limb_end_nodes": [n for n in limbs if n.endswith("_end")], "anim_stacks": stacks, "skin_clusters": clusters}


def action_fcurves(act):
    out = []
    for layer in act.layers:
        for strip in layer.strips:
            for cb in strip.channelbags:
                out.extend(cb.fcurves)
    return out


def reimport(path, kw_import, names, parent):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    res = bpy.ops.import_scene.fbx(filepath=str(path), **kw_import)
    log(f"REIMPORT {path.name}: import_scene.fbx {res}")
    for o in sorted(bpy.data.objects, key=lambda o: o.name):
        loc, rot, sc = o.matrix_world.decompose()
        log(f"  node {o.name!r} ({o.type}) parent {o.parent.name if o.parent else None!r}: world loc "
            f"{v3(loc * 1000.0, 4)} mm, rot {v3([math.degrees(a) for a in rot.to_euler()], 4)} deg, scale {v3(sc, 7)}")
        if o.type == "ARMATURE":
            bn = [b.name for b in o.data.bones]
            par_ok = all((o.data.bones[n].parent.name if o.data.bones[n].parent else None) == parent.get(n)
                         for n in bn if n in parent)
            log(f"  armature {o.name!r}: {len(bn)} bones, name set == blend {set(bn) == set(names)}, parents == blend "
                f"{par_ok}, missing {sorted(set(names) - set(bn))}, extra {sorted(set(bn) - set(names))}, connected by "
                f"the importer {sum(1 for b in o.data.bones if b.use_connect)}: "
                f"{[b.name for b in o.data.bones if b.use_connect]}")
        elif o.type == "MESH":
            me = o.data
            log(f"  mesh {o.name!r}: verts {len(me.vertices)}, polys {len(me.polygons)}, vertex groups "
                f"{len(o.vertex_groups)}, materials {len(me.materials)}, modifiers "
                f"{[(m.type, m.object.name if getattr(m, 'object', None) else None) for m in o.modifiers]}")
    for a in bpy.data.actions:
        fcs = action_fcurves(a)
        keys = sorted({round(k.co.x, 3) for fc in fcs for k in fc.keyframe_points})
        log(f"  action {a.name!r}: fcurves {len(fcs)}, frame_range {v3(a.frame_range, 3)}, distinct keyed frames "
            f"{len(keys)}" + (f" ({keys[0]}..{keys[-1]})" if keys else ""))


def main():
    preset = json.loads(PRESET.read_text(encoding="utf-8"))
    kw_exp = preset["export_scene_fbx"]
    kw_imp = preset["blender_reimport"]["import_scene_fbx"]
    log(f"PRESET {PRESET} (sha256 {sha(PRESET)}): {kw_exp}")
    log(f"SOURCE {SRC_BLEND} sha256 {sha(SRC_BLEND)}")
    EXPORT.mkdir(parents=True, exist_ok=True)

    # 1. rest stage -> Player.fbx
    arm, mesh = open_src()
    names = [b.name for b in arm.data.bones]
    parent = {b.name: (b.parent.name if b.parent else None) for b in arm.data.bones}
    log(f"CHECK {ARM}: {len(names)} bones, connected {sum(1 for b in arm.data.bones if b.use_connect)}, deform "
        f"{sum(1 for b in arm.data.bones if b.use_deform)}; {ARM} matrix identity {arm.matrix_world == Matrix.Identity(4)}, "
        f"{MESH} matrix identity {mesh.matrix_world == Matrix.Identity(4)}, {MESH} modifiers "
        f"{[(m.type, m.object.name) for m in mesh.modifiers]}, materials {len(mesh.data.materials)}")
    if any(b.use_connect for b in arm.data.bones):
        raise RuntimeError("connected bones in the crude rig")
    ad = arm.animation_data
    if ad is not None:
        ad.action = None
    for pb in arm.pose.bones:
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        pb.scale = (1.0, 1.0, 1.0)
    bpy.context.scene.frame_set(1)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(STAGE_REST), copy=True)
    log(f"STAGE {STAGE_REST.name}: action unassigned, pose identity, frame 1")
    export_fbx(FBX_BODY, [arm, mesh], False, kw_exp)

    # 2. clip stage -> Player@p1test.fbx
    arm, mesh = open_src()
    scene = bpy.context.scene
    old = scene.name
    scene.name = CLIP
    act = arm.animation_data.action if arm.animation_data else None
    if act is None or act.name != CLIP:
        raise RuntimeError(f"active action {act.name if act else None!r}, expected {CLIP!r}")
    scene.frame_start, scene.frame_end = (int(round(x)) for x in act.frame_range)
    scene.frame_set(scene.frame_start)
    bpy.ops.wm.save_as_mainfile(filepath=str(STAGE_CLIP), copy=True)
    log(f"STAGE {STAGE_CLIP.name}: scene {old!r} -> {scene.name!r}, fps {scene.render.fps}, frames "
        f"{scene.frame_start}..{scene.frame_end}, action {act.name!r} slot {arm.animation_data.action_slot.identifier!r}")
    export_fbx(FBX_CLIP, [arm, mesh], True, kw_exp)

    # 3. summaries + reimport self-check
    for p in (FBX_BODY, FBX_CLIP):
        s = fbx_summary(p)
        log(f"FILE {p.name}: FBX {s['fbx_version']}; header {s['header']}; nodes {s['nodes']}; limb nodes "
            f"{s['limb_nodes']}, _end {s['limb_end_nodes']}; skin clusters {s['skin_clusters']}; anim stacks "
            f"{s['anim_stacks']}")
    for p in (FBX_BODY, FBX_CLIP):
        reimport(p, kw_imp, names, parent)

    # 4. copy to Unity
    UNITY_DIR.mkdir(parents=True, exist_ok=True)
    for p in (FBX_BODY, FBX_CLIP):
        dst = UNITY_DIR / p.name
        shutil.copy2(p, dst)
        log(f"COPY {p.name} -> {dst} sha256 {sha(dst)}")
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
