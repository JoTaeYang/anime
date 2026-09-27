"""s11_export_clip.py - T70 (A3.1-A3.3 data): export one goblin clip (stage + FBX + Unity copy).

Input : the work blend given with -Blend (any blend holding the --action action, e.g. rig/gob_r07_swing_anim.blend,
        rig/gob_r08_idle.blend; opened by s07b.build_export, never saved),
        rig/data/export_preset.json, rig/data/canonical_skeleton.json (read only)
Output: rig/export/stage_<action>.blend   (s07b_export_rig.build_export)
        rig/export/goblin@<action>.fbx    armature `goblin` + goblin_mesh + the active action (AnimStack <action>)
        unity/AvatarCheck/Assets/Goblin/goblin@<action>.fbx  (byte copy of the FBX)

CLI:  bl.ps1 -Script s11_export_clip.py -Blend gob_r07_swing_anim.blend -- --action attack_swing

- Export keywords = export_preset.json `export_scene_fbx` (asserted equal to s08 COMMON + PRESET) with bake_anim True.
- Renames in memory on the opened stage (never saved), as s08: GOB_export -> `goblin`; the scene -> <action>, since
  the active-action path (bake_anim_use_all_actions False) names its single AnimStack after the scene.
- Self-check (stdout only, not a gate checker): FBX header / node summary (s08.fbx_summary), one reimport with the
  preset reimport options: node list, bone count / names / parents, `_end` leaves, keyed frame count, and the pelvis
  world position vs stage on every frame, as imported and after use_connect False on all reimported bones
  (G8 pitfall; reimport frame = stage frame + anim offset).
"""
import argparse
import os
import shutil
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402
import s07b_export_rig as s07b  # noqa: E402
import s08_export_fbx as s08  # noqa: E402

import bpy  # noqa: E402

PRESET_JSON = "export_preset.json"
UNITY_GOBLIN = goblib.RIG.parents[2] / "unity" / "AvatarCheck" / "Assets" / "Goblin"
ARM_OUT = s08.ARM_OUT
MESH_OUT = s08.MESH_OUT


def log(msg):
    print(f"[s11] {msg}")
    sys.stdout.flush()


def v3(v, nd=6):
    return s08.v3(v, nd)


def preset_kwargs(preset):
    kw = dict(preset["export_scene_fbx"])
    ref = dict(s08.COMMON, **s08.PRESET)
    if kw != ref:
        diff = sorted(k for k in set(kw) | set(ref) if kw.get(k) != ref.get(k))
        raise RuntimeError(f"export_preset.json export_scene_fbx differs from s08 COMMON+PRESET in {diff}")
    kw["object_types"] = set(kw["object_types"])
    kw["bake_anim"] = True
    return kw


def export_clip(path, arm, mesh, kw):
    sel = s08.select_only([arm, mesh])
    res = bpy.ops.export_scene.fbx(filepath=str(path), **kw)
    if res != {"FINISHED"}:
        raise RuntimeError(f"export_scene.fbx {path.name} returned {res}")
    log(f"EXPORT {path.name}: selected {sel}, bake_anim True, scene {bpy.context.scene.name!r} frames "
        f"{bpy.context.scene.frame_start}..{bpy.context.scene.frame_end} -> {res}")


def reimport_check(path, imp, canon_names, canon_parent, stage_pelvis, stage_f0):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    res = bpy.ops.import_scene.fbx(filepath=str(path), **imp)
    log(f"REIMPORT {path.name}: import_scene.fbx {res} (options = export_preset.json blender_reimport)")
    for o in sorted(bpy.data.objects, key=lambda o: o.name):
        loc, rot, sc = o.matrix_world.decompose()
        log(f"  node {o.name!r} ({o.type}) parent {o.parent.name if o.parent else None!r}: world loc "
            f"{v3(loc * 1000.0, 4)} mm, scale {v3(sc, 7)}")
    g = bpy.data.objects.get(ARM_OUT)
    if g is None or g.type != "ARMATURE":
        raise RuntimeError(f"reimport: no armature {ARM_OUT!r}")
    names = [b.name for b in g.data.bones]
    par_ok = all((g.data.bones[n].parent.name if g.data.bones[n].parent else None) == canon_parent.get(n)
                 for n in names if n in canon_parent)
    log(f"  armature {g.name!r}: {len(names)} bones, name set == canonical {set(names) == set(canon_names)}, "
        f"parents == canonical {par_ok}, missing {sorted(set(canon_names) - set(names))}, extra "
        f"{sorted(set(names) - set(canon_names))}, _end leaves {[n for n in names if n.endswith('_end')]}")
    for m in bpy.data.objects:
        if m.type == "MESH":
            me = m.data
            sk = me.shape_keys
            kfc = s08.action_fcurves(sk.animation_data.action) if sk and sk.animation_data and sk.animation_data.action else []
            log(f"  mesh {m.name!r}: verts {len(me.vertices)}, tris {sum(len(p.vertices) - 2 for p in me.polygons)}, "
                f"vertex groups {len(m.vertex_groups)}, modifiers "
                f"{[(x.type, x.object.name if getattr(x, 'object', None) else None) for x in m.modifiers]}; HR2 shape "
                f"keys {[kb.name for kb in sk.key_blocks] if sk else []}, Key action curves "
                f"{[(fc.data_path, len(fc.keyframe_points), round(min(k.co.y for k in fc.keyframe_points), 4), round(max(k.co.y for k in fc.keyframe_points), 4)) for fc in kfc if len(fc.keyframe_points)]}, "
                f"materials {[x.name if x else None for x in me.materials]}")
    ad = g.animation_data
    act = ad.action if ad else None
    if act is None:
        raise RuntimeError("reimport: armature has no action")
    for a in bpy.data.actions:
        fcs = s08.action_fcurves(a)
        keys = sorted({round(k.co.x, 3) for fc in fcs for k in fc.keyframe_points})
        log(f"  action {a.name!r}: slots {[s.identifier for s in a.slots]}, fcurves {len(fcs)}, frame_range "
            f"{v3(a.frame_range, 3)}, distinct keyed frames {len(keys)}" + (f" ({keys[0]}..{keys[-1]})" if keys else ""))
    log(f"  actions {len(bpy.data.actions)}; scene fps {bpy.context.scene.render.fps}")

    off = int(round(act.frame_range[0])) - stage_f0
    scene = bpy.context.scene

    def errs():
        out = []
        for f, w in stage_pelvis.items():
            scene.frame_set(f + off)
            out.append((f, round(((g.matrix_world @ g.pose.bones["pelvis"].matrix).translation - w).length * 1000.0, 4)))
        return out

    connected = [b.name for b in g.data.bones if b.use_connect]
    e_imp = errs()
    bpy.context.view_layer.objects.active = g
    bpy.ops.object.mode_set(mode="EDIT")
    for eb in g.data.edit_bones:
        eb.use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")
    e_dis = errs()
    worst_imp = max(e_imp, key=lambda t: t[1])
    worst_dis = max(e_dis, key=lambda t: t[1])
    log(f"PELVIS SELFCHECK {path.name} (reimport frame = stage frame + {off}), {len(e_dis)} frames: as imported max "
        f"{worst_imp[1]} mm at stage frame {worst_imp[0]} (reimported use_connect True: {connected}); after use_connect "
        f"False max {worst_dis[1]} mm at stage frame {worst_dis[0]}")
    log(f"PELVIS after use_connect False [stage frame, mm]: {e_dis}")


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="s11_export_clip.py")
    ap.add_argument("--action", required=True)
    a = ap.parse_args(argv)
    if not bpy.data.filepath:
        raise RuntimeError("no source blend open (use bl.ps1 -Blend <work blend>)")
    src = Path(bpy.data.filepath).resolve()
    if bpy.data.actions.get(a.action) is None:
        raise RuntimeError(f"action {a.action!r} not in {src.name} (actions {[x.name for x in bpy.data.actions]})")
    clip = a.action
    stage = goblib.EXPORT / f"stage_{clip}.blend"
    fbx = goblib.EXPORT / f"goblin@{clip}.fbx"
    preset = goblib.load_json(PRESET_JSON)
    kw = preset_kwargs(preset)
    imp = dict(preset["blender_reimport"]["import_scene_fbx"])
    canon = goblib.load_json("canonical_skeleton.json")
    canon_names = [b["name"] for b in canon["bones"]]
    canon_parent = {b["name"]: b["parent"] for b in canon["bones"]}

    # 1. stage (the work blend is only opened by build_export, never saved)
    s07b.build_export(src, clip, stage)

    # 2. goblin@<clip>.fbx from the stage (active action only)
    arm, mesh = s08.open_stage(stage)
    scene = bpy.context.scene
    old_scene = scene.name
    scene.name = clip
    ad = arm.animation_data
    act = ad.action if ad else None
    log(f"STAGE {stage.name}: scene {old_scene!r} -> {scene.name!r} (AnimStack name), fps {scene.render.fps}, frames "
        f"{scene.frame_start}..{scene.frame_end}; {ARM_OUT} action {act.name if act else None!r} slot "
        f"{ad.action_slot.identifier if ad and ad.action_slot else None!r} frame_range "
        f"{v3(act.frame_range, 3) if act else None}; actions in file {[x.name for x in bpy.data.actions]}; NLA tracks "
        f"{len(ad.nla_tracks) if ad else 0}; objects {[(o.name, o.type) for o in bpy.data.objects]}")
    if act is None or act.name != clip:
        raise RuntimeError(f"stage: {ARM_OUT} active action is {act.name if act else None!r}, expected {clip!r}")
    if scene.name != clip:
        raise RuntimeError(f"scene rename gave {scene.name!r}")
    f_cur = scene.frame_current
    stage_f0 = scene.frame_start
    stage_pelvis = {}
    for f in range(scene.frame_start, scene.frame_end + 1):
        scene.frame_set(f)
        stage_pelvis[f] = (arm.matrix_world @ arm.pose.bones["pelvis"].matrix).translation.copy()
    scene.frame_set(f_cur)
    export_clip(fbx, arm, mesh, kw)
    s = s08.fbx_summary(fbx)
    log(f"FILE {fbx.name}: {fbx.stat().st_size} bytes; FBX {s['fbx_version']}; header {s['header']}; nodes "
        f"{s['nodes']}; limb nodes {s['limb_nodes']}, _end {s['limb_end_nodes']}; anim stacks {s['anim_stacks']}")

    # 3. Unity copy
    UNITY_GOBLIN.mkdir(parents=True, exist_ok=True)
    dst = UNITY_GOBLIN / fbx.name
    shutil.copy2(fbx, dst)
    log(f"COPY {fbx} -> {dst} ({dst.stat().st_size} bytes, sha256 equal "
        f"{goblib.sha256(dst) == goblib.sha256(fbx)})")

    # 4. self-check reimport
    reimport_check(fbx, imp, canon_names, canon_parent, stage_pelvis, stage_f0)
    log("DONE")


if __name__ == "__main__":
    goblib.run_main(main)
