"""Export the retargeted WalkKimodo action as a Unity-ready clip FBX.

    blender --background CLIP.blend --python scripts/mocap/export_clip.py
        -- OUT.fbx [FIRST LAST]

Mirrors scripts/stages/04_export.py's conventions -- the pre-rotate trick that
leaves the armature at identity in Unity, FBX_SCALE_ALL against the 100x bone
scale trap, and no leaf bones. Armature only: the clip carries no mesh, the
character model.fbx already supplies the avatar.
"""
import math
import sys
from pathlib import Path

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.blender_utils import op_kwargs  # noqa: E402

argv = sys.argv[sys.argv.index("--") + 1:]
OUT_FBX = argv[0]
# Default to the whole action: a range is only needed when trimming to a loop.
clip = next(o for o in bpy.data.objects
            if o.type == "ARMATURE").animation_data.action
FIRST = int(argv[1]) if len(argv) > 1 else int(min(clip.frame_range))
LAST = int(argv[2]) if len(argv) > 2 else int(max(clip.frame_range))

rig = next(o for o in bpy.data.objects if o.type == "ARMATURE")
scene = bpy.context.scene
scene.frame_start, scene.frame_end = FIRST, LAST
print(f"RESULT range={FIRST}..{LAST} frames={LAST - FIRST + 1}")

# Drop the real mesh -- a clip needs no geometry -- but leave a token skinned
# mesh behind. Unity folds a childless armature NULL into the file-named root,
# which makes Hips' parent 'WalkKimodo' instead of 'DummyRig' and CopyFromOther
# rejects the hierarchy. A single skinned triangle keeps DummyRig a real node.
for obj in [o for o in bpy.data.objects if o.type != "ARMATURE"]:
    bpy.data.objects.remove(obj, do_unlink=True)

proxy_mesh = bpy.data.meshes.new("ClipProxy")
proxy_mesh.from_pydata([(0, 0, 0), (0.01, 0, 0), (0, 0, 0.01)], [], [(0, 1, 2)])
proxy_mesh.update()
proxy = bpy.data.objects.new("ClipProxy", proxy_mesh)
bpy.context.scene.collection.objects.link(proxy)
proxy.parent = rig
group = proxy.vertex_groups.new(name="Hips")
group.add([0, 1, 2], 1.0, "REPLACE")
modifier = proxy.modifiers.new("Armature", "ARMATURE")
modifier.object = rig
print(f"RESULT proxy_parent={proxy.parent.name}")

# 04_export's axis trick: bake the rest to Y-up, leave an unapplied +90 X so the
# FBX axis conversion cancels it and Unity sees an identity armature node.
bpy.context.view_layer.objects.active = rig
bpy.ops.object.mode_set(mode="OBJECT")
bpy.ops.object.select_all(action="DESELECT")
for obj in (rig, proxy):
    obj.select_set(True)
    obj.rotation_euler = (math.radians(-90), 0.0, 0.0)
bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
for obj in (rig, proxy):
    obj.rotation_euler = (math.radians(90), 0.0, 0.0)

kwargs = dict(
    filepath=OUT_FBX,
    use_selection=False,
    object_types={"ARMATURE", "MESH"},
    apply_unit_scale=True,
    global_scale=1.0,
    apply_scale_options="FBX_SCALE_ALL",
    axis_forward="-Z",
    axis_up="Y",
    use_space_transform=True,
    bake_space_transform=False,
    add_leaf_bones=False,
    primary_bone_axis="Y",
    secondary_bone_axis="X",
    armature_nodetype="NULL",
    use_armature_deform_only=False,
    bake_anim=True,
    bake_anim_use_all_bones=False,
    bake_anim_use_nla_strips=False,
    bake_anim_use_all_actions=False,
    bake_anim_force_startend_keying=True,
    mesh_smooth_type="OFF",
    path_mode="AUTO",
)
Path(OUT_FBX).parent.mkdir(parents=True, exist_ok=True)
bpy.ops.export_scene.fbx(**op_kwargs(bpy.ops.export_scene.fbx, **kwargs))
print("RESULT exported=" + OUT_FBX)
print(f"RESULT size_kb={Path(OUT_FBX).stat().st_size / 1024:.0f}")
