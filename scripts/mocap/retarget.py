"""Retarget a kimodo SOMA-30 BVH onto a humanoid rig.

    blender --background TARGET.blend --python scripts/mocap/retarget.py
        -- CLIP.bvh OUT.blend [--fbx MODEL.fbx] [--abduct auto|DEGREES]

Rest-pose-relative: the source's rotation delta from its own rest pose is what
transfers, so differing bone rolls, limb proportions, and even a T-pose vs
A-pose rest all survive without leaking into the result.

--abduct auto solves for the smallest shoulder opening that keeps the arms out
of the body, using the same face-overlap test the gate runs. Every character
needs its own value; see scripts/mocap/penetration.py for why.
"""
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mocap import rigs  # noqa: E402
from mocap.penetration import MAX_DEPTH, scan  # noqa: E402

argv = sys.argv[sys.argv.index("--") + 1:]
BVH_PATH, OUT_BLEND = argv[0], argv[1]
FBX_PATH = argv[argv.index("--fbx") + 1] if "--fbx" in argv else None
ABDUCT_ARG = argv[argv.index("--abduct") + 1] if "--abduct" in argv else "0"
AUTO = ABDUCT_ARG == "auto"
ABDUCT = 0.0 if AUTO else math.radians(float(ABDUCT_ARG))
# Coarse-then-fine search. Monotone in practice -- more opening is more
# clearance -- so the first clean coarse step brackets the answer.
COARSE_STEP, MAX_ABDUCT = 4, 20

if FBX_PATH:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    # ignore_leaf_bones drops the last bone of every chain, which on our own
    # Rigify export is the toe -- and a walk without toes has nothing to plant.
    # A CC rig survived it only because its chains end in finger and toe digits.
    bpy.ops.import_scene.fbx(filepath=FBX_PATH, ignore_leaf_bones=False,
                             automatic_bone_orientation=False)

target = next(o for o in bpy.data.objects if o.type == "ARMATURE")

# A CC FBX arrives at 0.01 scale (centimetre source). Bake it so armature space
# and world space are both metres and the maths below needs no unit handling.
if tuple(round(v, 4) for v in target.scale) != (1.0, 1.0, 1.0) or \
        tuple(round(v, 4) for v in target.rotation_euler) != (0.0, 0.0, 0.0):
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.select_all(action="DESELECT")
    for obj in [target] + [o for o in bpy.data.objects if o.parent == target]:
        obj.select_set(True)
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    print("RESULT applied_transform=1")

kind, BONE_MAP = rigs.detect(target)
names = {bone.name for bone in target.data.bones}
print(f"RESULT rig={kind} matched={len(names & set(BONE_MAP.values()))}/{len(BONE_MAP)}")

CHILD, PARENT_OF = rigs.CHILD, rigs.PARENT_OF
SRC_HAND, TGT_HAND = rigs.SRC_HAND, rigs.TGT_HAND[kind]
HIPS = BONE_MAP["Hips"]
TOES = (BONE_MAP["LeftToeBase"], BONE_MAP["RightToeBase"])

existing = {o.name for o in bpy.data.objects}
bpy.ops.import_anim.bvh(filepath=BVH_PATH, global_scale=1.0, rotate_mode="NATIVE",
                        update_scene_fps=True, update_scene_duration=True)
source = next(o for o in bpy.data.objects
              if o.type == "ARMATURE" and o.name not in existing)

action = source.animation_data.action
frame_start = int(min(action.frame_range))
frame_end = int(max(action.frame_range))
scene = bpy.context.scene

heights = []
for index in range(frame_start, frame_end + 1):
    scene.frame_set(index)
    heights.append(source.pose.bones["Hips"].matrix.translation.z)
src_hip_height = sum(heights) / len(heights)
tgt_hip_rest = target.data.bones[HIPS].matrix_local.translation.z
scale = tgt_hip_rest / src_hip_height
print(f"RESULT src_hip={src_hip_height:.4f} tgt_hip={tgt_hip_rest:.4f} scale={scale:.4f}")

for bone in target.pose.bones:
    bone.rotation_mode = "QUATERNION"

ordered = []


def walk(bone):
    ordered.append(bone.name)
    for child in bone.children:
        walk(child)


for root in target.data.bones:
    if root.parent is None:
        walk(root)

KEYED = set(BONE_MAP.values())
src_rest = {n: source.data.bones[n].matrix_local.to_3x3() for n in BONE_MAP}
tgt_rest = {b.name: b.matrix_local for b in target.data.bones}
inverse_map = {v: k for k, v in BONE_MAP.items()}


def rest_head(rig, bone):
    return rig.matrix_world @ rig.data.bones[bone].head_local


UP = Vector((0, 0, 1))
FACE = Vector((0, -1, 0))       # both rigs rest facing -Y in Blender space
FORWARD = Vector((0, -1, 0))    # the clip travels -Y in Blender space


def frame(direction, reference):
    """Orthonormal frame from a bone direction plus a roll reference.

    Aiming the bone alone leaves roll undefined, which is what let the arms
    transfer twisted: forearms rolled over so the palms faced outward. Pinning
    a second axis makes the alignment a full orientation, not just a heading.
    """
    x = direction.normalized()
    perpendicular = reference - x * reference.dot(x)
    z = perpendicular.normalized()
    y = z.cross(x)
    return Matrix((x, y, z)).transposed()


# Rest-pose alignment. Delta transfer maps "source rest" onto "target rest", so
# any pose baked into the target's rest survives into every frame -- a CC rig
# rests with its elbows bent ~54 degrees, which is why a straight-armed source
# still came out bent. Rotate each target bone so its rest segment points where
# the source's does, and apply the delta on top of that instead.
align = {}
for name in CHILD:
    if name in SRC_HAND and all(b in names for b in TGT_HAND[name]):
        src_mid, src_thumb = SRC_HAND[name]
        tgt_mid, tgt_thumb = TGT_HAND[name]
        src_wrist, tgt_wrist = rest_head(source, name), rest_head(target, BONE_MAP[name])
        align[name] = (frame(rest_head(source, src_mid) - src_wrist,
                             rest_head(source, src_thumb) - src_wrist)
                       @ frame(rest_head(target, tgt_mid) - tgt_wrist,
                               rest_head(target, tgt_thumb) - tgt_wrist).inverted())
        continue

    child = CHILD[name]
    if child is None:
        # Head and toe have no segment of their own to aim; keep them consistent
        # with the limb they cap.
        align[name] = align[PARENT_OF[name]]
        continue
    src_dir = (rest_head(source, child) - rest_head(source, name)).normalized()
    tgt_dir = (rest_head(target, BONE_MAP[child])
               - rest_head(target, BONE_MAP[name])).normalized()
    # Legs and spine run parallel to world up, where up is useless as a roll
    # reference; those take the facing direction instead. The source picks for
    # both rigs so the two frames are always built the same way.
    reference = FACE if abs(src_dir.dot(UP)) > 0.85 else UP
    align[name] = frame(src_dir, reference) @ frame(tgt_dir, reference).inverted()

worst_rest = max(align, key=lambda n: align[n].to_quaternion().angle)
print(f"RESULT largest_rest_correction={worst_rest} "
      f"{math.degrees(align[worst_rest].to_quaternion().angle):.1f}deg")


# "Outward" for each arm, taken from the rest shoulders rather than assumed to
# be a world axis, so no chirality convention has to be right for this to work.
OUTWARD = {}
_left_shoulder = rest_head(source, "LeftArm")
_right_shoulder = rest_head(source, "RightArm")
OUTWARD["Left"] = (_left_shoulder - _right_shoulder).normalized()
OUTWARD["Right"] = -OUTWARD["Left"]


def arm_chain(radians):
    """One shared rotation for every bone of each arm, for the current frame.

    The whole chain takes the same rotation so the limb swings out rigidly
    about the shoulder; applying it to the upper arm alone would leave the
    forearm behind and break the elbow, since every bone here is posed in
    absolute terms.

    The axis is `arm direction x outward`, recomputed per frame, NOT a fixed
    world axis. Rotating about the fixed travel axis was the first attempt and
    it is wrong: when the arm is swung forward or back -- exactly the frames
    that collide -- the limb lies along that axis, so the rotation degenerates
    into a roll and moves the hand nowhere. That is why opening the shoulders
    plateaued at 60/90 frames and then got *worse* at 20 degrees. Rotating
    about `d x outward` moves the hand along the component of `outward`
    perpendicular to the arm, which is the most displacement any rigid
    rotation about the shoulder can give, at every phase of the swing.

    Call this after scene.frame_set; it reads the source's current pose.
    """
    if not radians:
        return {}
    out = {}
    for side in ("Left", "Right"):
        shoulder = source.pose.bones[side + "Arm"].matrix.translation
        hand = source.pose.bones[side + "Hand"].matrix.translation
        direction = (hand - shoulder).normalized()
        axis = direction.cross(OUTWARD[side])
        if axis.length < 1e-6:      # already pointing straight out; nothing to open
            continue
        swing = Matrix.Rotation(radians, 3, axis.normalized())
        for part in ("Arm", "ForeArm", "Hand"):
            out[side + part] = swing
    return out


def bake(name, z_offset, level, radians):
    target.animation_data_clear()
    target.animation_data_create()
    clip = bpy.data.actions.new(name)
    target.animation_data.action = clip
    if hasattr(clip, "slots"):
        target.animation_data.action_slot = clip.slots.new(id_type="OBJECT", name=name)

    for index in range(frame_start, frame_end + 1):
        scene.frame_set(index)
        # Per frame: the opening axis follows the arm, so it has to be rebuilt
        # once the source pose for this frame is loaded.
        chain = arm_chain(radians)
        posed = {}
        for bone_name in ordered:
            bone = target.data.bones[bone_name]
            rest = tgt_rest[bone_name]
            if bone.parent is None:
                inherited = rest.copy()
            else:
                inherited = (posed[bone.parent.name]
                             @ tgt_rest[bone.parent.name].inverted() @ rest)

            source_name = inverse_map.get(bone_name)
            if source_name:
                src_pose = source.pose.bones[source_name].matrix.to_3x3()
                delta = src_pose @ src_rest[source_name].inverted()
                if source_name == "Head":
                    delta = level @ delta
                elif source_name in chain:
                    delta = chain[source_name] @ delta
                rotation = (delta @ align[source_name] @ rest.to_3x3()).normalized()
            else:
                rotation = inherited.to_3x3().normalized()

            location = inherited.translation.copy()
            if bone_name == HIPS:
                location = source.pose.bones["Hips"].matrix.translation * scale
                location.z += z_offset

            desired = Matrix.Translation(location) @ rotation.to_4x4()
            posed[bone_name] = desired
            pose_bone = target.pose.bones[bone_name]
            pose_bone.matrix_basis = inherited.inverted() @ desired
            if bone_name in KEYED:
                pose_bone.keyframe_insert("rotation_quaternion", frame=index)
                if bone_name == HIPS:
                    pose_bone.keyframe_insert("location", frame=index)
    return clip


def lowest_toe():
    low = None
    for index in range(frame_start, frame_end + 1):
        scene.frame_set(index)
        for toe in TOES:
            z = (target.matrix_world @ target.pose.bones[toe].head).z
            low = z if low is None else min(low, z)
    return low


probe = bake("Probe", 0.0, Matrix.Identity(3), 0.0)
rest_toe = min((target.matrix_world @ target.data.bones[t].head_local).z for t in TOES)
drop = lowest_toe() - rest_toe

# kimodo walks its characters with the chin up -- about 14 degrees above the
# horizon for the whole cycle, which is in the generated data, not the
# transfer. Measure the gaze on the posed target (the only place the alignment
# and the delta are both already applied) and cancel the constant part, leaving
# the frame-to-frame head motion intact.
HEAD = BONE_MAP["Head"]
head_rest = target.data.bones[HEAD].matrix_local.to_3x3()
gaze = []
for index in range(frame_start, frame_end + 1):
    scene.frame_set(index)
    aimed = (target.pose.bones[HEAD].matrix.to_3x3() @ head_rest.inverted()) @ FORWARD
    gaze.append(math.asin(max(-1.0, min(1.0, aimed.z))))
head_bias = sum(gaze) / len(gaze)
print(f"RESULT ground_drop={drop:.4f} rest_toe={rest_toe:.4f}")
print(f"RESULT head_bias={math.degrees(head_bias):.1f}deg (levelled)")

level_matrix = Matrix.Rotation(head_bias, 3, "X")
bpy.data.actions.remove(probe)


def bake_final(radians):
    previous = target.animation_data.action if target.animation_data else None
    clip = bake("WalkKimodo", -drop, level_matrix, radians)
    if previous is not None and previous is not clip:
        bpy.data.actions.remove(previous)
    return clip


def clean_count(quiet=True):
    """Frames where the arm stays within MAX_DEPTH of the body surface.

    Solving to zero contact is the wrong target twice over: a sub-millimetre
    graze is invisible, and contacts with runtime-simulated cloth are measured
    against a rest pose the cloth will not be in, so chasing those would open
    the arms to clear something that has already swung away."""
    per_frame, _ = scan(target, frame_start, frame_end, quiet=quiet)
    return (sum(1 for _, depth, _ in per_frame if depth <= MAX_DEPTH),
            len(per_frame))


if AUTO:
    # Ascending search: take the smallest opening that clears, so the pose is
    # disturbed as little as the target's proportions allow.
    coarse = list(range(0, MAX_ABDUCT + 1, COARSE_STEP))
    chosen, best = None, (-1, 0)
    for degrees in coarse:
        bake_final(math.radians(degrees))
        clean, total = clean_count(quiet=degrees != coarse[0])
        print(f"RESULT sweep abduct={degrees}deg clean={clean}/{total}")
        best = max(best, (clean, degrees))
        if clean == total:
            chosen = degrees
            break
    if chosen is None:
        # Save the least-bad attempt before failing. A gate that leaves nothing
        # to look at just moves the diagnosis to the next session; the frames
        # that still hit are exactly what has to be judged by eye.
        bake_final(math.radians(best[1]))
        Path(OUT_BLEND).parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=OUT_BLEND)
        raise RuntimeError(
            f"no shoulder opening up to {MAX_ABDUCT}deg keeps the arms within "
            f"{MAX_DEPTH * 1000:.0f}mm of the body -- the best was {best[1]}deg "
            f"at {best[0]}/{frame_end - frame_start + 1} frames, saved to "
            f"{OUT_BLEND} for inspection. Look at the frames before doing more: "
            f"if they read as a graze the limit is too tight for this "
            f"character, and if they read as a defect the motion needs a "
            f"shorter arm swing or a collision-aware IK pass.")
    # Refine downwards a degree at a time: the coarse step brackets the answer
    # but usually overshoots, and every extra degree shows in the pose.
    for degrees in range(chosen - COARSE_STEP + 1, chosen):
        if degrees <= 0:
            continue
        bake_final(math.radians(degrees))
        clean, total = clean_count()
        print(f"RESULT refine abduct={degrees}deg clean={clean}/{total}")
        if clean == total:
            chosen = degrees
            break
    ABDUCT = math.radians(chosen)
    print(f"RESULT abduct_solved={chosen}deg")

bake_final(ABDUCT)
if ABDUCT:
    print(f"RESULT abduction={math.degrees(ABDUCT):.1f}deg")

scene.frame_start, scene.frame_end = frame_start, frame_end

# Verify every mapped segment against the source rather than spot-checking:
# after rest alignment each target segment should point exactly where the
# source's does, so any non-trivial angle here is a transfer bug. The arm chain
# carries the abduction on purpose, so it is allowed exactly that much.
segments = [(n, c) for n, c in CHILD.items() if c]
worst_dir, worst_roll = {}, {}
for index in range(frame_start, frame_end + 1):
    scene.frame_set(index)
    for name, child in segments:
        src_dir = (source.pose.bones[child].matrix.translation
                   - source.pose.bones[name].matrix.translation).normalized()
        tgt_dir = ((target.matrix_world @ target.pose.bones[BONE_MAP[child]].head)
                   - (target.matrix_world
                      @ target.pose.bones[BONE_MAP[name]].head)).normalized()
        worst_dir[name] = max(worst_dir.get(name, 0.0),
                              math.degrees(src_dir.angle(tgt_dir, 0.0)))
        # Roll is invisible to a direction check -- that is how twisted arms got
        # through. Undo the target's rest alignment and compare full orientation
        # against the source's bone.
        src_rot = source.pose.bones[name].matrix.to_3x3().normalized()
        undo = (align[name] @ tgt_rest[BONE_MAP[name]].to_3x3()).inverted()
        tgt_rot = (target.pose.bones[BONE_MAP[name]].matrix.to_3x3()
                   @ undo @ src_rest[name]).normalized()
        angle = (src_rot.inverted() @ tgt_rot).to_quaternion().angle
        worst_roll[name] = max(worst_roll.get(name, 0.0),
                               math.degrees(min(angle, 2 * math.pi - angle)))

# The arm chain is deliberately offset by the abduction; everything else
# must match the source exactly.
ARM_BONES = {s + p for s in ("Left", "Right") for p in ("Arm", "ForeArm", "Hand")}
offset = {n: math.degrees(ABDUCT) for n in ARM_BONES} if ABDUCT else {}
failures = []
for name in sorted(worst_roll, key=lambda n: -worst_roll[n]):
    allowed = offset.get(name, 0.0) + 1.0
    bad = max(worst_dir[name], worst_roll[name]) > allowed and name != "Neck1"
    if bad:
        failures.append(name)
    print(f"RESULT verify {name:14s} dir={worst_dir[name]:5.1f} "
          f"orient={worst_roll[name]:5.1f}deg{'  <-- CHECK' if bad else ''}")

# Hands separately: they are leaves, so the segment check above never sees them.
for name in SRC_HAND:
    src_mid, src_thumb = SRC_HAND[name]
    tgt_mid, tgt_thumb = TGT_HAND[name]
    worst = 0.0
    for index in range(frame_start, frame_end + 1):
        scene.frame_set(index)
        wrist = source.pose.bones[name].matrix.translation
        src_frame = frame(source.pose.bones[src_mid].matrix.translation - wrist,
                          source.pose.bones[src_thumb].matrix.translation - wrist)
        twrist = target.matrix_world @ target.pose.bones[BONE_MAP[name]].head
        tgt_frame = frame((target.matrix_world @ target.pose.bones[tgt_mid].head) - twrist,
                          (target.matrix_world @ target.pose.bones[tgt_thumb].head) - twrist)
        angle = (src_frame.inverted() @ tgt_frame).to_quaternion().angle
        worst = max(worst, math.degrees(min(angle, 2 * math.pi - angle)))
    limit = math.degrees(ABDUCT) + 1.0
    if worst > limit:
        failures.append(name)
    print(f"RESULT verify {name:14s} palm_orient={worst:5.1f}deg"
          f"{'  <-- CHECK' if worst > limit else ''}")

bpy.data.objects.remove(source, do_unlink=True)

Path(OUT_BLEND).parent.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT_BLEND)
print("RESULT saved=" + OUT_BLEND)

if failures:
    raise RuntimeError(f"rotation transfer is off on {failures}; "
                       f"the clip was saved so the poses can be inspected")
print("RETARGET OK")
