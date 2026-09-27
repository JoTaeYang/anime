"""Arm-through-torso detection for a retargeted clip.

Why this exists: joint-angle verification says a retarget transferred the
rotations correctly, and says nothing at all about whether the result looks
right, because the source skeleton has no volume. A rotation-only retarget
cannot know the target has shorter arms or a fuller torso than the body the
motion was generated for, so the hands swing into the hips. That defect passed
angle checks on 54% of the frames of the first CC walk.

What is gated is DEPTH, not whether an intersection exists. Face-pair counts
say an intersection is present, never whether anyone would see it, and the two
cases need opposite decisions. Measured on the same clip and rig family:

    CC character, no correction (the defect a human spotted on sight)
        39/90 frames, max 25.1mm, mean 8.9mm
    our character, 8 degrees of opening
        14/90 frames, max  3.8mm, mean 1.8mm

An order of magnitude apart. 25mm on a 1.4m body reads as a hand inside a hip;
3.8mm on a 1.9m body is a graze nobody can see. MAX_DEPTH sits in that gap.
Gating on "any intersecting face" failed the second clip, which is wrong.

Depth is signed against the nearest body face's normal, not by ray parity: the
skin is not a closed surface, and parity on open geometry is what produced 69cm
readings on a 1.4m character.

Method: face-pair overlap (BVHTree.overlap). Two earlier approaches measured
nothing useful and are recorded here so they are not tried again --
  * sampling the bone centreline misses flesh that overlaps while the bone
    does not, and a bone is always inside its own limb;
  * ray-parity "is this point inside" needs a closed surface, and a shirt or a
    pair of shorts is an open tube. That is what produced 69cm "depths" on a
    1.4m character.
Face-pair overlap assumes nothing about closedness and reports exactly what
renders as poke-through.
"""


# Deepest the arm may sit inside the body before it counts as a defect. See the
# calibration table in the module docstring for where this number comes from.
MAX_DEPTH = 0.005


def _dominant_group(vertex, group_names):
    best = None
    for item in vertex.groups:
        if best is None or item.weight > best.weight:
            best = item
    return group_names.get(best.group, "") if best else ""


def simulated_bones(armature):
    """Bones a runtime simulation drives, which the clip does not animate.

    Our character's skirt, scarf and tail are spring bones in Unity: the
    Humanoid clip carries no curves for them at all, so in Blender they sit in
    their rest pose for the whole clip. Testing an arm against that pose asks
    whether the hand passes through cloth *that will not be there* -- at
    runtime the skirt has swung. So these are measured and reported, never
    gated. The chain list comes from the active profile, which is where the
    project already declares them; a rig with no such profile gets an empty
    set and nothing changes.

    Note what this does NOT claim: that an arm through the skirt is fine. It
    claims only that this gate, posing cloth at rest, cannot judge it. Whether
    the arm needs to be a collider for the spring chains is a runtime question
    for the spring-bone setup.
    """
    try:
        from lib.profiles import get_profile
        rename = get_profile().appendage_bone_rename()
    except (ImportError, AttributeError):
        return set()
    return set(rename.values()) & {bone.name for bone in armature.data.bones}


def classify_bones(armature, bone_map, simulated=frozenset()):
    """Split the skeleton into arm-side and body-side bone names.

    Derived from the hierarchy rather than from name prefixes, so it works on
    any rig the bone map covers.

    Arm means forearm downwards, not the whole limb. The deltoid and the
    armpit rest against the ribs on every frame of every pose, so including
    the upper arm reports 90/90 frames intersecting on a clip that renders
    clean. Everything from the clavicle to the elbow is therefore neutral --
    in neither set -- which also leaves a gap band at the elbow, the same
    trick that keeps the armpit from firing in partition_faces.
    """
    bones = armature.data.bones

    def subtree(root):
        return {root} | {bone.name for bone in bones[root].children_recursive}

    arm = subtree(bone_map["LeftForeArm"]) | subtree(bone_map["RightForeArm"])
    shoulders = subtree(bone_map["LeftShoulder"]) | subtree(bone_map["RightShoulder"])
    body = {bone.name for bone in bones} - shoulders - set(simulated)
    return arm, body, set(simulated)


def partition_faces(mesh_object, arm_bones, body_bones, simulated_bone_names=frozenset()):
    """Faces whose every vertex is arm-, body-, or simulation-weighted.

    Faces that straddle a boundary fall into no set. That gap band is
    load-bearing: without it the armpit, where arm and torso surfaces are one
    continuous mesh, would register as a permanent intersection.
    """
    group_names = {group.index: group.name for group in mesh_object.vertex_groups}
    label = {v.index: _dominant_group(v, group_names)
             for v in mesh_object.data.vertices}
    arm_faces, body_faces, soft_faces = [], [], []
    for polygon in mesh_object.data.polygons:
        labels = [label[i] for i in polygon.vertices]
        if all(name in arm_bones for name in labels):
            arm_faces.append(tuple(polygon.vertices))
        elif all(name in body_bones for name in labels):
            body_faces.append(tuple(polygon.vertices))
        elif all(name in simulated_bone_names for name in labels):
            soft_faces.append(tuple(polygon.vertices))
    return arm_faces, body_faces, soft_faces


def find_meshes(armature, arm_bones, body_bones):
    """(skin, garments) among the meshes deformed by this armature.

    Skin is the mesh weighted to the most of the skeleton. A garment is any
    other deformed mesh that is mostly body-side -- which keeps a shirt and a
    pair of shorts as obstacles while dropping a bracelet, whose every vertex
    rides the forearm and would otherwise read as a permanent hit.
    """
    import bpy

    deformed = [obj for obj in bpy.data.objects
                if obj.type == "MESH" and obj.vertex_groups
                and any(m.type == "ARMATURE" and m.object is armature
                        for m in obj.modifiers)]
    if not deformed:
        raise RuntimeError("no meshes are deformed by the armature; "
                           "the penetration check needs the character's skin")

    skin = max(deformed, key=lambda o: len({g.name for g in o.vertex_groups}
                                           & (arm_bones | body_bones)))
    garments = []
    for obj in deformed:
        if obj is skin:
            continue
        names = {group.index: group.name for group in obj.vertex_groups}
        counts = {"arm": 0, "body": 0}
        for vertex in obj.data.vertices:
            name = _dominant_group(vertex, names)
            if name in arm_bones:
                counts["arm"] += 1
            elif name in body_bones:
                counts["body"] += 1
        if counts["body"] > counts["arm"]:
            garments.append(obj)
    return skin, garments


def scan(armature, start, end, quiet=False):
    """Per-frame count of arm/body face intersections.

    Returns (per_frame, by_obstacle) where per_frame is a list of
    (frame, depth_metres, soft_hits) and by_obstacle maps an obstacle name to
    {"frames": n, "max": worst face-pair count}. Soft hits are contacts with
    runtime-simulated cloth, counted separately and never gated -- see
    simulated_bones().
    """
    import bpy
    from mathutils.bvhtree import BVHTree

    from .rigs import detect

    _, bone_map = detect(armature)
    simulated = simulated_bones(armature)
    arm_bones, body_bones, soft_bones = classify_bones(armature, bone_map, simulated)
    skin, garments = find_meshes(armature, arm_bones, body_bones)
    arm_faces, body_faces, soft_faces = partition_faces(
        skin, arm_bones, body_bones, soft_bones)
    if not quiet:
        print(f"RESULT skin={skin.name} garments={[g.name for g in garments]}")
        print(f"RESULT arm_faces={len(arm_faces)} body_faces={len(body_faces)} "
              f"simulated_faces={len(soft_faces)} "
              f"simulated_bones={len(soft_bones)}")
    if not arm_faces or not body_faces:
        raise RuntimeError(
            f"face partition degenerate (arm={len(arm_faces)}, "
            f"body={len(body_faces)}); the skin is not weighted to the mapped bones")

    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    per_frame, by_obstacle = [], {}

    def tree_of(obj):
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        matrix = obj.matrix_world
        tree = BVHTree.FromPolygons(
            [matrix @ v.co for v in mesh.vertices],
            [tuple(p.vertices) for p in mesh.polygons],
            all_triangles=False, epsilon=0.0)
        evaluated.to_mesh_clear()
        return tree

    for frame in range(start, end + 1):
        scene.frame_set(frame)
        evaluated = skin.evaluated_get(depsgraph)
        deformed = evaluated.to_mesh()
        matrix = skin.matrix_world
        vertices = [matrix @ v.co for v in deformed.vertices]
        arm_tree = BVHTree.FromPolygons(vertices, arm_faces,
                                        all_triangles=False, epsilon=0.0)
        body_tree = BVHTree.FromPolygons(vertices, body_faces,
                                         all_triangles=False, epsilon=0.0)
        soft_tree = BVHTree.FromPolygons(vertices, soft_faces,
                                         all_triangles=False,
                                         epsilon=0.0) if soft_faces else None
        evaluated.to_mesh_clear()

        obstacles = [(skin.name, body_tree)]
        obstacles += [(garment.name, tree_of(garment)) for garment in garments]

        depth = 0.0
        for name, tree in obstacles:
            pairs = arm_tree.overlap(tree)
            if not pairs:
                continue
            entry = by_obstacle.setdefault(name, {"frames": 0, "max": 0})
            entry["frames"] += 1
            entry["max"] = max(entry["max"], len(pairs))
            if tree is not body_tree:
                # Depth is measured against the body only. Skin is *supposed*
                # to sit inside clothing, so "behind the garment surface" is
                # not a defect signal; a garment crossing is reported by face
                # count and judged by eye.
                continue
            # Only vertices of faces that actually intersect can be inside;
            # testing every arm vertex each frame would dominate the runtime.
            suspect = {i for arm_index, _ in pairs for i in arm_faces[arm_index]}
            for index in suspect:
                point = vertices[index]
                location, normal, _, _ = tree.find_nearest(point)
                if location is None:
                    continue
                signed = (point - location).dot(normal)
                if signed < 0:              # behind the outward-facing surface
                    depth = max(depth, -signed)
        soft = len(arm_tree.overlap(soft_tree)) if soft_tree else 0
        per_frame.append((frame, depth, soft))

    return per_frame, by_obstacle
