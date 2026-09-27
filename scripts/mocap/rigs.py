"""Source (SOMA-30) to target-rig bone mapping, shared by retarget and checks.

SOMA calls the thigh "LeftLeg" and the calf "LeftShin". Every target rig names
those two differently again, so all three mappings are written out by hand -- a
name-matching retarget would put thigh motion on the shin.
"""

MAPS = {
    # Unity Humanoid names, as this project's Rigify export produces.
    "unity": {
        "Hips": "Hips", "Spine1": "Spine", "Spine2": "Chest", "Chest": "UpperChest",
        "Neck1": "Neck", "Head": "Head",
        "LeftShoulder": "LeftShoulder", "LeftArm": "LeftUpperArm",
        "LeftForeArm": "LeftLowerArm", "LeftHand": "LeftHand",
        "LeftLeg": "LeftUpperLeg", "LeftShin": "LeftLowerLeg",
        "LeftFoot": "LeftFoot", "LeftToeBase": "LeftToes",
        "RightShoulder": "RightShoulder", "RightArm": "RightUpperArm",
        "RightForeArm": "RightLowerArm", "RightHand": "RightHand",
        "RightLeg": "RightUpperLeg", "RightShin": "RightLowerLeg",
        "RightFoot": "RightFoot", "RightToeBase": "RightToes",
    },
    # Reallusion Character Creator 3/4.
    "cc": {
        "Hips": "Hip", "Spine1": "Waist", "Spine2": "Spine01", "Chest": "Spine02",
        "Neck1": "NeckTwist01", "Head": "Head",
        "LeftShoulder": "L_Clavicle", "LeftArm": "L_Upperarm",
        "LeftForeArm": "L_Forearm", "LeftHand": "L_Hand",
        "LeftLeg": "L_Thigh", "LeftShin": "L_Calf",
        "LeftFoot": "L_Foot", "LeftToeBase": "L_ToeBase",
        "RightShoulder": "R_Clavicle", "RightArm": "R_Upperarm",
        "RightForeArm": "R_Forearm", "RightHand": "R_Hand",
        "RightLeg": "R_Thigh", "RightShin": "R_Calf",
        "RightFoot": "R_Foot", "RightToeBase": "R_ToeBase",
    },
}

# Source-skeleton parent/child chain, used to aim each bone during rest-pose
# alignment. A leaf (Head, ToeBase, Hand) has no segment of its own.
CHILD = {
    "Hips": "Spine1", "Spine1": "Spine2", "Spine2": "Chest", "Chest": "Neck1",
    "Neck1": "Head", "Head": None,
    "LeftShoulder": "LeftArm", "LeftArm": "LeftForeArm",
    "LeftForeArm": "LeftHand", "LeftHand": None,
    "RightShoulder": "RightArm", "RightArm": "RightForeArm",
    "RightForeArm": "RightHand", "RightHand": None,
    "LeftLeg": "LeftShin", "LeftShin": "LeftFoot", "LeftFoot": "LeftToeBase",
    "LeftToeBase": None,
    "RightLeg": "RightShin", "RightShin": "RightFoot", "RightFoot": "RightToeBase",
    "RightToeBase": None,
}
PARENT_OF = {child: parent for parent, child in CHILD.items() if child}

# A hand has no single bone to aim, and inheriting the forearm's alignment
# leaves the palm rolled wherever the two rigs' wrists happen to differ. The
# middle finger gives direction, the thumb pins the palm plane.
SRC_HAND = {"LeftHand": ("LeftHandMiddleEnd", "LeftHandThumbEnd"),
            "RightHand": ("RightHandMiddleEnd", "RightHandThumbEnd")}
TGT_HAND = {
    "unity": {"LeftHand": ("LeftMiddleProximal", "LeftThumbProximal"),
              "RightHand": ("RightMiddleProximal", "RightThumbProximal")},
    "cc": {"LeftHand": ("L_Mid1", "L_Thumb1"),
           "RightHand": ("R_Mid1", "R_Thumb1")},
}


def detect(armature):
    """Pick the bone map whose names the armature actually carries.

    Returns (kind, bone_map). Raises if the best match is still incomplete --
    a partial map would silently leave limbs unanimated.
    """
    names = {bone.name for bone in armature.data.bones}
    kind = max(MAPS, key=lambda k: len(names & set(MAPS[k].values())))
    bone_map = MAPS[kind]
    missing = sorted(set(bone_map.values()) - names)
    if missing:
        raise RuntimeError(
            f"rig looks like '{kind}' but is missing mapped bones: {missing}. "
            f"Add a map to scripts/mocap/rigs.py for this skeleton.")
    return kind, bone_map
