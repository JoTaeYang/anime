def _sides(template_l: dict) -> dict:
    """{'DEF-x.L': 'LeftY'} 템플릿을 좌우 양쪽으로 확장한다."""
    out = {}
    for src, dst in template_l.items():
        out[src] = dst
        out[src.replace(".L", ".R")] = dst.replace("Left", "Right")
    return out


_SPINE = {
    "DEF-spine": "Hips",
    "DEF-spine.001": "Spine",
    "DEF-spine.002": "Chest",
    "DEF-spine.003": "UpperChest",
    "DEF-spine.004": "Neck",
    "DEF-spine.005": "NeckUpper",   # 여분 (Unity 미매핑, Neck-Head 사이 중간 본)
    "DEF-spine.006": "Head",
}

_LIMBS_L = {
    "DEF-pelvis.L": "LeftPelvis",             # 여분
    "DEF-shoulder.L": "LeftShoulder",
    "DEF-upper_arm.L": "LeftUpperArm",
    "DEF-upper_arm.L.001": "LeftUpperArmTwist",  # 여분
    "DEF-forearm.L": "LeftLowerArm",
    "DEF-forearm.L.001": "LeftLowerArmTwist",    # 여분
    "DEF-hand.L": "LeftHand",
    "DEF-thigh.L": "LeftUpperLeg",
    "DEF-thigh.L.001": "LeftUpperLegTwist",      # 여분
    "DEF-shin.L": "LeftLowerLeg",
    "DEF-shin.L.001": "LeftLowerLegTwist",       # 여분
    "DEF-foot.L": "LeftFoot",
    "DEF-toe.L": "LeftToes",
    "DEF-palm.01.L": "LeftPalm1",  # 여분
    "DEF-palm.02.L": "LeftPalm2",
    "DEF-palm.03.L": "LeftPalm3",
    "DEF-palm.04.L": "LeftPalm4",
}

_FINGER_SEGS = {"01": "Proximal", "02": "Intermediate", "03": "Distal"}
_FINGER_NAMES = {
    "thumb": "Thumb", "f_index": "Index", "f_middle": "Middle",
    "f_ring": "Ring", "f_pinky": "Little",
}

_FINGERS_L = {
    f"DEF-{rig}.{seg}.L": f"Left{uni}{segname}"
    for rig, uni in _FINGER_NAMES.items()
    for seg, segname in _FINGER_SEGS.items()
}

BONE_RENAME = {**_SPINE, **_sides(_LIMBS_L), **_sides(_FINGERS_L)}

# Unity Humanoid 필수 15본: humanName -> 우리 본 이름 (여기서는 동일 문자열이지만
# Unity 쪽 검증이 이 테이블을 정답지로 쓰므로 명시적으로 둔다)
REQUIRED_HUMAN_BONES = {
    n: n for n in [
        "Hips", "Spine", "Head",
        "LeftUpperLeg", "LeftLowerLeg", "LeftFoot",
        "RightUpperLeg", "RightLowerLeg", "RightFoot",
        "LeftUpperArm", "LeftLowerArm", "LeftHand",
        "RightUpperArm", "RightLowerArm", "RightHand",
    ]
}

OPTIONAL_HUMAN_BONES = {
    n: n for n in [
        "Chest", "UpperChest", "Neck",
        "LeftShoulder", "RightShoulder", "LeftToes", "RightToes",
    ]
}
