def _sides(template_l: dict) -> dict:
    """{'DEF-x.L': 'LeftY'} 템플릿을 좌우 양쪽으로 확장한다 (정직한 매핑).

    Blender .L 본(해부학적 왼쪽, 월드 +X) → Unity 'Left*',
    Blender .R 본(월드 −X)               → Unity 'Right*'.

    RESOLVED (2026-07-18): 예전엔 여기서 L/R을 스왑해 .L → 'Right*'로 부여했다. 근거였던
    "FBX X-미러"는 실재하지 않았다. Blender→Unity FBX는 손대칭(chirality)을 보존한다:
    Blender 해부학적 왼쪽(+X)은 Unity −X로 가는데, 왼손 좌표계인 Unity에서 +Z를 바라보는
    캐릭터의 '왼쪽'이 바로 −X 이다(오른쪽 = +X). 따라서 정직한 매핑이 옳다. 스왑은
    faces_plus_z의 손 검사(당시 lh.x>rh.x — Unity 왼손을 +X로 가정한 방향 오류)를
    통과시키려 넣은 보상이었을 뿐이고, 좌우대칭 더미라 잘못된 이론이 검증을 통과했다.
    비대칭 마커 실험(config 매트릭스 A/B/C)으로, 스왑 없이도 Unity −X(왼쪽)에 Left* 본이
    온다는 것을 확인했다. 04_export.py의 pre-rotate 트릭과는 무관(그건 루트 회전만 담당).
    상세: .superpowers/sdd/lr-swap-investigation.md"""
    out = {}
    for src, dst in template_l.items():
        out[src] = dst                                             # DEF-x.L → Left*
        out[src.replace(".L", ".R")] = dst.replace("Left", "Right")  # DEF-x.R → Right*
    return out


_SPINE = {
    "DEF-spine": "Hips",
    "DEF-spine.001": "Spine",
    "DEF-spine.002": "Chest",
    "DEF-spine.003": "UpperChest",
    "DEF-spine.004": "Neck",
    "DEF-spine.005": "Head",        # Neck 다음 본을 Head로 (Unity 매퍼가 Neck 다음을 Head로 집음)
    "DEF-spine.006": "HeadTop",     # 여분 leaf (Unity 미매핑; 이전엔 이 본이 Head라 off-by-one 발생)
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

_FINGER_UNITY_NAMES = [
    f"{side}{finger}{seg}"
    for side in ("Left", "Right")
    for finger in ("Thumb", "Index", "Middle", "Ring", "Little")
    for seg in ("Proximal", "Intermediate", "Distal")
]

OPTIONAL_HUMAN_BONES = {
    n: n for n in [
        "Chest", "UpperChest", "Neck",
        "LeftShoulder", "RightShoulder", "LeftToes", "RightToes",
    ] + _FINGER_UNITY_NAMES
}
