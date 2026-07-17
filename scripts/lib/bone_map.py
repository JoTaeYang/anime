def _sides(template_l: dict) -> dict:
    """{'DEF-x.L': 'LeftY'} 템플릿을 좌우 양쪽으로 확장한다.

    좌우 스왑: Blender(오른손 좌표계)를 Unity(왼손 좌표계)로 FBX 내보내면 X축이
    반사(mirror)된다 — 이는 축 프리셋으로 되돌릴 수 없다(정방향을 뒤집지 않는 한).
    Unity 휴머노이드 매퍼는 본 이름을 우선 사용하므로, Blender의 .L 본(월드 +X)을
    Unity 이름 'Right*'로, .R 본을 'Left*'로 부여해 미러를 상쇄한다. 더미는 좌우
    대칭이라 지오메트리상 문제 없음. (Task 9: faces_plus_z 교정)

    PHASE 1 BLOCKER: 이 L/R 스왑은 04_export.py의 사전 회전(pre-rotate) 트릭과 결합된
    보상일 가능성이 크다. 비대칭 캐릭터 반입 전, 비대칭 테스트 더미로 스왑 없이
    익스포트 설정만으로 미러 없는 정상 좌우가 나오는지 재검증할 것. (04_export.py 참조)"""
    out = {}
    for src, dst in template_l.items():
        out[src] = dst.replace("Left", "Right")            # DEF-x.L → Right* (FBX 미러 상쇄)
        out[src.replace(".L", ".R")] = dst                 # DEF-x.R → Left*
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
