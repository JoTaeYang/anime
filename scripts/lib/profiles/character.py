from lib.bone_map import BONE_RENAME
from lib.profiles.character_landmarks import LANDMARKS as _LM

NAME = "character"
MESH_OBJECT = "Character"
SOURCE = {"kind": "blend", "path": "assets/character.blend", "object": "Mesh_0"}
LANDMARKS = _LM
METARIG_SCALE = None          # 랜드마크 직접 배치
HIPS_HEIGHT = 1.00            # Task 6 수렴: spine.head[2] 실측 골반 높이
HIPS_TOL = 0.12
HEIGHT_RANGE = (1.85, 1.95)
FBX_NAME = "character.fbx"
USE_LATERALITY_MARKER = False

# 부속물 체인 초안. 좌표는 previews/char_front_negY.png, previews/char_side.png 관찰 기반.
# Blender 좌표계: 캐릭터는 -Y를 바라본다 (front = -Y, back = +Y), 해부학적 왼쪽 = +X.
# 좌표는 초안이며 Task 6의 렌더 루프에서 시각적으로 보정한다.
APPENDAGES = [
    # 꼬리: 천골(spine, z≈1.0)에서 뒤(+Y)로 나와 아래로 늘어지는 큰 복슬 꼬리.
    # 실측: 꼬리 부피 z0.44-1.10, 뒤 최대 y≈+0.40. 위로 솟지 않고 뒤-아래로 흐른다.
    {"chain": "tail", "rigify_type": "spines.basic_tail", "parent": "spine",
     "points": [[0.0, 0.12, 0.98], [0.0, 0.24, 0.92], [0.0, 0.32, 0.80], [0.0, 0.36, 0.66], [0.0, 0.34, 0.54], [0.0, 0.28, 0.44]],
     "rename": {"DEF-tail": "Tail1", "DEF-tail.001": "Tail2", "DEF-tail.002": "Tail3", "DEF-tail.003": "Tail4", "DEF-tail.004": "Tail5"}},

    # 치마: 골반(spine) 둘레 45도 간격 8체인, 각 2본(허리 부착점 -> 중간 -> 뾰족한 하단 자락).
    # parent 배정: 중앙(x=0) 2체인만 spine에 남기고, 좌/우 6체인은 pelvis.L/R로 옮긴다.
    # 이유: 익스포트된 Hips가 8개 스커트 체인 루트를 전부 자식으로 떠안으면(+다리2+spine+
    # tail+골반여분2 = ~14) Unity Humanoid 오토매퍼가 형제 수에 압도되어 역할을
    # 한 칸씩 밀어 배정한다(Spine 역할<-Chest 본, Chest 역할<-UpperChest 본, UpperChest
    # 미배정) — 이름은 이미 올바른데도 발생하는 구조적(형제 수) 오류. 좌/우 체인을
    # pelvis.L/R(사람 메타리그에 이미 존재 -> DEF-pelvis.L/R -> LeftPelvis/RightPelvis)에
    # 재부착하면 Hips 자식 수가 ~8로 줄고, pelvis 본은 각 3자식(레그 여분 없음)만 갖게 되어
    # 오토매퍼가 다시 정상 동작할 가능성을 회복시킨다.
    {"chain": "skirt_f", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[0.0, -0.09, 0.88], [0.0, -0.136, 0.64], [0.0, -0.20, 0.44]],
     "rename": {"DEF-skirt_f": "SkirtF1a", "DEF-skirt_f.001": "SkirtF1b"}},
    {"chain": "skirt_fr", "rigify_type": "limbs.simple_tentacle", "parent": "pelvis.R",
     "points": [[-0.089, -0.089, 0.88], [-0.144, -0.144, 0.64], [-0.212, -0.212, 0.44]],
     "rename": {"DEF-skirt_fr": "SkirtFR1a", "DEF-skirt_fr.001": "SkirtFR1b"}},
    {"chain": "skirt_r", "rigify_type": "limbs.simple_tentacle", "parent": "pelvis.R",
     "points": [[-0.168, 0.0, 0.88], [-0.272, 0.0, 0.64], [-0.40, 0.0, 0.44]],
     "rename": {"DEF-skirt_r": "SkirtR1a", "DEF-skirt_r.001": "SkirtR1b"}},
    {"chain": "skirt_br", "rigify_type": "limbs.simple_tentacle", "parent": "pelvis.R",
     "points": [[-0.095, 0.095, 0.88], [-0.154, 0.154, 0.64], [-0.226, 0.226, 0.44]],
     "rename": {"DEF-skirt_br": "SkirtBR1a", "DEF-skirt_br.001": "SkirtBR1b"}},
    {"chain": "skirt_b", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[0.0, 0.101, 0.88], [0.0, 0.163, 0.64], [0.0, 0.24, 0.44]],
     "rename": {"DEF-skirt_b": "SkirtB1a", "DEF-skirt_b.001": "SkirtB1b"}},
    {"chain": "skirt_bl", "rigify_type": "limbs.simple_tentacle", "parent": "pelvis.L",
     "points": [[0.095, 0.095, 0.88], [0.154, 0.154, 0.64], [0.226, 0.226, 0.44]],
     "rename": {"DEF-skirt_bl": "SkirtBL1a", "DEF-skirt_bl.001": "SkirtBL1b"}},
    {"chain": "skirt_l", "rigify_type": "limbs.simple_tentacle", "parent": "pelvis.L",
     "points": [[0.168, 0.0, 0.88], [0.272, 0.0, 0.64], [0.40, 0.0, 0.44]],
     "rename": {"DEF-skirt_l": "SkirtL1a", "DEF-skirt_l.001": "SkirtL1b"}},
    {"chain": "skirt_fl", "rigify_type": "limbs.simple_tentacle", "parent": "pelvis.L",
     "points": [[0.089, -0.089, 0.88], [0.144, -0.144, 0.64], [0.212, -0.212, 0.44]],
     "rename": {"DEF-skirt_fl": "SkirtFL1a", "DEF-skirt_fl.001": "SkirtFL1b"}},

    # 목도리: 목(spine.004) 기준 좌/우 두 갈래, 오른쪽이 더 길게 옆구리 쪽으로 늘어짐.
    {"chain": "scarf_l", "rigify_type": "limbs.simple_tentacle", "parent": "spine.004",
     "points": [[0.09, -0.12, 1.44], [0.11, -0.20, 1.16], [0.12, -0.22, 0.92]],
     "rename": {"DEF-scarf_l": "ScarfL1a", "DEF-scarf_l.001": "ScarfL1b"}},
    {"chain": "scarf_r", "rigify_type": "limbs.simple_tentacle", "parent": "spine.004",
     "points": [[-0.09, -0.12, 1.44], [-0.11, -0.16, 1.08], [-0.13, -0.12, 0.86]],
     "rename": {"DEF-scarf_r": "ScarfR1a", "DEF-scarf_r.001": "ScarfR1b"}},

    # 후드 귀: 머리 위(spine.005) 좌우 코너에서 위-바깥으로 뻗는 고양이 귀.
    {"chain": "hood_ear_l", "rigify_type": "basic.super_copy", "parent": "spine.005",
     "points": [[0.15, -0.05, 1.70], [0.17, -0.12, 1.85]],
     "rename": {"DEF-hood_ear_l": "HoodEarL"}},
    {"chain": "hood_ear_r", "rigify_type": "basic.super_copy", "parent": "spine.005",
     "points": [[-0.15, -0.05, 1.70], [-0.17, -0.12, 1.85]],
     "rename": {"DEF-hood_ear_r": "HoodEarR"}},
]

EXTRA_POSES = [
    {"name": "tail_lift", "view": "side",
     "ops": [("rotate", "Tail1", 'X', -40), ("rotate", "Tail2", 'X', -25), ("rotate", "Tail3", 'X', -15)]},
    {"name": "leg_spread", "view": "front",
     "ops": [("rotate", "LeftUpperLeg", 'Y', -35), ("rotate", "RightUpperLeg", 'Y', 35)]},
]


def appendage_bone_rename():
    out = {}
    for app in APPENDAGES:
        out.update(app["rename"])
    return out


def bone_rename():
    return {**BONE_RENAME, **appendage_bone_rename()}


def expected_def_bones():
    return set(bone_rename().keys())


_r = appendage_bone_rename()
assert len(_r) == len(set(_r.values())), "appendage rename collision"
assert not (set(_r.values()) & set(BONE_RENAME.values())), "appendage name collides with body"
