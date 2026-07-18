from lib.bone_map import BONE_RENAME
from lib.profiles.character_landmarks import LANDMARKS as _LM

NAME = "character"
MESH_OBJECT = "Character"
SOURCE = {"kind": "blend", "path": "assets/character.blend", "object": "Mesh_0"}
LANDMARKS = _LM
METARIG_SCALE = None          # 랜드마크 직접 배치
HIPS_HEIGHT = 0.4558          # 초안(키 1.899의 24%) — Task 3/6에서 실측값으로 갱신
HIPS_TOL = 0.12
HEIGHT_RANGE = (1.85, 1.95)
FBX_NAME = "character.fbx"
USE_LATERALITY_MARKER = False

# 부속물 체인 초안. 좌표는 previews/char_front_negY.png, previews/char_side.png 관찰 기반.
# Blender 좌표계: 캐릭터는 -Y를 바라본다 (front = -Y, back = +Y), 해부학적 왼쪽 = +X.
# 좌표는 초안이며 Task 6의 렌더 루프에서 시각적으로 보정한다.
APPENDAGES = [
    # 꼬리: 등 뒤(+Y)로 낮은 등에서 위쪽 어깨뼈 높이까지 부풀며 솟는 큰 복슬 꼬리. (브리프 예시 그대로 사용)
    {"chain": "tail", "rigify_type": "spines.basic_tail", "parent": "spine",
     "points": [[0.0, 0.10, 1.00], [0.0, 0.28, 1.02], [0.0, 0.45, 1.08], [0.0, 0.58, 1.18], [0.0, 0.66, 1.30], [0.0, 0.70, 1.42]],
     "rename": {"DEF-tail": "Tail1", "DEF-tail.001": "Tail2", "DEF-tail.002": "Tail3", "DEF-tail.003": "Tail4", "DEF-tail.004": "Tail5"}},

    # 치마: 골반(spine) 둘레 45도 간격 8체인, 각 2본(허리 부착점 -> 중간 -> 뾰족한 하단 자락).
    {"chain": "skirt_f", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[0.0, -0.12, 0.92], [0.0, -0.25, 0.75], [0.0, -0.20, 0.55]],
     "rename": {"DEF-skirt_f": "SkirtF1a", "DEF-skirt_f.001": "SkirtF1b"}},
    {"chain": "skirt_fr", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[-0.0849, -0.0849, 0.92], [-0.1768, -0.1768, 0.75], [-0.1414, -0.1414, 0.55]],
     "rename": {"DEF-skirt_fr": "SkirtFR1a", "DEF-skirt_fr.001": "SkirtFR1b"}},
    {"chain": "skirt_r", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[-0.12, 0.0, 0.92], [-0.25, 0.0, 0.75], [-0.20, 0.0, 0.55]],
     "rename": {"DEF-skirt_r": "SkirtR1a", "DEF-skirt_r.001": "SkirtR1b"}},
    {"chain": "skirt_br", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[-0.0849, 0.0849, 0.92], [-0.1768, 0.1768, 0.75], [-0.1414, 0.1414, 0.55]],
     "rename": {"DEF-skirt_br": "SkirtBR1a", "DEF-skirt_br.001": "SkirtBR1b"}},
    {"chain": "skirt_b", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[0.0, 0.12, 0.92], [0.0, 0.25, 0.75], [0.0, 0.20, 0.55]],
     "rename": {"DEF-skirt_b": "SkirtB1a", "DEF-skirt_b.001": "SkirtB1b"}},
    {"chain": "skirt_bl", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[0.0849, 0.0849, 0.92], [0.1768, 0.1768, 0.75], [0.1414, 0.1414, 0.55]],
     "rename": {"DEF-skirt_bl": "SkirtBL1a", "DEF-skirt_bl.001": "SkirtBL1b"}},
    {"chain": "skirt_l", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[0.12, 0.0, 0.92], [0.25, 0.0, 0.75], [0.20, 0.0, 0.55]],
     "rename": {"DEF-skirt_l": "SkirtL1a", "DEF-skirt_l.001": "SkirtL1b"}},
    {"chain": "skirt_fl", "rigify_type": "limbs.simple_tentacle", "parent": "spine",
     "points": [[0.0849, -0.0849, 0.92], [0.1768, -0.1768, 0.75], [0.1414, -0.1414, 0.55]],
     "rename": {"DEF-skirt_fl": "SkirtFL1a", "DEF-skirt_fl.001": "SkirtFL1b"}},

    # 목도리: 목(spine.004) 기준 좌/우 두 갈래, 오른쪽이 더 길게 옆구리 쪽으로 늘어짐.
    {"chain": "scarf_l", "rigify_type": "limbs.simple_tentacle", "parent": "spine.004",
     "points": [[0.08, -0.08, 1.40], [0.10, -0.12, 1.22], [0.11, -0.13, 1.05]],
     "rename": {"DEF-scarf_l": "ScarfL1a", "DEF-scarf_l.001": "ScarfL1b"}},
    {"chain": "scarf_r", "rigify_type": "limbs.simple_tentacle", "parent": "spine.004",
     "points": [[-0.08, -0.08, 1.40], [-0.10, -0.10, 1.15], [-0.12, -0.05, 0.92]],
     "rename": {"DEF-scarf_r": "ScarfR1a", "DEF-scarf_r.001": "ScarfR1b"}},

    # 후드 귀: 머리(spine.005) 위 좌우 단일 본.
    {"chain": "hood_ear_l", "rigify_type": "basic.super_copy", "parent": "spine.005",
     "points": [[0.06, 0.0, 1.75], [0.05, 0.02, 1.85]],
     "rename": {"DEF-hood_ear_l": "HoodEarL"}},
    {"chain": "hood_ear_r", "rigify_type": "basic.super_copy", "parent": "spine.005",
     "points": [[-0.06, 0.0, 1.75], [-0.05, 0.02, 1.85]],
     "rename": {"DEF-hood_ear_r": "HoodEarR"}},
]

EXTRA_POSES = []              # Task 7에서 채움


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
