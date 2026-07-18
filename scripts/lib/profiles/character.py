from lib.bone_map import BONE_RENAME
from lib.profiles.character_landmarks import LANDMARKS as _LM

NAME = "character"
MESH_OBJECT = "Character"
SOURCE = {"kind": "blend", "path": "assets/character.blend", "object": "Mesh_0"}
LANDMARKS = _LM
METARIG_SCALE = None          # 랜드마크 직접 배치
HIPS_HEIGHT = 1.03            # 초안(키 1.899의 54%) — Task 3/6에서 실측값으로 갱신
HIPS_TOL = 0.12
HEIGHT_RANGE = (1.85, 1.95)
FBX_NAME = "character.fbx"
USE_LATERALITY_MARKER = False
APPENDAGES = []               # Task 5에서 채움
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
