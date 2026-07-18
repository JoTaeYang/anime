from lib.proportions import P, HIPS_HEIGHT as _HIPS
from lib.bone_map import BONE_RENAME

NAME = "dummy"
MESH_OBJECT = "Dummy"
SOURCE = {"kind": "procedural"}
LANDMARKS = P
METARIG_SCALE = 0.858759
HIPS_HEIGHT = _HIPS
HIPS_TOL = 0.10
HEIGHT_RANGE = (1.55, 1.80)
FBX_NAME = "dummy.fbx"
USE_LATERALITY_MARKER = True
APPENDAGES = []
EXTRA_POSES = []


def appendage_bone_rename():
    return {}


def bone_rename():
    return {**BONE_RENAME, **appendage_bone_rename()}


def expected_def_bones():
    return set(bone_rename().keys())
