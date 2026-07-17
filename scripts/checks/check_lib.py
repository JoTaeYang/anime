import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lib.bone_map import BONE_RENAME, REQUIRED_HUMAN_BONES, OPTIONAL_HUMAN_BONES
from lib.proportions import P, HIPS_HEIGHT

assert len(BONE_RENAME) == 71, f"expected 71 DEF bones, got {len(BONE_RENAME)}"
vals = list(BONE_RENAME.values())
assert len(vals) == len(set(vals)), "duplicate Unity bone names"
assert all(k.startswith("DEF-") for k in BONE_RENAME)
for name in list(REQUIRED_HUMAN_BONES) + list(OPTIONAL_HUMAN_BONES):
    assert name in vals, f"human bone {name} not produced by BONE_RENAME"
assert abs(HIPS_HEIGHT - 0.8673) < 1e-6
assert abs(P["spine.006"]["tail"][2] - 1.70) < 1e-6, "head top must be exactly 1.70"

# proportions의 모든 메타리그 본은 대응하는 DEF 본이 매핑 테이블에 있어야 한다
for k in P:
    assert f"DEF-{k}" in BONE_RENAME, f"proportions key {k} has no DEF-{k} in BONE_RENAME"

print("CHECK_LIB OK")
