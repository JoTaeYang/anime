# Phase 0 파이프라인 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **실행 완료 (2026-07-17): 최종 코드가 규범이다.** 아래 코드 블록 일부는 실행 중 수정 전의 계획안이다 (spine.005/006 매핑, 02_anim 키잉 방식, check_03의 fcurves 접근 등). 실제 동작 코드는 `scripts/`와 `unity/AvatarCheck/`를 보라. Phase 1 착수 전 필수 규명 항목: **bone_map의 L/R 스왑과 04_export의 사전 회전 트릭의 결합** — 비대칭 더미로 재검증할 것.

**Goal:** 더미 마네킹으로 Blender→Unity 파이프라인(메시→리깅→애니메이션→베이크→FBX→Unity Humanoid 검증) 전 구간을 관통시킨다.

**Architecture:** 5개 스테이지가 `.blend` 스냅샷 체인(`build/00_mesh.blend` → … → `exports/dummy.fbx`)으로 연결된다. 각 스테이지는 headless Blender 스크립트이고, 스테이지마다 검증 스크립트(check)가 붙는다. 최종 검증은 Unity batchmode가 FBX를 임포트해 단언하고 JSON 리포트를 남긴다. 스펙: `docs/superpowers/specs/2026-07-17-phase0-pipeline-design.md`.

**Tech Stack:** Blender 5.1.2 bpy (Rigify, FBX 익스포터), Unity 6000.3.20f1 에디터 C#, PowerShell 5.1 래퍼.

## Global Constraints

- Blender 실행 파일: `C:\Program Files\Blender Foundation\Blender 5.1\blender.exe`
- Unity 실행 파일: `C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe`
- Blender headless 호출은 **항상** `--background --factory-startup --python-exit-code 1` 플래그를 쓴다. `--python-exit-code 1`이 없으면 파이썬 예외가 나도 Blender가 exit 0을 반환한다(파이프라인 침묵 실패의 원인). `--factory-startup`은 사용자 프리퍼런스(MCP 애드온 등)로부터 격리한다.
- Rigify는 factory-startup에서 비활성 상태다. **`bpy.ops.preferences.addon_enable(module="rigify")`로 활성화**한다. `addon_utils.enable("rigify")`는 Blender 5.1.2에서 등록이 반쯤 실패하며(`KeyError: 'rigify'` 출력 후) 이후 `rigify_generate`가 `'RigifyParameters' object has no attribute 'make_custom_pivot'`으로 죽는다. 사용 금지.
- `bpy.ops.*`에 대한 `hasattr` 검사는 **항상 True를 반환하므로 무의미**하다. 연산자 존재 확인은 `op.get_rna_type()`(없으면 예외)으로 한다.
- 더미 키 1.7m. 메타리그 스케일 계수 **0.858759** (기본 메타리그 최고점 1.9796m 실측 → 1.7/1.9796). 힙(spine 헤드) 높이 실측 **0.8673m**.
- 애니메이션: fps **24**, 프레임 **1–48**, 액션 이름 **"Idle"**.
- 본 이름의 단일 원천은 `scripts/lib/bone_map.py`. 다른 파일에 본 이름 리터럴을 중복하지 않는다(체크 스크립트가 import해서 쓴다).
- 스테이지 계약: 입력 `.blend` 하나(`00_mesh`는 없음), 출력 `.blend` 하나, 단독 실행 가능.
- 산출 FBX: `exports/dummy.fbx` 하나.
- 커밋 메시지는 conventional commit(`feat:`, `chore:`, `fix:`). 각 태스크 끝에 커밋.

---

### Task 1: 저장소 골격 + 실행 래퍼 + 스모크 테스트

**Files:**
- Create: `.gitignore`
- Create: `scripts/run.ps1`
- Create: `scripts/lib/__init__.py` (빈 파일)
- Create: `scripts/lib/paths.py`
- Create: `scripts/lib/blender_utils.py`
- Create: `scripts/stages/smoke.py`

**Interfaces:**
- Produces: `paths.PROJECT_ROOT / BUILD_DIR / EXPORTS_DIR / PREVIEWS_DIR: Path`, `paths.blend_path(stage: str) -> Path`, `paths.ensure_dirs() -> None`
- Produces: `blender_utils.ensure_rigify() -> None`, `blender_utils.clean_scene() -> None`, `blender_utils.open_blend(path) -> None`, `blender_utils.save_as(path) -> None`, `blender_utils.op_kwargs(op, **kwargs) -> dict`
- Produces: `scripts/run.ps1 <stage>` — 이후 모든 태스크의 실행 진입점. 스테이지명: `smoke|mesh|rig|anim|bake|export|unity|sheet|all`

- [ ] **Step 1: `.gitignore` 작성**

```gitignore
build/
previews/
exports/
__pycache__/
*.blend1
unity/AvatarCheck/Library/
unity/AvatarCheck/Temp/
unity/AvatarCheck/Logs/
unity/AvatarCheck/UserSettings/
unity/AvatarCheck/obj/
unity/AvatarCheck/Assets/Dummy/
```

- [ ] **Step 2: `scripts/lib/paths.py` 작성**

```python
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
BUILD_DIR = PROJECT_ROOT / "build"
EXPORTS_DIR = PROJECT_ROOT / "exports"
PREVIEWS_DIR = PROJECT_ROOT / "previews"


def blend_path(stage: str) -> Path:
    return BUILD_DIR / f"{stage}.blend"


def ensure_dirs() -> None:
    for d in (BUILD_DIR, EXPORTS_DIR, PREVIEWS_DIR):
        d.mkdir(parents=True, exist_ok=True)
```

- [ ] **Step 3: `scripts/lib/blender_utils.py` 작성**

```python
import bpy


def ensure_rigify() -> None:
    """factory-startup에서 rigify를 켠다. addon_utils.enable은 5.1.2에서 깨지므로 금지."""
    try:
        bpy.ops.object.armature_human_metarig_add.get_rna_type()
        return
    except Exception:
        pass
    bpy.ops.preferences.addon_enable(module="rigify")
    bpy.ops.object.armature_human_metarig_add.get_rna_type()  # 실패 시 여기서 예외


def clean_scene() -> None:
    bpy.ops.wm.read_homefile(use_empty=True)


def open_blend(path) -> None:
    bpy.ops.wm.open_mainfile(filepath=str(path))


def save_as(path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(path))


def op_kwargs(op, **kwargs) -> dict:
    """연산자가 실제로 받는 파라미터만 남긴다. Blender 버전 간 시그니처 차이 방어."""
    props = {p.identifier for p in op.get_rna_type().properties}
    return {k: v for k, v in kwargs.items() if k in props}
```

- [ ] **Step 4: `scripts/stages/smoke.py` 작성**

모든 스테이지 스크립트는 이 헤더 패턴(sys.path 주입)으로 시작한다 — headless 실행 시 `scripts/`가 모듈 경로에 없기 때문.

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib.blender_utils import ensure_rigify
from lib import paths

print("BLENDER", bpy.app.version_string)
assert bpy.app.version_string.startswith("5.1"), f"unexpected: {bpy.app.version_string}"
ensure_rigify()
paths.ensure_dirs()
print("SMOKE OK")
```

- [ ] **Step 5: `scripts/run.ps1` 작성**

```powershell
param([Parameter(Position = 0)][string]$Stage = "all")

$Root = Split-Path -Parent $PSScriptRoot
$Blender = "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"
$Unity = "C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe"

function Invoke-Blender([string]$RelScript) {
    Write-Host ">>> blender: $RelScript"
    & $Blender --background --factory-startup --python-exit-code 1 --python (Join-Path $Root $RelScript)
    if ($LASTEXITCODE -ne 0) {
        Write-Host "FAILED: $RelScript (exit $LASTEXITCODE)" -ForegroundColor Red
        exit $LASTEXITCODE
    }
}

function Invoke-UnityCheck {
    Write-Host ">>> unity: AvatarCheck.Run"
    & $Unity -batchmode -nographics -quit `
        -projectPath (Join-Path $Root "unity\AvatarCheck") `
        -executeMethod AvatarCheck.Run `
        -logFile (Join-Path $Root "unity\AvatarCheck\Logs\check.log")
    $code = $LASTEXITCODE
    $report = Join-Path $Root "unity\AvatarCheck\report.json"
    if (Test-Path $report) { Get-Content $report }
    if ($code -ne 0) {
        Write-Host "FAILED: Unity check (exit $code)" -ForegroundColor Red
        exit $code
    }
}

$Pipeline = [ordered]@{
    "mesh"   = @("scripts\stages\00_mesh.py", "scripts\checks\check_00.py")
    "rig"    = @("scripts\stages\01_rig.py", "scripts\checks\check_01.py")
    "anim"   = @("scripts\stages\02_anim.py", "scripts\checks\check_02.py")
    "bake"   = @("scripts\stages\03_bake.py", "scripts\checks\check_03.py")
    "export" = @("scripts\stages\04_export.py", "scripts\checks\check_04.py")
}

switch ($Stage) {
    "smoke" { Invoke-Blender "scripts\stages\smoke.py" }
    "unity" { Invoke-UnityCheck }
    "sheet" { Invoke-Blender "scripts\preview\contact_sheet.py" }
    "all" {
        Invoke-Blender "scripts\stages\smoke.py"
        foreach ($pair in $Pipeline.Values) { foreach ($s in $pair) { Invoke-Blender $s } }
        Invoke-UnityCheck
        Invoke-Blender "scripts\preview\contact_sheet.py"
        Write-Host "ALL STAGES PASSED" -ForegroundColor Green
    }
    default {
        if (-not $Pipeline.Contains($Stage)) {
            Write-Host "usage: run.ps1 [smoke|mesh|rig|anim|bake|export|unity|sheet|all]"
            exit 2
        }
        foreach ($s in $Pipeline[$Stage]) { Invoke-Blender $s }
    }
}
```

- [ ] **Step 6: 스모크 실행 — 성공 경로**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 smoke`
Expected: `BLENDER 5.1.2`, `SMOKE OK` 출력, exit 0. `build/`, `exports/`, `previews/` 디렉터리 생성됨.

- [ ] **Step 7: 실패 전파 확인**

일부러 예외를 내는 임시 파일로 exit code 전파를 검증한다 (검증 후 삭제).

```powershell
Set-Content scripts\stages\_fail.py "raise RuntimeError('must fail')" -Encoding utf8
& "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe" --background --factory-startup --python-exit-code 1 --python scripts\stages\_fail.py
echo "exit=$LASTEXITCODE"   # 1 이어야 함. 0이면 래퍼가 침묵 실패한다 — 플래그 확인
Remove-Item scripts\stages\_fail.py
```

Expected: `exit=1`

- [ ] **Step 8: 커밋**

```powershell
git add .gitignore scripts
git commit -m "feat: repo scaffold, headless blender wrapper, smoke stage"
```

---

### Task 2: 비율 테이블 + 본 매핑 테이블 (데이터 라이브러리)

**Files:**
- Create: `scripts/lib/proportions.py`
- Create: `scripts/lib/bone_map.py`
- Create: `scripts/checks/check_lib.py`

**Interfaces:**
- Consumes: 없음 (순수 데이터)
- Produces: `proportions.SCALE: float`, `proportions.P: dict[str, dict[str, list[float]]]` — 메타리그 본 이름 → `{"head": [x,y,z], "tail": [x,y,z]}` (1.7m 스케일 적용값, 왼쪽만; 오른쪽은 x 부호 반전)
- Produces: `bone_map.BONE_RENAME: dict[str, str]` — 전체 DEF 본(71개) → Unity 이름. `bone_map.REQUIRED_HUMAN_BONES: dict[str, str]` — Unity Humanoid 필수 15본 humanName → 우리 본 이름. `bone_map.OPTIONAL_HUMAN_BONES: dict[str, str]` — 매핑을 기대하는 선택 본(목/가슴/어깨/발끝/손가락).

- [ ] **Step 1: `scripts/lib/proportions.py` 작성**

값은 Blender 5.1.2 기본 human 메타리그를 0.858759배(최고점 1.9796 → 1.70)로 스케일해 실측한 좌표다. 좌표계는 Blender (x=우+, y=뒤+, z=상+), 미터 단위.

```python
# Blender 5.1.2 human metarig, scaled so total height = 1.70 m. Left side only.
SCALE = 0.858759

P = {
    "spine":       {"head": [0.0, 0.0474, 0.8673],  "tail": [0.0, 0.0148, 0.9938]},
    "spine.001":   {"head": [0.0, 0.0148, 0.9938],  "tail": [0.0, 0.0003, 1.1103]},
    "spine.002":   {"head": [0.0, 0.0003, 1.1103],  "tail": [0.0, 0.0051, 1.2587]},
    "spine.003":   {"head": [0.0, 0.0051, 1.2587],  "tail": [0.0, 0.0098, 1.4240]},
    "spine.004":   {"head": [0.0, 0.0098, 1.4240],  "tail": [0.0, -0.0112, 1.4768]},
    "spine.005":   {"head": [0.0, -0.0112, 1.4768], "tail": [0.0, -0.0212, 1.5297]},
    "spine.006":   {"head": [0.0, -0.0212, 1.5297], "tail": [0.0, -0.0212, 1.7000]},
    "shoulder.L":  {"head": [0.0157, -0.0587, 1.3784], "tail": [0.1455, 0.0176, 1.3783]},
    "upper_arm.L": {"head": [0.1677, 0.0229, 1.3608],  "tail": [0.3799, 0.0760, 1.2444]},
    "forearm.L":   {"head": [0.3799, 0.0760, 1.2444],  "tail": [0.5663, 0.0423, 1.1216]},
    "hand.L":      {"head": [0.5663, 0.0423, 1.1216],  "tail": [0.6212, 0.0354, 1.0807]},
    "thigh.L":     {"head": [0.0842, 0.0106, 0.9206],  "tail": [0.0842, -0.0246, 0.4613]},
    "shin.L":      {"head": [0.0842, -0.0246, 0.4613], "tail": [0.0842, 0.0139, 0.0732]},
    "foot.L":      {"head": [0.0842, 0.0139, 0.0732],  "tail": [0.0842, -0.0802, 0.0143]},
    "toe.L":       {"head": [0.0842, -0.0802, 0.0143], "tail": [0.0842, -0.1379, 0.0143]},
}

HIPS_HEIGHT = P["spine"]["head"][2]  # 0.8673
```

- [ ] **Step 2: `scripts/lib/bone_map.py` 작성**

Rigify가 생성하는 DEF 본(얼굴·breast 제거 후 71개, Blender 5.1.2 실측)을 전부 커버한다. Humanoid 매핑 대상은 Unity 관례 이름, 나머지(트위스트·팜·펠비스·목 상단)는 매핑 안 되는 여분 본으로 이름만 정리한다.

```python
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
```

- [ ] **Step 3: `scripts/checks/check_lib.py` 작성 (자기 일관성 테스트)**

```python
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
```

- [ ] **Step 4: 실행해서 통과 확인**

Run: `& "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe" --background --factory-startup --python-exit-code 1 --python scripts\checks\check_lib.py`
Expected: `CHECK_LIB OK`, exit 0.

- [ ] **Step 5: 커밋**

```powershell
git add scripts\lib scripts\checks
git commit -m "feat: proportion table and DEF-to-Unity bone map (single source of truth)"
```

---

### Task 3: 00_mesh — 더미 마네킹 절차 생성

**Files:**
- Create: `scripts/stages/00_mesh.py`
- Create: `scripts/checks/check_00.py`

**Interfaces:**
- Consumes: `lib.proportions.P`, `lib.paths`, `lib.blender_utils`
- Produces: `build/00_mesh.blend` — 단일 메시 오브젝트 **"Dummy"** (원점 월드 0, 트랜스폼 적용 완료, 체커 재질 "Checker", 메타리그 레스트 포즈(팔 ~30° 하향 — P 좌표 그대로), 키 ≈1.7m, 정면 −Y). Unity가 T포즈 경고를 내면 Task 9에서 대응.

- [ ] **Step 1: `scripts/checks/check_00.py` 작성 (먼저)**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend

open_blend(paths.blend_path("00_mesh"))

ob = bpy.data.objects.get("Dummy")
assert ob is not None and ob.type == 'MESH', "Dummy mesh missing"
assert 1.55 < ob.dimensions.z < 1.80, f"height {ob.dimensions.z}"
assert 1000 < len(ob.data.vertices) < 50000, f"verts {len(ob.data.vertices)}"
# 트랜스폼 적용 확인: 원점 월드 0, 스케일 1, 회전 0
assert ob.location.length < 1e-6, f"origin not at world zero: {tuple(ob.location)}"
assert all(abs(s - 1.0) < 1e-6 for s in ob.scale), f"scale not applied: {tuple(ob.scale)}"
assert all(abs(a) < 1e-6 for a in ob.rotation_euler), "rotation not applied"
mat = ob.data.materials[0] if ob.data.materials else None
assert mat is not None and mat.name == "Checker", "checker material missing"
assert any(n.bl_idname == "ShaderNodeTexChecker" for n in mat.node_tree.nodes)
print("CHECK_00 OK")
```

- [ ] **Step 2: 실패 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 mesh`
Expected: FAIL — `00_mesh.py`가 아직 없어 Blender가 파일을 못 찾고 exit ≠ 0.

- [ ] **Step 3: `scripts/stages/00_mesh.py` 작성**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from mathutils import Vector
from lib import paths
from lib.blender_utils import clean_scene, save_as
from lib.proportions import P

clean_scene()
paths.ensure_dirs()


def box_along(name, head, tail, thickness, cuts=3):
    """head→tail을 잇는 세그먼트 박스. 로컬 +Z를 본 방향에 정렬."""
    head, tail = Vector(head), Vector(tail)
    vec = tail - head
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(head + tail) / 2)
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = (thickness, thickness, vec.length)
    ob.rotation_mode = 'QUATERNION'
    ob.rotation_quaternion = vec.to_track_quat('Z', 'Y')
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.subdivide(number_cuts=cuts)
    bpy.ops.object.mode_set(mode='OBJECT')
    return ob


def mirror(v):
    return [-v[0], v[1], v[2]]


def h(bone):
    return P[bone]["head"]


def t(bone):
    return P[bone]["tail"]


HAND_TIP_L = [0.70, 0.02, 1.00]  # 손끝(손가락 포함 미튼) 근사 — hand.L 헤드에서 연장

parts = []
# 몸통·머리 (중앙)
parts.append(box_along("torso", h("spine"), t("spine.003"), 0.30, cuts=6))
parts.append(box_along("neck", h("spine.004"), h("spine.006"), 0.09, cuts=2))
parts.append(box_along("head", h("spine.006"), t("spine.006"), 0.17, cuts=2))

# 사지 (좌우)
for side, m in (("L", lambda v: v), ("R", mirror)):
    parts.append(box_along(f"upper_arm.{side}", m(h("upper_arm.L")), m(t("upper_arm.L")), 0.075, cuts=4))
    parts.append(box_along(f"forearm.{side}", m(h("forearm.L")), m(t("forearm.L")), 0.065, cuts=4))
    parts.append(box_along(f"hand.{side}", m(h("hand.L")), m(HAND_TIP_L), 0.075, cuts=3))
    parts.append(box_along(f"thigh.{side}", m(h("thigh.L")), m(t("thigh.L")), 0.12, cuts=4))
    parts.append(box_along(f"shin.{side}", m(h("shin.L")), m(t("shin.L")), 0.10, cuts=4))
    parts.append(box_along(f"foot.{side}", m(h("foot.L")), m(t("toe.L")), 0.08, cuts=3))

# 하나로 합침
bpy.ops.object.select_all(action='DESELECT')
for ob in parts:
    ob.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
bpy.ops.object.join()
dummy = bpy.context.active_object
dummy.name = "Dummy"
# join 후 원점은 첫 파트(몸통) 중심에 남는다 — 월드 0으로 정리
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# 체커 재질
mat = bpy.data.materials.new("Checker")
mat.use_nodes = True
bsdf = mat.node_tree.nodes["Principled BSDF"]
tex = mat.node_tree.nodes.new("ShaderNodeTexChecker")
tex.inputs["Scale"].default_value = 40.0
mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
dummy.data.materials.append(mat)

save_as(paths.blend_path("00_mesh"))
print("STAGE 00_mesh OK")
```

- [ ] **Step 4: 실행해서 통과 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 mesh`
Expected: `STAGE 00_mesh OK`, `CHECK_00 OK`, exit 0.

- [ ] **Step 5: 커밋**

```powershell
git add scripts\stages\00_mesh.py scripts\checks\check_00.py
git commit -m "feat: stage 00 procedural dummy mannequin (1.7m, checker, T-pose)"
```

---

### Task 4: 01_rig — 메타리그 적용, Rigify 생성, 자동 웨이트

**Files:**
- Create: `scripts/stages/01_rig.py`
- Create: `scripts/checks/check_01.py`

**Interfaces:**
- Consumes: `build/00_mesh.blend`, `lib.proportions.SCALE`, `lib.bone_map.BONE_RENAME`(키 집합을 기대 DEF 목록으로 사용)
- Produces: `build/01_rigged.blend` — 오브젝트 **"rig"**(Rigify 컨트롤 리그, DEF 71본), **"metarig"**(보존), **"Dummy"**(rig에 ARMATURE_AUTO로 바인딩, 버텍스 그룹은 `DEF-*` 이름)

- [ ] **Step 1: `scripts/checks/check_01.py` 작성 (먼저)**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend
from lib.bone_map import BONE_RENAME

open_blend(paths.blend_path("01_rigged"))

rig = bpy.data.objects.get("rig")
assert rig is not None and rig.type == 'ARMATURE', "generated rig missing"

def_bones = {b.name for b in rig.data.bones if b.name.startswith("DEF-")}
expected = set(BONE_RENAME.keys())
assert def_bones == expected, (
    f"DEF mismatch\n missing: {sorted(expected - def_bones)}\n extra: {sorted(def_bones - expected)}"
)

dummy = bpy.data.objects.get("Dummy")
assert dummy is not None
mods = [m for m in dummy.modifiers if m.type == 'ARMATURE']
assert len(mods) == 1 and mods[0].object == rig, "armature modifier not bound to rig"

vg_names = {vg.name for vg in dummy.vertex_groups}
missing_vg = expected - vg_names
assert len(missing_vg) < len(expected) * 0.15, f"too many DEF bones without vertex group: {sorted(missing_vg)[:10]}"

weighted = 0
for v in dummy.data.vertices:
    if any(g.weight > 0.01 for g in v.groups):
        weighted += 1
ratio = weighted / len(dummy.data.vertices)
assert ratio > 0.98, f"only {ratio:.1%} vertices weighted"
print("CHECK_01 OK")
```

- [ ] **Step 2: 실패 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 rig`
Expected: FAIL (`01_rig.py` 없음).

- [ ] **Step 3: `scripts/stages/01_rig.py` 작성**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import ensure_rigify, open_blend, save_as
from lib.proportions import SCALE, P

open_blend(paths.blend_path("00_mesh"))
ensure_rigify()

# 1) 메타리그 추가 + 1.7m 스케일
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.armature_human_metarig_add()
meta = bpy.context.active_object
meta.scale = (SCALE,) * 3
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

# 2) 얼굴 서브트리 + breast 제거 (박스 더미에 얼굴 본 100개는 노이즈)
bpy.ops.object.mode_set(mode='EDIT')
eb = meta.data.edit_bones
doomed = set()


def mark(bone):
    doomed.add(bone.name)
    for c in bone.children:
        mark(c)


if "face" in eb:
    mark(eb["face"])
for n in ("breast.L", "breast.R"):
    if n in eb:
        doomed.add(n)
for n in doomed:
    eb.remove(eb[n])
bpy.ops.object.mode_set(mode='OBJECT')

# 3) 스케일 검증 (proportions 테이블과 일치해야 함)
hips_z = meta.data.bones["spine"].head_local.z
assert abs(hips_z - P["spine"]["head"][2]) < 0.005, f"metarig scale drift: hips at {hips_z}"

# 4) 컨트롤 리그 생성 → "rig" 오브젝트
bpy.ops.pose.rigify_generate()
rig = bpy.context.active_object
assert rig.name == "rig", f"unexpected rig name {rig.name}"

# 5) 자동 웨이트 바인딩 (DEF 본만 use_deform=True이므로 DEF에만 붙는다)
dummy = bpy.data.objects["Dummy"]
bpy.ops.object.select_all(action='DESELECT')
dummy.select_set(True)
rig.select_set(True)
bpy.context.view_layer.objects.active = rig
bpy.ops.object.parent_set(type='ARMATURE_AUTO')

save_as(paths.blend_path("01_rigged"))
print("STAGE 01_rig OK")
```

- [ ] **Step 4: 실행해서 통과 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 rig`
Expected: `STAGE 01_rig OK`, `CHECK_01 OK`, exit 0.

알려진 실패 모드: `parent_set`이 "Bone Heat Weighting: failed to find solution" 오류를 내면 `00_mesh.py`의 `cuts`를 1씩 올려 재시도한다 (형상 밀도 부족이 원인).

- [ ] **Step 5: 커밋**

```powershell
git add scripts\stages\01_rig.py scripts\checks\check_01.py
git commit -m "feat: stage 01 rigify generate + auto weights (face pruned, 71 DEF bones)"
```

---

### Task 5: 02_anim — 절차적 idle 액션

**Files:**
- Create: `scripts/stages/02_anim.py`
- Create: `scripts/checks/check_02.py`

**Interfaces:**
- Consumes: `build/01_rigged.blend` (컨트롤 본 "torso" — Blender 5.1.2 Rigify 실측 확인됨)
- Produces: `build/02_animated.blend` — rig에 액션 **"Idle"** (프레임 1–48, fps 24, torso 호흡 루프: 1과 48 포즈 동일)

- [ ] **Step 1: `scripts/checks/check_02.py` 작성 (먼저)**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend

open_blend(paths.blend_path("02_animated"))

scene = bpy.context.scene
assert scene.render.fps == 24, f"fps {scene.render.fps}"
assert (scene.frame_start, scene.frame_end) == (1, 48)

rig = bpy.data.objects["rig"]
act = rig.animation_data.action if rig.animation_data else None
assert act is not None and act.name == "Idle", f"action: {act and act.name}"

def torso_matrix(frame):
    scene.frame_set(frame)
    return rig.pose.bones["torso"].matrix.copy()

m1, m24 = torso_matrix(1), torso_matrix(24)
delta = (m1.translation - m24.translation).length
assert delta > 0.005, f"torso barely moves between f1/f24: {delta}"

m48 = torso_matrix(48)
assert (m1.translation - m48.translation).length < 1e-4, "loop not closed (f1 != f48)"
print("CHECK_02 OK")
```

- [ ] **Step 2: 실패 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 anim`
Expected: FAIL (`02_anim.py` 없음).

- [ ] **Step 3: `scripts/stages/02_anim.py` 작성**

torso는 Rigify 몸통 루트 컨트롤이다. 다리는 기본 IK이므로 torso가 내려가면 무릎이 자동으로 굽는다 — IK 경로가 베이크까지 살아있는지를 검증하는 부수 효과가 있다.

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend, save_as

open_blend(paths.blend_path("01_rigged"))

scene = bpy.context.scene
scene.render.fps = 24
scene.frame_start = 1
scene.frame_end = 48

rig = bpy.data.objects["rig"]
bpy.context.view_layer.objects.active = rig
bpy.ops.object.mode_set(mode='POSE')

torso = rig.pose.bones["torso"]
# 호흡: 중간(f24)에 1.5cm 하강, 시작/끝 동일 포즈로 루프 성립
for frame, offset in ((1, 0.0), (24, -0.015), (48, 0.0)):
    torso.location = (0.0, 0.0, 0.0)
    scene.frame_set(frame)
    # torso 본 로컬축 방향과 무관하게 월드 Z 이동을 주기 위해 matrix 기반 이동
    m = torso.matrix.copy()
    m.translation.z += offset
    torso.matrix = m
    torso.keyframe_insert(data_path="location", frame=frame)

bpy.ops.object.mode_set(mode='OBJECT')
rig.animation_data.action.name = "Idle"

save_as(paths.blend_path("02_animated"))
print("STAGE 02_anim OK")
```

- [ ] **Step 4: 실행해서 통과 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 anim`
Expected: `STAGE 02_anim OK`, `CHECK_02 OK`, exit 0.

알려진 실패 모드: `torso.matrix` 대입 후 키가 원위치로 들어가면(check_02의 delta 실패) `scene.frame_set(frame)` 뒤에 `bpy.context.view_layer.update()`를 추가하고, 그래도 안 되면 matrix 방식 대신 `torso.location.y`(본 로컬축) 직접 키잉으로 바꾼 뒤 check_02의 방향 무관 단언이 잡아주는지 확인한다.

- [ ] **Step 5: 커밋**

```powershell
git add scripts\stages\02_anim.py scripts\checks\check_02.py
git commit -m "feat: stage 02 procedural idle breathing loop (24fps, 48f)"
```

---

### Task 6: 03_bake — deform 본 베이크, 컨트롤 제거, Unity 리네임

**Files:**
- Create: `scripts/stages/03_bake.py`
- Create: `scripts/checks/check_03.py`

**Interfaces:**
- Consumes: `build/02_animated.blend`, `lib.bone_map.BONE_RENAME`
- Produces: `build/03_baked.blend` — 오브젝트 **"DummyRig"**(구 rig: Unity 이름 71본만 남은 순수 deform 아마추어, 유일 액션 "Idle" 베이크 완료), **"Dummy"**(버텍스 그룹도 Unity 이름). metarig/위젯 삭제됨. 이후 태스크(04, 컨택트 시트)는 이 파일의 Unity 본 이름(Hips, LeftUpperArm…)을 직접 참조한다.

- [ ] **Step 1: `scripts/checks/check_03.py` 작성 (먼저)**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend
from lib.bone_map import BONE_RENAME

open_blend(paths.blend_path("03_baked"))

rig = bpy.data.objects.get("DummyRig")
assert rig is not None and rig.type == 'ARMATURE'

names = {b.name for b in rig.data.bones}
assert names == set(BONE_RENAME.values()), (
    f"bone set mismatch\n missing: {sorted(set(BONE_RENAME.values()) - names)}\n"
    f" extra: {sorted(names - set(BONE_RENAME.values()))}"
)

b = rig.data.bones
assert b["Hips"].parent is None


def is_ancestor(anc, child):
    p = b[child].parent
    while p is not None:
        if p.name == anc:
            return True
        p = p.parent
    return False


# 모든 본이 Hips를 루트로 하는 단일 트리에 속한다
for bone in b:
    if bone.name != "Hips":
        assert is_ancestor("Hips", bone.name), f"{bone.name} not rooted at Hips"

# Unity Humanoid가 요구하는 조상 순서 (중간 여분 본 — 트위스트 등 — 은 허용)
CHAINS = [
    ("Hips", "Spine"), ("Spine", "Chest"), ("Chest", "UpperChest"),
    ("UpperChest", "Neck"), ("Neck", "Head"),
]
for side in ("Left", "Right"):
    CHAINS += [
        ("UpperChest", f"{side}Shoulder"), (f"{side}Shoulder", f"{side}UpperArm"),
        (f"{side}UpperArm", f"{side}LowerArm"), (f"{side}LowerArm", f"{side}Hand"),
        ("Hips", f"{side}UpperLeg"), (f"{side}UpperLeg", f"{side}LowerLeg"),
        (f"{side}LowerLeg", f"{side}Foot"), (f"{side}Foot", f"{side}Toes"),
    ]
for anc, child in CHAINS:
    assert is_ancestor(anc, child), f"{child} not descended from {anc}"

acts = list(bpy.data.actions)
assert len(acts) == 1 and acts[0].name == "Idle", [a.name for a in acts]
paths_in_action = {fc.data_path for fc in acts[0].fcurves}
assert any('pose.bones["Hips"]' in p for p in paths_in_action), "Hips not keyed in baked action"

dummy = bpy.data.objects["Dummy"]
vg = {g.name for g in dummy.vertex_groups}
assert not any(n.startswith("DEF-") for n in vg), "vertex groups not renamed"
assert dummy.modifiers[0].object == rig

leftovers = [o.name for o in bpy.data.objects if o.name.startswith("WGT-") or o.name == "metarig"]
assert not leftovers, f"leftover objects: {leftovers}"

# 베이크가 실제 움직임을 담았는지 (IK 경유 다리 포함)
scene = bpy.context.scene
def hips_z(f):
    scene.frame_set(f)
    return rig.pose.bones["Hips"].matrix.translation.z
assert abs(hips_z(1) - hips_z(24)) > 0.005, "baked Idle does not move Hips"
print("CHECK_03 OK")
```

- [ ] **Step 2: 실패 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 bake`
Expected: FAIL (`03_bake.py` 없음).

- [ ] **Step 3: `scripts/stages/03_bake.py` 작성**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend, save_as, op_kwargs
from lib.bone_map import BONE_RENAME

open_blend(paths.blend_path("02_animated"))

rig = bpy.data.objects["rig"]
bpy.context.view_layer.objects.active = rig
bpy.ops.object.mode_set(mode='POSE')

# 1) DEF 본만 선택해 비주얼 키잉 베이크 (컨트롤/IK가 만든 최종 자세를 DEF에 굽는다)
bpy.ops.pose.select_all(action='DESELECT')
for pb in rig.pose.bones:
    if pb.name.startswith("DEF-"):
        pb.select = True   # Blender 5.1: 선택은 PoseBone.select (Bone.select 제거됨)

rig.animation_data.action.name = "Idle_src"
bpy.ops.nla.bake(**op_kwargs(
    bpy.ops.nla.bake,
    frame_start=1, frame_end=48, step=1,
    only_selected=True, visual_keying=True,
    clear_constraints=True, clear_parents=False,
    use_current_action=False, bake_types={'POSE'},
))
rig.animation_data.action.name = "Idle"
bpy.ops.object.mode_set(mode='OBJECT')

# 2) 각 DEF 본의 부모가 될 DEF 본을 삭제 전에 계산 — 2단계:
#    (a) Blender 부모 체인에서 DEF 조상 탐색 (연결 체인: 척추/팔다리/손가락 내부)
#    (b) 없으면 ORG 계층 우회: Rigify는 분기점(어깨→팔, 골반→다리, 손→손가락)에서
#        DEF 본을 ORG 본의 "형제"로 두므로, DEF-X ↔ ORG-X 대응으로 ORG 조상 중
#        DEF 대응이 존재하는 첫 본을 부모로 삼는다 (Blender 5.1.2 실측 토폴로지)
bones = rig.data.bones


def def_parent(bone):
    p = bone.parent
    while p is not None:
        if p.name.startswith("DEF-"):
            return p.name
        p = p.parent
    org = bones.get("ORG-" + bone.name[len("DEF-"):])
    p = org.parent if org else None
    while p is not None:
        cand = None
        if p.name.startswith("ORG-"):
            cand = "DEF-" + p.name[len("ORG-"):]
        elif p.name.startswith("DEF-"):
            cand = p.name
        if cand and cand in bones:
            return cand
        p = p.parent
    return None


parent_map = {}
for bone in bones:
    if bone.name.startswith("DEF-"):
        parent_map[bone.name] = def_parent(bone)

# 3) DEF가 아닌 본 전부 삭제 + DEF 계층 재부모화
bpy.ops.object.mode_set(mode='EDIT')
eb = rig.data.edit_bones
for bone in list(eb):
    if not bone.name.startswith("DEF-"):
        eb.remove(bone)
for name, pname in parent_map.items():
    eb[name].use_connect = False
    eb[name].parent = eb[pname] if pname else None
bpy.ops.object.mode_set(mode='OBJECT')

# 4) Unity 이름으로 리네임 (본 리네임은 fcurve 경로를 자동 갱신한다)
for old, new in BONE_RENAME.items():
    rig.data.bones[old].name = new

# 5) 버텍스 그룹도 명시적으로 리네임 (자동 동기화에 의존하지 않는다)
dummy = bpy.data.objects["Dummy"]
for old, new in BONE_RENAME.items():
    vg = dummy.vertex_groups.get(old)
    if vg is not None:
        vg.name = new

# 6) 잔여물 정리: 위젯, 메타리그, 원본 액션, 드라이버
for ob in list(bpy.data.objects):
    if ob.name.startswith("WGT-") or ob.name == "metarig":
        bpy.data.objects.remove(ob, do_unlink=True)
for act in list(bpy.data.actions):
    if act.name != "Idle":
        bpy.data.actions.remove(act)
if rig.animation_data:
    for drv in list(rig.animation_data.drivers):
        rig.animation_data.drivers.remove(drv)

rig.name = "DummyRig"
rig.data.name = "DummyRig"

save_as(paths.blend_path("03_baked"))
print("STAGE 03_bake OK")
```

- [ ] **Step 4: 실행해서 통과 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 bake`
Expected: `STAGE 03_bake OK`, `CHECK_03 OK`, exit 0.

알려진 실패 모드: headless에서 `bpy.ops.nla.bake`가 컨텍스트 오류를 내면 `with bpy.context.temp_override(active_object=rig, selected_pose_bones=[...])` 블록으로 감싸 재시도한다.

- [ ] **Step 5: 커밋**

```powershell
git add scripts\stages\03_bake.py scripts\checks\check_03.py
git commit -m "feat: stage 03 bake to deform bones, strip controls, rename to Unity convention"
```

---

### Task 7: 04_export — FBX 익스포트

**Files:**
- Create: `scripts/stages/04_export.py`
- Create: `scripts/checks/check_04.py`

**Interfaces:**
- Consumes: `build/03_baked.blend`
- Produces: `exports/dummy.fbx` — Unity에 자기 완결적으로 임포트되는 FBX (아마추어 "DummyRig" + 메시 "Dummy" + 클립 Idle)

- [ ] **Step 1: `scripts/checks/check_04.py` 작성 (먼저)**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lib import paths

fbx = paths.EXPORTS_DIR / "dummy.fbx"
assert fbx.exists(), "dummy.fbx missing"
size = fbx.stat().st_size
assert size > 100_000, f"suspiciously small fbx: {size} bytes"
print("CHECK_04 OK")
```

- [ ] **Step 2: 실패 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 export`
Expected: FAIL (`04_export.py` 없음).

- [ ] **Step 3: `scripts/stages/04_export.py` 작성**

설정값이 이 파이프라인의 "익스포트 설정 고정" 그 자체다. Unity 검증(Task 9)에서 축/스케일 단언이 실패하면 **이 kwargs만** 조정한다.

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend, op_kwargs

open_blend(paths.blend_path("03_baked"))
paths.ensure_dirs()

kwargs = dict(
    filepath=str(paths.EXPORTS_DIR / "dummy.fbx"),
    use_selection=False,
    object_types={'ARMATURE', 'MESH'},
    apply_unit_scale=True,
    global_scale=1.0,
    apply_scale_options='FBX_SCALE_ALL',   # 본 스케일 100배 함정 방지의 핵심
    axis_forward='-Z',
    axis_up='Y',
    use_space_transform=True,
    bake_space_transform=True,             # 아마추어 루트 -89.98° 회전 방지
    add_leaf_bones=False,                  # `_end` 본 생성 방지
    primary_bone_axis='Y',
    secondary_bone_axis='X',
    armature_nodetype='NULL',
    use_armature_deform_only=False,        # 이미 03에서 deform만 남김
    bake_anim=True,
    bake_anim_use_all_bones=True,
    bake_anim_use_nla_strips=False,
    bake_anim_use_all_actions=False,       # 활성 액션(Idle)만
    bake_anim_force_startend_keying=True,
    mesh_smooth_type='OFF',
)
bpy.ops.export_scene.fbx(**op_kwargs(bpy.ops.export_scene.fbx, **kwargs))
print("STAGE 04_export OK")
```

- [ ] **Step 4: 실행해서 통과 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 export`
Expected: `STAGE 04_export OK`, `CHECK_04 OK`, exit 0.

- [ ] **Step 5: 커밋**

```powershell
git add scripts\stages\04_export.py scripts\checks\check_04.py
git commit -m "feat: stage 04 fbx export with pinned Unity-safe settings"
```

---

### Task 8: Unity 검증 프로젝트 부트스트랩

**Files:**
- Create: `unity/AvatarCheck/` (Unity `-createProject`로 생성)
- Create: `unity/AvatarCheck/Assets/Editor/AvatarCheck.cs`

**Interfaces:**
- Consumes: `exports/dummy.fbx` (경로는 프로젝트 상대 `../../exports/dummy.fbx`)
- Produces: batchmode 진입점 **`AvatarCheck.Run`** — exit 0/1 + `unity/AvatarCheck/report.json` (`{"assertions": [{"name","pass","detail"}], "allPass": bool}`)

- [ ] **Step 1: Unity 프로젝트 생성 (최초 1회, 수 분 소요)**

```powershell
& "C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe" -batchmode -nographics -quit -createProject "C:\Users\whxod\orca\anime\unity\AvatarCheck" -logFile "C:\Users\whxod\orca\anime\unity\AvatarCheck\Logs\create.log"
echo "exit=$LASTEXITCODE"
```

Expected: `exit=0`, `unity/AvatarCheck/Assets`, `ProjectSettings`, `Packages` 생성. 실패 시 `Logs\create.log`에서 라이선스 오류를 확인한다(라이선스는 Hub 로그인으로 활성화된 상태).

- [ ] **Step 2: `unity/AvatarCheck/Assets/Editor/AvatarCheck.cs` 작성**

```csharp
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

public static class AvatarCheck
{
    const string FbxAssetPath = "Assets/Dummy/dummy.fbx";
    static readonly List<string> ImportErrors = new List<string>();

    // Blender 쪽 scripts/lib/bone_map.py의 REQUIRED_HUMAN_BONES와 일치해야 한다
    static readonly Dictionary<HumanBodyBones, string> Required = new Dictionary<HumanBodyBones, string>
    {
        { HumanBodyBones.Hips, "Hips" }, { HumanBodyBones.Spine, "Spine" }, { HumanBodyBones.Head, "Head" },
        { HumanBodyBones.LeftUpperLeg, "LeftUpperLeg" }, { HumanBodyBones.LeftLowerLeg, "LeftLowerLeg" }, { HumanBodyBones.LeftFoot, "LeftFoot" },
        { HumanBodyBones.RightUpperLeg, "RightUpperLeg" }, { HumanBodyBones.RightLowerLeg, "RightLowerLeg" }, { HumanBodyBones.RightFoot, "RightFoot" },
        { HumanBodyBones.LeftUpperArm, "LeftUpperArm" }, { HumanBodyBones.LeftLowerArm, "LeftLowerArm" }, { HumanBodyBones.LeftHand, "LeftHand" },
        { HumanBodyBones.RightUpperArm, "RightUpperArm" }, { HumanBodyBones.RightLowerArm, "RightLowerArm" }, { HumanBodyBones.RightHand, "RightHand" },
    };

    public static void Run()
    {
        var results = new List<(string name, bool pass, string detail)>();
        GameObject instance = null;
        try
        {
            CopyFbxIntoProject();

            Application.logMessageReceived += CaptureLog;
            AssetDatabase.ImportAsset(FbxAssetPath, ImportAssetOptions.ForceSynchronousImport);
            var importer = (ModelImporter)AssetImporter.GetAtPath(FbxAssetPath);
            importer.animationType = ModelImporterAnimationType.Human;
            importer.SaveAndReimport();
            Application.logMessageReceived -= CaptureLog;

            results.Add(("import_clean", ImportErrors.Count == 0,
                ImportErrors.Count == 0 ? "no errors/warnings" : string.Join(" | ", ImportErrors.Take(5))));

            var avatar = AssetDatabase.LoadAllAssetsAtPath(FbxAssetPath).OfType<Avatar>().FirstOrDefault();
            bool avatarOk = avatar != null && avatar.isValid && avatar.isHuman;
            results.Add(("avatar_valid_human", avatarOk,
                avatar == null ? "no avatar" : $"isValid={avatar.isValid} isHuman={avatar.isHuman}"));

            if (avatarOk)
            {
                var human = avatar.humanDescription.human.ToDictionary(h => h.humanName, h => h.boneName);
                var missing = new List<string>();
                foreach (var kv in Required)
                {
                    string humanName = HumanTrait.BoneName[(int)kv.Key];
                    if (!human.TryGetValue(humanName, out var mapped) || mapped != kv.Value)
                        missing.Add($"{humanName}->({(human.ContainsKey(humanName) ? human[humanName] : "UNMAPPED")}) expected {kv.Value}");
                }
                results.Add(("required_15_mapped", missing.Count == 0,
                    missing.Count == 0 ? "all mapped to intended bones" : string.Join(" | ", missing)));

                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(FbxAssetPath);
                instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                var anim = instance.GetComponent<Animator>();

                float hipsY = anim.GetBoneTransform(HumanBodyBones.Hips).position.y;
                results.Add(("hips_height", hipsY > 0.77f && hipsY < 0.97f, $"hipsY={hipsY:F4} expected 0.87±0.10"));

                var lh = anim.GetBoneTransform(HumanBodyBones.LeftHand).position;
                var rh = anim.GetBoneTransform(HumanBodyBones.RightHand).position;
                var lf = anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
                var lt = anim.GetBoneTransform(HumanBodyBones.LeftToes).position;
                bool facing = lh.x > rh.x && lt.z > lf.z;
                results.Add(("faces_plus_z", facing, $"leftHand.x={lh.x:F3} rightHand.x={rh.x:F3} toeZ-footZ={(lt.z - lf.z):F3}"));

                var badRots = new List<string>();
                foreach (Transform child in instance.transform)
                {
                    float angle = Quaternion.Angle(child.localRotation, Quaternion.identity);
                    if (angle > 1f) badRots.Add($"{child.name}:{angle:F1}deg");
                }
                results.Add(("root_children_identity_rotation", badRots.Count == 0,
                    badRots.Count == 0 ? "clean" : string.Join(" | ", badRots)));

                var clip = AssetDatabase.LoadAllAssetsAtPath(FbxAssetPath).OfType<AnimationClip>()
                    .FirstOrDefault(c => !c.name.StartsWith("__preview"));
                bool clipOk = clip != null && clip.length > 1.5f && clip.length < 2.5f;
                results.Add(("idle_clip_present", clipOk, clip == null ? "no clip" : $"{clip.name} len={clip.length:F3}s"));

                if (clipOk)
                {
                    var chest = anim.GetBoneTransform(HumanBodyBones.Spine);
                    clip.SampleAnimation(instance, 0f);
                    Vector3 p0 = chest.position;
                    clip.SampleAnimation(instance, clip.length * 0.5f);
                    Vector3 pMid = chest.position;
                    float delta = (p0 - pMid).magnitude;
                    results.Add(("idle_actually_moves", delta > 0.004f, $"spine delta={delta:F4}m at t=0 vs t=mid"));
                }
            }
        }
        catch (Exception e)
        {
            results.Add(("exception", false, e.ToString()));
        }
        finally
        {
            if (instance != null) UnityEngine.Object.DestroyImmediate(instance);
        }

        WriteReport(results);
        bool all = results.All(r => r.pass);
        Debug.Log($"AvatarCheck: {(all ? "PASS" : "FAIL")} ({results.Count(r => r.pass)}/{results.Count})");
        EditorApplication.Exit(all ? 0 : 1);
    }

    public static void CompileSmoke()
    {
        Debug.Log("AvatarCheck compile smoke OK");
        EditorApplication.Exit(0);
    }

    static void CopyFbxIntoProject()
    {
        string src = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "exports", "dummy.fbx"));
        string dst = Path.GetFullPath(Path.Combine(Application.dataPath, "Dummy", "dummy.fbx"));
        if (!File.Exists(src)) throw new FileNotFoundException($"export first: {src}");
        Directory.CreateDirectory(Path.GetDirectoryName(dst));
        File.Copy(src, dst, true);
        AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
    }

    static void CaptureLog(string condition, string stackTrace, LogType type)
    {
        if (type == LogType.Error || type == LogType.Exception || type == LogType.Warning)
            ImportErrors.Add($"[{type}] {condition}");
    }

    static void WriteReport(List<(string name, bool pass, string detail)> results)
    {
        var sb = new StringBuilder();
        sb.Append("{\"allPass\":").Append(results.All(r => r.pass) ? "true" : "false").Append(",\"assertions\":[");
        sb.Append(string.Join(",", results.Select(r =>
            $"{{\"name\":\"{Escape(r.name)}\",\"pass\":{(r.pass ? "true" : "false")},\"detail\":\"{Escape(r.detail)}\"}}")));
        sb.Append("]}");
        string path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "report.json"));
        File.WriteAllText(path, sb.ToString());
    }

    static string Escape(string s) => s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", " ");
}
```

- [ ] **Step 3: 컴파일 스모크**

```powershell
& "C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe" -batchmode -nographics -quit -projectPath "C:\Users\whxod\orca\anime\unity\AvatarCheck" -executeMethod AvatarCheck.CompileSmoke -logFile "C:\Users\whxod\orca\anime\unity\AvatarCheck\Logs\smoke.log"
echo "exit=$LASTEXITCODE"
```

Expected: `exit=0`. 컴파일 에러 시 `Logs\smoke.log`의 CS 에러를 고친다.

- [ ] **Step 4: 커밋**

```powershell
git add unity\AvatarCheck\Assets unity\AvatarCheck\Packages unity\AvatarCheck\ProjectSettings
git commit -m "feat: Unity batchmode avatar verification project (AvatarCheck.Run)"
```

---

### Task 9: Unity 검증 통과 — 교정 루프

이 태스크는 새 파일을 만들지 않는다. `run.ps1 unity`를 돌리고, 실패한 단언에 따라 **원인 스테이지의 상수만** 고쳐 전체를 재실행하는 루프다. Phase 0의 존재 이유(함정 선해결)가 이 태스크에서 소화된다.

**Files:**
- Modify (필요시에만): `scripts/stages/04_export.py` (축/스케일), `scripts/lib/bone_map.py` (매핑), `scripts/stages/03_bake.py` (계층)

**Interfaces:**
- Consumes: Task 1–8 전부
- Produces: `unity/AvatarCheck/report.json`의 `allPass: true`

- [ ] **Step 1: 전체 파이프라인 실행**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all`
Expected (첫 시도): unity 단계까지 진행. 실패 시 report.json의 실패 단언 확인.

- [ ] **Step 2: 실패 단언별 대응표에 따라 교정**

| 실패 단언 | 원인 후보 | 손댈 곳 |
|---|---|---|
| `hips_height` (100배 또는 0.01배) | 스케일 처리 | `04_export.py`의 `apply_scale_options` — `'FBX_SCALE_ALL'` ↔ `'FBX_SCALE_UNITS'` 교차 시도 |
| `faces_plus_z` | 축 매핑 | `04_export.py`의 `axis_forward`/`bake_space_transform`. Blender에서 더미가 −Y를 보는지 먼저 확인 |
| `root_children_identity_rotation` (−89.98°) | 스페이스 변환 | `04_export.py`의 `bake_space_transform=True` 확인; 파라미터가 5.1에서 사라졌다면(`op_kwargs`가 걸러냄) 03 단계에서 아마추어에 회전 적용으로 대체 |
| `required_15_mapped` (UNMAPPED) | 자동 매핑 실패 | `bone_map.py`의 해당 본 이름을 Mixamo식(`LeftArm` 등)으로 바꿔 재시도 |
| `required_15_mapped` (좌우 뒤바뀜) | 미러링 | `04_export.py` 축 설정, 또는 03의 리네임 좌우 확인 |
| `idle_actually_moves` | 베이크 누락 | `03_bake.py`의 `visual_keying=True`, 선택 본 확인 |
| `import_clean`의 경고 | 다양 | 경고 내용 읽고 개별 판단. 무해 판정 시 C# 쪽에서 해당 문자열만 화이트리스트 |

한 번에 하나만 바꾸고 `run.ps1 all`로 재검증한다 (원인 격리).

- [ ] **Step 3: 통과 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all` (sheet 단계는 Task 10 전이라 실패해도 됨 — unity까지 exit 0이면 통과)
Expected: report.json `"allPass":true`.

- [ ] **Step 4: 커밋 (교정이 있었던 경우)**

```powershell
git add -A scripts unity\AvatarCheck\Assets
git commit -m "fix: export/mapping corrections discovered by Unity avatar assertions"
```

---

### Task 10: 컨택트 시트 (에이전트 눈 검사)

**Files:**
- Create: `scripts/preview/contact_sheet.py`

**Interfaces:**
- Consumes: `build/03_baked.blend` (Unity 본 이름 사용)
- Produces: `previews/contact_sheet.png` — 1536×1536, 3×3 타일(행 우선): [정면, 측면, 3/4, 후면, 팔들기, 쪼그리기, 비틀기, idle f1, idle f24]

- [ ] **Step 1: `scripts/preview/contact_sheet.py` 작성**

```python
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
import numpy as np
from mathutils import Matrix
from lib import paths
from lib.blender_utils import open_blend

TILE = 512
open_blend(paths.blend_path("03_baked"))
paths.ensure_dirs()
tiles_dir = paths.PREVIEWS_DIR / "tiles"
tiles_dir.mkdir(exist_ok=True)

scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'   # 5.1에서 EEVEE 식별자 (실측). GPU 없으면 'CYCLES'+16샘플로 폴백
scene.render.resolution_x = TILE
scene.render.resolution_y = TILE
scene.render.image_settings.file_format = 'PNG'

world = bpy.data.worlds.new("W") if scene.world is None else scene.world
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.15, 0.15, 0.15, 1)

sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", 'SUN'))
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(50), 0, math.radians(30))
scene.collection.objects.link(sun)

cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
scene.camera = cam

rig = bpy.data.objects["DummyRig"]

VIEWS = {  # (위치, 회전) — 타깃 (0,0,0.85), 거리 4m
    "front": ((0, -4, 0.85), (math.radians(90), 0, 0)),
    "side": ((4, 0, 0.85), (math.radians(90), 0, math.radians(90))),
    "threequarter": ((2.8, -2.8, 0.85), (math.radians(90), 0, math.radians(45))),
    "back": ((0, 4, 0.85), (math.radians(90), 0, math.radians(180))),
}


def set_view(name):
    cam.location, cam.rotation_euler = VIEWS[name]


def render_tile(filename):
    scene.render.filepath = str(tiles_dir / filename)
    bpy.ops.render.render(write_still=True)


def reset_pose():
    for pb in rig.pose.bones:
        pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()


def rotate_world(bone, axis, deg):
    """본을 자기 헤드 기준으로 월드축 회전 (부모부터 순서대로 호출할 것)"""
    pb = rig.pose.bones[bone]
    bpy.context.view_layer.update()
    head = pb.matrix.to_translation()
    R = Matrix.Translation(head) @ Matrix.Rotation(math.radians(deg), 4, axis) @ Matrix.Translation(-head)
    pb.matrix = R @ pb.matrix
    bpy.context.view_layer.update()


def translate_world(bone, dz):
    pb = rig.pose.bones[bone]
    bpy.context.view_layer.update()
    m = pb.matrix.copy()
    m.translation.z += dz
    pb.matrix = m
    bpy.context.view_layer.update()


order = []

# 1) 레스트 4각도
rig.data.pose_position = 'REST'
if rig.animation_data:
    rig.animation_data.action = None
for view in ("front", "side", "threequarter", "back"):
    set_view(view)
    render_tile(f"rest_{view}.png")
    order.append(f"rest_{view}.png")

# 2) 극단 포즈 3종 (POSE 모드, 액션 없음, deform 본 직접 회전)
rig.data.pose_position = 'POSE'

reset_pose()
# Task 9의 L/R 리네임 스왑(FBX X-미러 보상) 이후 "LeftUpperArm"은 Blender −X쪽 본이다
# → 팔을 위로 들려면 부호가 원안과 반대
rotate_world("LeftUpperArm", 'Y', 80)
rotate_world("RightUpperArm", 'Y', -80)
set_view("front")
render_tile("pose_armsup.png")
order.append("pose_armsup.png")

reset_pose()
translate_world("Hips", -0.30)
for side in ("Left", "Right"):
    rotate_world(f"{side}UpperLeg", 'X', -60)
    rotate_world(f"{side}LowerLeg", 'X', 100)
    rotate_world(f"{side}Foot", 'X', -40)
set_view("side")
render_tile("pose_crouch.png")
order.append("pose_crouch.png")

reset_pose()
rotate_world("Spine", 'Z', 25)
rotate_world("Chest", 'Z', 25)
set_view("front")
render_tile("pose_twist.png")
order.append("pose_twist.png")

# 3) idle 2프레임
reset_pose()
act = bpy.data.actions.get("Idle")
rig.animation_data.action = act
set_view("threequarter")
for frame in (1, 24):
    scene.frame_set(frame)
    render_tile(f"idle_f{frame}.png")
    order.append(f"idle_f{frame}.png")

# 4) 3x3 합성 (Blender 이미지 원점은 좌하단 → 행을 위에서부터 채우려면 뒤집기)
sheet = np.zeros((TILE * 3, TILE * 3, 4), dtype=np.float32)
for i, name in enumerate(order):
    img = bpy.data.images.load(str(tiles_dir / name))
    px = np.array(img.pixels[:], dtype=np.float32).reshape(TILE, TILE, 4)
    row, col = divmod(i, 3)
    y0 = TILE * 3 - (row + 1) * TILE
    sheet[y0:y0 + TILE, col * TILE:(col + 1) * TILE] = px

out = bpy.data.images.new("sheet", TILE * 3, TILE * 3, alpha=True)
out.pixels.foreach_set(sheet.ravel())
out.filepath_raw = str(paths.PREVIEWS_DIR / "contact_sheet.png")
out.file_format = 'PNG'
out.save()
print("CONTACT SHEET OK:", paths.PREVIEWS_DIR / "contact_sheet.png")
```

- [ ] **Step 2: 실행**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 sheet`
Expected: `CONTACT SHEET OK`, `previews/contact_sheet.png` 생성. EEVEE가 GPU 문제로 죽으면 `scene.render.engine = 'CYCLES'; scene.cycles.samples = 16; scene.cycles.device = 'CPU'`로 바꾼다.

- [ ] **Step 3: 에이전트 눈 검사 (이 계획의 실행자가 직접)**

`previews/contact_sheet.png`를 Read 도구로 읽고 확인한다: (a) 9칸 전부 렌더됨, (b) 레스트 4각도에서 사람 형상·T포즈·정면 방향 정상, (c) 팔들기/쪼그리기/비틀기에서 메시가 본을 따라감(따라가지 않으면 웨이트 바인딩 문제 — Task 4로), (d) idle 2칸이 포즈 파손 없음. **품질(변형의 아름다움)은 판정하지 않는다 — 스펙 2절.**

- [ ] **Step 4: 커밋**

```powershell
git add scripts\preview
git commit -m "feat: 3x3 contact sheet renderer for agent-side visual check"
```

---

### Task 11: 전 구간 통합 실행 + PLAN 갱신

**Files:**
- Modify: `PLAN.md` (Phase 0 체크박스)

**Interfaces:**
- Consumes: Task 1–10 전부

- [ ] **Step 1: 클린 상태에서 전체 실행**

```powershell
Remove-Item -Recurse -Force build, exports, previews -ErrorAction SilentlyContinue
powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all
echo "exit=$LASTEXITCODE"
```

Expected: `ALL STAGES PASSED`, `exit=0`. 중간 산출물이 전부 재생성되고 Unity 리포트 `allPass:true`.

- [ ] **Step 2: 컨택트 시트 재확인**

새로 생성된 `previews/contact_sheet.png`를 Read로 읽고 Task 10 Step 3 기준 재확인.

- [ ] **Step 3: `PLAN.md` Phase 0 체크박스 갱신**

Phase 0의 완료 항목을 `[x]`로 바꾼다. 단 마지막 항목(완료 기준)은 "사용자 확인" 부분이 남았으므로, 사용자가 Unity 에디터에서 직접 열어본 뒤에만 체크한다. 사용자에게 안내할 것: `unity/AvatarCheck`를 Unity Hub로 열면 `Assets/Dummy/dummy.fbx`가 Humanoid로 임포트되어 있다 — 아바타 구성 화면과 Idle 재생을 눈으로 확인해달라고 요청한다.

- [ ] **Step 4: 최종 커밋**

```powershell
git add PLAN.md
git commit -m "chore: mark Phase 0 pipeline tasks complete (user Unity eyeball pending)"
```

---

## 계획 자기검토 결과 (작성 시 반영됨)

- 스펙 5a의 단언 5종 ↔ Task 8 C# 단언 대응 확인: isValid/isHuman+임포트 로그(1,2), 필수 15본+의도 매핑(3), 힙 높이(4), +Z 방향+루트 회전(5), 클립 존재+실제 움직임(6,7). 스펙의 "0.9m ±10%"는 실측 힙 높이 0.8673 기준 0.87±0.10으로 구체화했다 (스펙의 값이 근사치였음).
- 스펙 6절 저장소 구조 ↔ Task 1/8 생성 경로 일치 확인.
- 본 이름 단일 원천(bone_map.py) ↔ C# `Required` 테이블은 언어 경계 때문에 중복이 불가피 — 대신 check_lib(Task 2)와 C# 쪽 테이블이 같은 15본을 쓰는지 Task 9 교정 루프가 실질 검증한다.
- Blender 5.1.2 API 확인 완료 항목: `preferences.addon_enable` 경로, `BLENDER_EEVEE` 식별자, `rigify_generate` 정상 동작, DEF 71본 목록(얼굴 제거 후), 컨트롤 본 `torso` 존재, FBX 파라미터 13종 존재. 미확인 항목(`bake_space_transform`, `nla.bake` 시그니처, `bake_anim_use_all_bones`)은 `op_kwargs` 필터로 방어하고 Task 9 교정 루프가 받는다.
