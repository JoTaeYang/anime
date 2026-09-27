# Phase 1 캐릭터 리깅 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `assets/character.blend`(Meshy AI 애니메 캐릭터)를 부속물(꼬리·치마·목도리·후드 귀) 포함으로 리깅해 Unity Humanoid 검증을 통과시키고, 사용자 품질 승인용 프리뷰까지 산출한다.

**Architecture:** Phase 0 파이프라인을 **프로필 시스템**으로 일반화한다(dummy/character 병행, dummy는 영구 회귀 테스트). 캐릭터 경로는 `00_intake`(반입·정규화)로 시작하고, `01_rig`가 프로필 랜드마크 테이블로 메타리그를 맞춘 뒤 부속물 체인을 심어 생성한다. 맞춤과 웨이트는 각각 에이전트 렌더 루프(오버레이/컨택트 시트)로 수렴시킨다. 스펙: `docs/superpowers/specs/2026-07-18-phase1-character-rigging-design.md`.

**Tech Stack:** Blender 5.1.2 bpy + numpy(번들), Rigify 0.6.10, Unity 6000.3.20f1 C#, PowerShell 5.1.

## Global Constraints

- 실행 파일·headless 플래그·PS 5.1 제약은 Phase 0 계획의 Global Constraints 그대로 (`--background --factory-startup --python-exit-code 1`, Unity는 run.ps1의 Start-Process 경유).
- **프로필 선택**: 환경변수 `ANIME_PROFILE` (run.ps1 `-Profile` 파라미터가 설정). 이 계획의 태스크들이 끝나기 전까지 run.ps1 기본값은 `dummy`, 마지막 통합 태스크에서 `character`로 전환.
- **더미 회귀는 항상 초록이어야 한다**: 어떤 태스크도 `run.ps1 all -Profile dummy`(9/9)를 깨뜨린 채 커밋할 수 없다.
- 본 이름 단일 원천은 여전히 `scripts/lib/bone_map.py`(몸체 71본) + **프로필의 부속물 매핑**. 다른 파일에 이름 리터럴 중복 금지.
- 캐릭터 실측(2026-07-18, Blender 5.1.2): 메시 오브젝트 `"Mesh_0"`, 정점 276,737, 전부 삼각형, bbox z **−0.951..+0.948**(원점 중심 부유 → intake에서 발 접지 필수), 키 1.899m, mean|x|=0.0094(대칭), 정면 −Y, T포즈, 트랜스폼 깨끗.
- Rigify 5.1 실측: `bpy.ops.object.metarig_sample_add`는 **존재하지 않는다**. 부속물은 에디트 본 체인을 직접 만들고 `pose.bones[루트본].rigify_type = "<타입>"`으로 지정한다(설정 가능 확인됨). 레지스트리에 `spines.basic_tail`, `limbs.simple_tentacle`, `basic.super_copy` 존재 확인됨.
- 좌우 감시(laterality centroid) 단언은 **더미 프로필 전용**. `faces_plus_z`는 공통.
- 커밋은 conventional commit + `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

## 파일 구조 (이 계획이 만들거나 고치는 것)

```
scripts/lib/profiles/__init__.py     # get_profile() 로더            [Task 1]
scripts/lib/profiles/dummy.py        # 더미 프로필                    [Task 1]
scripts/lib/profiles/character.py    # 캐릭터 프로필                  [Task 2]
scripts/lib/profiles/character_landmarks.py  # 관절 좌표 (도구 생성) [Task 3, 루프로 갱신]
scripts/stages/00_intake.py          # 캐릭터 반입·정규화             [Task 2]
scripts/tools/estimate_landmarks.py  # 랜드마크 초안 추정             [Task 3]
scripts/preview/metarig_overlay.py   # 본 오버레이 렌더               [Task 4]
scripts/lib/weight_tools.py          # 웨이트 보정 헬퍼               [Task 8]
scripts/preview/user_preview.py      # 턴테이블 MP4 + 스틸           [Task 9]
(수정) run.ps1, 00_mesh.py, 01_rig.py, 03_bake.py, 04_export.py,
       check_00/01/03.py, contact_sheet.py, AvatarCheck.cs
```

---

### Task 1: 프로필 시스템 도입 + 전 스테이지 일반화 (더미 회귀 유지)

가장 침습적인 리팩터. 목적: 하드코딩("Dummy", `proportions.P`, "dummy.fbx", 마커 단언)을 프로필 경유로 바꾸되, **동작은 한 치도 안 바뀐다** — 이 태스크의 성공 기준은 더미 파이프라인이 이전과 동일하게 9/9 초록.

**Files:**
- Create: `scripts/lib/profiles/__init__.py`, `scripts/lib/profiles/dummy.py`
- Modify: `scripts/run.ps1`, `scripts/stages/00_mesh.py`, `scripts/stages/01_rig.py`, `scripts/stages/03_bake.py`, `scripts/stages/04_export.py`, `scripts/checks/check_00.py`, `scripts/checks/check_01.py`, `scripts/checks/check_03.py`, `scripts/checks/check_04.py`, `scripts/preview/contact_sheet.py`, `unity/AvatarCheck/Assets/Editor/AvatarCheck.cs`

**Interfaces:**
- Produces: `profiles.get_profile() -> module` — 환경변수 `ANIME_PROFILE`(기본 "dummy")로 프로필 모듈 반환.
- Produces (프로필 계약 — dummy.py가 구현, character.py도 동일 계약):
  - `NAME: str`, `MESH_OBJECT: str` ("Dummy"), `SOURCE: dict` ({"kind": "procedural"})
  - `LANDMARKS: dict` (메타리그 본→head/tail; dummy는 `proportions.P` 그대로)
  - `METARIG_SCALE: float | None` (dummy 0.858759; character는 None=랜드마크 배치가 대신함)
  - `HIPS_HEIGHT: float`, `HIPS_TOL: float` (dummy 0.8673 / 0.10)
  - `HEIGHT_RANGE: tuple` (dummy (1.55, 1.80))
  - `FBX_NAME: str` ("dummy.fbx"), `USE_LATERALITY_MARKER: bool` (True)
  - `APPENDAGES: list` (dummy는 `[]` — 항목 형식은 Task 5에서 정의)
  - `appendage_bone_rename() -> dict` (dummy는 `{}`), `expected_def_bones() -> set` (BONE_RENAME 키 ∪ 부속물 DEF)
  - `EXTRA_POSES: list` (dummy `[]` — 컨택트 시트 추가 포즈, 형식 Task 7)
- Produces: `exports/<NAME>_meta.json` — `{"fbx": FBX_NAME, "expectedHipsY": .., "hipsTol": .., "checkLateralityMarker": bool, "appendageBones": [Unity이름...]}` (04_export가 기록, AvatarCheck가 읽음)

- [ ] **Step 1: 프로필 로더와 dummy 프로필 작성**

`scripts/lib/profiles/__init__.py`:
```python
import importlib
import os


def get_profile():
    name = os.environ.get("ANIME_PROFILE", "dummy")
    return importlib.import_module(f"lib.profiles.{name}")
```

`scripts/lib/profiles/dummy.py`:
```python
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
```

- [ ] **Step 2: run.ps1에 -Profile 파라미터 추가**

`param` 줄을 다음으로 교체:
```powershell
param(
    [Parameter(Position = 0)][string]$Stage = "all",
    [string]$Profile = "dummy"
)
$env:ANIME_PROFILE = $Profile
```
그리고 최종 성공 메시지에 프로필 표시: `Write-Host "ALL STAGES PASSED ($Profile)" -ForegroundColor Green`

- [ ] **Step 3: 스테이지·체크의 하드코딩을 프로필 경유로 교체**

각 파일 상단(기존 `sys.path.insert` 뒤)에 `from lib.profiles import get_profile` + `PROFILE = get_profile()`를 추가하고, 다음 치환을 적용한다:

| 파일 | 기존 | 변경 |
|---|---|---|
| `00_mesh.py` | `from lib.proportions import P` 사용 | `P = PROFILE.LANDMARKS` (절차 생성 로직은 그대로). 파일 첫머리에 가드: `assert PROFILE.SOURCE["kind"] == "procedural", "00_mesh는 procedural 프로필 전용 — character는 00_intake 사용"` |
| `00_mesh.py` | 마커 블록 무조건 실행 | `if PROFILE.USE_LATERALITY_MARKER:` 로 감싼다 |
| `00_mesh.py` | `dummy.name = "Dummy"` | `dummy.name = PROFILE.MESH_OBJECT` |
| `01_rig.py` | `from lib.proportions import SCALE, P` | `P = PROFILE.LANDMARKS`; 스케일 블록을 `if PROFILE.METARIG_SCALE: meta.scale = (PROFILE.METARIG_SCALE,)*3; ...apply...` 로 |
| `01_rig.py` | `bpy.data.objects["Dummy"]` | `bpy.data.objects[PROFILE.MESH_OBJECT]` |
| `03_bake.py` | `from lib.bone_map import BONE_RENAME` → 사용처 | `RENAME = PROFILE.bone_rename()` 로 통일 (베이크 선택/리네임/버텍스그룹 루프 전부 `RENAME` 사용) |
| `03_bake.py` | `bpy.data.objects["Dummy"]` | `bpy.data.objects[PROFILE.MESH_OBJECT]` |
| `04_export.py` | `"dummy.fbx"` | `PROFILE.FBX_NAME` |
| `check_00.py` | `objects.get("Dummy")`, `1.55 < dims.z < 1.80` | `PROFILE.MESH_OBJECT`, `PROFILE.HEIGHT_RANGE[0] < dims.z < PROFILE.HEIGHT_RANGE[1]` |
| `check_00.py` | 재질 "Checker" 단언 | `if PROFILE.SOURCE["kind"] == "procedural":` 가드 (캐릭터는 자체 재질) |
| `check_01.py` | `set(BONE_RENAME.keys())` | `PROFILE.expected_def_bones()` |
| `check_01.py` | `objects.get("Dummy")` | `PROFILE.MESH_OBJECT` |
| `check_03.py` | `set(BONE_RENAME.values())`, `objects["Dummy"]` | `set(PROFILE.bone_rename().values())`, `PROFILE.MESH_OBJECT` |
| `check_04.py` | `"dummy.fbx"` | `PROFILE.FBX_NAME` |
| `contact_sheet.py` | (변경 최소) | 포즈 목록 뒤에 `PROFILE.EXTRA_POSES` 순회 렌더 추가 — 형식은 Task 7에서 채우므로 지금은 `for pose in PROFILE.EXTRA_POSES: pass` 자리만 두지 말고, Task 7 형식을 선반영: 아래 Step 4 코드 참조 |

- [ ] **Step 4: contact_sheet.py 추가 포즈 지원 (선반영)**

idle 렌더 블록 앞에 삽입 (형식: `EXTRA_POSES = [{"name": str, "view": str, "ops": [("rotate", 본, 축, 도) | ("translate", 본, dz)]}]`):
```python
for pose in PROFILE.EXTRA_POSES:
    reset_pose()
    for op in pose["ops"]:
        if op[0] == "rotate":
            rotate_world(op[1], op[2], op[3])
        elif op[0] == "translate":
            translate_world(op[1], op[2])
    set_view(pose["view"])
    render_tile(f"pose_{pose['name']}.png")
    order.append(f"pose_{pose['name']}.png")
```
그리고 시트 격자를 가변으로: `rows = math.ceil(len(order) / 3)`; `sheet = np.zeros((TILE * rows, TILE * 3, 4), ...)`; `y0 = TILE * rows - (row + 1) * TILE`. (9칸 고정 가정 제거 — dummy는 여전히 9칸 3×3.)

- [ ] **Step 5: 04_export가 meta.json 기록**

익스포트 호출 뒤에 추가:
```python
import json
meta = {
    "fbx": PROFILE.FBX_NAME,
    "expectedHipsY": PROFILE.HIPS_HEIGHT,
    "hipsTol": PROFILE.HIPS_TOL,
    "checkLateralityMarker": PROFILE.USE_LATERALITY_MARKER,
    "appendageBones": sorted(PROFILE.appendage_bone_rename().values()),
}
with open(paths.EXPORTS_DIR / f"{PROFILE.NAME}_meta.json", "w") as f:
    json.dump(meta, f, indent=1)
print("META OK:", meta)
```

- [ ] **Step 6: AvatarCheck.cs 일반화**

수정 지점 4곳 (기존 단언 로직·문턱은 유지):
1. 클래스 상단에 meta 로드 — `FbxAssetPath`/원본 경로의 `"dummy.fbx"` 리터럴을 meta의 `fbx` 값으로 대체:
```csharp
[Serializable]
class PipelineMeta {
    public string fbx; public float expectedHipsY; public float hipsTol;
    public bool checkLateralityMarker; public string[] appendageBones;
}
static PipelineMeta LoadMeta() {
    string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "exports"));
    var files = Directory.GetFiles(dir, "*_meta.json");
    if (files.Length != 1) throw new FileNotFoundException($"expected exactly one *_meta.json in {dir}, found {files.Length}");
    return JsonUtility.FromJson<PipelineMeta>(File.ReadAllText(files[0]));
}
```
`Run()` 첫 줄에서 `var meta = LoadMeta();`, `CopyFbxIntoProject(meta.fbx)` 로 변경 (대상 프로젝트 경로는 `Assets/Dummy/model.fbx` 고정 — 프로필 무관 단일 자리).
2. `hips_height` 단언: `hipsY > meta.expectedHipsY - meta.hipsTol && hipsY < meta.expectedHipsY + meta.hipsTol` (기존 0.77/0.97 리터럴 제거).
3. `laterality` 단언 블록: `if (meta.checkLateralityMarker) { ... }` 가드. 가드로 건너뛴 경우 report에 `("laterality", true, "skipped: profile has no marker")` 기록.
4. 새 단언 `appendages_present`: `foreach (var b in meta.appendageBones)` — 인스턴스 계층에서 `FindDeep(instance.transform, b) != null` (재귀 탐색 헬퍼 추가), 없으면 fail. `meta.appendageBones`가 빈 배열이면 `("appendages_present", true, "none declared")`.

- [ ] **Step 7: 더미 회귀 실행**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile dummy`
Expected: `ALL STAGES PASSED (dummy)`, report.json `allPass:true`, 단언 10개(기존 9 + appendages_present none). 실패 시 이 태스크는 완료 아님 — 리팩터가 동작을 바꾼 것.

- [ ] **Step 8: 커밋**

```powershell
git add scripts unity\AvatarCheck\Assets\Editor\AvatarCheck.cs
git commit -m "refactor: profile system generalizing pipeline (dummy regression green)"
```

---

### Task 2: character 프로필 + 00_intake

**Files:**
- Create: `scripts/lib/profiles/character.py`, `scripts/stages/00_intake.py`
- Modify: `scripts/run.ps1` (mesh 스테이지가 프로필에 따라 00_mesh/00_intake 선택)

**Interfaces:**
- Consumes: 프로필 계약(Task 1), `assets/character.blend` (실측: "Mesh_0", z −0.951..+0.948)
- Produces: `build/00_mesh.blend` — 오브젝트 **"Character"**, 발 z=0, 키 1.899m, 트랜스폼 적용 완료. character 프로필: `MESH_OBJECT="Character"`, `HIPS_HEIGHT`는 Task 3 랜드마크 확정 후 그 값으로 갱신(초안 1.03 = 키의 54%), `USE_LATERALITY_MARKER=False`, `FBX_NAME="character.fbx"`

- [ ] **Step 1: character.py 작성**

```python
from lib.bone_map import BONE_RENAME
from lib.profiles.character_landmarks import LANDMARKS as _LM  # Task 3 생성; 그 전까지는 아래 참조

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
```
주: Task 3 전까지 `character_landmarks.py`가 없으므로 이 태스크에서는 **빈 초안**을 함께 만든다: `LANDMARKS = {}` 한 줄 파일. (01_rig는 character에서 아직 실행 불가 — intake까지만 이 태스크의 범위.)

- [ ] **Step 2: 00_intake.py 작성**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import save_as
from lib.profiles import get_profile

PROFILE = get_profile()
assert PROFILE.SOURCE["kind"] == "blend", "00_intake는 blend 소스 프로필 전용"

src = paths.PROJECT_ROOT / PROFILE.SOURCE["path"]
bpy.ops.wm.open_mainfile(filepath=str(src))
paths.ensure_dirs()

ob = bpy.data.objects[PROFILE.SOURCE["object"]]
assert ob.type == 'MESH'

# 사전 점검: 트랜스폼 적용 상태, 대칭
assert all(abs(s - 1.0) < 1e-4 for s in ob.scale), f"scale not applied: {tuple(ob.scale)}"
assert all(abs(a) < 1e-4 for a in ob.rotation_euler), "rotation not applied"
import numpy as np
n = len(ob.data.vertices)
co = np.empty(n * 3, dtype=np.float32)
ob.data.vertices.foreach_get("co", co)
co = co.reshape(n, 3)
assert abs(float(co[:, 0].mean())) < 0.02, "mesh not X-symmetric"

# 정규화: 발 접지 (bbox min z -> 0), 원점 월드 0
ob.location.z -= float(co[:, 2].min()) + ob.location.z
bpy.ops.object.select_all(action='DESELECT')
ob.select_set(True)
bpy.context.view_layer.objects.active = ob
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

ob.name = PROFILE.MESH_OBJECT
# 정면 -Y 확인: 발끝(낮은 z 슬라이스)의 y 중심이 음수여야 함 (발이 앞으로 나옴)
low = co[co[:, 2] < co[:, 2].min() + 0.12]
assert float(low[:, 1].mean()) < 0.0, f"feet not pointing -Y: {float(low[:, 1].mean())}"

save_as(paths.blend_path("00_mesh"))
print("STAGE 00_intake OK height=", round(ob.dimensions.z, 3))
```
(주: `low`는 정규화 전 좌표 배열이지만 y축은 이동하지 않으므로 판정에 유효. z 이동만 있었음.)

- [ ] **Step 3: run.ps1 — mesh 스테이지 분기**

`$Pipeline` 정의를 함수화하거나, `"mesh"` 항목을 실행 직전에 프로필로 결정:
```powershell
$MeshStage = "scripts\stages\00_mesh.py"
if ($Profile -ne "dummy") { $MeshStage = "scripts\stages\00_intake.py" }
$Pipeline = [ordered]@{
    "mesh"   = @($MeshStage, "scripts\checks\check_00.py")
    ...나머지 동일...
}
```
(정확히는 kind 기반이 맞지만 PS에서 파이썬 프로필을 읽을 수 없으므로 이름 규칙으로: dummy만 절차 생성. 프로필이 늘어나면 재검토 — YAGNI.)

- [ ] **Step 4: 실행 검증 (TDD: check_00은 Task 1에서 이미 일반화됨)**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 mesh -Profile character`
Expected: `STAGE 00_intake OK height= 1.899`, `CHECK_00 OK` (HEIGHT_RANGE 1.85–1.95 통과), exit 0.
Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 mesh -Profile dummy`
Expected: 여전히 통과 (회귀).

- [ ] **Step 5: 커밋**

```powershell
git add scripts\lib\profiles scripts\stages\00_intake.py scripts\run.ps1
git commit -m "feat: character profile and 00_intake normalization stage"
```

---

### Task 3: 랜드마크 추정 도구 + 초안 좌표

**Files:**
- Create: `scripts/tools/estimate_landmarks.py`, `scripts/lib/profiles/character_landmarks.py`(도구 출력으로 생성)

**Interfaces:**
- Consumes: `build/00_mesh.blend` (character)
- Produces: `character_landmarks.LANDMARKS: dict` — Phase 0 `proportions.P`와 동일 형식(메타리그 본명 → {"head":[x,y,z], "tail":[x,y,z]}, 왼쪽만; 최소 집합은 P와 같은 15개 본). 이후 Task 5/6이 이 테이블로 메타리그를 배치·수정한다.

- [ ] **Step 1: estimate_landmarks.py 작성**

전략: z-슬라이스 폭 프로파일로 주요 관절 높이를 찾고, 팔은 x-극값 클러스터로 찾는다. T포즈·대칭 전제(intake가 보장).

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
import numpy as np
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile

PROFILE = get_profile()
open_blend(paths.blend_path("00_mesh"))
ob = bpy.data.objects[PROFILE.MESH_OBJECT]
n = len(ob.data.vertices)
co = np.empty(n * 3, dtype=np.float32)
ob.data.vertices.foreach_get("co", co)
co = co.reshape(n, 3)
H = float(co[:, 2].max())


def slice_at(z, band=0.015):
    return co[(co[:, 2] > z - band) & (co[:, 2] < z + band)]


def width(z):
    s = slice_at(z)
    return float(s[:, 0].max() - s[:, 0].min()) if len(s) else 0.0


# --- 팔: x 극값에서 손끝, 몸통 경계에서 어깨 ---
hand_tip_x = float(co[:, 0].max())
arm_band = co[co[:, 0] > hand_tip_x * 0.55]          # 팔 영역 (몸통 밖)
arm_z = float(np.median(arm_band[:, 2]))              # 팔 높이 (T포즈 수평)
arm_y = float(np.median(arm_band[:, 1]))
# 몸통 반폭: 팔 높이에서 몸통만 남긴 슬라이스 (|x| 작은 클러스터의 최대 |x|)
torso_sl = slice_at(arm_z)
torso_half = float(np.percentile(np.abs(torso_sl[:, 0]), 35))
shoulder_x = torso_half * 1.05
wrist_x = hand_tip_x * 0.80                            # 손 길이를 팔 20%로 가정(초안)
elbow_x = (shoulder_x + wrist_x) / 2

# --- 다리/몸통 높이: 폭 프로파일 특징점 ---
zs = np.linspace(0.02 * H, 0.98 * H, 97)
ws = np.array([width(z) for z in zs])
# 가랑이: 하반신(15..45%H)에서 폭이 최소가 되는 높이 부근의 위쪽
low_i = (zs > 0.15 * H) & (zs < 0.45 * H)
crotch_z = float(zs[low_i][np.argmin(ws[low_i])])
hips_z = crotch_z + 0.06 * H
# 무릎/발목: 가랑이-바닥 구간의 등분(초안; 루프에서 실루엣 보고 보정)
knee_z = crotch_z * 0.55
ankle_z = 0.045 * H
# 다리 x: 하반신 슬라이스의 |x| 중앙값
leg_sl = slice_at(knee_z)
leg_x = float(np.median(np.abs(leg_sl[:, 0])))
# 목/머리: 상단에서 폭 최소(목) — 팔 위 영역만
top_i = (zs > arm_z + 0.03 * H) & (zs < 0.93 * H)
neck_z = float(zs[top_i][np.argmin(ws[top_i])]) if top_i.any() else 0.85 * H
head_top = H
# 척추 분할: hips..neck을 P와 같은 비율로 4분할
spine_zs = np.linspace(hips_z, neck_z, 5)

LM = {
    "spine":       {"head": [0.0, 0.0, round(hips_z, 4)], "tail": [0.0, 0.0, round(float(spine_zs[1]), 4)]},
    "spine.001":   {"head": [0.0, 0.0, round(float(spine_zs[1]), 4)], "tail": [0.0, 0.0, round(float(spine_zs[2]), 4)]},
    "spine.002":   {"head": [0.0, 0.0, round(float(spine_zs[2]), 4)], "tail": [0.0, 0.0, round(float(spine_zs[3]), 4)]},
    "spine.003":   {"head": [0.0, 0.0, round(float(spine_zs[3]), 4)], "tail": [0.0, 0.0, round(neck_z, 4)]},
    "spine.004":   {"head": [0.0, 0.0, round(neck_z, 4)], "tail": [0.0, 0.0, round(neck_z + (head_top - neck_z) * 0.18, 4)]},
    "spine.005":   {"head": [0.0, 0.0, round(neck_z + (head_top - neck_z) * 0.18, 4)], "tail": [0.0, 0.0, round(neck_z + (head_top - neck_z) * 0.36, 4)]},
    "spine.006":   {"head": [0.0, 0.0, round(neck_z + (head_top - neck_z) * 0.36, 4)], "tail": [0.0, 0.0, round(head_top, 4)]},
    "shoulder.L":  {"head": [round(shoulder_x * 0.15, 4), round(arm_y, 4), round(arm_z + 0.02, 4)],
                    "tail": [round(shoulder_x, 4), round(arm_y, 4), round(arm_z + 0.02, 4)]},
    "upper_arm.L": {"head": [round(shoulder_x, 4), round(arm_y, 4), round(arm_z, 4)],
                    "tail": [round(elbow_x, 4), round(arm_y, 4), round(arm_z, 4)]},
    "forearm.L":   {"head": [round(elbow_x, 4), round(arm_y, 4), round(arm_z, 4)],
                    "tail": [round(wrist_x, 4), round(arm_y, 4), round(arm_z, 4)]},
    "hand.L":      {"head": [round(wrist_x, 4), round(arm_y, 4), round(arm_z, 4)],
                    "tail": [round(hand_tip_x * 0.97, 4), round(arm_y, 4), round(arm_z, 4)]},
    "thigh.L":     {"head": [round(leg_x, 4), 0.0, round(hips_z - 0.02, 4)],
                    "tail": [round(leg_x, 4), 0.0, round(knee_z, 4)]},
    "shin.L":      {"head": [round(leg_x, 4), 0.0, round(knee_z, 4)],
                    "tail": [round(leg_x, 4), 0.0, round(ankle_z, 4)]},
    "foot.L":      {"head": [round(leg_x, 4), 0.0, round(ankle_z, 4)],
                    "tail": [round(leg_x, 4), -0.09, round(0.015, 4)]},
    "toe.L":       {"head": [round(leg_x, 4), -0.09, 0.015], "tail": [round(leg_x, 4), -0.14, 0.015]},
}

out = paths.PROJECT_ROOT / "scripts" / "lib" / "profiles" / "character_landmarks.py"
with open(out, "w", encoding="utf-8") as f:
    f.write("# estimate_landmarks.py 초안 + 오버레이 루프 수동 보정. 좌표는 미터.\n")
    f.write("LANDMARKS = ")
    import pprint
    f.write(pprint.pformat(LM, width=100))
    f.write("\n")
print("LANDMARKS WRITTEN:", out)
for k, v in LM.items():
    print(" ", k, v)
```

- [ ] **Step 2: 실행**

Run: `$env:ANIME_PROFILE="character"; & "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe" --background --factory-startup --python-exit-code 1 --python scripts\tools\estimate_landmarks.py`
Expected: `LANDMARKS WRITTEN`, `character_landmarks.py` 생성, 좌표들이 상식 범위(hips_z ≈ 0.9–1.1, arm_z ≈ 1.2–1.5).

- [ ] **Step 3: character.py의 HIPS_HEIGHT를 출력된 `spine.head[2]`로 갱신**

- [ ] **Step 4: 커밋**

```powershell
git add scripts\tools\estimate_landmarks.py scripts\lib\profiles\character_landmarks.py scripts\lib\profiles\character.py
git commit -m "feat: landmark estimation tool and initial character joint table"
```

---

### Task 4: 메타리그 오버레이 렌더

**Files:**
- Create: `scripts/preview/metarig_overlay.py`

**Interfaces:**
- Consumes: `build/01_rigged.blend` (없으면 메타리그만 배치된 상태를 자체 구성: 00_mesh 로드 + 랜드마크로 메타리그 배치 — 01_rig 이전에도 맞춤 확인 가능해야 함)
- Produces: `previews/overlay_{front,side,threequarter}.png` — 메시 반투명 + 본 프록시(머리→꼬리 원뿔대) 발광 렌더. Task 6 루프의 눈.

- [ ] **Step 1: metarig_overlay.py 작성**

```python
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from mathutils import Vector
from lib import paths
from lib.blender_utils import open_blend, ensure_rigify
from lib.profiles import get_profile

PROFILE = get_profile()
rigged = paths.blend_path("01_rigged")

if rigged.exists():
    open_blend(rigged)
    arm = bpy.data.objects.get("metarig") or bpy.data.objects["rig"]
else:
    # 01_rig 이전: 00_mesh 위에 랜드마크로 메타리그만 배치해서 본다
    open_blend(paths.blend_path("00_mesh"))
    ensure_rigify()
    bpy.ops.object.armature_human_metarig_add()
    arm = bpy.context.active_object
    bpy.ops.object.mode_set(mode='EDIT')
    eb = arm.data.edit_bones
    for name, ht in PROFILE.LANDMARKS.items():
        for side_name, mirror in ((name, 1.0), (name.replace(".L", ".R"), -1.0)):
            if side_name in eb:
                eb[side_name].head = (ht["head"][0] * mirror, ht["head"][1], ht["head"][2])
                eb[side_name].tail = (ht["tail"][0] * mirror, ht["tail"][1], ht["tail"][2])
    bpy.ops.object.mode_set(mode='OBJECT')

# 본 프록시 메시 (원뿔대: head가 굵고 tail이 뾰족)
mat = bpy.data.materials.new("BoneProxy")
mat.use_nodes = True
bsdf = mat.node_tree.nodes["Principled BSDF"]
bsdf.inputs["Emission Color"].default_value = (1.0, 0.1, 0.1, 1.0)
bsdf.inputs["Emission Strength"].default_value = 3.0

proxies = []
for b in arm.data.bones:
    if b.name.startswith(("ORG-", "MCH-", "WGT-")):
        continue
    head, tail = Vector(b.head_local), Vector(b.tail_local)
    vec = tail - head
    if vec.length < 1e-5:
        continue
    bpy.ops.mesh.primitive_cone_add(radius1=max(0.012, vec.length * 0.07), radius2=0.002,
                                    depth=vec.length, location=(head + tail) / 2)
    p = bpy.context.active_object
    p.rotation_mode = 'QUATERNION'
    p.rotation_quaternion = vec.to_track_quat('Z', 'Y')
    p.data.materials.append(mat)
    proxies.append(p)

# 메시 반투명
mesh = bpy.data.objects[PROFILE.MESH_OBJECT]
for slot in mesh.material_slots:
    m = slot.material
    if m and m.use_nodes:
        pb = m.node_tree.nodes.get("Principled BSDF")
        if pb:
            pb.inputs["Alpha"].default_value = 0.35
        m.blend_method = 'BLEND'

scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_x = 900
scene.render.resolution_y = 900
scene.render.image_settings.file_format = 'PNG'
world = scene.world or bpy.data.worlds.new("W")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.1, 0.1, 0.1, 1)
sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", 'SUN'))
sun.data.energy = 2.0
sun.rotation_euler = (math.radians(50), 0, math.radians(30))
scene.collection.objects.link(sun)
cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
scene.camera = cam
cz = mesh.dimensions.z / 2
for name, loc, rz in (("front", (0, -4, cz), 0), ("side", (4, 0, cz), 90), ("threequarter", (2.8, -2.8, cz), 45)):
    cam.location = loc
    cam.rotation_euler = (math.radians(90), 0, math.radians(rz))
    scene.render.filepath = str(paths.PREVIEWS_DIR / f"overlay_{name}.png")
    bpy.ops.render.render(write_still=True)
    print("OVERLAY", name)
print("OVERLAY OK")
```

- [ ] **Step 2: run.ps1에 `overlay` 스테이지 등록**

switch에 추가: `"overlay" { Invoke-Blender "scripts\preview\metarig_overlay.py" }`

- [ ] **Step 3: 실행 (01_rig 전이므로 자체 배치 경로)**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 overlay -Profile character`
Expected: `OVERLAY OK`, PNG 3장. 에이전트가 Read로 열어 본이 몸 안에 대략 들어갔는지 확인 (정밀 수렴은 Task 6).

- [ ] **Step 4: 커밋**

```powershell
git add scripts\preview\metarig_overlay.py scripts\run.ps1
git commit -m "feat: metarig overlay renderer for fitting loop"
```

---

### Task 5: 01_rig 랜드마크 배치 + 부속물 체인

**Files:**
- Modify: `scripts/stages/01_rig.py`, `scripts/lib/profiles/character.py` (APPENDAGES 채움)

**Interfaces:**
- Consumes: `PROFILE.LANDMARKS`, `PROFILE.APPENDAGES`
- Produces: APPENDAGES 항목 형식(이 형식이 계약):
```python
{"chain": "tail", "rigify_type": "spines.basic_tail", "parent": "spine",
 "points": [[0.0, 0.10, 1.00], [0.0, 0.28, 1.02], [0.0, 0.45, 1.08], [0.0, 0.58, 1.18], [0.0, 0.66, 1.30], [0.0, 0.70, 1.42]],
 "rename": {"DEF-tail": "Tail1", "DEF-tail.001": "Tail2", "DEF-tail.002": "Tail3", "DEF-tail.003": "Tail4", "DEF-tail.004": "Tail5"}}
```
  (points N+1개 → 본 N개 연결 체인. 좌표 초안은 렌더 관찰 기반 — Task 6 루프에서 보정. 치마 8체인×2본, 목도리 2체인×2본은 `limbs.simple_tentacle`, 후드 귀는 1본 `basic.super_copy`.)
- Produces: `build/01_rigged.blend` (character) — DEF = 몸체 71 + 부속물

- [ ] **Step 1: character.py에 APPENDAGES 초안 기입**

꼬리(위 예시), 치마 8체인(가랑이 높이 링: 각도 45° 간격, 반지름 ~0.25, 각 체인 [상단점, 중간, 하단] 2본), 목도리 2체인, 후드 귀 2개. 전체 좌표는 `previews/char_*.png` 관찰로 초안을 넣고 Task 6에서 보정한다. rename 규칙: `SkirtF1a/F1b`, `SkirtB1a/…`, `ScarfL1a/…`, `HoodEarL` 형식 — **모든 rename 값은 유일해야 하며 check_lib 스타일 검증을 profiles에 추가**:
```python
# character.py 말미
_r = appendage_bone_rename()
assert len(_r) == len(set(_r.values())), "appendage rename collision"
assert not (set(_r.values()) & set(BONE_RENAME.values())), "appendage name collides with body"
```

- [ ] **Step 2: 01_rig.py 확장**

스케일 블록(Task 1에서 조건화) 다음에, 랜드마크 배치와 부속물 체인 추가:
```python
bpy.ops.object.mode_set(mode='EDIT')
eb = meta.data.edit_bones
# 1) 랜드마크 배치 (양측 미러)
for name, ht in PROFILE.LANDMARKS.items():
    for side_name, mirror in ((name, 1.0), (name.replace(".L", ".R"), -1.0)):
        if side_name in eb:
            eb[side_name].head = (ht["head"][0] * mirror, ht["head"][1], ht["head"][2])
            eb[side_name].tail = (ht["tail"][0] * mirror, ht["tail"][1], ht["tail"][2])
# 2) 손가락: 손 본 변화에 맞춰 이동+스케일 (wrist 기준 상대 변환)
#    기본 메타리그 hand.L 대비 새 hand.L의 (이동, 길이비)을 손가락·팜 전체에 적용
#    (기본값은 lib.proportions.P["hand.L"] — dummy 좌표가 기본 메타리그 스케일본이므로 재사용)
from lib.proportions import P as _DEFAULT
from mathutils import Vector as _V
for side, mirror in (("L", 1.0), ("R", -1.0)):
    old_wrist = _V(_DEFAULT["hand.L"]["head"]); old_wrist.x *= mirror
    new_wrist = _V(PROFILE.LANDMARKS["hand.L"]["head"]); new_wrist.x *= mirror
    old_len = (_V(_DEFAULT["hand.L"]["tail"]) - _V(_DEFAULT["hand.L"]["head"])).length
    new_len = (_V(PROFILE.LANDMARKS["hand.L"]["tail"]) - _V(PROFILE.LANDMARKS["hand.L"]["head"])).length
    k = new_len / old_len if old_len > 1e-6 else 1.0
    for b in eb:
        if b.name.endswith(f".{side}") and any(b.name.startswith(p) for p in ("thumb", "f_", "palm")):
            b.head = new_wrist + (b.head - old_wrist) * k
            b.tail = new_wrist + (b.tail - old_wrist) * k
# 3) 부속물 체인 생성
for app in PROFILE.APPENDAGES:
    pts = [(_V(p)) for p in app["points"]]
    prev = None
    for i in range(len(pts) - 1):
        name = app["chain"] if i == 0 else f"{app['chain']}.{i:03d}"
        nb = eb.new(name)
        nb.head, nb.tail = pts[i], pts[i + 1]
        nb.use_connect = i > 0
        nb.parent = prev if prev else eb[app["parent"]]
        prev = nb
bpy.ops.object.mode_set(mode='OBJECT')
# 4) rigify 타입 지정 (포즈 본에)
for app in PROFILE.APPENDAGES:
    meta.pose.bones[app["chain"]].rigify_type = app["rigify_type"]
```
기존 "3) 스케일 검증" 블록은 `if PROFILE.METARIG_SCALE:` 가드 안에만 두고, character 경로에서는 `meta.data.bones["spine"].head_local.z ≈ PROFILE.HIPS_HEIGHT` 검증으로 대체.
(주: 랜드마크 미러 배치와 부속물 생성은 dummy에서도 실행되지만 dummy의 LANDMARKS=P·APPENDAGES=[]이므로 결과 불변 — 회귀로 증명.)

- [ ] **Step 3: 실행 — 두 프로필 모두**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 rig -Profile dummy`
Expected: 여전히 `CHECK_01 OK` (회귀 — 배치가 P 좌표와 동일하므로).
Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 rig -Profile character`
Expected: `STAGE 01_rig OK`, `CHECK_01 OK` (expected_def_bones에 부속물 포함). 55만 정점 자동 웨이트는 수 분 걸림.
알려진 실패: `rigify_generate`가 부속물 타입 파라미터 오류를 내면 해당 rigify_type의 요구 조건(최소 본 수 등)을 오류 메시지로 확인 — basic_tail은 체인 ≥2본, simple_tentacle ≥2본, 단일 본은 super_copy.

- [ ] **Step 4: 커밋**

```powershell
git add scripts\stages\01_rig.py scripts\lib\profiles\character.py
git commit -m "feat: landmark-driven metarig fitting and appendage chains"
```

---

### Task 6: 맞춤 수렴 루프 (에이전트 반복 — 새 코드 없음)

**Files:**
- Modify (데이터만): `scripts/lib/profiles/character_landmarks.py`, `character.py`의 APPENDAGES 좌표

**Interfaces:**
- Consumes: Task 4 오버레이, Task 5 배치 경로
- Produces: 수렴된 좌표 테이블 (이후 태스크의 전제)

- [ ] **Step 1: 오버레이 + 컨택트 시트 생성 → 에이전트 판독**

Run: `run.ps1 overlay -Profile character` 후 `previews/overlay_*.png` 3장을 Read.
수렴 기준 (전부 만족할 때까지 Step 2 반복):
- 머리 본(spine.006)이 머리 부피 중앙을 관통 (후드 아님, 두개골 기준)
- 어깨·팔꿈치·손목 본 끝점이 실루엣 관절 위치와 일치 (팔 본이 살 밖으로 안 나감)
- 힙(spine head)이 가랑이 위 골반 중심, 무릎 본이 무릎 실루엣에
- 꼬리 체인이 꼬리 부피 안을 따라 흐름, 치마 체인이 치마 천 안에, 귀 본이 후드 귀 안에
- 손가락 클러스터가 손 부피 안에 (개별 손가락 정밀도는 요구하지 않음 — 미튼 수준 허용)

- [ ] **Step 2: 어긋난 좌표를 데이터 파일에서 수정 → 재실행**

`character_landmarks.py`(몸체) / `character.py` APPENDAGES(부속물) 좌표 수정 → `run.ps1 rig -Profile character` → Step 1 재판독. 오버레이는 01_rigged가 있으면 그걸 쓰므로 rig 재실행 후 overlay 재실행.

- [ ] **Step 3: 수렴 시 좌표 커밋**

```powershell
git add scripts\lib\profiles\character_landmarks.py scripts\lib\profiles\character.py
git commit -m "fix: converged character landmark and appendage coordinates"
```

---

### Task 7: 부속물 검증 관통 (03/04/Unity/시트 확장)

**Files:**
- Modify: `scripts/lib/profiles/character.py` (EXTRA_POSES), (필요시) `scripts/checks/check_03.py`

**Interfaces:**
- Consumes: Task 1의 meta.json 배선(appendageBones 단언 포함 — 이미 구현됨), Task 5의 rename
- Produces: character 프로필로 bake→export→unity 전 구간 통과

- [ ] **Step 1: EXTRA_POSES 기입**

```python
EXTRA_POSES = [
    {"name": "tail_lift", "view": "side",
     "ops": [("rotate", "Tail1", 'X', -40), ("rotate", "Tail2", 'X', -25), ("rotate", "Tail3", 'X', -15)]},
    {"name": "leg_spread", "view": "front",
     "ops": [("rotate", "LeftUpperLeg", 'Y', -35), ("rotate", "RightUpperLeg", 'Y', 35)]},
]
```

- [ ] **Step 2: check_03의 계층 CHAINS는 몸체 전용이므로 부속물 루트 확인 1줄 추가**

`CHAINS` 단언 뒤에:
```python
for uni in PROFILE.appendage_bone_rename().values():
    assert uni in names, f"appendage bone {uni} missing after bake"
```
(rootedness 단언은 이미 전 본을 커버 — 부속물도 Hips 트리에 속해야 통과.)

- [ ] **Step 3: 전 구간 실행 — 두 프로필**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile character`
Expected: unity 단계까지 통과 — report.json `allPass:true`, `appendages_present` 통과, `hips_height`는 meta 기대치 기준, `laterality`는 skipped 기록. 컨택트 시트는 11칸(9+2) 4행 시트로 생성.
주의: exports에 meta.json이 dummy 것과 공존하면 LoadMeta가 2개를 발견해 실패한다 — **04_export가 자기 프로필 것만 남기도록 기존 `*_meta.json` 삭제 후 기록**하는 줄을 추가:
```python
for old in paths.EXPORTS_DIR.glob("*_meta.json"):
    old.unlink()
```
Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile dummy`
Expected: 회귀 초록.

- [ ] **Step 4: 커밋**

```powershell
git add scripts
git commit -m "feat: appendage verification through bake/export/unity, extended pose sheet"
```

---

### Task 8: 웨이트 품질 루프

**Files:**
- Create: `scripts/lib/weight_tools.py`
- Create: `scripts/tools/fix_weights.py` (보정 실행 스크립트 — 발견된 문제에 맞춰 이 파일에 보정 호출을 누적)
- Modify: `scripts/stages/01_rig.py` (자동 웨이트 직후 fix_weights 호출)

**Interfaces:**
- Produces: `weight_tools.select_verts_by_box(obj, min_co, max_co) -> list[int]`, `weight_tools.reduce_influence(obj, group_name, vert_indices, factor)`, `weight_tools.smooth_groups(obj, group_names, factor=0.5, repeat=3)`, `weight_tools.limit_total(obj, max_influences=4)`

- [ ] **Step 1: weight_tools.py 작성**

```python
import bpy


def select_verts_by_box(obj, min_co, max_co):
    """월드 AABB 안 정점 인덱스 (트랜스폼 적용된 메시 전제)."""
    out = []
    for v in obj.data.vertices:
        c = v.co
        if all(min_co[i] <= c[i] <= max_co[i] for i in range(3)):
            out.append(v.index)
    return out


def reduce_influence(obj, group_name, vert_indices, factor):
    """지정 정점에서 특정 그룹 웨이트를 factor배로 (0이면 제거)."""
    vg = obj.vertex_groups.get(group_name)
    if vg is None:
        return
    idx = vg.index
    for vi in vert_indices:
        for g in obj.data.vertices[vi].groups:
            if g.group == idx:
                if factor <= 0.0:
                    vg.remove([vi])
                else:
                    vg.add([vi], g.weight * factor, 'REPLACE')
                break


def smooth_groups(obj, group_names, factor=0.5, repeat=3):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    prev_mode = obj.mode
    bpy.ops.object.mode_set(mode='WEIGHT_PAINT')
    for name in group_names:
        vg = obj.vertex_groups.get(name)
        if vg is None:
            continue
        obj.vertex_groups.active_index = vg.index
        bpy.ops.object.vertex_group_smooth(group_select_mode='ACTIVE', factor=factor, repeat=repeat)
    bpy.ops.object.mode_set(mode=prev_mode if prev_mode != 'WEIGHT_PAINT' else 'OBJECT')


def limit_total(obj, max_influences=4):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.vertex_group_limit_total(limit=max_influences)
    bpy.ops.object.vertex_group_normalize_all(lock_active=False)
```

- [ ] **Step 2: fix_weights.py 골격 작성 + 01_rig 말미 호출**

`scripts/tools/fix_weights.py`:
```python
"""발견된 웨이트 문제의 보정 누적 스크립트. 01_rig가 자동 웨이트 직후 호출한다.
새 문제를 발견하면 여기에 보정 호출을 추가하고 주석으로 증상을 기록한다."""
from lib import weight_tools
from lib.profiles import get_profile


def apply(obj):
    profile = get_profile()
    if profile.NAME != "character":
        return
    weight_tools.limit_total(obj, max_influences=4)
    # 이후 루프에서 발견되는 보정을 여기 누적:
    # 예) 치마가 왼다리에 끌려감:
    # skirt = weight_tools.select_verts_by_box(obj, (-0.4, -0.35, 0.55), (0.4, 0.35, 0.95))
    # weight_tools.reduce_influence(obj, "DEF-thigh.L", skirt, 0.3)
```
`01_rig.py`의 `parent_set` 다음에:
```python
from tools.fix_weights import apply as _fix_weights  # scripts/가 sys.path 루트
_fix_weights(dummy)
```
(주: 01_rig의 변수명 `dummy`는 Task 1에서 PROFILE.MESH_OBJECT를 담는 기존 변수 — 이름은 리팩터하지 않는다, 회귀 최소화.)
`tools/`에 `__init__.py` 빈 파일 생성.

- [ ] **Step 3: 루프 실행 — 에이전트 판독 기준**

`run.ps1 all -Profile character`(또는 rig부터) → `previews/contact_sheet.png` Read → 판독 기준:
- **치마**: leg_spread 칸에서 치마가 찢어지거나 한 다리에 통째로 끌려가지 않음 (치마 체인이 주도, 다리 영향은 부드러운 경사)
- **꼬리**: tail_lift 칸에서 꼬리 뿌리가 엉덩이 살을 크게 끌고 가지 않음
- **겨드랑이/어깨**: arms_up 칸에서 몸통 측면 함몰 없음
- **쪼그리기**: 엉덩이-허벅지 경계 파손 없음
문제 발견 → `fix_weights.py`에 보정 추가(박스 좌표는 시트에서 추정) → 재실행. 각 보정에 증상 주석 필수.

- [ ] **Step 4: 수렴 시 커밋**

```powershell
git add scripts\lib\weight_tools.py scripts\tools\fix_weights.py scripts\tools\__init__.py scripts\stages\01_rig.py
git commit -m "feat: weight correction toolkit and accumulated character fixes"
```

---

### Task 9: 사용자 프리뷰 (턴테이블 MP4 + 고해상도 스틸)

**Files:**
- Create: `scripts/preview/user_preview.py`
- Modify: `scripts/run.ps1` (`userpreview` 스테이지)

**Interfaces:**
- Consumes: `build/03_baked.blend` (Unity 이름 본)
- Produces: `previews/user/turntable.mp4` (1080p, 5초 360°, 레스트), `previews/user/idle.mp4` (3/4 뷰 idle 2루프), `previews/user/still_<pose>.png` (1536², EXTRA_POSES + 기본 극단 3종)

- [ ] **Step 1: user_preview.py 작성**

```python
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from mathutils import Matrix
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile

PROFILE = get_profile()
open_blend(paths.blend_path("03_baked"))
out_dir = paths.PREVIEWS_DIR / "user"
out_dir.mkdir(parents=True, exist_ok=True)

scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
world = scene.world or bpy.data.worlds.new("W")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.12, 0.12, 0.13, 1)
sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", 'SUN'))
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(55), 0, math.radians(35))
scene.collection.objects.link(sun)
fill = bpy.data.objects.new("Fill", bpy.data.lights.new("Fill", 'SUN'))
fill.data.energy = 1.0
fill.rotation_euler = (math.radians(60), 0, math.radians(-120))
scene.collection.objects.link(fill)

rig = bpy.data.objects["DummyRig"]
mesh = bpy.data.objects[PROFILE.MESH_OBJECT]
cz = mesh.dimensions.z * 0.52

# 턴테이블 리그: 카메라를 엠티 자식으로, 엠티 회전 키
pivot = bpy.data.objects.new("Pivot", None)
scene.collection.objects.link(pivot)
pivot.location = (0, 0, cz)
cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
cam.parent = pivot
cam.location = (0, -3.6, 0)
cam.rotation_euler = (math.radians(90), 0, 0)
scene.camera = cam


def render_video(filepath, frame_end, setup):
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.image_settings.file_format = 'FFMPEG'
    scene.render.ffmpeg.format = 'MPEG4'
    scene.render.ffmpeg.codec = 'H264'
    scene.render.fps = 24
    scene.frame_start = 1
    scene.frame_end = frame_end
    scene.render.filepath = str(filepath)
    setup()
    bpy.ops.render.render(animation=True)


def reset_pose():
    for pb in rig.pose.bones:
        pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()


# 1) 레스트 턴테이블 (5초 = 120f)
def setup_turntable():
    rig.animation_data.action = None
    rig.data.pose_position = 'REST'
    pivot.rotation_euler = (0, 0, 0)
    pivot.keyframe_insert("rotation_euler", frame=1)
    pivot.rotation_euler = (0, 0, math.radians(360))
    pivot.keyframe_insert("rotation_euler", frame=120)
    for fc_container in (pivot.animation_data.action.layers[0].strips[0].channelbag(pivot.animation_data.action.slots[0]).fcurves
                          if pivot.animation_data.action.layers else []):
        for kp in fc_container.keyframe_points:
            kp.interpolation = 'LINEAR'


render_video(out_dir / "turntable.mp4", 120, setup_turntable)

# 2) idle 재생 (2루프 = 96f, 고정 3/4 뷰)
def setup_idle():
    pivot.animation_data_clear()
    pivot.rotation_euler = (0, 0, math.radians(45))
    rig.data.pose_position = 'POSE'
    rig.animation_data.action = bpy.data.actions.get("Idle")


scene.frame_start = 1
render_video(out_dir / "idle.mp4", 96, setup_idle)

# 3) 극단 포즈 스틸 1536²
import importlib
cs = importlib.import_module("preview.contact_sheet") if False else None  # 순환 방지 — 회전 헬퍼는 로컬 재정의


def rotate_world(bone, axis, deg):
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


STILL_POSES = [
    {"name": "armsup", "view_rz": 0, "ops": [("rotate", "LeftUpperArm", 'Y', -80), ("rotate", "RightUpperArm", 'Y', 80)]},
    {"name": "crouch", "view_rz": 90, "ops": [("translate", "Hips", -0.30),
        ("rotate", "LeftUpperLeg", 'X', -60), ("rotate", "RightUpperLeg", 'X', -60),
        ("rotate", "LeftLowerLeg", 'X', 100), ("rotate", "RightLowerLeg", 'X', 100),
        ("rotate", "LeftFoot", 'X', -40), ("rotate", "RightFoot", 'X', -40)]},
    {"name": "twist", "view_rz": 0, "ops": [("rotate", "Spine", 'Z', 25), ("rotate", "Chest", 'Z', 25)]},
] + [{"name": p["name"], "view_rz": {"front": 0, "side": 90, "threequarter": 45}[p["view"]], "ops": p["ops"]}
     for p in PROFILE.EXTRA_POSES]

scene.render.image_settings.file_format = 'PNG'
scene.render.resolution_x = 1536
scene.render.resolution_y = 1536
rig.animation_data.action = None
rig.data.pose_position = 'POSE'
for pose in STILL_POSES:
    reset_pose()
    for op in pose["ops"]:
        if op[0] == "rotate":
            rotate_world(op[1], op[2], op[3])
        else:
            translate_world(op[1], op[2])
    pivot.rotation_euler = (0, 0, math.radians(pose["view_rz"]))
    scene.render.filepath = str(out_dir / f"still_{pose['name']}.png")
    bpy.ops.render.render(write_still=True)
    print("STILL", pose["name"])
print("USER PREVIEW OK:", out_dir)
```
주의: arms_up 부호는 좌우 스왑 폐기 후 규약(2026-07-18 해소분)과 일치 — `contact_sheet.py`의 현행 부호를 따른다(계획 값이 현행과 다르면 **현행 코드가 규범**).
알려진 위험: 레이어드 액션 API로 pivot 키프레임 보간 설정이 어긋나면(setup_turntable의 fcurve 순회 실패) 보간은 기본값으로 두어도 무방 — 시각적 차이 미미. 실패 시 해당 루프를 try/except로 감싸고 주석.

- [ ] **Step 2: run.ps1 등록 + 실행**

switch에 `"userpreview" { Invoke-Blender "scripts\preview\user_preview.py" }` 추가.
Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 userpreview -Profile character`
Expected: `USER PREVIEW OK`, mp4 2개(각 수 MB) + 스틸 5장. 렌더 216프레임이라 수 분.

- [ ] **Step 3: 커밋**

```powershell
git add scripts\preview\user_preview.py scripts\run.ps1
git commit -m "feat: user-facing turntable/idle videos and hi-res pose stills"
```

---

### Task 10: 통합 — 기본 프로필 전환, 전 구간 클린 실행, PLAN 갱신, 승인 게이트

**Files:**
- Modify: `scripts/run.ps1` (기본 프로필 `character`로), `PLAN.md`

- [ ] **Step 1: run.ps1 기본값 전환**

`[string]$Profile = "dummy"` → `"character"`. `all` 시퀀스 끝에 userpreview 추가 여부는 **하지 않는다** (사용자 프리뷰는 승인 시점에만 수동 실행 — 매 실행 수 분 낭비 방지).

- [ ] **Step 2: 클린 전 구간 — 두 프로필**

```powershell
Remove-Item -Recurse -Force build, exports, previews -ErrorAction SilentlyContinue
powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile character
powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile dummy
```
Expected: 둘 다 `ALL STAGES PASSED`, character report `allPass:true`(laterality skipped, appendages_present 통과), dummy 회귀 초록.

- [ ] **Step 3: 컨택트 시트 최종 판독 (에이전트) + user_preview 생성**

시트 Read로 완료 기준 2번(극단 포즈 5종 파손 없음) 확인 → `run.ps1 userpreview -Profile character` 재생성 (character all 직후의 03_baked 기준).

- [ ] **Step 4: PLAN.md Phase 1 체크박스 갱신**

구현 항목들 `[x]`, 완료 기준 중 "사용자 승인" 항목은 미체크 + 안내 주석 (턴테이블/스틸 경로, Unity 확인 안내).

- [ ] **Step 5: 커밋 + 사용자 승인 요청**

```powershell
git add scripts\run.ps1 PLAN.md
git commit -m "chore: phase 1 pipeline complete, default profile=character (user approval pending)"
```
컨트롤러가 사용자에게: `previews/user/` 파일들 확인 요청 + Unity 최종 확인 안내(완료 기준 3·4). 승인 후 마지막 체크박스 마감.

---

## 계획 자기검토 결과 (작성 시 반영)

- 스펙 커버리지: §2 프로필(Task 1–2), §3 스테이지(Task 1·2·5·7), §4 부속물(Task 5), §5 맞춤 루프(Task 3·4·6), §6 웨이트 루프(Task 8), §7 검증 3층(Task 1·7·9), §8 완료 기준(Task 10), 더미 회귀(전 태스크 Step에 명시) — 전 항목 태스크 존재.
- 실측 반영: `metarig_sample_add` 부재 → `rigify_type` 직접 지정(레지스트리 확인), 캐릭터 원점 부유 → intake 접지, mean|x|=0.0094 → 대칭 단언 0.02, EEVEE 식별자·레이어드 액션 API 주의는 Phase 0 확정 사실 재사용.
- 형식 일관성: APPENDAGES/EXTRA_POSES 형식은 Task 5/7이 정의하고 Task 1(contact_sheet 선반영)·Task 9가 동일 형식 사용. meta.json 필드는 Task 1 C#과 Task 1 Step 5가 동일 스키마.
- 유보(정직): 랜드마크 초안 수치·부속물 좌표는 추정이며 Task 6 루프가 수렴 책임을 진다(수렴 기준 명문화됨). fix_weights는 증상 기반 누적 구조로, 사전에 코드화할 수 없는 부분을 루프 규율로 대체.
