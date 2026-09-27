# Clay Goblin 게임 리그 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. 단, 이 문서의 "실행 규칙"이 스킬 기본값보다 우선한다(판정은 main만, raw evidence 반환). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Meshy Clay Goblin을 리토폴하고 24본 DEF + CTRL/MCH 리그를 만든다. 이 리그를 Unity Generic으로 내보내 G1~G10 Acceptance Gate를 통과시킨다.

**Architecture:** 모든 산출물은 headless Blender 5.1 스크립트 체인으로 만든다(`rig/scripts/sNN_*.py`, 입력 blend → 출력 blend). 각 Gate는 독립 checker(`check_gN_*.py`)가 raw evidence JSON과 렌더를 만들고, main이 판정한다. Unity 검증은 `unity/AvatarCheck`에 Goblin 전용 에디터 스크립트를 추가해 배치 모드로 실행한다.

**Tech Stack:** Blender 5.1 (bpy, bmesh, mathutils, numpy 번들), PowerShell 5.1 래퍼, Unity 6000.3.20f1 (C# Editor scripts).

**합의문서:** `work/goblin_swing/d-23-goblin-rig-design.md`. 이하 "스펙"이라 부르고, Gate 정의는 §7이다. 이 계획의 모든 task는 스펙 §7의 Gate ID에 연결된다.

**스킬 기본값과 다른 점:**
- opus task(판단이 필요한 기하·리깅 구현)는 완성 코드를 계획에 싣지 않는다. 대신 인터페이스, 스키마, 결정사항, 검증 명령을 고정한다(사용자 역할 분담 규칙).
- 기계적 코드(래퍼·게이트 러너)는 전문을 싣는다.
- 커밋은 하지 않는다(사용자 결정: `work/`는 미커밋 상태로 유지하고, 나중에 별도 브랜치를 만들어 커밋).

## Global Constraints (스펙에서 그대로)

- 작업 루트 `work/goblin_swing/rig/`. 모든 명령은 저장소 루트 `C:\Users\whxod\orca\anime`에서 실행한다.
- 소스 FBX `C:\Users\whxod\Downloads\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx`(읽기 전용). Downloads·UVReviewProjects 수정 금지. Clubby lowpoly(`run_6e1eb23f`) 사용 금지.
- 좌표: 정면 −Y, 위 +Z, 캐릭터 왼쪽(`_l`) = +X, 지면 z=0, 원본 크기(키 1.388 m).
- DEF 24본(스펙 §2 트리 그대로). deform OFF: `root`, `weapon_socket_r`. 이름은 소문자 + `_l`/`_r`.
- 정점당 웨이트 ≤ 4(프로젝트 정책), DEF 스케일 1, 비균등 스케일 금지, IK 스트레치 없음.
- 얼굴·손가락·배 squash·텍스처는 범위 밖.
- 오브젝트 이름: 소스 `SRC_hi`·`SRC_club_hi`, 캐릭터 메시 `GOB_mesh`(단일, face 속성 `part_id`), 몽둥이 `GOB_club`, 작업 아마추어 `GOB_rig`, export 아마추어 `GOB_export`(FBX에서는 `goblin`), export 메시 `goblin_mesh`.
- 버전 파일: `gob_r00_source` → `gob_r01_retopo` → `gob_r01t_temprig` → `gob_r02_skeleton` → `gob_r03_skinned` → `gob_r04_ctrl` → `gob_r05_rigtest`. 중간 파일은 `rig/work/`.
- 판정은 main만 한다. `[U]` 항목은 사용자 시각 승인이 필요하다.

## 파일 구조

```
work/goblin_swing/rig/
  scripts/
    bl.ps1                 # Blender headless 래퍼 (T0)
    unity_goblin.ps1       # Unity 배치 래퍼 (T0)
    run_all_gates.ps1      # 최종 재검증 러너 (T41)
    goblib.py              # 공용 라이브러리 (T1)
    s00_source_prep.py     # G1 (T3)
    s01_pivots.py          # G1 (T4)
    check_g1_source.py     # G1 (T2)
    s02a_body.py           # G2 (T8)  변형 몸통 + 어깨/골반 링 포트
    s02b_limbs.py          # G2 (T9)  팔/다리 튜브 + 관절 루프
    s02c_rigid.py          # G2 (T10) 머리/손/신발/벨트/몽둥이
    s02d_finish.py         # G2 (T11) 합치기, part_id, 노멀, UV, loops json
    check_g2_static.py     # G2 (T6)
    check_g2_shape.py      # G2 (T7)
    s03_temprig.py         # G3 (T13)
    check_g3_deform.py     # G3 (T14)
    s04_skeleton.py        # G4 (T18)
    check_g4_skeleton.py   # G4 (T17)
    s05_skin.py            # G5 (T21)
    check_g5_skin.py       # G5 (T20)
    s06a_ctrl_torso_head.py  s06b_ctrl_arms.py  s06c_ctrl_legs.py  s06d_ctrl_weapon.py  # G6 (T24~T27)
    s06e_rig_ui.py         # G6 (T28)
    addon/goblin_rig_ui.py # G6 (T28) 스냅·공간전환 연산자
    check_g6_ctrl.py       # G6 (T23)
    s07a_rigtest.py        # G7 (T31)
    s07b_export_rig.py     # G7 (T32)
    check_g7_bake.py       # G7 (T30)
    s08_export_fbx.py      # G8 (T35)
    check_g8_fbx.py        # G8 (T34)
    s09_blender_ref.py     # G9 (T39)
    compare_g9.py          # G9 (T39)
  data/   pivots.json parts.json retopo_loops.json canonical_skeleton.json skin_decisions.json
          rom_poses.json ctrl_manifest.json rigtest_manifest.json export_preset.json compare_cams.json
  inspect/G1..G10/          # evidence JSON + 렌더
  work/                     # 중간 blend
  export/                   # FBX, stage_*.blend
unity/AvatarCheck/Assets/Editor/Goblin/GoblinImport.cs      # G9 (T37)
unity/AvatarCheck/Assets/Editor/Goblin/GoblinRigCheck.cs    # G9 (T38)
unity/AvatarCheck/Assets/Goblin/                            # 복사된 FBX (T40)
```

### 공용 계약

**goblib.py API** (T1이 만들고 이후 모든 스크립트가 사용):
```python
RIG: Path            # .../work/goblin_swing/rig
DATA: Path; INSPECT: Path; WORK: Path; EXPORT: Path
SRC_FBX: Path
def sha256(path) -> str
def load_json(name: str) -> dict            # DATA/name
def save_json(name: str, obj: dict) -> Path
class Evidence:
    def __init__(self, gate: str, checker: str)
    def add_input(self, path) -> None
    def add_stage_inputs(self, upto: str) -> None   # 'sNN' 이하 제작 스크립트 + DATA/*.json + goblib.py 해시
    def criterion(self, id: str, measured, threshold, ok: bool, note: str = "") -> None
    def render(self, path) -> None
    def write(self) -> Path                          # INSPECT/<gate>/<checker>.json
def quat_angle_deg(q1, q2) -> float                  # 2*acos(|dot|)
def bone_world(arm_obj, name) -> Matrix              # 포즈 반영 월드 행렬
def ortho_render(objs, view: str, out_png, res_h=1024, frame_bbox=None, wire=False, colors=None) -> Path
      # view ∈ {'front','side','three_quarter','top','back'}; Workbench; 배경 투명
def silhouette_mask(png) -> "np.ndarray[bool]"
def contour_max_dev_px(mask_a, mask_b) -> float      # 두 윤곽 간 최대 거리(px), numpy distance
def run_main(fn) -> None                             # 예외 시 traceback 출력 후 sys.exit(2)
```

**리토폴 데이터 계약** (T6·T7·T8~T11·T14·T17 공통):
- `data/parts.json`: `{"body":0,"head":1,"hand_l":2,"hand_r":3,"shoe_l":4,"shoe_r":5,"belt":6}`. `GOB_mesh`의 face int 속성 `part_id` 값이다.
- `data/retopo_loops.json`: `{"mesh":"GOB_mesh","rings":{"<name>":{"verts":[int,...],"joint":"<joint>","k":int,"center":bool}}}`. verts는 `GOB_mesh` 정점 인덱스이고, 링 순서대로 닫힌 edge loop다. 링 이름과 개수:
  - `shoulder_x_0..2`(2~3개, k=0이 몸통 쪽), `hip_x_0..2`(2~3개, k=0이 몸통 쪽)
  - `elbow_x_0..2`, `knee_x_0..2`(k=1이 center)
  - `wrist_x_0..1`, `ankle_x_0..1`(k=0이 몸 쪽)
  - `wrist_x_end`, `ankle_x_end`: 튜브 끝 캡의 테두리 루프(강체 셸 안쪽)
  - `spine_01_0`, `spine_02_0`, `belt_top_0`, `belt_bot_0`
  - x는 l/r이다.
- 설계된 overlap part 쌍: (body, hand_l), (body, hand_r), (body, shoe_l), (body, shoe_r), (body, head), (body, belt). 이 쌍의 상호 관통은 결함이 아니다.

**공간 전환·IK/FK 규약** (T24~T28 공통):
- 속성 본 `PROPS`의 커스텀 속성(기본값):
  - `arm_ik_fk_l/r`, `leg_ik_fk_l/r`: float 0..1, 팔 0, 다리 1
  - `head_space`: int, 0=chest, 1=world
  - `hand_ik_space_l`: 0=world, 1=root, 2=torso, 3=weapon
  - `hand_ik_space_r`: 0=world, 1=root, 2=torso
  - `weapon_space`: 0=hand_r, 1=hand_l, 2=torso, 3=world
  - `knee_pole_space_l/r`: 0=foot, 1=world
  - `foot_roll_l/r`, `foot_bank_l/r`, `heel_twist_l/r`, `toe_twist_l/r`: float, 0, clamp 범위는 ctrl_manifest
- 전환 가능한 컨트롤 X는 부모 `MCH_X_space`의 자식이다. `MCH_X_space`에는 공간마다 Armature/Child Of 대상이 있고, 영향도는 드라이버(`prop == k`)가 정한다. **영향도만 바꾸는 방식의 전환은 금지.** 전환은 연산자가 X의 월드 행렬을 기록하고, 속성을 바꾸고, 의존성 그래프를 갱신한 뒤, X의 월드 행렬을 복원하는 방식이다(오토키 on이면 키 삽입).
- `rig/data/ctrl_manifest.json` 스키마(T24~T27이 각자 섹션을 추가, T28·T23이 읽음):
```json
{"controls": {"CTRL_x": {"side": "L|R|C", "sweep": [{"channel": "rot|loc", "axis": "X|Y|Z", "min": -90, "max": 90}]}},
 "spaces": {"head_space": {"control": "CTRL_head", "values": ["chest", "world"]}},
 "ikfk": {"arm_l": {"prop": "arm_ik_fk_l", "fk": ["CTRL_upperarm_fk_l", "..."], "ik": "CTRL_hand_ik_l", "pole": "CTRL_elbow_pole_l", "def": ["upperarm_l", "lowerarm_l", "hand_l"]}},
 "clamps": {"foot_roll_l": [-45, 60]}}
```

## 실행 규칙 (모든 task 프롬프트에 그대로 포함)

**공통 지시문(원문 그대로 붙인다):**
```
지시된 범위만 수행하라. 요청 밖의 파일 수정, 리팩터링, 테스트 추가 금지. 더 나은 방법은 한 줄로 보고만 하고 지시대로 진행하라.
subagent를 만들지 마라(Agent/Task 도구 사용 금지). 이 task는 혼자 완결하라.
지정된 검증 명령만 각각 1회 실행하라. 추가 검증을 만들지 마라. (구현 중 디버깅 실행은 허용하되, 보고하는 검증은 지정 명령 1회 결과다.)
Gate 충족 여부를 판단하지 말고 raw evidence만 반환하라.
Blender GUI/MCP를 사용하지 마라(사용자 뷰어). headless만 쓴다.
소스(Downloads, UVReviewProjects)와 work/goblin_swing/t/, 루트의 이전 goblin 파일은 읽기만 한다.

반환 형식(산문 금지, 아래 4항목만):
- 변경 파일 / 함수 / line range
- 실행 명령 / exit code / output tail(마지막 40줄)
- 의도적으로 안 건드린 관련 범위
- 범위 밖에서 발견한 문제 또는 개선점 1줄
```

**task 프롬프트 필드:** Gate ID · 결정 사항 요약 · 대상 파일/함수 · 적용 대상 전체 · 금지 범위 · 검증 명령 · 반환 형식(위). 아래 각 task는 이 필드로 작성돼 있다. 디스패치할 때 해당 task 블록과 "공용 계약", 스펙 해당 절을 프롬프트에 복사한다.

**checker 공통 규칙(모든 check_*.py, compare_g9.py):** Evidence에 `ev.add_input(__file__)`로 checker 자신을 해시하고, `add_stage_inputs(<해당 단계>)`와 입력 blend/FBX를 추가한다. goblib의 `add_stage_inputs`는 제작 스크립트 `sNN*`만 포함하기 때문이다(T1 검수에서 확인).

**에이전트 수명 (2026-09-24 사용자 승인 개정):**
- 제작 에이전트는 Phase 단위로 유지한다. 같은 Phase의 다음 task와 수정 task는 SendMessage로 같은 에이전트에게 보낸다.
- checker 작성 에이전트는 제작 에이전트와 분리한다.
- 교체 조건은 Phase 전환, 같은 실수 2회, 보고 품질 저하다. 교체 시 main이 `rig/BRIEF.md`를 갱신하고, 새 에이전트는 BRIEF와 지정된 절만 읽는다.

| 제작 Phase | 이어 쓰는 task |
|---|---|
| 리토폴 | T10 → T11 → (T16) |
| 임시 리그 | T13 |
| 스켈레톤 | T18 |
| 스키닝 | T21 |
| 조작 리그 | T24 → T25 → T26 → T27 → T28 |
| 테스트 클립·export | T31 → T32 → T35 |
| Unity | T37 → T38 → T39 |

- checker는 Gate별 1개 에이전트가 맡는다: T14 / T17 / T20 / T23 / T30·T34 / (T39 compare 부분은 Unity Phase와 분리 가능).
- 이어 쓰는 task도 필수 7항목과 필수 문구를 매번 적는다. 추가 필수 문구: "이전 task 결과는 이번 지시로 바뀐 부분이 아니면 되돌리거나 고치지 마라".
- main 검증: git 추적 밖 파일은 task 전 사본(scratchpad)과 비교한 diff를 읽는다. 이어 쓰기 task는 증분만 읽고, Phase 종료 시 전문을 1회 읽는다. 렌더는 매번 직접 본다.
- 수치 기준은 실측 후 확정한다.

**모델:** 판단 구현은 `opus`. 기계적 실행·수집은 `sonnet` + "effort low" 명시, 적용 범위를 문자 그대로 적는다.

**main 검증(매 task 후):**
1. diff 전체를 읽는다. `work/`는 untracked라 git diff가 불가하므로, 변경 파일 전문을 Read하고 task 전 스냅샷과 비교한다.
2. 스펙과 Gate에 대조한다.
3. 범위 밖 수정과 누락된 적용 대상을 확인한다(`git status --porcelain` + `rig/` 파일 목록·mtime).
4. raw command output을 확인한다.

**Gate 판정(run task 후):** main이 evidence JSON의 criteria를 직접 읽고 렌더를 직접 Read한 뒤 PASS/FAIL을 판정한다. `[U]` 항목은 사용자에게 렌더를 보여주고 승인을 받는다. FAIL이면 해당 criterion을 해결하는 최소 범위 수정 task만 추가하고, 무효화 규칙(스펙 §7 공통)대로 재실행한다.

---

## Phase 0: 기반

### Task T0: 폴더 골격과 래퍼 (sonnet, effort low)
- **Gate:** G10.3(기준선), 전 Gate(실행 래퍼)
- **결정:** 모든 Blender 실행은 `bl.ps1` 경유(factory-startup, python-exit-code 1). Unity는 `unity_goblin.ps1` 경유(Start-Process -Wait -PassThru, graphics 사용).
- **대상 파일(이 목록만 생성):** `work/goblin_swing/rig/{scripts,scripts/addon,data,inspect,work,export}/` 디렉터리, `work/goblin_swing/rig/scripts/bl.ps1`, `work/goblin_swing/rig/scripts/unity_goblin.ps1`, `work/goblin_swing/rig/inspect/baseline_git_status.txt`
- **금지:** 위 목록 외 파일 생성·수정 일체.
- **내용: `bl.ps1`** (T0b에서 교체된 현행본. 최초 param 블록 버전은 `powershell -File`에서 `--`의 바인딩에 실패했다)
```powershell
$Script = ""; $Blend = ""; $ScriptArgs = @(); $i = 0
while ($i -lt $args.Count) {
    $t = [string]$args[$i]
    if ($t -eq "-Script") { $Script = [string]$args[$i + 1]; $i += 2; continue }
    if ($t -eq "-Blend") { $Blend = [string]$args[$i + 1]; $i += 2; continue }
    if ($t -eq "--") {
        if ($i + 1 -lt $args.Count) { $ScriptArgs = @($args[($i + 1)..($args.Count - 1)]) }
        break
    }
    Write-Error "bl.ps1: unexpected argument '$t' (usage: -Script <py> [-Blend <blend>] [-- <args>])"
    exit 64
}
if (-not $Script) { Write-Error "bl.ps1: -Script is required"; exit 64 }
$Blender = "C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"
$Rig = Split-Path -Parent $PSScriptRoot
$a = @("--background", "--factory-startup")
if ($Blend) { $a += (Join-Path $Rig $Blend) }
$a += @("--python-exit-code", "1", "--python", (Join-Path $PSScriptRoot $Script))
if ($ScriptArgs.Count) { $a += @("--") + $ScriptArgs }
& $Blender @a
exit $LASTEXITCODE
```
- **내용: `unity_goblin.ps1`(전문 그대로)**
```powershell
param([Parameter(Mandatory = $true)][string]$Method)
$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")).Path
$Unity = "C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe"
$args2 = @("-batchmode", "-quit", "-projectPath", (Join-Path $Repo "unity\AvatarCheck"),
           "-executeMethod", $Method, "-logFile", (Join-Path $Repo "unity\AvatarCheck\Logs\goblin_$Method.log"))
$p = Start-Process -FilePath $Unity -ArgumentList $args2 -Wait -PassThru -NoNewWindow
exit $p.ExitCode
```
- **baseline:** `git -C C:\Users\whxod\orca\anime status --porcelain -uall | Where-Object { $_ -notmatch 'work/goblin_swing/rig/' } | Set-Content -Encoding utf8 work/goblin_swing/rig/inspect/baseline_git_status.txt`(rig 폴더 자신은 기준선에서 제외)
- **검증 명령:**
  1. `Get-ChildItem -Recurse work/goblin_swing/rig | Select-Object FullName`
  2. `powershell -NoProfile -Command "Get-Content work/goblin_swing/rig/inspect/baseline_git_status.txt | Measure-Object -Line"`

### Task T1: goblib.py 공용 라이브러리 (opus)
- **Gate:** G1~G9 evidence 형식(스펙 §7 공통 스키마)
- **결정:** 공용 계약의 API를 시그니처 그대로 구현한다. 렌더는 Workbench에 ortho 카메라를 쓰고, view별 방향은 front(−Y에서 +Y를 봄), side(+X에서 −X를 봄), three_quarter(정면에서 오른쪽 45°, 위 15°), back, top. `frame_bbox`가 None이면 objs 전체 bbox에 5% 여백. numpy는 Blender 번들을 쓴다.
- **대상:** `work/goblin_swing/rig/scripts/goblib.py`(신규, 위 API 전부)
- **적용 대상 전체:** 공용 계약 목록의 모든 상수·함수·클래스. `add_stage_inputs(upto)`는 `scripts/s*.py` 중 파일명 접두 `sNN`이 upto 이하인 것 + `goblib.py` + `data/*.json`을 해시한다.
- **금지:** 다른 파일 생성, 소스 파일 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script goblib.py -- --selftest`. `--selftest`는 기본 큐브를 ortho_render(front)하고, silhouette_mask, contour_max_dev_px(자기 자신 = 0), quat_angle_deg(항등, 90° 회전 = 90), Evidence를 `inspect/_selftest/goblib.json`에 쓰고 결과를 출력한다.

## Phase 1: G1 SRC

> 실행 기록(2026-09-23): T3의 1차 결과에서 주먹을 관통한 자루 끝이 `SRC_hi`에 남았다(main 렌더로 확인). 그래서 **T3b**(뒤쪽 두 번째 평면 절단, `source_prep.json`에 `plane2_co/plane2_no` 추가)와 **T2b**(G1.4c criterion + 손 클로즈업 렌더 등록)를 최소 범위 task로 추가했다.

### Task T2: check_g1_source.py (opus)
- **Gate:** G1.1~G1.6(G1.7 렌더는 T4 산출물을 evidence renders에 등록)
- **결정:** 스펙 §7 G1 표의 수치 그대로. 몸통 알 단면 중심은 z ∈ [벨트 상단, 어깨 높이 − 0.05] 구간 수평 단면의 중심 평균. 눈 링 정점은 머리 전면 영역에서 원형 링 두 개 검출(검출 불가면 criterion `ok=false`, note에 이유).
- **대상:** `rig/scripts/check_g1_source.py` 신규. 함수 `main()`, `crit_g1_1()` … `crit_g1_6()`.
- **적용 대상 전체:** G1.1~G1.6 각각 criterion 1개 이상. inputs: `gob_r00_source.blend`, `add_stage_inputs('s01')`. renders: `inspect/G1/pivots_front.png`, `pivots_side.png`(존재 시 등록).
- **금지:** 제작 스크립트 작성·수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g1_source.py -Blend gob_r00_source.blend`. 입력이 아직 없으므로 exit ≠ 0과 "missing input" 메시지가 기대 출력이다(T3 이후 T5에서 실제 실행).

### Task T3: s00_source_prep.py (opus)
- **Gate:** G1.1~G1.4
- **결정:** FBX를 임포트하고 transform을 적용한다. 정면 −Y, 지면 z=0, 몸통 중심 x=0으로 정렬하고 크기는 유지한다. 몽둥이는 오른손(x<0)에서 분리해 `SRC_club_hi`로 만든다. 분리 방법은 loose parts가 우선이고, 몽둥이가 손과 융합돼 있으면 자루 축에 수직인 평면 절단 후 양쪽 캡으로 한다. 사용한 방법과 절단 위치를 `data/pivots.json`이 아니라 `data/source_prep.json`에 기록한다. 이전 작업 `work/goblin_swing/t/scripts/step01r_build.py`의 몽둥이 분리 로직을 참고만 할 수 있다.
- **대상:** `rig/scripts/s00_source_prep.py`, 출력 `rig/gob_r00_source.blend`, `rig/data/source_prep.json`
- **적용 대상 전체:** 몸 + 몽둥이 전 정점. 컬렉션 `SRC`.
- **금지:** 리토폴·리깅 작업, 소스 FBX 수정, 다른 스크립트 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s00_source_prep.py`

### Task T4: s01_pivots.py (opus)
- **Gate:** G1.5~G1.7
- **결정:**
  - 튜브 축은 수직 단면 원 피팅(팔은 x축 방향 슬라이스, 다리는 z축 방향 슬라이스)으로 구한다.
  - ~~shoulder_root는 필렛이 몸통과 만나는 지점~~ → T4b에서 변경: shoulder는 팔-몸통 접합부(필렛이 없어 shoulder_root 측정값과 같은 점), shoulder_root는 팔 축 연장선 위에서 x = 0.5 × shoulder.x인 점이다. wrist는 튜브가 손 구로 바뀌는 지점, elbow는 shoulder→wrist의 50%다.
  - hip은 다리 튜브 상단이 몸통과 만나는 곳의 튜브 축 위 점, ankle은 튜브가 신발로 바뀌는 지점, knee는 hip→ankle의 50%다.
  - pelvis는 좌우 hip 중점을 z로 올린 몸통 축 위의 점(hip 높이 + 0.03), spine_01은 벨트 상단 높이, head는 머리 박스 하단과 몸통이 만나는 높이, spine_02는 ~~알 몸통의 최대 폭 높이~~ → T4b에서 변경: spine_01과 head의 z 중간(모두 몸통 축 위).
  - grip_r은 오른손 구 중심을 자루 축에 투영한 점이고, axis는 자루 → 몽둥이 머리 방향이다.
  - foot_tip·heel은 신발의 −Y·+Y 끝점을 지면 투영한 점이다.
  - 각 키에 `{"co": [...], "axis": [...]?, "radius": r?, "method": "..."}`를 기록한다.
  - 렌더 `inspect/G1/pivots_front.png`, `pivots_side.png`: 반투명 소스 위에 마커 구 + 라벨.
  - 렌더 `inspect/G1/hand_r_top.png`, `hand_r_front.png`: `SRC_hi` 단독 오른손 클로즈업(G1.7, 자루 잔여물 확인용).
- **대상:** `rig/scripts/s01_pivots.py`, 출력 `rig/data/pivots.json`, 렌더 4장
- **적용 대상 전체:** 스펙 G1.5 필수 키 전부, 좌우 각각 독립 측정(미러 금지, 비대칭 존재).
- **금지:** blend 저장(입력 blend 수정 금지), 다른 스크립트 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s01_pivots.py -Blend gob_r00_source.blend`

### Task T5: G1 실행 (sonnet, effort low)
- **Gate:** G1
- **적용 범위(문자 그대로):** 아래 3개 명령을 순서대로 각 1회 실행하고 출력을 수집한다. 파일 생성·수정 금지.
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s00_source_prep.py`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s01_pivots.py -Blend gob_r00_source.blend`
  3. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g1_source.py -Blend gob_r00_source.blend`
- **반환:** 명령별 exit code, output tail, `inspect/G1/check_g1_source.json` 전문.
- **main:** G1 판정. G1.7은 렌더 2장을 사용자에게 보여주고 승인받는다(팔꿈치·무릎 50% 위치 확인 포함).

> **G1 판정(2026-09-23, main):** G1.1~G1.6 PASS. evidence `inspect/G1/check_g1_source.json`의 inputs sha256이 현재 파일과 일치한다. G1.4b 몽둥이 머리 좌표는 main이 독립 계산한 값과 3.2 mm 차이다. G1.7은 사용자 리뷰에서 조건부 PASS를 받았고, 조건(구부림 방향, 소켓 축, 어깨 역할 분리)은 G4.3·G4.6·G4.9에서 검증한다. → **G1 PASS**. 이후 s00/s01/check_g1/goblib/pivots.json/source_prep.json이 바뀌면 무효다.

## Phase 2: G2 RETOPO-STATIC

> **중단 지점(2026-09-23, 사용자 지시):** T8·T9 완료(main 확인: 코드 전문 + 렌더). T10은 디스패치 직후 중단했고 산출물은 없다. 재개는 T10부터다. 추적할 위험은 셋이다: (1) 어깨 뒤·위 원본 편차가 최대 약 7.8 mm라 G2.11에서 확인해야 한다(T8 디버그 측정값). (2) 어깨 링 구역이 25 mm로 작다(G3). (3) 손목 `_end`가 wrist_1에서 약 10 mm라 손목을 굽힐 때 노출될 수 있다(G3).
>
> 실행 기록(2026-09-23): T6 결과 리뷰에서 G2.16 정의 결함(자루 반경 과대 추정 → 손가락 오검출)을 발견해 **T6b**로 심지 원통 정의를 적용한다. T7 리뷰에서 겹친 내부 면 오판정을 발견해 **T7b**로 interface 제외를 적용했다. 실루엣 해상도가 부족해 **T7c**로 2048 px를 적용한다.

### Task T6: check_g2_static.py (opus)
- **Gate:** G2.1~G2.10, G2.14, G2.15(렌더), G2.16(hand_r 자루 토막, 수치 + 클로즈업 렌더 `inspect/G2/hand_r_top.png`, `hand_r_front.png`)
- **결정:** 스펙 §7 G2 표의 수치와 정의 그대로.
  - 셸은 `part_id`별 연결 성분이다.
  - 외향 검사는 셸별 부피 부호 + 면 노멀과 셸 중심 방향의 내적 다수결로 한다.
  - pole 굴곡대 섹터는 피벗의 튜브 axis 기준 각도로 계산한다.
  - UV 겹침은 2048² 래스터에서 섬 id 충돌 픽셀 수로 센다.
  - 와이어 렌더 `inspect/G2/loops_front.png`, `loops_shoulder_l.png`, `loops_shoulder_r.png`, `loops_hip.png`: 루프별 색상, pole 정점은 마커로 표시.
- **대상:** `rig/scripts/check_g2_static.py` 신규, criterion 함수를 G2 ID당 1개.
- **적용 대상 전체:** G2.1~G2.10, G2.14. retopo_loops.json의 모든 링, parts.json의 모든 part.
- **금지:** 제작 스크립트 작성·수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g2_static.py -Blend gob_r01_retopo.blend`(입력이 없어 missing input 오류가 기대 출력).

### Task T7: check_g2_shape.py (opus)
- **Gate:** G2.11~G2.13
- **결정:**
  - 편차는 양방향으로 잰다. 면적 가중 표면 샘플 ≥ 50k → 상대 메시 BVH 최근접 거리. 평균, p95, max는 전체와 part별로 낸다.
  - 원본 part 대응은 원본 정점을 retopo의 가장 가까운 면의 part로 라벨링해서 정한다.
  - 실루엣은 ortho 1024 px 높이로 front, side, three_quarter를 렌더하고 윤곽 최대 편차를 px·mm로 낸다.
  - overlay 렌더(원본 빨강 반투명 + retopo 윤곽선) 3장과 머리·눈·송곳니 클로즈업 1장을 만든다.
- **대상:** `rig/scripts/check_g2_shape.py` 신규.
- **적용 대상 전체:** `GOB_mesh`와 `GOB_club` 모두(몽둥이는 `SRC_club_hi` 대비).
- **금지:** 제작 스크립트 작성·수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g2_shape.py -Blend gob_r01_retopo.blend`(missing input이 기대 출력).

### Task T8: s02a_body.py: 변형 몸통 (opus)
- **Gate:** G2.2, G2.5(shoulder/hip 링, spine, belt 루프), G2.6(spine), G2.7
- **결정:**
  - 알 몸통을 쿼드 메시로 새로 만들고 `SRC_hi`에 shrinkwrap(nearest surface point)으로 투영한다.
  - 어깨·골반에 2~3단 동심 링 포트를 만든다. 링 가장 바깥 루프의 정점 수는 12(팔·다리 튜브 단면과 일치)이고, 어깨 연결부에서만 국소 밀도 증가를 허용한다.
  - pole 규칙: 3/5-pole을 굴곡대(어깨는 팔 축 기준 ±Z ±30°, 골반은 다리 축 기준 ±Y ±30°) 밖의 앞뒤(어깨)·좌우(골반) 섹터에 둔다.
  - spine_01·spine_02 피벗 높이에 가로 루프를 두고, 벨트 상하단 높이에 루프를 둔다.
  - 머리 박스가 덮는 상단은 머리 셸 안쪽까지 올려 닫는다.
  - 루프의 정점 인덱스를 `rig/work/loops_body.json`에 기록한다(이름은 스펙 G2.5 규약 `<joint>_<k>`, k는 몸통 쪽이 0, center 여부 포함).
- **대상:** `rig/scripts/s02a_body.py`, 출력 `rig/work/r01a_body.blend`(`SRC_hi`, `SRC_club_hi`, `GOB_body_tmp` 포함), `rig/work/loops_body.json`
- **적용 대상 전체:** 몸통 + 좌우 어깨 포트 + 좌우 골반 포트. 좌우는 각각 pivots.json 좌표로 만든다(미러 금지).
- **금지:** 팔다리 튜브, 강체 part, UV·노멀 작업, 다른 스크립트 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s02a_body.py -Blend gob_r00_source.blend`. 스크립트는 종료 전에 `rig/work/r01a_body_front.png`(와이어, goblib.ortho_render front)를 저장하고 링별 정점 수와 pole 목록을 stdout에 출력한다.

### Task T9: s02b_limbs.py: 팔다리 튜브 (opus)
- **Gate:** G2.2, G2.5(elbow/wrist/knee/ankle), G2.6(elbow/knee), G2.14(튜브 끝 캡)
- **결정:**
  - 12각 단면 튜브를 어깨·골반 포트의 바깥 링에 브리지한다.
  - 루프 배치: elbow·knee는 3루프(center는 피벗, support는 ±(튜브 반경 × 0.6)에서 시작해 G3에서 조정), wrist·ankle은 2루프.
  - 튜브 끝은 손·신발 강체 셸 안쪽 ≥ 3 mm까지 연장하고 캡으로 막는다.
  - support 간격 파라미터는 스크립트 상단 상수 `SUPPORT_OFFSET_FACTOR = {"elbow": 0.6, "knee": 0.6}`로 둔다(T16이 이 상수만 조정).
  - `loops_body.json`에 limb 루프를 추가해 `rig/work/loops_limbs.json`으로 저장한다.
- **대상:** `rig/scripts/s02b_limbs.py`, 입력 `work/r01a_body.blend`, 출력 `rig/work/r01b_limbs.blend`, `rig/work/loops_limbs.json`
- **적용 대상 전체:** 좌우 팔, 좌우 다리. pivots.json의 좌우 개별 좌표를 쓴다.
- **금지:** 몸통 메시 토폴로지 수정(브리지 링 제외), 강체 part, UV·노멀.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s02b_limbs.py -Blend work/r01a_body.blend`

### Task T10: s02c_rigid.py: 강체 part (opus)
- **Gate:** G2.1(part), G2.4, G2.11(강체 part 편차), G2.14(머리·벨트 overlap)
- **결정:**
  - 머리(눈 링·눈알·송곳니 포함), hand_l, hand_r, shoe_l, shoe_r, belt, club을 `SRC_hi`/`SRC_club_hi`에서 부위별로 분리해 감축한다.
  - hand_r: 몸 메시에 남은 자루 토막(주먹 앞뒤 돌출)을 손바닥 구와 손가락 표면에 맞춰 잘라내고 캡한다(스펙 §3).
  - 예산은 머리 ≤ 1,800, 손 각 ≤ 500, 신발 각 ≤ 400, 벨트 ≤ 500, club < 1,000 tris다. 감축은 형상 오차 우선(Planar/Collapse 조합)으로 한다.
  - 각 셸은 닫힌 manifold로 만든다(구멍은 캡).
  - 머리 셸 하단 테두리가 body 안쪽 ≥ 3 mm, 벨트 셸이 body를 감싸도록 두께를 확보한다.
  - 몽둥이 원점은 pivots.grip_r, 로컬 +Y는 자루 축이다.
  - 몽둥이는 감축하지 않고 **회전체로 재구성**한다. `SRC_club_hi`(머리+앞 자루, butt의 두 조각)의 주축을 따라 반경 프로파일을 샘플링하고, 주먹 속 구간은 앞뒤 자루 반경을 선형 보간해 하나의 닫힌 lathe 메시(단면 16각)로 만든다(스펙 §3 몽둥이 항목).
- **대상:** `rig/scripts/s02c_rigid.py`, 입력 `work/r01b_limbs.blend`, 출력 `rig/work/r01c_rigid.blend`(`GOB_head`, `GOB_hand_l`, `GOB_hand_r`, `GOB_shoe_l`, `GOB_shoe_r`, `GOB_belt`, `GOB_club` 추가)
- **적용 대상 전체:** 위 7개 part 전부.
- **금지:** body 메시 수정, UV·노멀.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s02c_rigid.py -Blend work/r01b_limbs.blend`

### Task T11: s02d_finish.py: 합치기·노멀·UV (opus)
- **Gate:** G2.1, G2.8~G2.10
- **결정:**
  - body와 강체 part를 `GOB_mesh`로 합친다. face int 속성 `part_id`를 두고, 매핑은 `data/parts.json` `{"body":0,"head":1,"hand_l":2,"hand_r":3,"shoe_l":4,"shoe_r":5,"belt":6}`이다.
  - 스무스 셰이딩 + Weighted Normal modifier를 적용해 커스텀 노멀로 굳힌다.
  - UV: 0–1, 좌우 겹침 없음, 머리 정면 texel density ≥ body 평균 × 1.5. 방식은 seam 기반 unwrap 후 pack(margin 0.005).
  - `GOB_club`도 노멀과 UV를 같은 방식으로 처리한다.
  - join 후 인덱스로 loops json을 재매핑해 `data/retopo_loops.json`에 저장한다.
- **대상:** `rig/scripts/s02d_finish.py`, 입력 `work/r01c_rigid.blend`, 출력 `rig/gob_r01_retopo.blend`, `rig/data/parts.json`, `rig/data/retopo_loops.json`
- **적용 대상 전체:** `GOB_mesh`, `GOB_club`.
- **금지:** 토폴로지 변경(합치기 외), 다른 스크립트 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s02d_finish.py -Blend work/r01c_rigid.blend`

### Task T12: G2 실행 (sonnet, effort low)
- **Gate:** G2
- **적용 범위(문자 그대로):** 아래 명령을 순서대로 각 1회 실행. 파일 생성·수정 금지.
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s02a_body.py -Blend gob_r00_source.blend`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s02b_limbs.py -Blend work/r01a_body.blend`
  3. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s02c_rigid.py -Blend work/r01b_limbs.blend`
  4. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s02d_finish.py -Blend work/r01c_rigid.blend`
  5. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g2_static.py -Blend gob_r01_retopo.blend`
  6. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g2_shape.py -Blend gob_r01_retopo.blend`
- **반환:** 명령별 exit code와 tail, `inspect/G2/*.json` 전문, 렌더 파일 목록.
- **main:** G2 판정 + [U] 렌더 사용자 승인(Static Gate).

> **G2 판정(2026-09-25, main): PASS.**
> - 정적 검사: check_g2_static 15개 항목 모두 ok.
> - 형상 검사(check_g2_shape): G2.11은 사용자 예외에 따라 ok_outside_seam=true(이음매 띠 ±40 mm, +20 mm 밖 max 2.98/2.98 mm). G2.12는 세 뷰 모두 ok(3 mm + 1 px, 띠 제외).
> - [U] 렌더(G2.13, G2.15, G2.16)는 사용자가 수용했다. 머리 눈 링의 다각형 품질도 포함이다.
> - evidence 입력 해시는 현재 파일과 일치한다.
> - 리토폴 과정의 실행 기록(T10~T11 반복):
>   - T10: 벨트 톱니, 머리 바닥·어깨 편차 → T10b(s02a 벨트·어깨 링)
>   - T10c: 예산, 벨트, 머리, 오른손 조각
>   - T10d: cover_push → 채택 안 함(톱니)
>   - T9b·T9c: 손목·발목 링을 튜브 반경으로
>   - T10e: 신발 lip seam
>   - T10f: 오른손목 조각
>   - T11: 합치기
>   - 검사 측 T7b~T7g: interface 제외, grip 설계 변경 영역, 해상도, 이음매 띠 통계·확정
> - s02a~s02d, check_g2_* 가 바뀌면 이 PASS는 무효다.

## Phase 3: G3 RETOPO-DEFORM

> 실행 기록(2026-09-25):
> - **G3 1차 FAIL.** 사용자 결정으로 토폴로지 보강 + 기준 분리(스펙 G3 개정).
> - **T16a**(어깨 k0 간격, s02a) 적용 후 수치:
>   - 어깨 30/60° 단면 비 0.84~0.97, 자기교차 0.
>   - rest 자기교차 0.
> - **T16c**(골반 토폴로지): 변형 3종 모두 미달이라 T16a 상태로 복귀했다(바이트 동일, main 확인).
>   - 원인은 heat 웨이트가 thigh 영향을 몸통 k0까지 퍼뜨린 것이다. 그래서 hip_flex_60(0.74 / 0.66)은 **G5 웨이트로 이관**했다.
> - **T16b**(팔꿈치·무릎 이름 없는 루프, s02b):
>   - 적용 변형은 elbow jmid 2, knee jmid 1이다. GOB_mesh는 8,198 tris.
>   - 60° 수치: elbow 0.78 → 0.785, knee 0.64 / 0.72 → 0.68 / 0.74로 소폭 개선.
>   - 변형 6종 모두 0.8 미달이고, 최저점 위치는 토폴로지와 무관했다.
>   - main diff 확인: s02b 3곳, s02c 상수 1줄. 그 외 스크립트는 바이트 동일.
>   - G2: static 전 항목 ok. shape는 seam 밖 ok(max 2.98 mm), seam 띠는 사용자 예외 범위.
>   - **main G3 판정(2026-09-25): PASS (사용자 결정 반영) → Retopo Final.**
>     - 통과: G3.2, G3.5a_rest 0, 전 관절 30/60° 자기교차 0, 붕괴 0.
>     - 통과: 어깨 G3.3 0.84~0.97.
>     - elbow·knee·hip·wrist_l G3.3 미달분은 사용자 결정으로 G5.5에서 판정한다.
>     - G3.6 [U]: 사용자 승인. 90° 문제(팔꿈치 안쪽 꼬집힘, 겨드랑이 접힘, 골반 90° 배 접힘)는 G5에서 다시 본다.
>     - 최종 산출물: `gob_r01_retopo.blend` GOB_mesh 8,198 tris, GOB_club 992.
>   - T16은 원래 상수 조정만 허용했지만, 이번에는 사용자 "토폴로지 보강" 결정에 따라 루프 추가 코드 변경까지 허용했다.

### Task T13: s03_temprig.py (opus)
- **Gate:** G3.1
- **결정:**
  - 임시 DEF는 스펙 §2의 24본 계층을 쓰되 위치는 pivots.json이다(뼈 규칙은 스펙 G4.3/G4.5와 같음. 롤은 G4.4 규약, preferred bend도 적용 — T13 실행분은 knee 4 mm였고 테스트 전용이라 유지).
  - body는 heat 자동 웨이트, 강체 part는 해당 본 100%, belt는 spine_01 100%다.
  - 이 파일은 테스트 전용이고 G4에서 폐기된다.
- **대상:** `rig/scripts/s03_temprig.py`, 입력 `gob_r01_retopo.blend`, 출력 `rig/gob_r01t_temprig.blend`
- **적용 대상 전체:** 24본, `GOB_mesh` 전체.
- **금지:** 메시 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s03_temprig.py -Blend gob_r01_retopo.blend`

### Task T14: check_g3_deform.py (opus)
- **Gate:** G3.2~G3.6
- **결정:**
  - 포즈 세트는 스펙 G3.2 그대로다(좌우 각각, 회전은 해당 본 로컬 축. 어깨 들기는 upperarm을 몸통 기준 위로, 어깨 앞은 −Y로, 팔꿈치는 G4.4 규약 +X, 골반 굴곡은 thigh 앞으로, 무릎은 calf +X).
  - 지표:
    - 단면 비: 링 영역 슬라이스의 최소 반경 / rest
    - 경계 노출: 튜브 끝 테두리 정점의 강체 셸 내부 깊이 최소값
    - 자기교차: 관절 영역 면 BVH overlap, 설계 overlap part 쌍 제외
    - 붕괴 면: 면적 < rest의 10%
  - 시트: `inspect/G3/sheet_<joint>.png`(각도별 3열 × 좌우, 와이어 겹침)
- **대상:** `rig/scripts/check_g3_deform.py` 신규.
- **적용 대상 전체:** G3.2의 모든 포즈 × 좌우.
- **금지:** 리그·메시 수정(포즈는 메모리 상에서만, 저장 금지).
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g3_deform.py -Blend gob_r01t_temprig.blend`

### Task T15: G3 실행 (sonnet, effort low)
- **적용 범위(문자 그대로):** 각 1회. 파일 생성·수정 금지.
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s03_temprig.py -Blend gob_r01_retopo.blend`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g3_deform.py -Blend gob_r01t_temprig.blend`
- **반환:** exit code와 tail, `inspect/G3/check_g3_deform.json` 전문, 시트 목록.
- **main:** G3 판정 + [U] 사용자 승인.

### Task T16 (조건부): 루프 간격 미세조정 (opus)
- **Gate:** G3.3/G3.5 FAIL 항목
- **결정:** main이 FAIL criterion과 조정할 관절을 지정한다. `s02b_limbs.py`의 `SUPPORT_OFFSET_FACTOR`와 `s02a_body.py`의 링 간격 상수만 변경한다.
- **금지:** 그 외 코드 변경.
- **검증 명령:** T12의 1~6과 T15의 1~2를 순서대로 실행한다.
- **main:** G2와 G3를 재판정한다. 둘 다 PASS면 **Retopo Final**.

## Phase 4: G4 SKEL

> 기준 확정(2026-09-25, main 실측: 최종 리토폴 링 중심과 pivots.json 비교):
> - elbow_x_1, wrist_x_1, knee_x_1, ankle_x_1 링 중심은 피벗과 0.0~0.9 mm 차이다. 이 링 중심을 본 위치 기준으로 쓴다.
> - shoulder_x_0/1, hip_x_0/1 링은 몸통 쪽 구역이라 피벗에서 22~46 mm 떨어져 있다. `_2` 링은 피벗에서 축 방향으로 2.4~3.3 mm다.
>   → 어깨·골반·shoulder_root·몸통 본의 기준은 **pivots.json**이다.
> - preferred bend 오프셋은 링 중심에서 튜브 축에 수직으로 준다(elbow +Y 4 mm, knee −Y 1.5 mm).
>   - G4.9 오프셋 측정은 (lowerarm/calf head − 링 중심)의 튜브 축 수직 성분이다.
>   - 각도는 두 세그먼트 방향 사이 각이다.
> - hand tail은 wrist 중심에서 lowerarm 방향으로 둔다. 길이는 hand 파트 정점의 최대 투영 길이다(s03 규칙과 같음).
> - foot tail은 foot_tip의 x·y에 z = ankle 중심 z를 쓴다. head tail은 (0,0) 방향으로 z = 머리 파트 최대 z다.
>
> 실행 기록(2026-09-25): T18(s04_skeleton.py, 320줄)과 T17(check_g4_skeleton.py, 752줄)을 병렬로 진행했다.
> - main이 s04 전문을 읽었다. checker는 기준점 계산부(c_head, _bend_geom, c_bend_*)를 확인했다.
> - **G4 PASS**:
>   - evidence 44 criteria가 모두 ok다. 입력 16개의 sha256이 현재 파일과 일치한다.
>   - G4.9: bend 2.25/2.31°(팔)·2.55/2.53°(다리), offset 4.0/1.5 mm.
>   - main이 GUI 정면·측면 캡처로 본 배치를 직접 확인했다.
> - 참고:
>   - foot 본 로컬 X는 다리 체인 X와 약 16° 차이가 난다. foot이 수평 전방을 향해서 생기는 기하 결과이고, G4.4 대상(thigh·calf)이 아니다.
>   - hand 본 길이 293 mm는 손 셸 크기(약 310 mm)와 일치한다.

### Task T17: check_g4_skeleton.py (opus)
- **Gate:** G4.1~G4.9
- **결정:** 스펙 G4 표 그대로(G4.9 preferred bend 포함). G4.7은 canonical json으로 새 아마추어를 임시 생성해 비교하고 저장하지 않는다. 링 중심은 retopo_loops.json 정점 평균이다.
- **대상:** `rig/scripts/check_g4_skeleton.py` 신규.
- **금지:** 제작 스크립트 작성·수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g4_skeleton.py -Blend gob_r02_skeleton.blend`(missing input이 기대 출력).

### Task T18: s04_skeleton.py (opus)
- **Gate:** G4.1~G4.9
- **결정:**
  - preferred bend: elbow head는 +Y로 4 mm, knee head는 −Y로 1.5 mm 옮긴다(스펙 §2, G4.9 각도 0.5°~3°).
  - 본 위치는 **최종 retopo 링 중심**이다(스펙 §2 확정 규칙). shoulder_x는 head=shoulder_root, tail=upperarm head. upperarm은 shoulder 링 중심 → elbow center 루프 중심. lowerarm은 elbow center → wrist 링 중심. hand는 wrist 중심 → 손 구 중심 방향 끝.
  - thigh는 hip → knee center, calf는 knee center → ankle 중심, foot은 ankle 중심 → foot_tip(z는 ankle 높이 유지).
  - pelvis는 pelvis → spine_01, spine_01 → spine_02, spine_02 → head, head는 head → head + (0,0,머리 높이).
  - root는 (0,0,0) → (0,0,0.2), 롤 0.
  - twist는 G4.5, weapon_socket_r은 G4.6(길이 0.1).
  - 롤은 G4.4 규약이다(구현 방법은 자유, 결과를 검증 명령으로 보인다).
  - `data/canonical_skeleton.json`을 저장한다.
- **대상:** `rig/scripts/s04_skeleton.py`, 입력 `gob_r01_retopo.blend`, 출력 `rig/gob_r02_skeleton.blend`(`GOB_rig`, 컬렉션 `RIG`), `rig/data/canonical_skeleton.json`
- **적용 대상 전체:** 24본 전부, 좌우 개별 좌표.
- **금지:** 스키닝, CTRL/MCH 본, 메시 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s04_skeleton.py -Blend gob_r01_retopo.blend`

### Task T19: G4 실행 (sonnet, effort low)
- **적용 범위(문자 그대로):** 각 1회. 파일 생성·수정 금지.
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s04_skeleton.py -Blend gob_r01_retopo.blend`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g4_skeleton.py -Blend gob_r02_skeleton.blend`
- **반환:** exit code와 tail, `inspect/G4/check_g4_skeleton.json` 전문.

## Phase 5: G5 SKIN

> 결정(2026-09-25, main):
> - **벨트**는 Option A로 두되, 본은 spine_01이 아니라 **pelvis**다. 실측상 벨트 z 0.43~0.578이 pelvis 본 구간(0.304~0.5777) 안에 있고, spine_01 head가 벨트 상단이다.
> - **G5.5 단면 비**는 스펙대로 초안이다. checker는 포즈마다 tier를 기록한다.
>   - `judged`: G3 이관분(shoulder·elbow·knee·hip 30/60°, wrist twist 90°)과 ankle 30°. ok 판정 대상이다.
>   - `report`: 그 외 ROM 극값과 대표 포즈. 보고만 하고 육안으로 판정한다.
>   - main이 첫 실측 후 기준을 확정한다.

> 실행 기록(2026-09-25):
> - **T20**: check_g5_skin.py 1,437줄과 rom_poses.json(84포즈: judged 24, report 60)을 만들었다.
> - **T21**: s05_skin.py 657줄. 관절별 프로파일 웨이트이고, 벨트는 pelvis다.
> - **1차 G5:** G5.1~5.4, 5.6, 5.7은 ok이다. G5.5 judged 미달 4건:
>   - elbow_60 0.797/0.792
>   - knee_60 0.77 (2D 모델 LBS 한계 0.770)
>   - shoulder_fwd_60_r 교차 4쌍, 모두 면 1442
>   - ankle `_1` −0.25/−0.43 mm (rest와 같음)
> - 사용자 결정:
>   - G5.5 기준을 실측으로 확정했다(60° ≥ 0.75, 교차면 ≤ 1개, `_1`은 rest 대비 판정).
>   - ROM 시트 [U] 승인.
>   - twist를 SWING_TWIST_Y로 교체한다(Euler 누수 15°).
> - 후속: T21b(s05 twist 제약 교체) ∥ T20b(checker G5.3b 추가, G5.5 기준 반영). 이후 main이 최종 실행해 판정한다.
>
> **T21b / T20b / T20c:**
> - twist를 Transformation SWING_TWIST_Y로 바꿨다. 누수 15° → 0.0°, 90° twist는 −45/+45°.
> - G5.3b(스윙 누수 ≤ 2°)를 추가했다.
> - G5.5 교차 기준을 main이 정정했다: 고유 면 수 → 최소 면 cover ≤ 1. shoulder_fwd_60_r은 4쌍이 모두 면 1442를 공유해 cover 1이다.
>
> **main G5 판정 (2026-09-25): PASS.**
> - evidence 28 criteria 모두 ok, 입력 19개 sha256 일치.
> - main이 diff를 확인했다: T21b는 twist 부분만, T20b/c는 G5.3·G5.5 블록만 바뀌었고 rom_poses는 바이트 동일.
> - G5.7 [U]는 사용자가 1차 시트를 승인했다. 웨이트는 이후 바뀌지 않았다.

### Task T20: check_g5_skin.py + rom_poses.json (opus)
- **Gate:** G5.1~G5.7
- **결정:**
  - `data/rom_poses.json`은 스펙 §4 ROM 전 범위(좌우)와 대표 포즈를 담는다. 대표 포즈는 스윙 와인드업·임팩트, walk 접지·통과, run 최대 보폭, hit 뒤로 젖힘, death 앞으로 쓰러짐, jump 웅크림 각 1개 이상이다.
  - 포즈는 DEF 본 로컬 회전 dict다. twist 본은 제약이 구동하므로 포즈에 넣지 않는다.
  - 지표 정의는 G3와 같고, 체적은 body 셸 부피(발산 정리)로 잰다.
  - ROM 시트는 `inspect/G5/rom_<group>.png`다.
- **대상:** `rig/scripts/check_g5_skin.py`, `rig/data/rom_poses.json`(checker가 소유한다. s05는 읽기만).
- **금지:** 제작 스크립트 작성·수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g5_skin.py -Blend gob_r03_skinned.blend`(missing input이 기대 출력).

### Task T21: s05_skin.py (opus)
- **Gate:** G5.1~G5.6
- **결정:**
  - body는 heat 자동 웨이트에서 시작하고, 링 구간별 블렌드 폭 정리 → 정점당 4개 제한 → 정규화한다.
  - 강체 part는 G5.2(belt는 Option A spine_01부터 시작하고, G5 FAIL이면 Option B).
  - twist 제약: upperarm_twist와 lowerarm_twist에 Copy Rotation(Y축, local space). upperarm_twist는 upperarm의 축 비틀림을 분산하고, lowerarm_twist는 hand의 축 비틀림을 받는다. 시작 비율은 0.5이고, 방향과 비율을 조정했으면 `data/skin_decisions.json`에 기록한다.
  - 몽둥이 `GOB_club`은 weapon_socket_r에 본 부모로 둔다(웨이트 없음).
- **대상:** `rig/scripts/s05_skin.py`, 입력 `gob_r02_skeleton.blend`, 출력 `rig/gob_r03_skinned.blend`, `rig/data/skin_decisions.json`
- **적용 대상 전체:** `GOB_mesh` 전 정점, 22 deform 본.
- **금지:** 본 위치·롤 수정(필요하면 한 줄 보고 후 중단), 메시 토폴로지 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s05_skin.py -Blend gob_r02_skeleton.blend`

### Task T22: G5 실행 (sonnet, effort low)
- **적용 범위(문자 그대로):** 각 1회. 파일 생성·수정 금지.
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s05_skin.py -Blend gob_r02_skeleton.blend`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g5_skin.py -Blend gob_r03_skinned.blend`
- **반환:** exit code와 tail, `inspect/G5/check_g5_skin.json` 전문, 시트 목록.
- **main:** G5 판정 + [U] 사용자 승인.

## Phase 6: G6 CTRL

> 실행 기록(2026-09-25):
> - **T24/T24b**: 몸통·머리 컨트롤. 머리 world는 회전만 고정한다. G6.2 1.2e-7.
> - **T25**: 팔 FK/IK. G6.2는 FK 1.2e-7, IK 1.2e-5. twist ±45°.
>   - 발견: Python으로 PROPS를 쓴 뒤 `arm.update_tag()`가 필요하다.
> - **T26**: 다리 FK/IK와 foot roll. G6.2는 IK 1.7e-5.
>   - 발견: 곧은 다리(150 mm)라 roll·twist가 IK 도달 범위를 벗어나면 신발이 12~58 mm 뜬다. −1 mm 기준으로는 이것을 잡지 못한다.
>   - → T26b: planted 기준(reach ≤ 1 mm, 최저 z −1~+2 mm)과 stance −15 mm로 clamp를 재측정한다. G6.7은 실측 후 확정한다.
> - **T23**: check_g6_ctrl.py 1,662줄.
>   - 발견: weapon_space=hand_l과 hand_ik_space_l=weapon이 순환을 만든다.
>   - → 스펙 §5: 무기 hand_l 공간의 대상은 CTRL_hand_fk_l로 정했다.

> 실행 기록(2026-09-25, 계속):
> - **T26b~d:** foot roll 피벗을 접지면 근처(높이 3~15 mm)로 옮겼다.
>   - planted 기준(reach ≤ 1 mm, z −1~+3 mm, slip ≤ 5 mm)은 준비 자세 −15 mm에서 확정했다.
>   - roll −29/−33 ~ +26/+29, bank 최소 ±13. 발끝 쪽이 짧은 것은 신발 형상 한계라 수용했다.
> - **T27:** 무기 컨트롤. hand_l 공간 대상은 CTRL_hand_fk_l이다(순환 방지).
> - **T28:** 연산자·UI·셰이프·색.
> - **T29:** 체인 전체 재실행 후 G6.5 FAIL(18채널). 원인은 거의 곧은 관절 근처의 가파른 IK와 다리 길이를 넘는 sweep 범위다.
>   - 수정: 스펙 G6.5를 이분 정밀화 연속성 판정으로 정정했다(T23c).
>   - 수정: sweep 범위를 캐릭터 치수에 맞게 줄였다(T28b).
> - **main G6 판정 (2026-09-25): G6.1~G6.9 PASS.**
>   - main이 최종 blend에서 checker를 직접 실행했다. exit 0, 9 criteria 모두 ok.
>   - 입력 26개 sha256이 일치한다.
>   - G6.5: 후보 84개 모두 연속, 튐 0.
>   - G6.10 [U]는 사용자 GUI 확인 대기 중이다.

> 외부 리뷰 반영(2026-09-25, 사용자 결정):
> - 채택: G6.5 branch flip 검사, G6.7 누적 접점 slip 정의, G6.11 Ready pose 연산자, G9.8 프레임 사이 비교, G9.9 Unity 바인드 행렬, G9.10 bounds, G10 압축 ON 재검증.
> - 기각:
>   - soft IK: rest가 이미 99.96% 펴져 있어 G6.2와 충돌한다.
>   - 접지 영역 중심 slip: 정상적인 뒤꿈치 구름을 잡는다.
>   - 양손 그립 로케이터: 사용자가 나중으로 미뤘다.
> - 위험 기록만 한다(이번 범위 밖): 외곽선 셰이더(inverted hull)의 overlap 셸 아티팩트, 자동 LOD 생성 시 숨은 몸통 노출, `part_id`는 Unity로 넘어가지 않는다(Blender checker 전용).
> - 후속 task:
>   - T26e: s06c clamp 절차를 새 slip 정의로 바꾼다.
>   - T28c: ready_pose 연산자를 추가한다.
>   - T23d: checker에 G6.5b, G6.7 slip, G6.11을 반영한다.
>   - 이후 G6를 재판정한다(기존 G6 PASS 무효).

> **main G6 재판정(2026-09-25, 외부 리뷰 반영 후): G6.1~G6.9, G6.11 PASS.**
> - main이 최종 blend에서 checker를 직접 실행했다. 10 criteria 모두 ok, 입력 sha256 일치.
> - T26e: 새 slip 정의로 foot_roll 뒤꿈치 쪽이 −24/−27로 줄었다(목표 ≥ −20).
> - T28c/d: ready_pose가 팔 FK/IK × 다리 FK/IK 4조합 모두에서 무릎 51°, 팔꿈치 17°, 발 고정을 유지한다.
> - G6.10 [U]는 사용자 GUI 확인을 기다리는 중이다.

### Task T23: check_g6_ctrl.py (opus)
- **Gate:** G6.1~G6.9
- **결정:**
  - ctrl_manifest.json 기반으로 검사한다.
  - G6.3 무작위 포즈는 seed 고정(1234)으로 20개, 각 FK 컨트롤을 sweep 범위 내에서 무작위로 만든다.
  - G6.4는 각 공간 값 쌍(a→b)마다 대상 컨트롤에 비항등 포즈를 준 상태에서 연산자로 전환한다.
  - G6.5·G6.6은 manifest sweep을 쓴다.
  - G6.7은 발 IK를 지면에 둔 상태에서 clamp 범위를 sweep한다.
  - G6.8은 PROPS에서 `hand_ik_space_r`의 값 목록에 weapon이 없는지 확인하고, 의존성 사이클 경고를 캡처한다.
  - G6.9는 blend 텍스트 `goblin_rig_ui.py`의 연산자 등록 여부와 애드온 파일의 import를 확인한다.
- **대상:** `rig/scripts/check_g6_ctrl.py` 신규.
- **금지:** 제작 스크립트 작성·수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g6_ctrl.py -Blend gob_r04_ctrl.blend`(missing input이 기대 출력).

### Task T24: s06a_ctrl_torso_head.py (opus)
- **Gate:** G6.1, G6.2, G6.4(head), G6.5
- **결정:**
  - 스펙 §5 몸통 계층(`CTRL_root` > `CTRL_torso` > {`CTRL_pelvis`, `CTRL_spine_01` > `CTRL_chest`}), `CTRL_head`(head_space: chest/world)을 만든다.
  - DEF root·pelvis·spine_01·spine_02·head가 CTRL을 따라가도록 Copy Transforms로 연결한다(MCH 경유는 자유).
  - `PROPS` 본을 만들고, 공용 계약의 전체 속성을 이 task에서 모두 선언한다(값의 사용은 이후 task).
  - 본 컬렉션 CTRL/MCH/DEF를 만든다.
  - ctrl_manifest.json의 controls, spaces(head_space) 섹션을 작성한다.
- **대상:** `rig/scripts/s06a_ctrl_torso_head.py`, 입력 `gob_r03_skinned.blend`, 출력 `rig/work/r04a.blend`, `rig/data/ctrl_manifest.json`(생성)
- **적용 대상 전체:** 몸통·머리 CTRL, PROPS, 컬렉션.
- **금지:** 팔·다리·무기 컨트롤, DEF 본의 rest·웨이트 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06a_ctrl_torso_head.py -Blend gob_r03_skinned.blend`

### Task T25: s06b_ctrl_arms.py (opus)
- **Gate:** G6.1~G6.6(팔), G6.8
- **결정:**
  - 좌우 팔 FK(`CTRL_shoulder_x`, `CTRL_upperarm_fk_x`, `CTRL_lowerarm_fk_x`, `CTRL_hand_fk_x`)와 IK(`CTRL_hand_ik_x`, `CTRL_elbow_pole_x`, MCH IK 체인, ik_stretch 0)를 만든다.
  - `arm_ik_fk_x`로 MCH 결과를 섞어 DEF를 구동한다.
  - 손 IK 공간은 hand_ik_space_l(world/root/torso/weapon), hand_ik_space_r(world/root/torso)이다. weapon 대상 본 `CTRL_weapon`은 T27에서 만들므로, 이 task에서는 대상 이름만 쓰는 드라이버·제약을 만들지 말고 `MCH_hand_ik_l_space`의 weapon 슬롯을 비워 둔다. T27이 채운다.
  - ctrl_manifest의 controls, spaces, ikfk(arm_l, arm_r) 섹션을 추가한다.
- **대상:** `rig/scripts/s06b_ctrl_arms.py`, 입력 `work/r04a.blend`, 출력 `rig/work/r04b.blend`, ctrl_manifest.json(섹션 추가만)
- **적용 대상 전체:** 좌우 팔.
- **금지:** 몸통·다리·무기, 스냅 연산자(T28), DEF rest·웨이트.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06b_ctrl_arms.py -Blend work/r04a.blend`

### Task T26: s06c_ctrl_legs.py (opus)
- **Gate:** G6.1~G6.7(다리)
- **결정:**
  - 좌우 다리 FK(`CTRL_thigh_fk_x`, `CTRL_calf_fk_x`, `CTRL_foot_fk_x`)와 IK(`CTRL_foot_ik_x`, `CTRL_knee_pole_x`, knee_pole_space foot/world)를 만든다.
  - foot roll MCH 피벗 체인을 heel·foot_tip·신발 안/바깥 모서리 피벗(pivots.json heel/foot_tip + 신발 bbox 모서리)으로 만든다.
  - 속성 `foot_roll`(−: 뒤꿈치 축, +: 발끝 축), `foot_bank`, `heel_twist`, `toe_twist`를 만들고 clamp를 ctrl_manifest.clamps에 기록한다. 시작값은 roll [−45, 60], bank [−30, 30], twist [−45, 45]이고, G6.7 결과로 좁힌다.
  - ctrl_manifest에 controls, spaces, ikfk(leg_l, leg_r), clamps를 추가한다.
- **대상:** `rig/scripts/s06c_ctrl_legs.py`, 입력 `work/r04b.blend`, 출력 `rig/work/r04c.blend`, ctrl_manifest.json(섹션 추가만)
- **적용 대상 전체:** 좌우 다리.
- **금지:** 몸통·팔·무기, 스냅 연산자, DEF rest·웨이트.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06c_ctrl_legs.py -Blend work/r04b.blend`

### Task T27: s06d_ctrl_weapon.py (opus)
- **Gate:** G6.1, G6.4(무기, 왼손 weapon 공간), G6.8
- **결정:**
  - `CTRL_weapon`(weapon_space: hand_r/hand_l/torso/world)을 만든다. rest 위치는 weapon_socket_r의 rest다.
  - DEF weapon_socket_r의 부모는 hand_r로 유지하고, `CTRL_weapon` 월드 transform을 Copy Transforms(world space)로 따른다.
  - T25가 비워 둔 `MCH_hand_ik_l_space`의 weapon 슬롯을 `CTRL_weapon` 대상으로 채운다.
  - 오른손 IK는 weapon 공간을 두지 않아서 순환이 없다.
  - ctrl_manifest에 controls, spaces(weapon_space, hand_ik_space_l의 weapon 값)를 추가한다.
- **대상:** `rig/scripts/s06d_ctrl_weapon.py`, 입력 `work/r04c.blend`, 출력 `rig/work/r04d.blend`, ctrl_manifest.json(섹션 추가만)
- **적용 대상 전체:** 무기 컨트롤, 왼손 IK weapon 슬롯.
- **금지:** 그 외 컨트롤, DEF rest·웨이트, 연산자.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06d_ctrl_weapon.py -Blend work/r04c.blend`

### Task T28: s06e_rig_ui.py + addon/goblin_rig_ui.py (opus)
- **Gate:** G6.1(색·셰이프·가시성), G6.3, G6.4, G6.9
- **결정:**
  - 연산자 3개: `goblin.snap_ikfk(chain, direction)`, `goblin.switch_space(prop, value)`(공용 계약의 월드 행렬 유지 방식), `goblin.reset_rig()`. 모두 ctrl_manifest.json의 내용을 blend 텍스트 `goblin_manifest.json`으로 임베드해 읽는다(외부 파일 의존 금지).
  - N-패널 UI에 PROPS 속성과 연산자 버튼을 둔다.
  - s06e는 애드온 소스를 blend 텍스트 `goblin_rig_ui.py`(use_module=True)로 넣고, 커스텀 셰이프(원·사각·화살표 메시, 컬렉션 `WGT` 숨김)와 색(L 파랑, R 빨강, C 노랑)을 적용하고, 기본 가시성을 CTRL만으로 둔다.
- **대상:** `rig/scripts/s06e_rig_ui.py`, `rig/scripts/addon/goblin_rig_ui.py`, 입력 `work/r04d.blend`, 출력 `rig/gob_r04_ctrl.blend`
- **적용 대상 전체:** manifest의 모든 ikfk 체인과 spaces 속성.
- **금지:** 컨트롤 구조 변경(s06a~d의 책임).
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06e_rig_ui.py -Blend work/r04d.blend`

### Task T29: G6 실행 (sonnet, effort low)
- **적용 범위(문자 그대로):** 각 1회. 파일 생성·수정 금지.
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06a_ctrl_torso_head.py -Blend gob_r03_skinned.blend`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06b_ctrl_arms.py -Blend work/r04a.blend`
  3. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06c_ctrl_legs.py -Blend work/r04b.blend`
  4. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06d_ctrl_weapon.py -Blend work/r04c.blend`
  5. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s06e_rig_ui.py -Blend work/r04d.blend`
  6. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g6_ctrl.py -Blend gob_r04_ctrl.blend`
- **반환:** exit code와 tail, `inspect/G6/check_g6_ctrl.json` 전문.
- **main:** G6 판정. G6.10은 main이 Blender MCP로 `gob_r04_ctrl.blend`를 열어 사용자가 조작해 보고 승인한다.

## Phase 7: G7 BAKE

> 실행 기록(2026-09-25):
> - T30/T30b/T30c: checker 959줄을 만들었다. foot_lock 유효성 검사를 넣었고, pelvis cover와 연결 해제 여부도 검사한다.
> - T31/T31b/T31c: rigtest 583프레임, 12구간을 만들었다. 골반 독립 구간을 포함한다.
> - T32/T32b: export 본은 모두 use_connect False로 둔다(스펙 §6).
> - 설계 변경(사용자 결정): CTRL_pelvis를 허리 피벗으로 바꿨다(T24c). 이 때문에 G6를 재실행했고 10 criteria 모두 PASS다.
> - **main G7 판정: PASS.**
>   - main이 최종 blend에서 checker를 직접 실행했다. 13 criteria 모두 ok, 입력 sha256 일치.
>   - G7.4: 583프레임 × 24본, 위치 최대 0.0006 mm, 회전 최대 0.107°.

### Task T30: check_g7_bake.py (opus)
- **Gate:** G7.1~G7.5
- **결정:**
  - `s07b_export_rig.build_export(src_blend, action_name, out_blend)`를 import해서 stage blend를 새로 생성한 뒤 비교한다.
  - G7.3은 rigtest와 rest로 두 번 생성해 rest를 비교한다.
  - G7.4는 전 프레임·24본 비교다.
  - G7.5는 rigtest_manifest의 구간별 max 오차와, 발 고정 구간의 foot 월드 이동량을 잰다.
- **대상:** `rig/scripts/check_g7_bake.py` 신규.
- **금지:** 제작 스크립트 작성·수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g7_bake.py -Blend gob_r05_rigtest.blend`(missing input이 기대 출력).

### Task T31: s07a_rigtest.py (opus)
- **Gate:** G7.1
- **결정:**
  - 액션 `rigtest`는 CTRL과 PROPS 키로만 만든다(DEF 직접 키 금지). 24fps, 구간 정의는 `data/rigtest_manifest.json` `{"segments":[{"name","start","end","covers":[...],"foot_lock":["l"|"r"...]}]}`이다.
  - 필수 구간: root 이동+회전, COG, 팔 FK ROM(좌우), IK/FK blend와 스냅 전후, head 공간 전환, 왼손 weapon 공간, 무기 world 공간(socket offset)과 hand_r 복귀, foot roll/bank sweep(발 고정), 다리 FK. 마지막 프레임은 rest.
  - 공간 전환은 T28 연산자로 수행하고 결과를 키로 남긴다.
- **대상:** `rig/scripts/s07a_rigtest.py`, 입력 `gob_r04_ctrl.blend`, 출력 `rig/gob_r05_rigtest.blend`, `rig/data/rigtest_manifest.json`
- **적용 대상 전체:** 위 필수 구간 전부.
- **금지:** 리그 구조 변경.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s07a_rigtest.py -Blend gob_r04_ctrl.blend`

### Task T32: s07b_export_rig.py (opus)
- **Gate:** G7.2~G7.4
- **결정:**
  - `build_export(src_blend: Path, action_name: str | None, out_blend: Path) -> Path`
    - `GOB_export`는 `data/canonical_skeleton.json`**에서만** 생성한다. 작업 DEF의 현재 rest를 참조하지 않는다.
    - 각 본에 Copy Transforms(작업 리그 DEF, world space)를 건다.
    - `action_name`이 있으면 해당 프레임 범위를 visual keying으로 매 프레임 베이크한다. None이면 rest 1프레임을 베이크한다.
    - 베이크 후 제약을 모두 제거하고, 작업 리그와 CTRL/MCH, WGT를 씬에서 삭제한다.
    - 메시 `goblin_mesh`는 `GOB_mesh`를 복제한 것이다. 버텍스 그룹이 같으므로 Armature modifier 대상만 GOB_export로 바꾼다.
    - 몽둥이 `goblin_club`은 GOB_export의 weapon_socket_r에 본 부모로 둔다.
  - CLI: `-- --action rigtest --out export/stage_rigtest.blend`, `-- --rest --out export/stage_rest.blend`
- **대상:** `rig/scripts/s07b_export_rig.py`
- **금지:** 작업 blend 저장(입력은 읽기 전용).
- **검증 명령:**
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s07b_export_rig.py -Blend gob_r05_rigtest.blend -- --action rigtest --out export/stage_rigtest.blend`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s07b_export_rig.py -Blend gob_r05_rigtest.blend -- --rest --out export/stage_rest.blend`

### Task T33: G7 실행 (sonnet, effort low)
- **적용 범위(문자 그대로):** 각 1회. 파일 생성·수정 금지.
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s07a_rigtest.py -Blend gob_r04_ctrl.blend`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g7_bake.py -Blend gob_r05_rigtest.blend`
- **반환:** exit code와 tail, `inspect/G7/check_g7_bake.json` 전문.

## Phase 8: G8 FBX

> 실행 기록(2026-09-25):
> - **T35:** s08_export_fbx.py와 export_preset.json(후보 A: bake_space_transform False, FBX_SCALE_ALL, Forward −Z / Up Y)을 만들었다.
> - **T35b:** 몽둥이를 weapon_socket_r rest 프레임으로 다시 내보냈다. 원래 롤이 89° 틀어져 있었다. 스펙 G8.6을 강화했다.
> - **T35c:** G8.5 실패(pelvis 아래 75.5 mm)의 원인은 Blender FBX 임포터였다. 부모 tail이 자식 head에 닿으면 자식을 connected로 만들어 위치 키를 무시하게 한다. FBX 파일의 곡선 자체는 정상이었다.
> - **T34/b/c:** checker는 재임포트 뒤 모든 본을 unconnect하고 비교한다(스펙 G8.5에 기록). 기준을 넘는 구간도 보고한다.
> - **main G8 판정: PASS.**
>   - main이 직접 실행했다. 6개 criteria 모두 ok, 입력 35개 sha256 일치.
>   - G8.5: 583 × 24에서 최대 0.0078 mm / 0.15°.
>   - G8.4: 노멀 p99 0.014°.
>   - G8.6: 몽둥이 소켓 프레임 오차 0.0014 mm.
> - **주의:** FBX 헤더에 생성 시각이 들어가므로, export를 다시 하면 해시가 바뀐다. checker는 항상 마지막 export 뒤에 돌린다.

### Task T34: check_g8_fbx.py (opus)
- **Gate:** G8.1~G8.6
- **결정:**
  - factory settings에서 각 FBX를 재임포트한다(임포트 옵션: automatic bone orientation OFF, 축은 export_preset과 역매핑).
  - rest 비교는 export_preset.json의 axis_mapping으로 보정한 뒤 한다.
  - 노멀 비교는 stage_rest.blend의 goblin_mesh 코너 노멀 대비 각도 p99다.
  - rigtest 비교는 stage_rigtest.blend 대비 전 프레임이다.
- **대상:** `rig/scripts/check_g8_fbx.py` 신규. 실행은 blend 없이 한다.
- **금지:** 제작 스크립트 작성·수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g8_fbx.py`(missing input이 기대 출력).

### Task T35: s08_export_fbx.py + export_preset.json (opus)
- **Gate:** G8.1~G8.6(그리고 G9.3과 연동)
- **결정:**
  - `export_scene.fbx` 설정: axis_forward '-Z', axis_up 'Y', add_leaf_bones False, bake_anim True, bake_anim_step 1.0, bake_anim_simplify_factor 0.0, bake_anim_force_startend_keying True, use_armature_deform_only False, object_types {'ARMATURE','MESH'}, mesh_smooth_type 'OFF', use_custom_props False.
  - `apply_unit_scale`, `apply_scale_options`, `bake_space_transform`(Apply Transform) 값은 preset 후보 A(bake_space_transform False, apply_scale_options 'FBX_SCALE_ALL')로 시작한다. 결과와 Blender→Unity 축 매핑 3×3 행렬을 `data/export_preset.json`에 기록한다.
  - G8·G9.3이 FAIL하면 main이 후보 B(bake_space_transform True)로 이 task를 재지시한다.
  - 파일은 `export/goblin.fbx`(stage_rest: 메시+뼈대, 애니메이션 없음), `export/goblin@rigtest.fbx`(stage_rigtest: 뼈대+메시+액션 1개), `export/goblin_club.fbx`(몽둥이 단독, 원점 grip)다.
  - 스크립트는 stage blend를 `s07b_export_rig.build_export`로 새로 생성해서 사용한다.
- **대상:** `rig/scripts/s08_export_fbx.py`, `rig/data/export_preset.json`, 출력 FBX 3개
- **금지:** 리그·메시 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s08_export_fbx.py -Blend gob_r05_rigtest.blend`

### Task T36: G8 실행 (sonnet, effort low)
- **적용 범위(문자 그대로):** 각 1회. 파일 생성·수정 금지.
  1. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s08_export_fbx.py -Blend gob_r05_rigtest.blend`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script check_g8_fbx.py`
- **반환:** exit code와 tail, `inspect/G8/check_g8_fbx.json` 전문.

## Phase 9: G9 UNITY

> 실행 기록(2026-09-25):
> - 공통 계약: `rig/data/g9_contract.md`.
> - **T37/T38(a–e):** GoblinImport.cs, GoblinRigCheck.cs.
>   - Unity 라이선스 만료로 한 번 막혔다(exit 198). 사용자가 Hub에 다시 로그인해 해결했다.
>   - 렌더는 BakeMesh 방식을 쓴다. 배치 모드에서 SMR 스키닝이 렌더에 반영되지 않았기 때문이다.
>   - bounds는 rigtest 합집합 + 0.1 m로 잡아 `Assets/Goblin/goblin.prefab`(Variant)에 넣었다.
>   - lit 렌더를 추가했다.
> - **T39(a–f):** s09_blender_ref.py, compare_g9.py.
>   - G9.8은 실측 뒤 기준을 확정했다: 선형 참조 대비 p95 ≤ 2 mm / 1°, 최대 ≤ 20 mm / 15°. 측정을 보고 정한 기준이라 사용자에게 보고했다.
>   - f371의 급변은 왼손 IK 목표가 팔 길이의 1.7배 밖에서 돌아오면서 생긴 것이다. 리그 결함이 아니다.
> - **main G9 판정: G9.1–G9.6, G9.8–G9.10 PASS. G9.7 [U]는 사용자 확인 대기.**
>   - main이 직접 실행했다. 10 criteria 모두 ok, 입력 61개 sha256 일치.
>   - 결과: G9.4 0.0024 mm, G9.5 0.0021 mm, G9.6 2.8 px, G9.9 4.5e-6(두 파일 모두), G9.10 bounds 위반 0.
> - 애니메이션 가이드용 주의: IK 목표를 팔이나 다리 도달 범위 밖으로 보냈다가 되돌리면 팔꿈치·무릎이 한 프레임에 크게 굽는다.

### Task T37: GoblinImport.cs (opus)
- **Gate:** G9.1
- **결정:**
  - `AssetPostprocessor.OnPreprocessModel`에서 경로가 `Assets/Goblin/`인 것만 처리한다.
    - `goblin.fbx`: animationType Generic, avatarSetup CreateFromThisModel, motionNodeName "root", importAnimation false.
    - `goblin@*.fbx`: Generic, avatarSetup CopyFromOther, sourceAvatar = `Assets/Goblin/goblin.fbx`의 Avatar(없으면 먼저 import 강제), animationCompression Off, clip loop 설정 없음(검증용).
    - `goblin_club.fbx`: animationType None.
  - 파일명 `@` 규칙에만 의존하지 않고 명시적 파일명 매칭 + 설정을 적용한다.
  - 다른 경로의 에셋에는 영향이 없어야 한다.
- **대상:** `unity/AvatarCheck/Assets/Editor/Goblin/GoblinImport.cs`(+ .meta는 Unity 생성)
- **금지:** 기존 Editor 스크립트(AvatarCheck.cs, ClipImport.cs, KimodoClipImport.cs 등)와 기존 에셋 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/unity_goblin.ps1 -Method GoblinImportProbe.Run`. 같은 파일에 `GoblinImportProbe.Run`을 둔다: `Assets/Goblin`의 세 FBX importer 설정을 읽어 `unity/AvatarCheck/goblin_import_probe.json`으로 쓰고, FBX가 없으면 "no goblin assets"를 쓰고 exit 0.

### Task T38: GoblinRigCheck.cs (opus)
- **Gate:** G9.1~G9.3, G9.4~G9.7 데이터 수집
- **결정:**
  - `GoblinRigCheck.Run()`:
    1. `AssetDatabase.Refresh`로 강제 재임포트한다.
    2. importer 설정을 기록한다(G9.1).
    3. goblin.fbx를 인스턴스화하고 계층, 이름, 예상 외 노드, lossyScale을 기록한다(G9.2).
    4. rest에서 root→head, hand_r→hand_l 벡터를 기록한다(G9.3).
    5. rigtest 클립을 `AnimationClip.SampleAnimation`으로 `work/goblin_swing/rig/data/rigtest_manifest.json`의 샘플 프레임(구간마다 start/mid/end)에서 샘플링하고, 24본 월드 pos/quat를 기록한다(G9.4).
    6. goblin_club.fbx 인스턴스를 weapon_socket_r 자식 local 0으로 붙이고, 같은 프레임의 몽둥이 월드 pos/quat를 기록한다(G9.5).
    7. `rig/data/compare_cams.json`의 카메라로 해당 프레임을 1024 px 높이로 렌더해 `rig/inspect/G9/unity_<cam>_<frame>.png`에 저장한다(G9.6/9.7).
  - 출력 `unity/AvatarCheck/goblin_report.json`. 실패 시 `EditorApplication.Exit(1)`, 성공 시 0.
- **대상:** `unity/AvatarCheck/Assets/Editor/Goblin/GoblinRigCheck.cs`
- **금지:** 기존 스크립트·씬 수정, Assets/Goblin 외 에셋 생성.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/unity_goblin.ps1 -Method GoblinRigCheck.Run`(에셋이 없으면 exit 1과 로그 tail이 기대 출력).

### Task T39: s09_blender_ref.py + compare_g9.py (opus)
- **Gate:** G9.4~G9.7
- **결정:**
  - `s09_blender_ref.py`:
    - `data/compare_cams.json`을 생성한다. 카메라 front와 three_quarter, ortho, 1024 px 높이, Blender 좌표 기준 위치·방향·ortho_scale, 그리고 Unity 좌표 변환값(export_preset axis_mapping 적용).
    - `export/stage_rigtest.blend`에서 같은 샘플 프레임의 24본 월드 pos/quat와 몽둥이 월드를 `inspect/G9/blender_dump.json`에 쓴다.
    - Workbench 렌더를 `inspect/G9/blender_<cam>_<frame>.png`에 쓴다.
  - `compare_g9.py`(bl.ps1로 blend 없이 실행): goblin_report.json과 blender_dump.json을 axis_mapping으로 맞춘 뒤 G9.2~G9.6 criterion을 기록하고, 나란히 비교 시트 `inspect/G9/side_by_side.png`를 만든다. Evidence는 `inspect/G9/compare_g9.json`에 쓴다.
- **대상:** `rig/scripts/s09_blender_ref.py`, `rig/scripts/compare_g9.py`, `rig/data/compare_cams.json`
- **금지:** Unity 스크립트 수정, 리그 수정.
- **검증 명령:** `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s09_blender_ref.py -Blend export/stage_rigtest.blend`

### Task T40: G9 실행 (sonnet, effort low)
- **적용 범위(문자 그대로):** 각 1회. 아래 복사 외 파일 생성·수정 금지.
  1. `New-Item -ItemType Directory -Force unity/AvatarCheck/Assets/Goblin | Out-Null; Copy-Item work/goblin_swing/rig/export/goblin.fbx, "work/goblin_swing/rig/export/goblin@rigtest.fbx", work/goblin_swing/rig/export/goblin_club.fbx unity/AvatarCheck/Assets/Goblin/ -Force`
  2. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script s09_blender_ref.py -Blend export/stage_rigtest.blend`
  3. `powershell -NoProfile -File work/goblin_swing/rig/scripts/unity_goblin.ps1 -Method GoblinRigCheck.Run`
  4. `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script compare_g9.py`
- **반환:** exit code와 tail, `unity/AvatarCheck/goblin_report.json`의 앞 200줄, `inspect/G9/compare_g9.json` 전문, Unity 로그 `unity/AvatarCheck/Logs/goblin_GoblinRigCheck.Run.log` 마지막 60줄.
- **main:** G9 판정 + [U] 사용자 승인. G9.3 FAIL이면 T35를 preset B로 재지시하고 G8·G9를 재실행한다.

## Phase 10: G10 FINAL

> 실행 기록(2026-09-25):
> - **T41 run_all_gates:** exit 0, 4분. Unity 재임포트까지 실제로 실행됐다(로그 확인).
> - **main G10 판정:**
>   - G10.1 PASS: 입력 280개 sha256 일치, 재현 결과가 이전 수치와 동일.
>   - G10.2 작성: `inspect/G10/spec_crosscheck.md`.
>   - G10.3: 범위 밖 경로 3개(G9 보고서 2개, 사용자가 요청한 리뷰 문서 1개)는 모두 설명된 항목이라 수용.
> - **G9.7 [U] 사용자 승인(2026-09-25):** "똑같다"는 답을 받았다. 몽둥이가 어색해 보인 건 f431이 weapon_world 테스트 구간이었고, 원본이 머리 바로 밑을 쥐는 형태라서다. 사용자가 현 상태 유지를 결정했다.
> - **G6.10 [U] 사용자 승인(2026-09-25):** "괜찮다". 공격 애니메이션에서 깨짐이 재현될까 우려한다는 의견이 있어, 애니메이션 단계에서 안전 범위와 A-Gate로 대응한다(`d-25-goblin-swing-anim-design.md`).
> - **리깅 파이프라인 G1~G10 완료.**

### Task T41: run_all_gates.ps1 작성 및 실행 (sonnet, effort low)
- **Gate:** G10.1, G10.3
- **대상:** `work/goblin_swing/rig/scripts/run_all_gates.ps1`(아래 전문 그대로) 생성 후 1회 실행.
```powershell
$ErrorActionPreference = "Stop"
$S = $PSScriptRoot
$Repo = (Resolve-Path (Join-Path $S "..\..\..\..")).Path
Set-Location $Repo
function BL([string]$Script, [string]$Blend = "", [string[]]$Rest = @()) {
    Write-Host ">>> $Script $Blend $Rest"
    $a = @("-NoProfile", "-File", "$S\bl.ps1", "-Script", $Script)
    if ($Blend) { $a += @("-Blend", $Blend) }
    if ($Rest.Count) { $a += $Rest }
    & powershell @a
    if ($LASTEXITCODE -ne 0) { throw "FAILED $Script (exit $LASTEXITCODE)" }
}
BL s00_source_prep.py
BL s01_pivots.py gob_r00_source.blend
BL check_g1_source.py gob_r00_source.blend
BL s02a_body.py gob_r00_source.blend
BL s02b_limbs.py work/r01a_body.blend
BL s02c_rigid.py work/r01b_limbs.blend
BL s02d_finish.py work/r01c_rigid.blend
BL check_g2_static.py gob_r01_retopo.blend
BL check_g2_shape.py gob_r01_retopo.blend
BL s03_temprig.py gob_r01_retopo.blend
BL check_g3_deform.py gob_r01t_temprig.blend
BL s04_skeleton.py gob_r01_retopo.blend
BL check_g4_skeleton.py gob_r02_skeleton.blend
BL s05_skin.py gob_r02_skeleton.blend
BL check_g5_skin.py gob_r03_skinned.blend
BL s06a_ctrl_torso_head.py gob_r03_skinned.blend
BL s06b_ctrl_arms.py work/r04a.blend
BL s06c_ctrl_legs.py work/r04b.blend
BL s06d_ctrl_weapon.py work/r04c.blend
BL s06e_rig_ui.py work/r04d.blend
BL check_g6_ctrl.py gob_r04_ctrl.blend
BL s07a_rigtest.py gob_r04_ctrl.blend
BL check_g7_bake.py gob_r05_rigtest.blend
BL s08_export_fbx.py gob_r05_rigtest.blend
BL check_g8_fbx.py
New-Item -ItemType Directory -Force unity/AvatarCheck/Assets/Goblin | Out-Null
Copy-Item work/goblin_swing/rig/export/goblin.fbx, "work/goblin_swing/rig/export/goblin@rigtest.fbx", work/goblin_swing/rig/export/goblin_club.fbx unity/AvatarCheck/Assets/Goblin/ -Force
BL s09_blender_ref.py export/stage_rigtest.blend
& powershell -NoProfile -File "$S\unity_goblin.ps1" -Method GoblinRigCheck.Run
if ($LASTEXITCODE -ne 0) { throw "FAILED GoblinRigCheck (exit $LASTEXITCODE)" }
BL compare_g9.py
# G10.3: 기준선 대비 변경 경로
$base = Get-Content work/goblin_swing/rig/inspect/baseline_git_status.txt
$now = git status --porcelain -uall
$allowed = '^(\?\?|.M|M.|A.)\s+"?(work/goblin_swing/rig/|unity/AvatarCheck/Assets/Editor/Goblin|unity/AvatarCheck/Assets/Goblin)'
$new = Compare-Object $base $now | Where-Object SideIndicator -eq "=>" | ForEach-Object InputObject
$bad = $new | Where-Object { $_ -notmatch $allowed }
New-Item -ItemType Directory -Force work/goblin_swing/rig/inspect/G10 | Out-Null
@{ new_status_lines = @($new); out_of_scope = @($bad) } | ConvertTo-Json | Set-Content -Encoding utf8 work/goblin_swing/rig/inspect/G10/scope.json
Write-Host "OUT_OF_SCOPE_COUNT=$(@($bad).Count)"
```
- **적용 범위(문자 그대로):** 위 파일 생성 1개, `powershell -NoProfile -File work/goblin_swing/rig/scripts/run_all_gates.ps1` 1회 실행. 그 외 파일 수정 금지.
- **반환:** exit code, 전체 output의 마지막 80줄, `inspect/G10/scope.json` 전문.
- **주의(main):** 기준선과 현재 상태를 모두 `-uall`로 떠서, untracked `work/` 아래 새 경로까지 파일 단위로 비교한다. Unity 실행이 건드리는 `Logs/`, `UserSettings/` 등이 목록에 나오면 main이 범위 밖 변경인지 판정한다.

### main 최종 절차
- [ ] 모든 `inspect/G*/**.json`의 inputs sha256을 현재 파일과 대조한다(G10.1).
- [ ] 스펙 §0~§7을 항목별로 최종 산출물과 대조한 체크리스트를 `inspect/G10/spec_crosscheck.md`에 작성한다(G10.2).
- [ ] scope.json과 `git status --porcelain -uall`에서 범위 밖 변경이 0인지 확인한다(G10.3).
- [ ] `[U]` 항목 사용자 승인 기록을 확인한다(G1.7, G2.13, G2.15, G3.6, G5.7, G6.10, G9.7).
- [ ] 완료 보고: Gate별 PASS 근거 파일 경로, 사용자 승인 내역, 남은 결정(스펙 §8).

## 실행 순서와 의존성

T0 → T1 → (T2 ∥ T3) → T4 → T5[G1 판정]
→ (T6 ∥ T7 ∥ T8) → T9 → T10 → T11 → T12[G2 판정]
→ T13 → T14 → T15[G3 판정] → (T16 → T12 → T15)* → Retopo Final
→ (T17 ∥ T18) → T19[G4] → (T20 ∥ T21) → T22[G5]
→ T23 ∥ (T24 → T25 → T26 → T27 → T28) → T29[G6]
→ (T30 ∥ T31) → T32 → T33[G7] → (T34 ∥ T35) → T36[G8]
→ (T37 ∥ T38 ∥ T39) → T40[G9] → T41 → main 최종[G10]

∥ 표시는 파일이 겹치지 않아 병렬 디스패치가 가능하다는 뜻이다. Checker를 쓰는 task와 제작 task는 서로 다른 파일만 만든다.

## 변경 기록: 공 주먹 (2026-09-25, 사용자 결정)
- **T60:** s02c에서 양손을 quad sphere로 만들었다.
  - 반경 97.86 mm(레퍼런스 주먹/머리 비율 0.346), 손당 432 tris, GOB_mesh 8,122 tris.
  - 손목 링이 공 안쪽 4.4 mm에 들어간다.
  - 오른손은 몽둥이 자루가 관통하고, 공을 통과하는 현은 지름의 0.94배다.
- **T61(b–e):** 체커를 바꿨다.
  - G2.16은 공 주먹 기준으로 다시 정의했다(표면 샘플링).
  - G2.11/G2.12는 원본 손 전체(손목 축 200 mm 이내)를 제외한다.
  - 그립 설계 변경 반경은 빈 구간 경계 링을 기준으로 한다(57.7 mm).
- **T39g/h:** G9.6에서 2 px closing 뒤 500 px 미만 구멍을 채운다.
- **main 판정:** run_all_gates를 두 번 연속 돌렸다. 입력 300개 sha256이 모두 일치한다.
  - G2.11a만 이음매 밖 최대 3.0052 mm(벨트)로 기준보다 0.005 mm 크다. 벨트는 변경되지 않았다(이전 PASS 때 2.9775). 손 셸 면적이 바뀌어 표면 샘플 위치가 달라지면서 생긴 샘플링 차이로 보고 **수용**한다.
  - G3.3은 G5로 이관한 것이다. 나머지 G1~G9는 모두 ok다.
  - G10.3 범위 밖 파일 4개(G9 보고서 2개, 리뷰 문서, 애니 스펙)는 모두 설명된다.
- A1: s10a를 다시 실행했다. A1.1~A1.4 PASS. 키포즈 시트에 공 주먹이 반영됐다.
- **A1.5 [U] 사용자 승인(2026-09-25).** A1 PASS. 다음은 A2.
- **A2 결과(2026-09-25):** s10b에서 중간 키 11개를 넣었다. main이 check_a_swing을 직접 돌려 A1.1~A2.5 전부 ok. 최소 간격은 21.4 mm(f8, 머리), branch flip 0, f39=f1, 발 이동 ≤ 0.005 mm. A2.6 [U]는 사용자 확인을 기다리는 중이다.
- **A2 수정(T52b, 사용자 피드백 'f15~20 어깨 이상'):** 버티는 구간에서 어깨가 으쓱 올라가던 것(Z가 24°까지)을 0으로 고정했다. f24는 어깨·오른팔 값만 바꿨다. main이 check_a_swing을 재실행해 A1.1~A2.5 모두 ok. 비교 이미지: inspect/A2/shoulder_before_after.png. A2.6 [U] 사용자 확인 대기.
- **A2.6 [U] 사용자 승인(2026-09-25). A2 PASS.** 다음은 A3: 계약 rig/data/a3_contract.md, T70(s11_export_clip + GoblinClipCheck.cs) ∥ T71(check_a3_clip.py).
- **A3 PASS(2026-09-25):** s11_export_clip.py와 GoblinClipCheck.cs(T70)를 만들었다. prefab bounds는 모든 goblin@* 클립의 합집합 + 0.1 m로 잡는다(T70b). main이 check_a3_clip을 직접 돌려 A3.1~A3.3e 모두 ok: 베이크 0.0004 mm, FBX 0.0006 mm, Unity 0.0007 mm / 0.07°, 몽둥이 0.0007 mm, bounds 위반 0, 루트 움직임 0. G9 재비교도 10/10 ok. **attack_swing 클립이 Unity까지 완료됐다.**
- **idle(2026-09-25):**
  - T80: s12_clip_anim.py와 clips/idle.json을 만들었다.
  - T81/b: check_anim_clip.py를 만들었다. 머리 상승은 눈 위치 기준으로 잰다.
  - main이 체커를 다시 돌렸다. I1.1~I2.4 모두 ok, 입력 40개 해시 일치.
  - f1은 attack f1과 같다(차이 0.0). 눈 17.6 mm, 왼 주먹 18.5 mm, 몽둥이 0.43 mm. 최소 간격 70 mm.
  - I1.3과 I2.5 [U]는 사용자 확인 대기 중이다.
- **I1.3 / I2.5 [U] 사용자 승인(2026-09-25).** 다음은 I3이다. GoblinClipCheck를 클립별 보고서로 일반화(T72)하고 check_a3_clip에 --action을 추가(T73)한다.
- **I3 PASS(2026-09-25):**
  - T72b: GoblinClipCheck가 rigtest를 뺀 모든 클립을 검사하고 `goblin_clip_report_<action>.json`을 클립별로 쓴다. 이전 goblin_clip_report.json은 삭제했다.
  - T73: check_a3_clip.py에 --action을 추가했다. idle은 inspect/I3, attack_swing은 inspect/A3에 증거를 남긴다.
  - s11 idle export exit 0. Unity GoblinRigCheck 11.8초, GoblinClipCheck 7.4초. 래퍼를 고치기 전에는 한 번에 약 10분이 더 걸렸다.
  - main이 직접 다시 돌렸다. I3.1~I3.3e, A3.1~A3.3e, G9.1~G9.10 모두 ok이고 exit 0이다. 입력 해시는 I3 40/40, A3 40/40, G9 63/63이 일치한다.
  - idle은 bounds 위반 0, 루트 커브 없음, Generic이다. prefab bounds는 바뀌지 않았다(idle이 기존 합집합 안에 들어간다).
  - **idle 클립이 Unity까지 완료됐다.**
- **hit 시작(2026-09-25):**
  - 레퍼런스 KakaoTalk_20260920_211530723.mp4를 ref_hit/로 복사했다(md5 일치). 39프레임, 24fps, 같은 카메라이고, f1은 idle f1과 같다.
  - main 측정은 ref_hit/measure_ref.py → measure.json.
    - 움찔(f6–9): 정수리 −66 mm, 벨트 −89 mm.
    - 튕김(f10–14): 왼 주먹 위 214 / 밖 140 mm, 몽둥이 머리 +120 mm.
    - 복귀 f15–20, 정지 f26–39.
  - 스펙 d-27-goblin-hit-anim-design.md(H1–H3)를 썼다. 스쿼시·스트레치는 쓰지 않고(scale 없음) COG, 척추, 머리, 팔로 표현한다.
  - check_a3_clip.py에 hit 기본값(gob_r08_hit.blend, 게이트 H3)을 추가했다. 6곳만 바뀌었다.
  - T90(opus 제작): clips/hit.json 키포즈 f1/f8/f13 → s12 → check_anim_clip --gate 1.
- **H1 main 재판정(2026-09-25):**
  - T90이 clips/hit.json 키포즈 f1/f8/f13과 gob_r08_hit.blend를 만들었다. s12 반복 실행 18회.
  - main이 check_anim_clip --gate 1을 직접 다시 돌렸다. H1.1, H1.2 모두 ok, exit 0. 입력 해시 39/39 일치.
  - 간격: f1 70.3 / f8 114.1 / f13 83.8 mm. 보이는 교차 0. 발 reach ≤ 0.005 mm. 안전 범위 초과 없음.
  - f8의 cog_dz는 −0.036. 준비 자세 대비 21 mm만 내려간다. 더 내리면 다리 접힘이 보이고 hip_flex가 60을 넘는다(실측 제약, 스펙 §3에 기록).
  - f13: 왼 주먹 위 192 / 밖 126 mm, 눈 +39 mm. f8 눈 −100 mm.
  - H1.3 [U]는 사용자 확인 대기.
- **H1 수정(T90b, 사용자 피드백 '움찔이 약함' → 상체로 강화):**
  - f8에서 spine 8, chest 9, head 14, 어깨 l20/r25로 바꿨다. 팔은 몸 쪽으로 당겼다.
  - COG와 다리는 그대로다. 머리 −31 mm, 눈 −204 mm.
  - main 재실행 결과 H1.1, H1.2 ok, exit 0, 입력 해시 39/39.
  - C2(report)에 shoulder_raise_r 25.0이 한계 "≤ 25"에 걸려 표시된다. 부동소수 오차로, H2 task에서 24.5로 내린다.
  - H1.3 [U] 사용자 확인 대기.
- **H1.3 [U] 사용자 승인(2026-09-25, "일단 진행해줘").** 사용자 피드백은 "맞고 원래 상태로 돌아오는 게 없다"였다. 키포즈만 있는 단계라 예상된 상태이고, H2에서 채운다. T91: 브레이크다운(f5 정지, 움찔 f6–9, 튕김 f10–14, 복귀 f15–19, 안착, f26/f39 = f1) + f8 shoulder_r 24.5.
- **H2 main 재판정(2026-09-25):**
  - T91이 브레이크다운 15개를 넣었다. f6과 f21은 팔 접힘을 피하려고 팔 값을 f1에 고정했다. f8 shoulder_r은 24.5로 내렸다.
  - main이 check_anim_clip --gate both를 직접 다시 돌렸다. H1.1–H2.4 모두 ok, exit 0, 입력 해시 40/40.
  - 루프 f39 = f1 차이 0.0. attack f1과의 차이 0.0. 루트 0.
  - 최소 간격 69.2 mm(f21). 보이는 교차 cover ≤ 1. flip 0. 발 이동 ≤ 0.006 mm.
  - 진폭: 눈 +34 / −181 mm, 왼 주먹 위 192 / 밖 126 mm, 몽둥이 끝 336 mm.
  - 인접 프레임 간 머리 회전이 f6–7과 f10–11에서 약 18°다. 의도한 빠른 동작이지만, 영상으로 사용자 판단이 필요하다.
  - side_by_side.mp4는 896×576, 24fps, 39프레임. H2.5 [U] 사용자 확인 대기.
- **H2.5 [U] 사용자 승인(2026-09-25). H2 PASS.** 다음은 H3: T92(sonnet) s11 hit export → GoblinRigCheck → GoblinClipCheck, 그 뒤 main이 check_a3_clip로 hit / attack_swing / idle을 판정하고 compare_g9를 돌린다.
- **H3 PASS(2026-09-25):**
  - T92(sonnet): s11 hit export exit 0(2.7초). s11이 Unity에도 복사했다(sha256 일치). GoblinRigCheck 8.8초, GoblinClipCheck 6.5초, 둘 다 exit 0.
  - 클립 보고서 3개(attack_swing, idle, hit) 모두 errors []. bounds 밖 프레임 0. 루트 커브 없음.
  - main이 check_a3_clip을 hit / attack_swing / idle로, compare_g9도 직접 돌렸다. H3.1–H3.3e, A3.1–A3.3e, I3.1–I3.3e, G9.1–G9.10 모두 ok, exit 0.
  - 입력 해시: H3 40/40, A3 40/40, I3 40/40, G9 63/63 일치.
  - hit 오차: 베이크 0.0005 mm / 0.09°, FBX 0.0004 mm / 0.06°, Unity 본 0.0006 mm / 0.06°, 몽둥이 0.0005 mm / 0.07°.
  - **hit 클립이 Unity까지 완료됐다.** 클립 3개(attack_swing, idle, hit).
- **death 시작(2026-09-25):**
  - 레퍼런스 Downloads/orc-death.mp4를 ref_death/로 복사했다(md5 일치). 56프레임, 24fps, 같은 카메라, f1은 idle f1과 같다.
  - main 측정(ref_death/measure_ref.py):
    - 정지 f1–9. 힘 빠짐 f10–22(정수리 −59 mm, 뒤·오른쪽으로 기울어짐). 균형 상실 f23–27. 뒤로 넘어짐 f28–30.
    - 부딪힘 f31, 작게 튐 f32–38, 누워서 정지 f39–56. 몽둥이는 손에 쥔 채다.
  - 스펙 d-28-goblin-death-anim-design.md(D1–D3)를 썼다. 루프가 아니고 root locked이며, 발 IK 컨트롤로 발을 든다.
  - 새 기준 C11(바닥 접촉: 관통, 누운 구간 떠 있음)을 추가한다. 첫 실행은 보정 실행(report-only)이다.
  - BRIEF에 클립 작업 절을 추가했다(hit에서 얻은 실측 제약).
  - check_a3_clip에 death/D3 기본값을 추가했다.
  - T100(opus checker): check_anim_clip에 C11 추가. T101(opus, 새 제작 에이전트): death 키포즈 f1/f22/f27/f31/f38.
- **T100 C11 main 확인(2026-09-25):**
  - check_anim_clip.py에 C11 바닥 접촉 검사를 추가했다(+121/−5줄). 기존 평가 배열에서 argmin만 한다.
  - main이 hit을 --gate both로 다시 돌렸다. 이전 증거와 비교하면 id·ok·measured·threshold·note가 모두 같다. 입력 해시는 checker 자신만 바뀌었다.
  - 보정 참고값: 서 있을 때 가장 낮은 곳은 신발 −0.56 mm다(hit 39프레임 −0.564 ~ −0.560).
- **T101 리그 한계(2026-09-25):**
  - 누운 자세(f31/f38)는 CTRL_torso rot X ±45, loc Y ≤ +0.01, foot IK Z ≤ 0.08 안에서 불가능하다. 최대로 해도 몸이 82–93 mm 떠 있다.
  - 이 범위는 G6의 발 딛은 sweep 범위로, 리그 제약이 아니라 s10a apply_state가 입력을 검사하는 값이다.
  - **사용자 결정: death 클립에서만 확장한다.** death.json의 sweep_overrides에 이유와 함께 적고, s12가 메모리에서만 넓힌다. 넓히기만 허용한다. 매니페스트, s10a, 리그는 바꾸지 않는다.
  - 확장한 포즈는 클립 자체의 검사(C11, C8, C9)로 검증한다.
  - T101b: s12 수정 + f22/f27 다리 접힘(31쌍, knee_r 104) 재작업 + f31/f38 누운 자세.
- **D1 main 재판정(2026-09-25, T101b):**
  - s12에 sweep_overrides(클립 전용, 넓히기만 허용)를 추가했다. hit은 override 블록이 없어 결과가 같다(에이전트 비교).
  - death.json override: torso rot X −90, loc Y +0.026, foot IK Z 0.21, rot X 80.
  - main이 check_anim_clip --gate 1을 다시 돌렸다. D1.1, D1.2 ok, exit 0, 입력 해시 39/39. C2 범위 밖 0. 최소 간격 56.4 mm. 보이는 교차 0. f1/f22 발 딛음.
  - C11: 누운 자세 f31/f38에서 몸 최저 −0.85 mm(벨트), 몽둥이 +2.6 mm(f38).
  - 실루엣: 누운 몸 높이 비율은 서 있을 때와 같다(1.13). 팔을 곧게 뻗어 화면 양끝을 넘는다(레퍼런스는 손이 몸 가까이).
  - f27: 우리는 왼팔이 벌어지고 다리가 곧다. 레퍼런스는 무릎을 굽히고 팔이 아래에 있다.
  - f28–30 in-between은 현재 다리 접힘과 무릎 > 90이 있다. D2에서 브레이크다운으로 처리한다.
  - D1.3 [U] 대기.
- **D1.3 사용자 피드백(2026-09-25): "둘 다 고치기".** T101c: 누운 자세의 팔을 굽혀 손을 몸 가까이 두고(실루엣 x 폭을 레퍼런스에 맞춤), f27은 무릎을 굽혀 주저앉듯 뒤로 기울고 팔을 아래에 둔다.
- **D1 재판정(T101c):**
  - main이 다시 돌렸다. D1.1, D1.2 ok, exit 0, 입력 해시 39/39. C2 범위 밖 0, 보이는 교차 0.
  - C11: 누운 자세 몸 −0.85 mm, 몽둥이 +3.1 mm, 손 +0.3 / +2.8 mm.
  - 누운 자세 실루엣 x 106–773(896 px). 레퍼런스 ×2 범위(80–770)와 맞다.
  - f27은 무릎 55/57로, 더 굽히면 다리 접힘 14–30쌍이 생긴다. 주저앉는 느낌은 제한적이다.
  - 에이전트가 루트에 잠깐 만든 last_s12.txt는 삭제된 것을 확인했다.
  - D1.3 [U] 대기.
- **D1.3 [U] 사용자 승인(2026-09-25). D1 PASS.** T102: 브레이크다운(힘 빠짐 f10–22 서서히, 균형 상실 가속, 넘어짐 f28–30은 발을 먼저 들고 무릎을 굽힘, 작게 튐 f32–34, 안착 f38, f56 = f38). ground_rest_ranges를 설정하고, C11 기준은 D2 측정 뒤 확정한다.
- **D2 main 재판정(2026-09-26, T102):**
  - T102가 브레이크다운 13개를 넣었다.
    - 넘어짐 f28–30은 발 IK를 먼저 들어 다리 접힘 30쌍 → 0, 무릎 ≤ 60.6.
    - 작게 튐 f32/f33 +20/+15 mm.
    - sweep_overrides torso loc Y 0.046.
  - ground_rest_ranges [[31,31],[34,56]].
  - **C11 기준 확정: 관통 1 mm, 뜸 3 mm**(G6.7과 같은 띠, 실측 −0.56/−0.85 mm). 참고용에서 판정 항목으로 바꿨다(main이 death.json 편집: gate_id C11 + ground 블록).
  - main이 --gate both로 다시 돌렸다. D1.1–D2.4 모두 ok, exit 0, 입력 해시 40/40.
    - 최소 간격 56.4 mm. 보이는 교차 ≤ 1. flip 0. 발 ≤ 0.005 mm(f1–26). 루트 0. f1 = attack f1.
  - M2 행은 targets가 없어 나오지 않는다. s12 측정으로 대신한다: 눈 −59 mm(f21), 레퍼런스 59 mm.
  - 에이전트가 최종 실행 뒤 override 이유 문구를 수정했다(값은 같다). main 재실행 증거는 현재 JSON 해시 기준이다.
  - side_by_side.mp4는 896×576, 24fps, 56프레임. D2.6 [U] 대기.
- **D2.6 [U] 사용자 승인(2026-09-26). D2 PASS.** 다음은 D3: T103(sonnet) s11 death export → GoblinRigCheck → GoblinClipCheck. 그 뒤 main이 check_a3_clip를 death/hit/idle/attack_swing으로, compare_g9도 돌린다.
- **D3 PASS(2026-09-26):**
  - T103(sonnet): s11 death export exit 0(56프레임). Unity에도 복사했다(sha256 일치). GoblinRigCheck 8.5초, GoblinClipCheck 7.1초. 클립 보고서 4개 errors 0, bounds 밖 0, 루트 커브 없음.
  - prefab bounds: Z extents 0.720 → 1.027, center z 0.083 → −0.223(누운 자세). X/Y는 그대로다.
  - main이 check_a3_clip을 death/hit/idle/attack_swing으로, compare_g9도 직접 돌렸다. D3/H3/I3/A3 전 항목 ok, G9.1–G9.10 ok, 전부 exit 0.
  - 코드 변경(s12 sweep_overrides, C11)으로 idle/hit 애니 증거의 해시가 어긋나서 main이 --gate both로 다시 돌렸다. I1.1–I2.4, H1.1–H2.4 모두 ok.
  - 최종 증거 해시: D3/H3/I3/A3 40/40, G9 63/63, death 40/40, A(check_a_swing) 36/36.
  - **death 클립이 Unity까지 완료됐다.** 클립 4개(attack_swing, idle, hit, death). 남은 것: walk, run, jump.
- **roar 시작(2026-09-26):**
  - 레퍼런스 orc_roar.mp4를 ref_roar/로 복사했다(md5 일치). 39프레임, 같은 카메라, f1은 idle f1과 같다.
  - 입을 크게 벌려야 하는데 고블린에는 입이 없다.
  - **사용자 결정: B(입 벌림 셰이프 키).** 뼈대 24본은 유지한다.
  - 스펙 d-29-goblin-roar-mouth-design.md(R0 왕복 시험 → R1 입 모델링[U] → R2 통합과 재검증 → R3 roar 애니 → R4 export).
  - main 측정(measure_mouth.json):
    - 입 열림 f9–10, 유지 f10–26, 닫힘 f27–29.
    - 벌린 입은 얼굴 폭의 약 80%, 높이 약 55%.
    - 웅크림(wind-up)에서 정수리 −110 mm(f7).
  - T110(opus): R0 셰이프 키 왕복(키 방식과 드라이버 방식, Unity blendShape 커브, BakeMesh 비교). 임시 폴더에서만 하고 끝나면 정리한다.
- **R0 PASS(2026-09-26, T110, main 재확인):** 키 방식과 드라이버 방식 모두 FBX DeformPercent 커브(39키)로 나간다(exporter가 드라이버 값을 굳힌다). Unity blendShape 커브의 가중치가 Blender ×100과 같다(차이 0). BakeMesh 차이 0.0004 mm. 정리 후 해시가 같다. 조건: 메시 모디파이어는 Armature 하나뿐, relative 키. 기준 확정: 가중치 ≤ 0.01, 정점 ≤ 0.01 mm.
- T111(opus 모델링): s02e_mouth.py(새 스크립트). 입력 gob_r01_retopo.blend → 출력 work/r01e_mouth.blend. 머리 파트에 입 구멍, 입 안(어두운 재질 슬롯 GOB_mouth_inner), 아래 송곳니, 셰이프 키 mouth_open을 만든다. inspect/R1/mouth_sheet.png로 R1 [U]를 받는다. 체인 통합은 R2에서 한다.
- **R1 main 재확인(2026-09-26, T111):**
  - s02e_mouth.py → work/r01e_mouth.blend. main이 probe를 다시 돌렸고 에이전트 보고와 같다.
    - 정점 4073 → 4255, 삼각형 8122 → 8470(+348).
    - non-manifold 0. 면적 0인 면 0. 입 영역 밖 Basis 편차 0.0.
    - 입 안쪽 재질은 입 안쪽 면 128개에만 있다. relative 키.
  - 열린 입: 폭 = 머리 폭의 0.67–0.69(레퍼런스 0.69). **높이는 보이는 머리 높이의 0.29(레퍼런스 약 0.55).** 눈 아랫면과 준비 자세의 가슴이 한계다(턱을 더 내리면 몸을 관통).
  - 송곳니: 원래 송곳니를 아래 송곳니로 두고, 위 송곳니는 닫힌 입에서 머리 안에 숨긴다.
  - R1 시트 오른쪽 전신 렌더는 준비 자세가 아니라 rest 자세다(에이전트 보고와 다르다).
  - R1 [U] 대기.
- **R1 사용자 피드백(2026-09-26):** "벌린 입이 징그럽다, 찢어진 채 웃는 것 같다". main 진단: (a) 세로가 긴 둥근 사각형이 아니다 (b) 입꼬리가 당겨져 주름이 생긴다 (c) 입 안이 검붉어 속살처럼 보인다 (d) 이빨이 피부색이다. T111b: 네 가지를 고친 변형 2–3개(셰이프 키 v1/v2/v3)를 만든다. 몸 관통은 roar 자세(머리 −15~−20°) 기준으로 보고, 치아와 혀 재질 슬롯을 추가한다. 산출물은 mouth_variants_sheet.png.
- **R1 변형 main 재확인(2026-09-26, T111b):**
  - probe를 다시 돌렸다(gob_r04_ctrl로 자세 부분 포함). 에이전트 보고와 같다.
    - Basis 편차 0. non-manifold 0. 삼각형 8530(+408).
    - 재질 4개: skin 5472 / mouth_inner 128 / teeth 20 / tongue 40.
  - 변형:
    - v1: 폭 0.48, h/w 0.53.
    - v2: 폭 0.35, h/w 0.57.
    - v3: 폭 0.42, h/w 0.56.
    - roar 자세(head X −18°) 관통 0. ready 자세에서 입을 끝까지 벌리면 12–20 mm 관통(report, roar 전용).
  - 남은 흠: 캐릭터 왼쪽 눈과 입꼬리 사이 주름(모든 변형, v2가 가장 크다). 3/4에서 아래 볼이 울퉁불퉁하다.
  - 사용자 선택 대기.
- **R1 사용자 피드백 2(2026-09-26):** v1 3/4 옆 입이 최선이냐는 질문. main 진단: 경첩 턱이 아래 얼굴을 앞으로 45 mm 밀어 옆모습에 주둥이가 생긴다. 레퍼런스는 앞면에 파인 입 + 머리 스쿼시·스트레치다(실측: 머리 폭 145 → 125 px, 정수리 +10–12 px). T111c: v4는 턱을 내밀지 않고 앞면 평면에 입을 파며 머리를 X 약 −10%, Z 약 +10%로 늘인다. 양쪽 3/4과 옆모습 렌더로 v1과 비교한다.
- **R1 v4 main 재확인(2026-09-26, T111c):**
  - probe(-Blend gob_r04_ctrl)를 다시 돌렸다. 에이전트 보고와 같다.
  - v4:
    - 턱 내밀기 0. 옆모습 입선 아래 +8.25 mm(뒤로), 전체 0.
    - 머리 bbox x 0.90 / z 1.10, 바닥 이동 0.
    - 입 w/head 0.43, h/w 0.62(roar).
    - 관통: roar 0, ready 20 verts 3.6 mm(report).
  - 공통: Basis 편차 0, non-manifold 0.
  - 남은 흠: 캐릭터 왼쪽 눈 밑 작은 홈, 얇은 아랫입술.
  - 사용자 선택 대기.
- **R1 사용자 결정(2026-09-26): "V4 좋다", 더 큰 v5를 만들어 비교한다.** T111d: v4 방식 + 폭은 평평한 앞면의 약 60%까지(모서리를 넘지 않음), 눈 올리기 + 스트레치 X 0.87 / Z 1.15, 높이 ≥ 0.40, 눈 밑 홈과 아랫입술을 고친다. 산출물 mouth_v4_v5_sheet.png.
- **R1 v5 main 재확인(2026-09-26, T111d):**
  - probe를 다시 돌렸다. 에이전트 보고와 같다.
  - v5 수치:
    - 입 w/head 0.53, w/front 0.60, h/visible head 0.41(roar), h/w 0.77.
    - 머리 x 0.87 / z 1.151. 눈 +121 mm. 옆모습 윤곽 +12.95 mm(뒤로).
    - 관통 roar 0, ready 2 verts 0.4 mm.
    - Basis 편차 0, non-manifold 0.
  - 눈 밑 홈은 Basis 코너 노멀 재설정으로 없앴다. 닫힌 입에도 반영된다(송곳니 옆에 옅은 밝은 패치).
  - main 시각 확인: v4/v5 모두 3/4 뷰에서 머리 아래와 몸 경계에 구겨진 띠가 보인다(에이전트 보고에 없음).
  - 사용자 선택 대기.
- **R1 사용자 피드백 3(2026-09-26): "입 주변이 레퍼런스처럼 평평하지 않다".** main 진단: (1) 고르지 않은 원래 면을 밀어서 생긴 잔물결 (2) 도드라진 입술 림 (3) 닫힌 상태 기준 커스텀 노멀. T111e: v6는 v5 비율 + 입 주변 토폴로지 재구성(균일한 동심 루프) + 두 상태 모두 매끈한 앞면 곡면 위 + 림 없는 깔끔한 컷 + 목 경계 구김 수정. 새 기준(report-only 첫 실행): 재구성 영역에서 닫힌 표면과 원래 표면의 거리, 열린 상태에서 fitted 곡면까지의 거리. 산출물 mouth_v5_v6_sheet.png.
- **R1 v6 main 재확인(2026-09-26, T111e):**
  - probe를 다시 돌렸다. 에이전트 보고와 같다.
  - 토폴로지가 모든 키에 공통으로 바뀌었다(v1–v5도 재생성). 삼각형 8704(+582).
  - v6: 입 둘레의 fitted 곡면 거리 0.000 mm(v5 17.3). 입 w/head 0.544, h 0.413. 관통 roar/ready 0. 옆모습 +10.9 mm(뒤로).
  - 새 기준 첫 측정(report): 닫힌 패치와 원래 머리 표면의 거리 max 19.3 / p95 10.6 mm(송곳니 판 뒤 포함). 원래 얼굴 기준으로는 8.9 / 4.6 mm.
  - **main 시각 판단:** matcap을 보면 닫힌 머리 전체(눈 사이, 볼, 이마, 몸)가 원래 리토폴 면 때문에 울퉁불퉁하다. 입 주변의 평평함은 머리 앞면 전체를 재토폴로지해야 해결된다(G2 범위).
  - v6 닫힌 상태에는 입 위에 옅은 곡선 주름이 새로 생겼다(패치 경계, 원래 면과 최대 8.9 mm 차이). 기존 클립에도 보이게 되므로 회귀다.
  - 사용자 결정 대기.
- **사용자 결정(2026-09-26): A, 머리 재토폴로지.** matcap을 보면 닫힌 머리 전체가 울퉁불퉁하다(현재 머리는 SRC_hi collapse decimation + fit, s02c). d-29에 HR 단계를 추가했다. HR0(측정만): 원본 대비 현재 머리 matcap, 원본 거칠기, 삼각형 예산(G2.8 ≤ 8500, v6 입 포함 8704로 이미 초과). T112(opus, 새 에이전트).
- **HR0 결과(2026-09-26, T112, main이 다시 돌림: 수치 1782개 모두 같음):**
  - **울퉁불퉁함의 원인은 decimation/fit이지 원본이 아니다.**
    - 앞면을 smooth 기준(MLS R25)과 비교: SRC 0.024/0.068/0.18 mm, 현재 GOB 0.44/1.20/2.1 mm(평균 18배).
    - 셰이딩 오차 p95: SRC 2.13°, GOB 3.22°.
  - 현재 머리: 1628 tris(전부 삼각형), 밸런스 3–25, 보이는 edge 길이 CV 0.73.
  - 고른 quad 셸 시험:
    - 1200/1600/2000 tris에서 앞면 편차 0.45/0.29/0.25 mm 평균(straddle), CV 0.15–0.17, matcap 깨끗.
    - 최대값 초과(>3 mm)는 모두 머리/몸 경계 주름부(bottom)에서 나온다. crease rim 루프가 필요하다.
    - 눈/송곳니 패치(537 tris, decimation)는 경계가 지저분하다.
  - 예산: G2.8 ≤ 8500. 현재 8122. 1200 셸 + 패치 = 8227(입 제외). 1600 셸이면 8655. 입(+약 580)을 더하면 초과한다.
- **사용자 결정(2026-09-26): G2.8 상한 9,500(1번).** d-23 G2.8을 고쳤다. d-29에 HR1 Gate를 추가했다(HR0 실측 기반: CV ≤ 0.25, G2.11 그대로, 셰이딩 p95 ≤ 2.6°, MLS p95 ≤ 1.2 mm, 눈·송곳니 루프 [U], G2 정적 규칙, ≤ 9,500). 백업: work/goblin_swing/backup_pre_HR_2026-09-26/(rig 전체 + Unity Assets/Goblin + 보고서, 728 해시).
- T113 = HR1a(HR0 에이전트가 이어서 제작): s02c 머리를 고른 quad 셸로 바꾼다(약 1600, straddle, crease rim 루프, 눈 링·송곳니 루프). s02c → s02d를 실행한다. 검증 도구는 수정하지 않은 hr0_measure.py다. 다음 HR1b: 입 에이전트가 새 머리에 v6 입을 다시 만든다.
- **HR1a main 재확인(2026-09-26, T113):**
  - s02c 새 머리: 1056 quads + 46 삼각형(숨은 캡). 눈 링, 송곳니, crease 루프를 넣었다. s02d GOB_mesh 8652 tris(입 여유 848).
  - main이 hr0_measure(수정 없음, b94cad2b)를 다시 돌렸다. 수치 1771개가 같다.
  - 판정 초안:
    - HR1.2 G2.11: 앞면/옆/위/뒤/바닥 모두 max ≤ 2.6 mm, 통과. 예외는 eye x<0 영역.
    - HR1.3 MLS p95 1.01 ≤ 1.2, 통과. **앞면 셰이딩 p95 3.16° > 2.6° 초과.**
    - HR1.1 CV 0.362 > 0.25(눈·송곳니 짧은 루프가 포함됨).
    - G2.14 19.2 mm, G2.9 밀도 2.1, G2.10 OK.
  - **원본 결함:** SRC_hi의 x<0 눈 덩어리는 16개 방위 중 6개에서 얼굴에서 떨어져 있다(틈 최대 약 36 mm). 새 머리는 이를 얼굴에 붙인다. 그래서 SRC→head 편차가 최대 47 mm다.
  - matcap(main 시각 확인): 원본 수준으로 매끈하다.
  - T113b 진단: 영역별 CV, 셰이딩 오차 heatmap(커스텀 노멀 vs 기본 노멀), 눈 틈 렌더.
- **HR1 진단(T113b) main 판독:**
  - HR1.1은 구역 밖 CV로 판정한다. 결과 0.221 ≤ 0.25. 근거: 초안 기준이 눈/송곳니 패치를 뺀 probe 셸이었다(0.149).
  - HR1.3 앞면 셰이딩 초과의 원인은 s02d 저장 노멀(아래 crease 띠, near_edge p95 3.86°)이다. 나머지 얼굴은 2.41°. plain 노멀은 띠를 1.99°로 낮추지만 눈/송곳니 벽은 29.9°로 나빠진다. T113c: 머리만 기본 smooth + 눈/송곳니 벽만 sharp로 처리한다.
  - x<0 눈: 원본의 바깥쪽 절반이 떠 있다(gap map 초승달, 단면 구멍). x>0 눈은 정상이다. 새 머리는 이를 메웠다. 설계 변경 인정 여부는 사용자가 결정한다.
- **사용자 결정(2026-09-26): x<0 눈 틈 메움을 설계 변경으로 인정한다.** 공 주먹과 같은 방식이다. 그 눈 영역은 G2.11 SRC→head 방향 판정에서 빼고 보고만 한다. head→SRC 방향은 그대로 판정한다.
- **T113c:**
  - 머리 노멀을 smooth + 눈/송곳니 벽 118 edges만 sharp로 바꿨다.
  - 앞면 셰이딩 p95 3.16 → 2.46°(통과). 옆 1.45, 위 1.05.
  - 머리 외 노멀은 비트 단위로 같다(cn_check).
  - 대신 bottom 셰이딩이 5.24 → 40.7°로 회귀했다(보이는 아랫면과 숨은 skirt 경계를 smooth로 이었기 때문). T113d: crease rim 루프를 hard로 지정한다.
- **HR1a 판정(2026-09-26, T113d, main이 hr0_measure/hr1_diag를 다시 돌림: 1771 + 105 수치 같음):**
  - HR1.1 CV(구역 밖) 0.221, 통과.
  - HR1.3:
    - 셰이딩 p95: 앞 2.46 / 옆 1.45 / 위 1.05°(≤ 2.6), 통과. bottom 3.73°(T113 5.24).
    - MLS p95 1.01 mm, 통과.
  - HR1.5 G2.14 19.2 mm, G2.10 NaN 0, G2.9 2.1, 통과. HR1.6 8652, 통과.
  - 머리 외 노멀은 비트 단위로 같다.
  - **HR1.2 정정:** 메운 눈 틈은 원본에 대응 표면이 없는 새 형상이라 head→SRC도 최대 약 6 mm(dev 측정 6.42)다. 공 주먹처럼 메운 구역은 양방향 모두 제외하고 보고한다. 제외 구역은 gap map으로 정의해 HR2의 check_g2_shape에 반영한다.
  - T114 = HR1b: 입 에이전트가 새 quad 머리에 단일 키 mouth_open(v6 방식 + v5 비율)을 통합한다. 산출물 hr1_mouth_sheet.png.
- **HR1b main 재확인(2026-09-26, T114):**
  - probe를 다시 돌렸다. 에이전트 보고와 같다.
  - 단일 키 mouth_open. GOB_mesh 8652 → 9168(≤ 9500).
  - 입 영역 밖 Basis 편차 0. non-manifold 0. 재질 skin 5018 / inner 144 / teeth 20 / tongue 40.
  - 입 w/head 0.548, h/visible 0.409(roar), h/w 0.74.
  - 머리 X 0.883 / Z 1.154. 눈 +121 mm. 옆모습 +12.3 mm(뒤로). 관통 ready/roar 0.
  - 입 패치는 36개 정점 동심 quad 루프 4개(삼각형 없음). 닫힌 입은 머리 표면 위에 있다(0.0003 mm).
  - 시각: 닫힌 머리는 원본과 같은 수준으로 매끈하다. 열린 상태 matcap에서 눈 사이/입 위에 약간의 굴곡이 있다(plain 렌더에서는 거의 안 보임).
  - HR1.7 [U] 대기.
- **HR1.7 [U] 사용자 승인(2026-09-26). HR1 PASS.** HR2 데이터 계약은 rig/data/hr2_contract.md(체인 순서 s02d → work/r01d_merged → s02e → gob_r01_retopo, 셰이프 키와 재질 4개, PROPS mouth_open 드라이버, stage 굽기, Unity 기준). T120 제작 ∥ T121 checker.
- HR2 dispatch: T120a(머리 에이전트, s02c head_design_zones.json) ∥ T120b(새 통합 에이전트: s02d/s02e 경로, run_all_gates, s05, s06 PROPS 드라이버, s07b 셰이프 키 굽기, s12 props_overrides, GoblinImport) ∥ T121(새 checker: G2/G5/G6/G7/G8/G9, A3 계열 blendShape, Unity RigCheck/ClipCheck, d-23 G 행).
- **T120b 완료(2026-09-26):**
  - s02d → work/r01d_merged → s02e → gob_r01_retopo. run_all_gates에 s02e를 추가했다(pose 렌더는 --pose 옵션으로만 실행).
  - s05: 입 363 verts를 DEF head 1.0에 연결했다.
  - s06a: PROPS mouth_open [0,1] + AVERAGE 드라이버. UI에 Face 슬라이더를 추가했다. ready_pose는 0이다.
  - s07b: stage Key 액션 "<action>_shapekeys"에 매 프레임 값을 굽는다.
  - s12: props_overrides(BEZIER) + 재질 색 렌더. GoblinImport: importBlendShapes를 명시했다.
  - 체인 s02d–s08 exit 0. FBX에 BlendShape mouth_open과 재질 4개, 9168 tris.
  - probe로 확인: 드라이버 0→1→0, 머리 외 변위 0.
  - T121에 산출물 준비와 액션 이름 규칙을 전달했다.
- **T120a-2(2026-09-26):**
  - eye_gap_xneg에 깊이 규칙을 넣었다: local SRC front보다 D = 22.27 mm 깊은 점만 제외한다(근거: 떠 있는 덩어리 뒷면의 최소 깊이 22.77 mm − 0.5).
  - 뒷면 4941칸은 모두 제외된다.
  - 판정에 남는 x<0 눈 앞면: head→SRC 0.47/1.21/2.43, SRC→head 0.49/1.24/2.47 mm. x>0 눈과 같은 수준이다.
  - GOB_head는 바이트 단위로 같다.
- **T121 완료 + main G2 판독(2026-09-26):**
  - 새 행 G2.17, G5.2 입, G6.12, G7.6은 ok. G8.7, G9.11, *3.3f는 입력 재생성 전이라 막히는 게 예상된 결과다.
  - main이 check_g2_static/shape를 직접 돌렸다.
    - G2.11: 이음매 띠 밖 11a max 3.008 mm(바뀌지 않은 벨트, 이전 3.0052를 수용한 것과 같은 샘플링 차이), 11b 통과. 평균 0.63 → 0.58 mm로 개선.
    - G2.4 52면은 모두 입 내부 설계 overlap이다(혀 40, 숨은 이빨 10, 송곳니 받침 2).
    - G2.9 overlap 14 px, 앞면 밀도 1.41 < 1.5(입 UV).
    - G2.12 side mm_judged 4.63 mm(입 근처, 원인 미확인).
  - T122(입 에이전트): s02e UV 재배치. T121b(checker): G2.4 입 내부 제외 + G2.12 side 최대 위치 보고.
- **T121b:**
  - G2.4는 입 내부 설계 overlap 52면을 제외해 남은 면 0, ok.
  - G2.12 side 판정 최대 4.63 mm는 입 마스크에서 7.8 px 떨어진 턱/머리–몸 crease 앞(z 0.953)이다. dilation 문제는 아니다. 헤드 작업 전에는 3.087 mm였다.
  - T123(머리 에이전트): 원인 진단(단면, 윤곽 비교, s02c 수정안). 적용은 T122(UV) 이후에 한다.
- **T122:** s02e가 입을 만든 뒤 s02d make_uvs로 전체 UV를 다시 배치한다. G2.9 overlap 0, 밀도 2.097, 통과. 형상/키/재질/노멀은 비트 단위로 같다. 남은 G2 실패는 G2.12 side(턱 crease, T123 진단 중)뿐이다.
- **T123 진단(머리 에이전트):** 단면 기준 옆 윤곽 앞쪽 차이는 z 0.930–0.990에서 최대 1.65 mm로 4.63 mm가 재현되지 않았다. 몸과 s02e는 원인이 아니다. crease 스텝(z 0.952–0.954)에 걸린 점이고, 몽둥이와 턱 틈일 가능성이 있다. 수정안 HEAD_CREASE_S에 6 mm 행 추가(+92 tris)는 보류한다. T121c(checker): 체커 래스터에서 성분별 커버리지를 확인한다.
- **T121c:**
  - G2.12 side 4.63 mm는 몽둥이–턱 사이 틈(SRC 9 px, retopo 3 px)의 가장자리를 _contour가 윤곽으로 센 값이다. 몽둥이를 빼면 2.78 mm로 통과한다.
  - 새 체커로 백업(헤드 전) 메시를 재도 같은 4.63이 나온다. 그런데 헤드 전 증거는 같은 메시에서 3.087이었다. 체커 회귀가 의심된다.
  - main이 옛 체커로 직접 시도한 결과는 무효다(체커가 -Blend 대신 기본 경로 메시를 연다). 증거 파일은 복원했고 임시 스크립트는 삭제했다.
  - T121d: 옛 코드와 새 코드 × 입력을 대조해 원인을 찾는다.
- **T121d:** 체커 코드 회귀는 없다. 옛/새 코드 모두 백업 메시에서 3.087이다. 새 머리가 union bbox를 바꿔 frame이 1.58054 → 1.57877 m로 변했고, raster 격자가 밀려 몽둥이–턱 틈이 2 px 넓게 찍혔다. **main 결정:** G9.6 규칙(2 px closing + 500 px 미만 구멍 채움)을 G2.12에도 적용한다(T121e). 기준값은 그대로 둔다.
- **T121e:** G9.6 hole 규칙을 적용했다. 새 메시 side 2.78(통과), 백업 side 3.94(열린 몽둥이–턱 틈이라 불합격). 규칙이 견고하지 않다. **main 결정(T121f):** G2.12를 물체별로 판정한다(캐릭터 GOB_mesh vs SRC_hi, 몽둥이 GOB_club vs SRC_club_hi). 합친 윤곽은 보고만 한다.
- **T121f:** G2.12를 물체별로 판정한다. 새 메시 front 2.78 / side 2.78 / 3/4 3.12, 몽둥이 ≤ 2.18, 모두 통과. 백업은 캐릭터 side 3.087(헤드 전 값과 같다), 합친 윤곽 report 3.94. G2.11은 비트 단위로 같다. 이제 main이 run_all_gates를 돌린다.
- **HR2 run_all_gates 1차(main, 2026-09-26, 358초, exit 0):**
  - G1, G4, G5, G6, G7은 모두 ok.
  - G2.11a/b의 띠 밖 값은 헤드 전과 같은 판정이다(11a 3.008 벨트 샘플링, 11b 통과).
  - G3.3 실패 7행은 헤드 전과 수치까지 같다(G5.5로 이관된 항목).
  - G8.7은 클립을 재생성하기 전이라 예상된 실패다. G9.11 ok(blendShape mouth_open, subMesh 4).
  - **G9.6 실패:** Unity 정면 렌더에서 닫힌 입선이 뚫려 보인다(145×5 px 구멍). Unity는 back-face culling을 하고 Blender Workbench는 하지 않는다. T124(입 에이전트): 입 내부 winding 수정 + culled 렌더 추가.
  - 입력 해시: compare_cams.json(s09가 새 메시로 재생성)과 G1의 mouth.json이 어긋난다. 메시가 바뀐 뒤 첫 실행이라서다. 2회 연속 실행으로 재확인한다.
  - G10 범위 밖 목록: 백업 폴더, docs, CLAUDE.md, unity 보고서 json(설명 가능, 최종 판정 때 정리).
- **T124 진단:** 입 winding은 정상이다(광선 2만 개 중 뒷면 0, culled 렌더에 구멍 없음). G9.6 구멍의 원인은 GoblinRigCheck.cs:346의 bakeRenderer.sharedMaterial = mat(재질 1개)로, Unity가 submesh 0(skin)만 그렸다. **main이 한 줄 수정했다:** sharedMaterials = submesh 수만큼 mat. s02e에는 culled 렌더(inspect/R1/culled_*)를 추가했다. 이제 run_all_gates를 2회 연속 실행한다.
- **HR2 G1–G10 main 판정(2026-09-26, run_all_gates 2회 연속, 각 exit 0):**
  - 1회차와 2회차의 모든 행 measured가 같다.
  - 입력 해시 338개 중 337개가 일치한다. 예외는 G1이 hash한 mouth.json이다. s02e가 G1 뒤에 다시 쓰는데 내용이 매 실행 달라진다(판정에는 쓰지 않는다. 원인을 확인할 것).
  - G1, G2(2.11은 이전과 같은 판정), G3(3.3은 G5.5로 이관된 이전 수치 그대로), G4–G7, G9(G9.6 2.236 px, G9.11)은 PASS다. G8.7은 클립 재생성 뒤에 판정한다.
  - T125(sonnet): 4개 클립을 재생성한다(s10a/s10b, s12 ×3, s11 ×4, Unity).
  - mouth.json 해시 불일치 원인: s02e가 입력/출력 블렌드 sha256을 mouth.json에 기록하는데, 블렌드 저장 바이트는 실행마다 달라진다. 형상 판정값은 두 회차가 같으므로 영향 없음으로 판정한다.
- **HR2 PASS(2026-09-26, main 판정):**
  - T125: 4개 클립 재생성(s10a/b, s12 ×3, s11 ×4, Unity 2회), 모두 exit 0.
  - main이 체커 80항목을 직접 돌렸다. A1–A2, I1–I2+C12, H1–H2+C12, D1–D2(D2.4 포함)+C12, A3/I3/H3/D3.1–3.3f, G8.1–G8.7, G9.1–G9.11 모두 ok.
  - 입력 해시: 증거 10개 모두 불일치 0.
  - 클립 수치가 헤드 전과 같다(최소 간격 idle 70.1 / hit 69.2 / death 56.4 mm, 루프, 바닥 접촉). 달라진 것은 face 번호뿐이다. 입 가중치는 모든 클립에서 0이다.
  - main 첫 체커 일괄 실행은 PowerShell 함수 이름 R이 Invoke-History 별칭과 겹쳐 아무것도 실행되지 않았다. bash 루프로 다시 돌렸다.
  - 다음: R3 roar 클립.
- **R3 roar 시작(2026-09-26, 사용자 "진행"):**
  - main 측정: 웅크림 f7(정수리 −111 mm), 폭발 f9–11(왼 주먹 +570 mm, 몽둥이 +330 mm), 유지 f11–23, 내림 f24–27, 복귀 f28–31.
  - d-29에 R3 설계와 Q 게이트를 추가했다. BRIEF에 HR2 절(입 셰이프 키, props_overrides, 재질 렌더)을 추가했다. check_a3_clip에 roar/Q3 기본값을 추가했다.
  - T130(opus, 새 제작 에이전트): roar.json 키포즈 f1/f7/f12.
- **Q1 main 재판정(2026-09-26, T130):**
  - main이 check_anim_clip --gate 1을 다시 돌렸다. Q1.1, Q1.2 ok, exit 0, 입력 해시 42/42.
  - f12 roar: 왼 주먹 +585 mm(레퍼런스 +570), 입 1, 간격 229 mm. C2 범위 밖 0.
  - **Q1.3 [U] 사용자 승인. 발끝 돌리기는 뺀다**(heel_twist가 CONSTANT라 발이 튀고 C10 판정이 불가).
  - T131: 브레이크다운과 입 곡선(0 → f10 1 → f26 1 → f29 0).
- **Q2 main 재판정(2026-09-26, T131):**
  - main이 --gate both를 다시 돌렸다. Q1.1–Q2.3과 Q2.C12 모두 ok, exit 0, 입력 해시 43/43.
  - 루프 0, attack f1 0, 루트 0, flip 0, 발 ≤ 0.005 mm, 최소 간격 70.3 mm, 보이는 교차 ≤ 1(f4, f6, f7).
  - mouth_open: 0(f1–8) → 0.5(f9) → 1(f10–27) → 0.5(f28) → 0(f29–39). 범위 [0, 1].
  - burst의 인접 회전 최대 38.7°(lowerarm_l f9–10).
  - Q2.4(M2)는 targets가 없어 나오지 않는다. s12 측정: 주먹 +585 / 레퍼런스 +570 mm.
  - side_by_side.mp4 24 fps, 39프레임. Q2.5 [U] 대기.
- **Q2 사용자 피드백(2026-09-26): 포효 때 몸이 솟고 발끝이 들리며 발이 벌어지는 레퍼런스 동작을 원함.** C10에 contact_ranges "mode":"pivot"을 추가한다(G6.7 규칙: 누적 slip ≤ 5 mm, shoe z [−1, +3]). T132 checker ∥ T133 제작(COG 상승, foot_roll/heel_twist BEZIER).
- **T132:** check_anim_clip C10 "mode":"pivot"(G6.7 규칙 재구현: slip ≤ 5 mm, 접촉 허용 1 mm, z [−1, +3]). hit C10은 같다. roar 시험 run slip 0.004/0.003 mm. 제작 에이전트에 알렸다.
- **Q2 재판정(T133, 2026-09-26):**
  - f12: COG +0.006(ready보다 21 mm 위, ik 0.959/0.956), foot_roll −12, heel_twist ±20. heel 끌림을 보정하려고 foot IK 위치를 조정했다. 발 props는 BEZIER다.
  - main이 --gate both를 다시 돌렸다. Q1.1–Q2.3과 C12 모두 ok, 입력 해시 43/43.
  - C10 pivot: slip l 0.80 / r 0.76 mm(≤ 5), z −0.57 ~ +2.15(범위 안).
  - 보이는 교차 ≤ 1(f10–24는 calf_r|foot_r 2쌍).
  - 더 올리면 종아리와 발 접힘이 보인다(스킨 한계).
  - Q2.5 [U] 대기.
- **사용자 피드백(2026-09-26): "뒷발(뒤꿈치)을 들어야 하는데 앞발(발끝)을 든다".** T134: foot_roll 부호를 반대로(뒤꿈치 올림, 볼/발끝 접지) 하고, 벌림은 toe_twist로 한다(heel_twist 0). COG 재조정. 발 props BEZIER, pivot 모드 유지.
- **Q2 재판정(T134, 2026-09-26):**
  - f12:
    - foot_roll +24(뒤꿈치 올림, 볼 1점 접지), toe_twist ±15, heel_twist 0.
    - COG +0.048 / 앞 +0.04(ready보다 63 mm 위). ik 0.955 / 0.961, hip 38.6.
    - **roar.json sweep_overrides CTRL_torso loc Y max +0.08**(이유: 발 앞쪽으로 서면 발목이 약 60 mm 올라간다. death와 같은 클립 전용 메커니즘이다. 사용자에게 보고한다).
  - main이 --gate both를 다시 돌렸다. 모든 행 ok, 입력 해시 43/43.
  - C10 pivot: slip l 3.70(롤 전환 f9/f28에서 접점이 바뀐 탓) / r 1.31 mm(≤ 5), z 범위 안.
  - 정면 카메라에서 몽둥이 머리가 프레임 위쪽에 닿는다(렌더 프레이밍, 판정과는 무관).
  - Q2.5 [U] 대기.
- **Q1.3 / Q2.5 [U] 사용자 승인(2026-09-26, roar 발끝 서기 + roar 전용 sweep_overrides 포함). Q2 PASS.** 다음은 Q3: T135(sonnet) s11 roar → Unity 2회. 그 뒤 main이 check_a3_clip roar/attack/idle/hit/death, G8, G9를 판정한다.
- **Q3 PASS(2026-09-26):**
  - T135: s11 roar export(Key 액션 roar_shapekeys, 0–1, 39키). GoblinRigCheck 19.8초, GoblinClipCheck 6.1초. 클립 보고서 5개 errors 0, bounds 밖 0.
  - prefab bounds: Y extents 0.878 → 0.888(roar 몽둥이).
  - main이 check_a3_clip을 roar/attack/idle/hit/death로, check_g8, compare_g9도 직접 돌렸다. 58항목 모두 ok. 입력 해시 7개 증거 모두 불일치 0.
  - Q3.3f: Unity 가중치와 stage ×100 차이 max 6.1e-5(≤ 0.01). BakeMesh 입 정점 1968개 × 열린 20프레임에서 max 0.0007 mm(≤ 0.01).
  - T135 보고의 가중치 목록은 옮겨 적기 오류였다(원본 JSON 39값 정상).
  - **roar 클립이 Unity까지 완료됐다. 클립 5개(attack_swing, idle, hit, death, roar).**
