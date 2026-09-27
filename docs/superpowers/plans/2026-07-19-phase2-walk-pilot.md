# Phase 2 walk 파일럿 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `refs/walk/walk_side_8frames.png` 레퍼런스로 캐릭터의 제자리 walk 사이클(24f, IK 발, 부속물 팔로스루)을 수제 제작해 Unity까지 관통시키고, 사용자 파일럿 판정 재료(걷기 영상)를 산출한다.

**Architecture:** `02_anim`을 프로필 기반 액션 빌더 디스패처로 확장한다(dummy=기존 운반체 불변, character=`scripts/anim/walk_build.py`). 포즈는 파라미터+키 오버라이드 데이터(`walk_poses.py`)로 분리해 에이전트 비교 루프(`walk_compare.py`의 2×8 시트)가 데이터만 고치며 수렴한다. 팔로스루는 몸체 키 완성 후 부속물 컨트롤에 지연·감쇠 키를 자동 생성하는 후처리다. 스펙: `docs/superpowers/specs/2026-07-19-phase2-walk-pilot-design.md`.

**Tech Stack:** Blender 5.1.2 bpy(Rigify 컨트롤 리그 — 이번에 처음 실사용), 기존 프로필/체크/Unity 검증 인프라.

## Global Constraints

- 실행·플래그·프로필 규약은 Phase 1 계획의 Global Constraints 그대로 (`ANIME_PROFILE`, `--python-exit-code 1`, 산출물 `build|exports|previews/<profile>/`, 더미 회귀 상시 초록).
- **컨트롤 본 실명 (build/character/01_rigged.blend 실측, 2026-07-19):** `foot_ik.L/R`, `foot_heel_ik.L/R`, `foot_spin_ik.L/R`, `torso`, `hips`, `chest`, `neck`, `head`, `upper_arm_fk.L/R`, `forearm_fk.L/R`, `hand_ik.L/R`, `thigh_ik.L/R`. **IK/FK 스위치: `thigh_parent.L/R["IK_FK"]=0.0`(다리 IK — 유지), `upper_arm_parent.L/R["IK_FK"]=0.0`(팔도 기본 IK — FK 스윙 쓰려면 1.0으로 키잉 필수).** 부속물 컨트롤: `tail`, `tail.001..004`, `tail_master`, `skirt_{f,fr,r,br,b,bl,l,fl}`+`.001`, `scarf_{l,r}`+`.001`, `hood_ear_{l,r}`.
- **레퍼런스 시트 실측:** 2172×724px, 8개 피규어 x구간 = `[(33,276),(318,542),(597,815),(880,1074),(1159,1399),(1455,1617),(1727,1891),(1975,2139)]` — 균등 분할이 아니므로 이 실측 박스를 슬라이서에 하드코딩.
- 사이클 규격: **24fps, f1–f24, f25=f1 루프, 키 프레임 [1,4,7,10,13,16,19,22]** (레퍼런스 8키 대응), 제자리(전진은 엔진 몫), 캐릭터 전방 = Blender −Y.
- 액션 이름: character **"Walk"**, dummy **"Idle"** — 하드코딩 금지, 프로필 필드 경유.
- 커밋: conventional + `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

## 파일 구조

```
scripts/anim/__init__.py            # 빈 파일                          [T2]
scripts/anim/walk_poses.py          # 걸음 파라미터 + 키 오버라이드     [T2, T5 루프로 갱신]
scripts/anim/walk_build.py          # 액션 빌더 (IK발/FK팔/몸통 키)    [T2]
scripts/anim/followthrough.py       # 부속물 지연·감쇠 후처리          [T3]
scripts/preview/walk_compare.py     # 2×8 비교 시트                    [T4]
(수정) profiles/{dummy,character}.py, 02_anim.py, check_02.py,        [T2]
       03_bake.py, check_03.py, 04_export.py, AvatarCheck.cs,         [T6]
       user_preview.py, PLAN.md                                       [T7]
(수정) AvatarCheck.cs 착륙 폴더 Assets/Import + 텍스처 추출           [T1]
```

---

### Task 1: Unity 검증 프로젝트 정리 (약속된 선행 정리)

Phase 1 말미에 확인된 두 혼란 요인 제거: 착륙 폴더명 `Assets/Dummy`(오해 유발), 뷰포트 흰 재질(임베드 텍스처 미추출).

**Files:**
- Modify: `unity/AvatarCheck/Assets/Editor/AvatarCheck.cs`, `.gitignore`

**Interfaces:**
- Produces: 착륙 경로 `Assets/Import/model.fbx` (이후 태스크·문서가 이 경로 사용). 임포트 직후 `importer.ExtractTextures()` 호출로 텍스처가 `Assets/Import/Textures/`로 추출되어 뷰포트에 재질이 보인다.

- [ ] **Step 1: AvatarCheck.cs 수정**

`FbxAssetPath` 상수의 `"Assets/Dummy/dummy... /model.fbx"` 경로를 `"Assets/Import/model.fbx"`로 변경 (CopyFbxIntoProject의 대상 디렉터리 생성 포함 — 기존 코드가 디렉터리를 만들므로 상수만 바꾸면 됨). `SaveAndReimport()` 직후에 텍스처 추출 추가:

```csharp
// 임베드 텍스처를 추출해 뷰포트에서 재질이 보이게 한다 (사용자 눈검증 UX).
// 반환값은 추출된 텍스처 수 관련 정보가 아니므로 무시; 실패해도 단언에는 영향 없음.
string texDir = "Assets/Import/Textures";
if (!Directory.Exists(texDir)) Directory.CreateDirectory(texDir);
importer.ExtractTextures(texDir);
AssetDatabase.ImportAsset(FbxAssetPath, ImportAssetOptions.ForceSynchronousImport);
```

- [ ] **Step 2: .gitignore 갱신**

`unity/AvatarCheck/Assets/Dummy/` 줄을 `unity/AvatarCheck/Assets/Import/` 로 교체.

- [ ] **Step 3: 구 폴더 삭제 + 검증 실행**

```powershell
Remove-Item -Recurse -Force unity\AvatarCheck\Assets\Dummy, unity\AvatarCheck\Assets\Dummy.meta -ErrorAction SilentlyContinue
powershell -ExecutionPolicy Bypass -File scripts\run.ps1 unity -Profile character
```
Expected: exit 0, `allPass:true` (10/10), `Assets/Import/model.fbx` 존재, `Assets/Import/Textures/`에 텍스처 파일 ≥1개.

- [ ] **Step 4: 커밋**

```powershell
git add unity\AvatarCheck\Assets\Editor\AvatarCheck.cs .gitignore
git commit -m "chore: rename Unity landing slot to Assets/Import, auto-extract textures"
```

---

### Task 2: 액션 디스패처 + walk 포즈 데이터/빌더 + check_02 일반화

**Files:**
- Create: `scripts/anim/__init__.py`(빈 파일), `scripts/anim/walk_poses.py`, `scripts/anim/walk_build.py`
- Modify: `scripts/lib/profiles/dummy.py`, `scripts/lib/profiles/character.py`, `scripts/stages/02_anim.py`, `scripts/checks/check_02.py`

**Interfaces:**
- Produces (프로필 신규 필드): `ANIM_ACTION: str` (dummy `"carrier"` / character `"walk"`), `ACTION_NAME: str` (dummy `"Idle"` / character `"Walk"`), `CLIP_LEN: float` (dummy `1.958` / character `1.0`), `FRAME_END: int` (dummy `48` / character `24`)
- Produces: `walk_build.build(rig, scene) -> None` — Walk 액션 생성 (팔로스루 전 몸체만)
- Produces: `walk_poses.PARAMS: dict`, `walk_poses.KEY_FRAMES: list`, `walk_poses.CONTACTS: dict`, `walk_poses.OVERRIDES: dict` — T5 루프의 수정 대상
- Produces: check_02가 프로필 기준으로 액션명/프레임/루프/발접지를 단언

- [ ] **Step 1: walk_poses.py 작성 (초안 — T5 루프가 수정)**

```python
"""walk 사이클 파라미터. 값 수정 = 애니메이션 수정 (T5 비교 루프의 대상).
전방 = -Y. 제자리 사이클: 접지한 발이 +Y로 등속 후퇴."""

FPS = 24
FRAME_END = 24                      # f25 = f1 (루프)
KEY_FRAMES = [1, 4, 7, 10, 13, 16, 19, 22]
# 접지 구간 (기계 검사용): 발별 (시작f, 끝f) — 이 구간 동안 발 z≈0, y 등속 후퇴
CONTACTS = {"L": (1, 12), "R": (13, 24)}

PARAMS = {
    "stride": 0.34,        # 보폭 (전후 총 이동량, m)
    "foot_lift": 0.07,     # 스윙 중 발 최고 높이
    "foot_x_L": 0.084,     # 발 좌우 폭 (랜드마크 다리 x)
    "torso_bob": 0.020,    # 몸통 상하 진폭 (접지 직후 최저, 통과 시 최고)
    "torso_lean_deg": 5.0, # 전방 숙임 (고정)
    "pelvis_yaw_deg": 6.0, # 골반 요잉 (왼발 전방일 때 왼골반 앞으로)
    "chest_counter_deg": 5.0,  # 가슴 반대 요잉
    "arm_down_deg": 72.0,  # T포즈에서 팔 내리는 기본 각 (rotate_world Y축: L +, R -)
    "arm_swing_deg": 24.0, # 팔 전후 스윙 진폭 (다리와 반대 위상, world X축 회전)
    "elbow_bend_deg": 14.0,# 팔꿈치 상시 굽힘
}

# 키프레임별 미세 오버라이드: {frame: {"<control>": {"rot_world": (axis, deg)} | {"loc_delta": (x,y,z)}}}
# T5 루프에서 레퍼런스와 어긋나는 관절을 여기에 추가한다.
OVERRIDES = {}
```

- [ ] **Step 2: walk_build.py 작성**

```python
"""character walk 액션 빌더. 02_anim이 디스패치한다.
IK 다리(foot_ik 위치 키) + FK 팔(IK_FK=1 전환 후 스윙) + 몸통 키."""
import math

import bpy
from mathutils import Matrix

from anim.walk_poses import FPS, FRAME_END, KEY_FRAMES, PARAMS, OVERRIDES


def _rotate_world(rig, pb, axis, deg):
    bpy.context.view_layer.update()
    head = pb.matrix.to_translation()
    r = Matrix.Translation(head) @ Matrix.Rotation(math.radians(deg), 4, axis) @ Matrix.Translation(-head)
    pb.matrix = r @ pb.matrix
    bpy.context.view_layer.update()


def _key_all(pb, frame):
    pb.keyframe_insert(data_path="location", frame=frame)
    if pb.rotation_mode == 'QUATERNION':
        pb.keyframe_insert(data_path="rotation_quaternion", frame=frame)
    else:
        pb.keyframe_insert(data_path="rotation_euler", frame=frame)


def _reset(rig):
    for pb in rig.pose.bones:
        pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()


def _foot_pose(phase, side):
    """phase 0..1 (사이클 위상, 접지 시작=0). (y, z) 반환. 전방=-Y.
    접지 절반: y -stride/2 -> +stride/2 등속, z=0. 스윙 절반: y 복귀, z 아치."""
    s = PARAMS["stride"] / 2.0
    if phase < 0.5:                       # 접지
        t = phase / 0.5
        return (-s + PARAMS["stride"] * t, 0.0)
    t = (phase - 0.5) / 0.5               # 스윙
    y = s - PARAMS["stride"] * t
    z = PARAMS["foot_lift"] * math.sin(math.pi * t)
    return (y, z)


def build(rig, scene):
    scene.render.fps = FPS
    scene.frame_start = 1
    scene.frame_end = FRAME_END

    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode='POSE')
    _reset(rig)
    pbs = rig.pose.bones

    # 팔 FK 전환 (f1에 키 — 베이크가 상태를 굽도록)
    for side in ("L", "R"):
        par = pbs[f"upper_arm_parent.{side}"]
        par["IK_FK"] = 1.0
        par.keyframe_insert(data_path='["IK_FK"]', frame=1)

    rest_foot = {s: pbs[f"foot_ik.{s}"].matrix.to_translation().copy() for s in ("L", "R")}

    frames = KEY_FRAMES + [FRAME_END + 1]          # f25 = f1 복제로 루프 폐쇄
    for f in frames:
        src_f = 1 if f == FRAME_END + 1 else f
        cycle_t = (src_f - 1) / FRAME_END          # 0..~1
        scene.frame_set(f)
        _reset(rig)

        # --- 다리 (IK): 위상 L=0시작, R=반주기 offset ---
        for side, offset in (("L", 0.0), ("R", 0.5)):
            phase = (cycle_t + offset) % 1.0
            y, z = _foot_pose(phase, side)
            fik = pbs[f"foot_ik.{side}"]
            base = rest_foot[side]
            m = fik.matrix.copy()
            m.translation.x = base.x
            m.translation.y = base.y + y           # rest y 기준 전후
            m.translation.z = base.z + z
            fik.matrix = m
            bpy.context.view_layer.update()

        # --- 몸통: 상하 bob(접지 최저/통과 최고) + 숙임 + 골반/가슴 요잉 ---
        bob = -PARAMS["torso_bob"] * math.cos(4 * math.pi * cycle_t)   # f1 최저, f7 최고 (반주기 2회)
        torso = pbs["torso"]
        mt = torso.matrix.copy()
        mt.translation.z += bob
        torso.matrix = mt
        bpy.context.view_layer.update()
        _rotate_world(rig, torso, 'X', -PARAMS["torso_lean_deg"])      # 전방(-Y) 숙임
        yaw = PARAMS["pelvis_yaw_deg"] * math.sin(2 * math.pi * cycle_t)
        _rotate_world(rig, pbs["hips"], 'Z', yaw)
        _rotate_world(rig, pbs["chest"], 'Z', -PARAMS["chest_counter_deg"] * math.sin(2 * math.pi * cycle_t))

        # --- 팔 (FK): 내리고, 다리 반대 위상 스윙, 팔꿈치 상시 굽힘 ---
        for side, sgn, leg_offset in (("L", +1.0, 0.5), ("R", -1.0, 0.0)):
            ua = pbs[f"upper_arm_fk.{side}"]
            _rotate_world(rig, ua, 'Y', sgn * PARAMS["arm_down_deg"])  # 팔 내림 (컨택트시트 부호 규약)
            swing = PARAMS["arm_swing_deg"] * math.sin(2 * math.pi * ((cycle_t + leg_offset) % 1.0))
            _rotate_world(rig, ua, 'X', swing)
            _rotate_world(rig, pbs[f"forearm_fk.{side}"], 'X', -PARAMS["elbow_bend_deg"])

        # --- 키 오버라이드 (T5 루프가 채움) ---
        for ctrl, spec in OVERRIDES.get(src_f, {}).items():
            pb = pbs[ctrl]
            if "rot_world" in spec:
                _rotate_world(rig, pb, spec["rot_world"][0], spec["rot_world"][1])
            if "loc_delta" in spec:
                m = pb.matrix.copy()
                m.translation.x += spec["loc_delta"][0]
                m.translation.y += spec["loc_delta"][1]
                m.translation.z += spec["loc_delta"][2]
                pb.matrix = m
                bpy.context.view_layer.update()

        # --- 키 삽입 ---
        for name in ("torso", "hips", "chest",
                     "foot_ik.L", "foot_ik.R",
                     "upper_arm_fk.L", "upper_arm_fk.R",
                     "forearm_fk.L", "forearm_fk.R"):
            _key_all(pbs[name], f)

    bpy.ops.object.mode_set(mode='OBJECT')
    rig.animation_data.action.name = "Walk"
```

- [ ] **Step 3: 프로필 필드 추가**

`dummy.py`에 `ANIM_ACTION = "carrier"`, `ACTION_NAME = "Idle"`, `CLIP_LEN = 1.958`, `FRAME_END = 48` 추가. `character.py`에 `ANIM_ACTION = "walk"`, `ACTION_NAME = "Walk"`, `CLIP_LEN = 1.0`, `FRAME_END = 24` 추가.

- [ ] **Step 4: 02_anim.py 디스패처화**

기존 torso 호흡 블록을 `if PROFILE.ANIM_ACTION == "carrier":` 아래로 이동(코드 그대로). else 분기:

```python
else:
    import importlib
    builder = importlib.import_module(f"anim.{PROFILE.ANIM_ACTION}_build")
    builder.build(rig, scene)
```
파일 말미의 `rig.animation_data.action.name = "Idle"` 줄은 carrier 분기 안으로 이동 (walk_build가 자체 명명).

- [ ] **Step 5: check_02.py 일반화 + 발 미끄러짐 단언 (TDD: 먼저 수정 → character에서 RED 확인 → 빌더 구현 순서로 진행해도 좋으나, 빌더와 체크가 한 태스크이므로 완성 후 GREEN만 필수)**

기존 fps/범위/액션명/루프 단언을 프로필 기준으로:
```python
assert scene.render.fps == 24
assert (scene.frame_start, scene.frame_end) == (1, PROFILE.FRAME_END)
assert act is not None and act.name == PROFILE.ACTION_NAME
```
torso 이동·루프 단언은 그대로(경계 프레임을 `PROFILE.FRAME_END`로). **character 전용 발접지 단언 추가**:

```python
if PROFILE.NAME == "character":
    from anim.walk_poses import CONTACTS
    for side, (f0, f1_) in CONTACTS.items():
        ys, zs = [], []
        for f in range(f0, f1_ + 1, 3):
            scene.frame_set(f)
            p = rig.pose.bones[f"foot_ik.{side}"].matrix.translation
            ys.append(p.y)
            zs.append(p.z)
        assert max(zs) - min(zs) < 0.01, f"foot {side} lifts during contact: {zs}"
        dys = [ys[i + 1] - ys[i] for i in range(len(ys) - 1)]
        assert all(d > 0 for d in dys), f"foot {side} not moving backward steadily: {dys}"
        avg = sum(dys) / len(dys)
        assert all(abs(d - avg) < 0.35 * abs(avg) for d in dys), f"foot {side} slide speed uneven: {dys}"
```

- [ ] **Step 6: 실행 — 양 프로필**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 anim -Profile character`
Expected: `STAGE 02_anim OK`(빌더 경유), `CHECK_02 OK` (Walk, 24f, 접지 단언 포함).
Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 anim -Profile dummy`
Expected: 회귀 초록 (Idle, 48f).
알려진 실패 모드: `foot_ik.matrix` 대입이 IK 평가 타이밍과 어긋나 키가 원위치로 들어가면(발 단언 실패) `matrix` 방식 대신 `pb.location`(foot_ik의 로컬 이동) 직접 키잉으로 전환 — foot_ik는 월드 정렬에 가까워 로컬≈월드. 실패 시 그 전환을 보고에 기록.

- [ ] **Step 7: 커밋**

```powershell
git add scripts\anim scripts\lib\profiles scripts\stages\02_anim.py scripts\checks\check_02.py
git commit -m "feat: walk action builder with IK feet and FK arm swing (profile-dispatched)"
```

---

### Task 3: 부속물 팔로스루 후처리

**Files:**
- Create: `scripts/anim/followthrough.py`
- Modify: `scripts/anim/walk_build.py` (말미 1줄 호출), `scripts/anim/walk_poses.py` (파라미터 추가)

**Interfaces:**
- Produces: `followthrough.apply(rig, scene, chains, delay=3, damp=0.6, axis='X') -> None` — 각 체인 컨트롤에 부모 세계 회전 변화의 지연·감쇠 사본을 키잉

- [ ] **Step 1: walk_poses.py에 파라미터 추가**

```python
FOLLOWTHROUGH = {
    "delay_frames": 3,     # 체인 단계당 지연
    "damp": 0.6,           # 단계당 감쇠
    # 체인 루트 컨트롤 → 체인 컨트롤 나열 (실측 컨트롤명, Global Constraints 참조)
    "chains": [
        ["tail", "tail.001", "tail.002", "tail.003", "tail.004"],
        ["skirt_f", "skirt_f.001"], ["skirt_fr", "skirt_fr.001"],
        ["skirt_r", "skirt_r.001"], ["skirt_br", "skirt_br.001"],
        ["skirt_b", "skirt_b.001"], ["skirt_bl", "skirt_bl.001"],
        ["skirt_l", "skirt_l.001"], ["skirt_fl", "skirt_fl.001"],
        ["scarf_l", "scarf_l.001"], ["scarf_r", "scarf_r.001"],
        ["hood_ear_l"], ["hood_ear_r"],
    ],
}
```

- [ ] **Step 2: followthrough.py 작성**

구동 신호 = **hips 컨트롤의 세계 z(상하)와 y(전후) 속도** (몸의 출렁임 원천). 각 체인 컨트롤 i에 대해 `signal(f - delay*(i+1)) * damp**(i+1)`를 로컬 X 회전각으로 키잉:

```python
"""부속물 팔로스루: 몸통 움직임의 지연·감쇠 사본을 부속물 컨트롤에 키잉.
walk_build.build() 말미에서 호출된다. 24f 루프 전제(신호 샘플을 주기적으로 순환)."""
import math

import bpy

from anim.walk_poses import FRAME_END, KEY_FRAMES, FOLLOWTHROUGH


def _hips_signal(rig, scene):
    """프레임별 hips 세계 z 속도(전후 흔들림 근사)를 도 단위 각으로 스케일."""
    zs = {}
    for f in range(1, FRAME_END + 1):
        scene.frame_set(f)
        zs[f] = rig.pose.bones["hips"].matrix.translation.z
    sig = {}
    for f in range(1, FRAME_END + 1):
        nxt = f % FRAME_END + 1
        sig[f] = (zs[nxt] - zs[f]) * 900.0          # m/frame → deg 근사 스케일
    return sig


def apply(rig, scene):
    delay = FOLLOWTHROUGH["delay_frames"]
    damp = FOLLOWTHROUGH["damp"]
    sig = _hips_signal(rig, scene)
    pbs = rig.pose.bones
    for chain in FOLLOWTHROUGH["chains"]:
        for i, name in enumerate(chain):
            pb = pbs[name]
            if pb.rotation_mode == 'QUATERNION':
                pb.rotation_mode = 'XYZ'
            k = damp ** (i + 1)
            d = delay * (i + 1)
            for f in KEY_FRAMES + [FRAME_END + 1]:
                src = ((1 if f == FRAME_END + 1 else f) - 1 - d) % FRAME_END + 1
                ang = math.radians(max(-25.0, min(25.0, sig[src] * k)))
                scene.frame_set(f)
                pb.rotation_euler = (ang, 0.0, 0.0)
                pb.keyframe_insert(data_path="rotation_euler", frame=f)
```

- [ ] **Step 3: walk_build.py 말미(오브젝트 모드 전환 전)에 호출 추가**

```python
    from anim.followthrough import apply as _followthrough
    _followthrough(rig, scene)
```

- [ ] **Step 4: 실행**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 anim -Profile character`
Expected: `CHECK_02 OK` 유지 + (수동 확인) `build/character/02_animated.blend`에서 tail 컨트롤에 키 존재. 더미 회귀: followthrough는 walk_build 안에서만 호출되므로 불변 — `run.ps1 anim -Profile dummy` 초록 재확인.

- [ ] **Step 5: 커밋**

```powershell
git add scripts\anim
git commit -m "feat: procedural follow-through pass for appendage controls"
```

---

### Task 4: 비교 시트 도구 (walk_compare)

**Files:**
- Create: `scripts/preview/walk_compare.py`
- Modify: `scripts/run.ps1` (`walkcompare` 스테이지 등록)

**Interfaces:**
- Consumes: `build/character/02_animated.blend`, `refs/walk/walk_side_8frames.png` (실측 박스)
- Produces: `previews/character/walk_compare.png` — 상단 레퍼런스 8칸 / 하단 우리 렌더 8칸 (각 384px 타일, 3072×768)

- [ ] **Step 1: walk_compare.py 작성**

```python
"""레퍼런스 시트 vs 우리 걸음 렌더의 2x8 비교 시트. T5 수렴 루프의 눈."""
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
import numpy as np
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile
from anim.walk_poses import KEY_FRAMES

PROFILE = get_profile()
TILE = 384
# 레퍼런스 8칸 실측 x구간 (2172x724px)
REF_BOXES = [(33, 276), (318, 542), (597, 815), (880, 1074),
             (1159, 1399), (1455, 1617), (1727, 1891), (1975, 2139)]
# 레퍼런스 피규어는 화면 오른쪽을 본다. 우리 캐릭터(-Y 전방)를 오른쪽 보기로 렌더하려면
# 카메라를 -X쪽에 두고 +X를 바라보게 한다. (첫 실행에서 방향 어긋나면 이 상수만 뒤집는다)
CAM_X = -4.0
CAM_ROT_Z = -90.0

open_blend(paths.blend_path("02_animated"))
scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_x = TILE
scene.render.resolution_y = TILE
scene.render.image_settings.file_format = 'PNG'
world = scene.world or bpy.data.worlds.new("W")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.92, 0.92, 0.92, 1)
sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", 'SUN'))
sun.data.energy = 3.0
sun.rotation_euler = (math.radians(50), 0, math.radians(-120))
scene.collection.objects.link(sun)
cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
scene.camera = cam
mesh = bpy.data.objects[PROFILE.MESH_OBJECT]
cz = mesh.dimensions.z * 0.5
cam.location = (CAM_X, 0, cz)
cam.rotation_euler = (math.radians(90), 0, math.radians(CAM_ROT_Z))

tiles_dir = paths.PREVIEWS_DIR / "walk_tiles"
tiles_dir.mkdir(parents=True, exist_ok=True)
ours = []
for i, f in enumerate(KEY_FRAMES):
    scene.frame_set(f)
    scene.render.filepath = str(tiles_dir / f"ours_{i}.png")
    bpy.ops.render.render(write_still=True)
    ours.append(tiles_dir / f"ours_{i}.png")

# 레퍼런스 슬라이스 → 타일 크기로 리샘플
ref_img = bpy.data.images.load(str(paths.PROJECT_ROOT / "refs" / "walk" / "walk_side_8frames.png"))
rw, rh = ref_img.size
ref_px = np.array(ref_img.pixels[:], dtype=np.float32).reshape(rh, rw, 4)


def _resize_nearest(a, out_h, out_w):
    ys = (np.arange(out_h) * a.shape[0] / out_h).astype(int)
    xs = (np.arange(out_w) * a.shape[1] / out_w).astype(int)
    return a[ys][:, xs]


sheet = np.ones((TILE * 2, TILE * 8, 4), dtype=np.float32)
for i, (x0, x1) in enumerate(REF_BOXES):
    crop = ref_px[:, x0:x1]
    h, w = crop.shape[0], crop.shape[1]
    scale = min(TILE / h, TILE / w)
    th, tw = int(h * scale), int(w * scale)
    tile = _resize_nearest(crop, th, tw)
    y0 = TILE + (TILE - th) // 2                   # 상단 행 (Blender 좌표: 위가 큰 y)
    x0o = i * TILE + (TILE - tw) // 2
    sheet[y0:y0 + th, x0o:x0o + tw] = tile
for i, p in enumerate(ours):
    img = bpy.data.images.load(str(p))
    px = np.array(img.pixels[:], dtype=np.float32).reshape(TILE, TILE, 4)
    sheet[0:TILE, i * TILE:(i + 1) * TILE] = px

out = bpy.data.images.new("cmp", TILE * 8, TILE * 2, alpha=True)
out.pixels.foreach_set(sheet.ravel())
out.filepath_raw = str(paths.PREVIEWS_DIR / "walk_compare.png")
out.file_format = 'PNG'
out.save()
print("WALK COMPARE OK:", paths.PREVIEWS_DIR / "walk_compare.png")
```

- [ ] **Step 2: run.ps1 switch에 `"walkcompare" { Invoke-Blender "scripts\preview\walk_compare.py" }` 추가 (usage 문자열에도)**

- [ ] **Step 3: 실행 + 에이전트 1차 판독**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 walkcompare -Profile character`
Expected: `WALK COMPARE OK`, PNG 생성. Read로 열어 (a) 상하 두 행이 모두 렌더됨, (b) 우리 캐릭터의 바라보는 방향이 레퍼런스와 일치(불일치 시 CAM_X/CAM_ROT_Z 부호 뒤집고 재실행 — 이 조정은 이 태스크에서 허용).

- [ ] **Step 4: 커밋**

```powershell
git add scripts\preview\walk_compare.py scripts\run.ps1
git commit -m "feat: walk reference comparison sheet (2x8)"
```

---

### Task 5: 수렴 루프 (에이전트 반복 — 데이터 수정만)

**Files:**
- Modify (데이터만): `scripts/anim/walk_poses.py` (PARAMS/OVERRIDES/FOLLOWTHROUGH)

**Interfaces:**
- Produces: 수렴된 walk (T6·T7의 전제)

- [ ] **Step 1: 루프 — `run.ps1 anim -Profile character` → `run.ps1 walkcompare -Profile character` → 시트 Read → walk_poses.py 수정 → 반복**

반복당 ~2분 (자동 웨이트 없음 — anim 수십 초 + 렌더 ~1분). 수렴 기준 (모두 충족):
- 8키 각각에서 다리 위상이 레퍼런스와 일치 (어느 다리가 앞인지, 무릎 굽힘 정도)
- 팔 스윙 위상이 다리와 반대, 진폭이 레퍼런스 수준
- 몸통 숙임·상하 흔들림이 레퍼런스의 실루엣과 유사
- 발이 지면에 붙는 구간이 자연스럽고 뚫림/부양 없음
- 부속물(특히 꼬리·목도리)이 정지해 보이지 않음
매 반복을 `.superpowers/sdd/walk-loop-log.md`에 기록 (본 것 → 바꾼 값 → 다음 결과). 10회 초과 시 BLOCKED 보고.

- [ ] **Step 2: 수렴 시 CHECK_02 최종 확인 + 커밋**

```powershell
git add scripts\anim\walk_poses.py
git commit -m "fix: converged walk cycle parameters against reference sheet"
```

---

### Task 6: 베이크→Unity 관통 (액션명/클립 단언 일반화)

**Files:**
- Modify: `scripts/stages/03_bake.py`, `scripts/checks/check_03.py`, `scripts/stages/04_export.py`, `unity/AvatarCheck/Assets/Editor/AvatarCheck.cs`

**Interfaces:**
- Produces: meta.json 신규 필드 `clipName: str`, `clipLen: float`. Unity 단언 `clip_present`/`clip_moves`가 meta 기준으로 판정 (기존 idle_* 단언 대체).

- [ ] **Step 1: 03_bake.py — `"Idle_src"`/`"Idle"` 리터럴을 `PROFILE.ACTION_NAME + "_src"` / `PROFILE.ACTION_NAME`으로. 베이크 프레임 범위 1..48 리터럴을 `1..PROFILE.FRAME_END`로.**

- [ ] **Step 2: check_03.py — `acts[0].name == "Idle"` → `== PROFILE.ACTION_NAME`; hips 이동 검사 프레임 (1, 24)를 `(1, PROFILE.FRAME_END // 2)`로.**

- [ ] **Step 3: 04_export.py meta에 `"clipName": PROFILE.ACTION_NAME, "clipLen": PROFILE.CLIP_LEN` 추가.**

- [ ] **Step 4: AvatarCheck.cs — `PipelineMeta`에 `public string clipName; public float clipLen;` 추가. `idle_clip_present` 단언을 `clip_present`로: 길이 조건 `clip.length > meta.clipLen - 0.25f && clip.length < meta.clipLen + 0.25f`. `idle_actually_moves` → `clip_moves` (로직 동일, 이름만). 명시 매핑·기타 단언 불변.**

- [ ] **Step 5: 전 구간 실행 — 양 프로필**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile character` → Expected: ALL STAGES PASSED, `clip_present` detail에 Walk/≈1.0s, 10/10.
Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile dummy` → Expected: 회귀 초록 (Idle/≈1.96s).

- [ ] **Step 6: 커밋**

```powershell
git add scripts unity\AvatarCheck\Assets\Editor\AvatarCheck.cs
git commit -m "feat: action-name and clip-length generalization through bake/export/unity"
```

---

### Task 7: 걷기 영상 + PLAN 갱신 + 파일럿 승인 게이트

**Files:**
- Modify: `scripts/preview/user_preview.py`, `PLAN.md`

**Interfaces:**
- Produces: `previews/character/user/walk_side.mp4` (측면 3루프 = 72f), `previews/character/user/walk_threequarter.mp4` (3/4뷰 3루프)

- [ ] **Step 1: user_preview.py 확장**

idle 영상 블록 뒤에 추가 (기존 `render_video`/pivot 재사용; 액션명은 하드코딩 금지 — `PROFILE.ACTION_NAME`):

```python
# 걷기(또는 프로필 액션) 루프 영상 2종 — 측면 / 3/4
act = bpy.data.actions.get(PROFILE.ACTION_NAME)
if act is not None and PROFILE.NAME == "character":
    def setup_walk_side():
        pivot.animation_data_clear()
        pivot.rotation_euler = (0, 0, math.radians(-90))   # 측면 (walk_compare와 동일 방향)
        rig.data.pose_position = 'POSE'
        rig.animation_data.action = act

    scene.frame_start = 1
    render_video(out_dir / "walk_side.mp4", PROFILE.FRAME_END * 3, setup_walk_side)

    def setup_walk_tq():
        pivot.rotation_euler = (0, 0, math.radians(45))

    render_video(out_dir / "walk_threequarter.mp4", PROFILE.FRAME_END * 3, setup_walk_tq)
```
주의: `render_video`의 프레임 순환 — 액션은 24f 루프이므로 72f 렌더 시 f25..72가 자동 순환되지 않음. 씬 `frame_end`만 늘리면 액션 밖 프레임은 마지막 포즈로 고정된다 — **액션에 Cyclic 모디파이어를 걸거나(fcurve cycles), 단순하게 씬 범위를 24f로 두고 ffmpeg 먹싱 시 `-stream_loop 2`로 3회 반복**한다. 후자 권장 (렌더 24f×2뷰로 절약): `render_video`에 `loop_count` 파라미터를 추가해 먹싱 명령에 `-stream_loop {loop_count-1}` 삽입.

- [ ] **Step 2: 실행 + 파일 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 userpreview -Profile character`
Expected: 기존 산출물 + walk_side.mp4/walk_threequarter.mp4 (>300KB).

- [ ] **Step 3: PLAN.md Phase 2 갱신**

2b 항목에 walk 파일럿 진행 상황 체크 (레퍼런스 반입 [x], 교정 루프 [x], 완료 기준 "사용자 액션별 승인"은 [ ] + 안내: `previews/character/user/walk_*.mp4`). 파일럿 판정이 Phase 2 방향(전부 수제 vs 하이브리드)을 결정한다는 노트 추가.

- [ ] **Step 4: 커밋 + 컨트롤러가 사용자에게 파일럿 판정 요청**

```powershell
git add scripts\preview\user_preview.py PLAN.md
git commit -m "feat: walk preview videos, pilot approval gate recorded in PLAN"
```

---

## 계획 자기검토 결과 (작성 시 반영)

- 스펙 커버리지: §2 디스패처(T2), §3 IK/FK 적용(T2 — IK_FK 실측 반영), §4 팔로스루(T3), §5 비교 시트(T4 — 실측 박스), §6 검증 3층(T2 발미끄럼 / T5 에이전트 / T6 기계 / T7 사용자), §7 완료 기준(T5·T6·T7), §8 YAGNI 준수(멀티 액션 없음 — Walk 단독), Unity 정리 약속(T1). 전 항목 태스크 존재.
- 실측 반영: 컨트롤 실명, 팔 기본 IK(→IK_FK 키잉), 시트 8칸 픽셀 박스, 카메라 방향 상수화(+1회 뒤집기 허용).
- 형식 일관성: PROFILE 신규 필드 4종(T2 정의)을 T6·T7이 동일 명칭으로 소비. KEY_FRAMES/CONTACTS는 walk_poses 단일 원천.
- 정직한 유보: 초안 PARAMS는 표준 보행 이론 기반 추정 — T5 루프가 수렴 책임(수렴 기준·로그·상한 명문화). foot_ik matrix 키잉의 IK 평가 타이밍 리스크는 대체 경로(location 직접 키잉)를 T2에 명시.
