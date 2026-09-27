# Clay Goblin 게임용 리그 설계 (2026-09-23)

상태: 설계 1~4 사용자 승인 완료. 이 문서는 기존 스윙 전용 Gate 계획(v01~v03)을 대체한다. 기존 산출물은 `work/goblin_swing/t/`에 증거로 보존하고 재사용하지 않는다(레퍼런스 분석 `ref/`는 애니메이션 단계에서 재사용).

## 0. 목표와 범위

- **목표:** Unity(Generic)로 내보내는 게임 캐릭터 1체분 리그. 모든 모션은 Blender 손키 → DEF 베이크 → FBX.
- **모션 범위:** 몽둥이 스윙(레퍼런스 영상), idle·walk·run, hit·death·jump.
- **범위 포함:** 리토폴, DEF 뼈대 + 스키닝, 조작 리그(CTRL/MCH), 무기 소켓, Unity 내보내기 파이프라인 + 검증.
- **범위 제외:** 얼굴 리깅(눈 깜빡임·턱), 손가락 뼈, 배 squash/출렁임, 텍스처, 런타임 무기 detach(물리 드랍은 Unity 측 작업).

### 소스 자산

| 항목 | 경로 | 비고 |
|---|---|---|
| 원본 메시 | `C:\Users\whxod\Downloads\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate.fbx` | 195,506 tris, 1.902×1.007×1.388 m, UV 없음, MD5 `0F937D58…` |
| 참고용 감축 메시 | `UVReviewProjects\Meshy_AI_Clay_Goblin_Warrior_0920141947_generate\runs\run_2a0171c1-…\lowpoly.blend` | 12k tris 자동 감축. 최종 메시 아님 |
| **사용 금지** | `UVReviewProjects\Meshy_AI_Clubby_Goblin_0920122145_generate_1\runs\run_6e1eb23f-…\lowpoly.blend` | 폐기된 Clubby 자산(팔 든 자세, 높이 1.904 m) |
| 모션 레퍼런스 | `work/goblin_swing/ref/` (reference.mp4, REFERENCE_ANALYSIS.md, metrics.json) | 스윙 애니메이션 단계에서 사용 |

소스 파일(Downloads, UVReviewProjects)은 수정하지 않는다.

### 역할 분담

- main: 오케스트레이션 + 각 Gate 재검수(렌더와 수치 직접 확인).
- 구현: opus 서브에이전트, 중첩 서브에이전트 금지.
- 보고는 **기계 검증 결과**와 **품질 판정 요청**을 구분한다. 품질 판정은 사용자가 한다.
- 제작은 headless Blender 5.1 스크립트로 한다. Blender MCP(GUI)는 사용자 뷰어 용도로만 쓰고, 새 산출물이 나오면 GUI로 열어준다.

### 작업 폴더

`work/goblin_swing/rig/`
- `gob_rNN_<stage>.blend`: 단계별 버전. 이전 버전은 덮어쓰지 않는다.
- `scripts/`: 모든 제작·검증 스크립트. 재실행 가능해야 한다.
- `inspect/<gate>/`: 검수 렌더와 JSON.
- `export/`: FBX 출력.

## 1. 파이프라인

```
리토폴 → Static Gate → 임시 DEF로 변형 테스트(30/60/90°) → 리토폴 미세조정 → Retopo Final Gate
→ DEF 위치 확정 + 스키닝 → Skin Gate
→ 조작 리그(CTRL/MCH) → Control Gate
→ Export 아마추어 + 베이크 → Bake Gate → FBX Gate → Unity Gate
```

Gate를 통과하지 못하면 다음 단계로 넘어가지 않는다. Gate마다 main이 재검수하고, 사용자가 품질을 판정한다.

## 2. DEF 뼈대 (설계 1)

```
root                              # Deform OFF, 지면 원점, 루트 모션
└ pelvis
  ├ spine_01
  │ └ spine_02
  │   ├ head                      # 목 없음. 몸통-머리 이음매가 피벗
  │   ├ shoulder_l
  │   │ └ upperarm_l
  │   │   ├ upperarm_twist_l      # deform helper, constraint-driven
  │   │   └ lowerarm_l
  │   │     ├ lowerarm_twist_l    # deform helper, constraint-driven
  │   │     └ hand_l
  │   └ shoulder_r
  │     └ upperarm_r
  │       ├ upperarm_twist_r
  │       └ lowerarm_r
  │         ├ lowerarm_twist_r
  │         └ hand_r
  │           └ weapon_socket_r   # Deform OFF, 무기 부착 Transform
  ├ thigh_l → calf_l → foot_l
  └ thigh_r → calf_r → foot_r
```

- 총 24본, deform 22본. toe 본은 없다(신발은 강체, foot roll은 조작 리그의 MCH 피벗으로 처리).
- twist 본은 관절 체인의 연결 본이 아니라 변형 보조 본이다. Blender 제약이 해당 세그먼트의 비틀림 일부를 받아가고, Unity에는 베이크 결과만 넘어간다.
- 이름은 소문자 + `_l`/`_r`(Blender 좌우 미러에서도 인식됨).
- **프로젝트 정책**(Unity 제약이 아님): 정점당 웨이트 최대 4개, 스케일 1, DEF에 비균등 스케일 키 금지, 롤을 일관되게 맞춰 팔다리 체인이 같은 로컬축으로 굽힘.
- **위치 확정 규칙:** 원본에서 회전 중심 후보를 측정 → 그 위치에 리토폴 관절 루프를 배치 → **최종 리토폴 메시의 실제 회전 중심**으로 DEF 위치를 확정한다. 특히 shoulder·elbow·knee가 해당된다.
- **측정 결과로 확정된 정의 (2026-09-23 사용자 결정):**
  - 이 모델은 팔 튜브가 필렛 없이 몸통에 바로 꽂혀 있다. 그래서 `shoulder_x`는 팔-몸통 접합부로 두고, `shoulder_root_x`(shoulder 본 head)는 팔 축 연장선 위에서 몸 중심축(x=0)과 접합부의 **중간 x**로 둔다. shoulder 본 길이는 약 0.15 m다.
  - 알 몸통의 최대 폭이 벨트 상단에 있어서, `spine_02`는 `spine_01`(벨트 상단)과 `head`(머리 이음매)의 **z 중간**으로 둔다.
- **피벗 → DEF 대응** (피벗 이름은 측정점 이름이고 DEF 본 이름이 아니다. heel·foot_tip·grip_r은 DEF 본이 되지 않는다):

  | 피벗 (pivots.json) | 쓰임 |
  |---|---|
  | shoulder_root_x | shoulder_x 본 head |
  | shoulder_x | shoulder_x tail = upperarm_x head |
  | elbow_x | upperarm_x tail = lowerarm_x head (preferred bend 적용 후) |
  | wrist_x | lowerarm_x tail = hand_x head |
  | hip_x / knee_x / ankle_x | thigh_x head / calf_x head (preferred bend 적용 후) / foot_x head |
  | pelvis, spine_01, spine_02, head | 같은 이름 본의 head |
  | grip_r (+axis) | weapon_socket_r head, 로컬 Y = axis |
  | heel_x, foot_tip_x | MCH foot roll 피벗 전용 (export 제외) |

- **Preferred bend (2026-09-23 리뷰 반영):** 원본 팔·다리 튜브는 곧아서 shoulder–elbow–wrist, hip–knee–ankle이 일직선이 된다. 그러면 IK 구부림 평면이 정의되지 않는다. 그래서 rest 본 체인에 미세한 구부림을 준다. elbow는 +Y(뒤)로 4 mm, knee는 −Y(앞)로 1.5 mm 옮긴다(구부림 각도 0.5°~3° 기준. 다리가 짧아 knee는 오프셋을 작게 둔다). 이것은 G4.4의 굽힘 방향(팔꿈치를 굽히면 손이 −Y, 무릎을 굽히면 발이 +Y)과 같은 방향이다. 메시는 바꾸지 않는다.
- 확정된 bind/rest 뼈대는 **canonical skeleton**으로 JSON에 저장한다(`rig/canonical_skeleton.json`: 이름, 부모, head/tail/roll, rest matrix). 이후 모든 export는 이 템플릿에서 생성한다.

## 3. 리토폴 (설계 2)

### 부위별 방식
- **변형부:** 팔 튜브, 어깨 필렛, 알 몸통, 다리 튜브, 골반-다리 연결부. 스크립트로 새 쿼드 메시를 만들고 원본 표면에 shrinkwrap으로 투영한다.
- **강체부:** 박스 머리(눈 링·눈알·송곳니 포함), 손, 신발, 벨트. 원본을 부위별로 감축해 형태를 보존한다. 겹치는 별도 셸을 허용한다(예: 머리-몸통 이음매).
- ~~손은 원본의 비대칭을 유지한다(왼손 검지 편 손, 오른손 주먹).~~ **(2026-09-25 사용자 결정으로 변경) 양손을 레퍼런스처럼 공 모양 주먹(닫힌 구형 셸)으로 바꾼다.**
  - 크기: 레퍼런스 프레임에서 잰 '주먹 지름 / 머리 폭' 비율로 정한다(실측값과 계산은 제작 스크립트와 BRIEF에 기록).
  - 중심: 손 본 축 위에 둔다. wrist `_1`~`_end` 튜브 끝이 공 안에 들어가야 한다(G2.14).
  - 오른손 공은 몽둥이 자루가 관통하는 설계된 overlap이다(자루가 주먹 위아래로 나온다). 주먹 구멍이나 캡은 없다.
  - 손가락 형상은 없다. 몽둥이는 별도 메시다.
- 몽둥이: 원본 자루가 주먹을 관통하기 때문에, 소스 분리 후에는 주먹 앞(머리+자루)과 주먹 뒤(butt) 두 조각이 된다. 리토폴에서는 `SRC_club_hi`의 반경 프로파일을 주축을 따라 샘플링해 **주먹 속까지 이어진 하나의 회전체**로 재구성한다. 사망 모션에서 떨어뜨렸을 때 끊김이 보이면 안 되기 때문이다.
- **(공 주먹으로 대체되어 폐기)** 오른손: 분리 후 몸 메시에 남은 자루 토막(주먹 앞 약 5 cm, 뒤 약 2 cm 돌출)은 리토폴 강체 hand_r에서 **손바닥 구와 손가락 표면에 맞춰 잘라내고 캡**한다. 작은 원형 절단면은 남는다(2026-09-23 사용자 결정).
  - 원본에서 아래 손가락이 자루와 붙어 있어, 자루를 제거하면 그 손가락 안에 약 6 cm 깊이의 빈 통로가 생긴다. 이것은 **빈 주먹 구멍으로 둔다**(2026-09-24 사용자 결정). 막대를 쥐던 주먹의 빈 통로로 읽히고, 몽둥이를 쥔 상태에서는 몽둥이가 채운다.

### 관절 토폴로지
- 팔다리 튜브는 12각 단면. 어깨 연결부만 국소적으로 밀도를 높일 수 있다.
- 팔꿈치·무릎 3루프(support/center/support), 손목·발목 2루프, 어깨·골반은 2~3단 동심 링, 몸통은 spine_01·spine_02 피벗 높이에 가로 루프, 벨트 위아래 경계에 루프.
- **Pole 규칙:** 어깨·골반 연결부의 3/5-pole은 최대 굴곡선에서 벗어난 저변형 영역에 둔다. 팔을 들 때 늘어나는 위쪽과 압축되는 아래쪽에 면적을 충분히 확보한다.
- 관절 루프 간격은 rest만 보고 확정하지 않는다. 임시 DEF로 30/60/90° 굴곡 테스트를 한 뒤 확정한다.

### 강체-변형 경계
- 변형부와 강체부의 이음매(손목↔손, 발목↔신발)는 필요하면 내부 overlap을 허용한다. 애니메이션 전 범위에서 틈이 보이지 않는 것을 우선한다.
- 벨트: 별도 메시로 두고 Option A(pelvis 또는 spine_01에 100%)로 먼저 테스트한다. 파고들거나 뜨거나 분리되면 Option B(2본에 부드러운 스키닝)로 전환한다.

### 예산 (폴리곤 배분 우선순위)
- 캐릭터 약 6~8.5k tris(꽉 채울 필요 없음, 2026-09-25 상한 8,500으로 조정 — 증가분은 관절 루프), 몽둥이 1k 미만.
- 배분 우선순위: 어깨·골반 연결부(매우 높음) > 머리 실루엣·팔꿈치·무릎(높음) > 몸통·손(중간) > 신발(낮음~중간) > 평평하거나 가려진 면(낮음).

### 노멀과 UV
- 스무스 셰이딩 + Weighted Normal. Blender에서 계산한 커스텀 노멀을 FBX로 넘긴다(Unity에서 modifier가 돌아가는 게 아님). 노멀맵은 없다.
- UV: 0–1 범위, **좌우 겹침 없음**(손 비대칭, 향후 좌우 다른 디테일 대비), 머리 정면은 texel density를 넉넉히, 눈·송곳니 등 강체 셸은 분리 가능.

### Retopo Gate
- **Geometry:** non-manifold 없음, 뒤집힌 노멀 없음, 불필요한 내부 면 없음, 관절부 quad flow 정상, 변형 영역의 pole이 굴곡 중심 밖에 있음.
- **Joint topology:** 어깨 링 2~3단, 팔꿈치 3루프, 손목 2루프, 골반 링 2~3단, 무릎 3루프, 발목 2루프, spine 피벗 높이에 몸통 루프.
- **Shape:** 정면·측면·¾ 실루엣 overlay PASS. 원본 표면 대비 편차 평균 1mm 이하, p95 2mm 이하, 최대 3mm 이하. 머리·눈·송곳니 등 실루엣에 민감한 부분은 수치보다 overlay 렌더 판정이 우선이다.
- **경계:** 손목·발목 틈 없음, 머리-몸통 이음매 문제 없음, 벨트 교차 없음.
- **Technical:** 캐릭터 약 6~8.5k tris, 무기 1k 미만, UV 유효, 커스텀 노멀 정상.
- **변형 테스트(필수):** 임시 DEF를 넣고 어깨·팔꿈치·골반·무릎을 30/60/90°로 굽혀 루프가 제 역할을 하는지 확인한다. 그 결과로 미세조정한 뒤 Final Gate를 한다.
- Unity 셰이딩 확인은 FBX/Unity Gate에서 한다.

## 4. 스키닝 (Skin Gate)

- 강체부는 해당 본에 100%: head(눈·송곳니 포함), hand_l/r, foot_l/r(신발), 벨트(위 Option A/B).
- twist 분산 규칙: upperarm_twist와 lowerarm_twist는 해당 세그먼트의 axial twist를 분산한다. 기본값 50%에서 시작하고, 방향과 비율은 변형 테스트로 확정한다.
- **ROM 테스트(좌우 각각):** 어깨 들기 0→170°, 어깨 앞뒤 ±90°, 윗팔 twist ±90°, 팔꿈치 0→140°, 손목 twist ±90°·bend ±60°, 골반 굴곡 ±60°·외전, 무릎 0→130°, spine·head 굽힘. 모션 범위(스윙·이동·피격·사망·점프)의 대표 포즈를 포함한다.
- **통과 기준:**
  - 강체부 잔차 1mm 미만, 찢어짐·뒤집힌 면 없음.
  - 튜브 최소 단면 반경이 rest 대비 0.8 이상(어깨·팔꿈치·손목·무릎·발목).
  - 체적 변화는 수 % 이내.
  - 경계 틈이 없고, ROM 렌더를 사용자가 판정한다.

## 5. 조작 리그 (설계 3)

역할: CTRL = 애니메이터 인터페이스 / MCH = 계산·피벗·제약 / DEF = Unity가 받는 결과.

### 몸통
```
CTRL_root            # DEF root 구동, 루트 모션
└ CTRL_torso         # COG 공간 자체를 이동
   ├ CTRL_pelvis     # COG 안에서 골반만 상대 이동. 피벗은 허리(DEF pelvis 끝 = spine_01 시작)
   └ CTRL_spine_01
      └ CTRL_chest
CTRL_head            # 공간: chest / world
```
- **CTRL_pelvis는 허리(pelvis 본 tail, 벨트 상단)를 피벗으로 회전한다.** 상체(척추)는 제자리에 있고, 엉덩이와 다리만 흔들린다(걷기 골반 흔들기). 구현은 다음과 같다: CTRL_pelvis를 허리에 두고, 그 자식 MCH_pelvis(rest = DEF pelvis)를 DEF pelvis에 Copy Transforms로 연결한다. (2026-09-25 사용자 결정: 전에는 피벗이 엉덩이 쪽이라 골반을 20° 돌리면 상체가 약 94 mm 밀리고 척추 컨트롤이 메시와 어긋났다)
- head_space의 world는 **회전만 월드 고정**이고, 위치는 항상 chest(목)를 따라간다(2026-09-25 main 결정: 위치까지 world에 두면 몸통을 움직일 때 머리가 제자리에 남는다).
- 공간 전환(head, 손 IK, 무기)은 모두 **스냅 유지 방식**이다. 전환할 때 현재 월드 transform을 유지하도록 새 공간 기준 offset을 계산하며, constraint influence만 바꾸는 방식은 금지한다.

### 팔
- FK: shoulder / upperarm / lowerarm / hand. IK: 손 컨트롤 + 팔꿈치 pole. `ik_fk`(0~1)로 섞는다. IK↔FK 스냅 연산자를 둔다.
- 손 IK 공간: world / root / torso. **weapon 공간은 왼손만** 허용한다(오른손 = 무기 구동, 왼손 = 선택적 추종). 순환 의존을 막기 위해서다.
- IK 스트레치는 없다(DEF 비균등 스케일 금지).

### 다리
- IK 기본 + FK(피격·사망·점프용), IK↔FK 스냅. 무릎 pole의 공간은 발 따라가기 / 월드.
- foot roll(MCH 피벗 체인, toe 본 없음): `roll`(−는 뒤꿈치, +는 발끝 축), `bank`(신발 안·바깥 모서리 축), `heel_twist`, `toe_twist`.
- 각 속성은 **정의된 사용 범위로 clamp**한다. roll은 애니메이션에 필요한 범위, bank는 신발 형상상 허용 범위, twist는 실루엣이 무너지지 않는 범위다.

### 무기
- `CTRL_weapon`의 공간: hand_r(기본) / hand_l / torso / world. 스냅으로 전환한다.
- **hand_l 공간의 대상은 `CTRL_hand_fk_l`(왼손 FK 컨트롤)이다.** DEF hand_l(IK/FK 합성 결과)을 대상으로 두면, 왼손 IK의 weapon 공간과 서로를 대상으로 하는 의존 순환이 생긴다. Blender는 influence가 0이어도 제약 관계를 그래프에 넣는다. 따라서 hand_l 공간은 왼팔 FK(arm_ik_fk_l = 0)일 때 의미가 있고, 전환 연산자와 UI가 이 조건을 알린다. (2026-09-25 main 결정, T23 발견)
- DEF `weapon_socket_r`은 계층상 항상 `hand_r`의 자식이다. Blender에서는 `CTRL_weapon`의 월드 transform을 따라가고, export 시 매 프레임 베이크 후 제약 없이 나간다. 결과는 **weapon socket offset animation**(계층 detach가 아님)이다.

### 기타
- 본 컬렉션 CTRL / MCH / DEF(평소 CTRL만 보이게). 색은 L 파랑 / R 빨강 / 중앙 노랑. 커스텀 셰이프를 쓴다. 속성은 속성 본 하나에 모은다.
- 스냅 연산자는 blend 내 텍스트 스크립트(Auto Run 필요)로 넣고, 같은 코드를 `rig/scripts/`에 애드온으로도 둔다.

### Control Gate
- 모든 컨트롤을 사용 범위 끝까지 움직여도 DEF가 뒤집히거나 튀지 않는다.
- IK↔FK 스냅 오차 1mm·1° 이하.
- 정의된 사용 범위 안에서 신발이 지면을 뚫지 않는다.
- 공간 전환 시 튐이 없다(head, 손 IK, 무기).
- 전 컨트롤을 0으로 두면 rest(T-pose)로 복원된다.

## 6. Unity 내보내기 (설계 4)

### Export 아마추어
```
WORK RIG (CTRL + MCH + DEF)
      ↓ Copy Transforms
GOB_export (24 export bones only; canonical skeleton 템플릿으로 생성)
      ↓ visual bake (매 프레임)
constraints removed
      ↓
FBX
```
- `GOB_export`의 rest matrix와 부모 관계는 `canonical_skeleton.json`에서 생성하며, **클립마다 절대 변하지 않는다.** 작업용 DEF의 현재 상태로 rest를 다시 만드는 것은 금지한다.
- `Only Deform Bones`에 의존하지 않는다(Deform OFF인 root·weapon_socket_r이 빠지기 때문).
- **`GOB_export`의 모든 본은 `use_connect = False`다.** canonical의 connect는 작업 리그용 정보다. 연결된 본은 위치 키를 무시한다. 작업 DEF에서 연결 본의 head가 부모 tail과 떨어지는 경우(FK/IK 블렌드 중의 미세한 이격 등)를 export가 그대로 따라가도록 연결을 끊는다. rest 행렬은 그대로이고 FBX·Unity에는 연결 개념이 없다. (2026-09-25 main 결정, T32에서 발견)
- 작업용 blend는 수정하지 않고, 매 export마다 스크립트로 생성한다. 메시는 같은 버텍스 그룹 이름으로 `GOB_export`에 스킨을 옮긴다.

### FBX 설정
- Forward −Z, Up Y. 오브젝트 스케일 1. Apply Unit = 프로젝트 preset, **Apply Transform = 검증된 preset**. 기준 FBX 하나를 Unity 왕복 테스트해서, root 회전이 기대값이고 아마추어·메시 스케일이 1이며 중간 100×/0.01× 노드가 없는지 확인한 뒤 preset을 잠근다.
- Add Leaf Bones OFF. 커스텀 노멀 포함.
- Bake Animation: sampling 1, Simplify 0, **Force Start/End Keying ON**.
- 아마추어 오브젝트 이름 `goblin`.

### 파일
- `goblin.fbx` = 메시 + 뼈대 + Avatar 소스(rest, 애니메이션 없음).
- `goblin@<clip>.fbx` = 같은 뼈대 계층 + 클립 1개.
- `goblin_club.fbx` = 무기 prop. 피벗·축을 weapon_socket_r에 맞춰, 소켓 밑 local position/rotation 0으로 제자리에 장착된다.
- Unity 임포트 설정은 파일명 `@` 규칙에 의존하지 않는다. 에디터 스크립트(ModelImporter / AssetPostprocessor)나 preset으로 명시적으로 적용한다.
  - `goblin.fbx`: Animation Type Generic, Avatar Create From This Model, Root node `root`.
  - 클립 FBX: Generic, Copy From Other Avatar, 소스는 goblin Avatar.
  - 검증 시 애니메이션 압축 Off. 사용 설정은 검증 후 정한다.

### 클립 목록 (루트 모션 정의)
pelvis 이동과 root 이동은 구분한다. 넉백 같은 "몸의 이동"을 root로 옮길지는 클립마다 명시한다.

| clip | loop | root translation | root rotation | weapon socket offset anim |
|---|---|---|---|---|
| idle | ○ | locked | locked | – |
| walk / run | ○ | animated | animated | – |
| attack_swing | – | locked | optionally locked | – |
| hit | – | knockback을 만들면 animated | optional | – |
| death | – | clip-specific | clip-specific | ○ |
| jump_start / jump_loop / jump_land | loop만 ○ | animated 또는 gameplay-driven(프로젝트 규칙으로 클립 제작 전에 결정) | project rule | – |

물리 드랍(detach + rigidbody)은 Unity 런타임 처리로, 이 범위에 포함하지 않는다.

### Bake Gate
- export 본 24개, 계층 정확히 일치, bind/rest matrix 정확히 일치, CTRL/MCH 없음, 제약 없음.
- 테스트 클립을 DEF만으로 재생해 작업 리그와 비교한다. 위치 오차(거리) 1mm 이하, 회전 오차(쿼터니언 각도 차) 1° 이하.
- twist, IK 결과, 공간 전환, 무기 offset이 유지되고 발이 미끄러지지 않는다.

### FBX Gate
- 아마추어·메시 오브젝트 스케일 1, 예상 밖 transform 노드 없음, leaf bone 없음, 본 이름과 rest 계층 정확히 일치, 커스텀 노멀 유지.

### Unity Gate (`unity/AvatarCheck` 재사용)
- 임포트 설정이 위 규칙대로 적용됐는지 확인한다. 본 24개, 계층이 일치한다.
- 에디터 스크립트가 각 클립을 지정 프레임에서 샘플링해 본 월드 transform을 JSON으로 덤프하고 Blender DEF와 대조한다. 위치는 거리 1mm 이하, 회전은 쿼터니언 각도 1° 이하.
- 무기 prefab을 소켓에 local 0으로 붙였을 때 정상 장착되고, 스윙·사망 클립에서 확인한다.
- 같은 카메라로 Blender와 Unity 렌더를 비교해 실루엣과 셰이딩(커스텀 노멀) PASS.

## 7. Acceptance Gates (구현 전 확정, 판정은 main만)

### 공통 규칙
- **판정:** PASS/FAIL은 main만 한다. `[U]` 표시 항목은 main이 수치를 확인한 뒤 **사용자의 시각 품질 승인**까지 받아야 PASS다.
- **Checker:** 각 Gate의 실행형 검증은 `rig/scripts/check_gN_*.py`이 한다. 출력은 `rig/inspect/GN/<checker>.json`이다.
  - 스키마: `{gate, checker, inputs:[{path, sha256}], criteria:[{id, measured, threshold, ok, note}], renders:[path]}`.
  - `inputs`에는 입력 blend/FBX, 해당 단계까지의 제작 스크립트 전부, `rig/data/*.json`의 sha256을 넣는다.
  - `ok`는 기계적 비교 결과일 뿐이고, 판정은 main이 한다.
  - Checker는 정상 완료 시 exit 0, 스크립트 오류 시 exit ≠ 0이다(판정 결과로 exit code를 바꾸지 않는다).
- **무효화:** Gate N의 `inputs` 중 하나라도 sha256이 바뀌면 GN과 그 하류 Gate의 PASS는 모두 무효가 되고 재실행한다. 하류 관계는 G1→G2→G3→G4→G5→G6→G7→G8→G9의 직선이다. 예외로 Unity 스크립트만 바뀐 경우는 G9만 무효다.
- **단위:** 거리는 m 단위로 측정해 mm로 보고한다. 각도는 도(°)이고, 회전 오차는 쿼터니언 각도 차 `2·acos(|q1·q2|)`로 계산한다.
- **좌표 규약:** Blender에서 캐릭터 정면은 −Y, 위는 +Z, 캐릭터 왼쪽(`_l`)은 +X, 지면은 z=0이다. 크기는 원본 그대로(키 1.388 m)다.

### G1 SRC: 소스 정규화와 피벗 측정
| ID | 조건 | 검증 |
|---|---|---|
| G1.1 | `rig/gob_r00_source.blend`에 `SRC_hi`(몸 전체), `SRC_club_hi`(몽둥이만)가 있고 둘 다 transform이 적용됨(loc 0, rot 0, scale 1) | check_g1_source |
| G1.2 | `SRC_hi` bbox 최소 z = 0 ±0.5 mm. 몸통 알(egg) 단면 중심 x = 0 ±1 mm. 키 1.388 ±0.002 m | check_g1_source |
| G1.3 | 정면이 −Y: 눈 링 정점의 평균 y < 몸통 중심 y | check_g1_source |
| G1.4 | `SRC_club_hi`의 모든 정점이 오른손 쪽(x < 0)에 있음. `SRC_hi`에 몽둥이 머리 형상이 남지 않음(몽둥이 머리 중심 반경 30 mm 안에 `SRC_hi` 정점 0개) | check_g1_source |
| G1.4c | 자루 끝(butt)이 몸에 남지 않음. `SRC_club_hi` 주축(PCA)을 기준으로, 자루 반경 + 3 mm 원통 안에 있으면서 몽둥이의 butt 쪽 끝보다 더 바깥에 있는 `SRC_hi` 정점이 0개. 자루 반경은 butt 쪽 20% 구간 club 정점의 반경 중앙값이고, 주축과 반경은 checker가 club 기하에서 직접 구함. (2026-09-23 추가: 1차 분리에서 주먹을 관통한 자루 끝 약 0.13 m가 몸에 남은 결함을 잡기 위한 항목) | check_g1_source |
| G1.5 | `rig/data/pivots.json`에 필수 키가 모두 있고 각 좌표가 메시 bbox 안에 있음. 필수 키: shoulder_root/shoulder/elbow/wrist/hip/knee/ankle의 `_l`·`_r`, pelvis, spine_01, spine_02, head, grip_r(+axis), foot_tip_l/r, heel_l/r | check_g1_source |
| G1.6 | 좌우 피벗의 미러 오차를 보고(판정 기준 아님, 참고값) | check_g1_source |
| G1.7 [U] | 피벗 마커 overlay 렌더(정면·측면)에서 마커가 관절 중심·튜브 축 위에 있음. 오른손 클로즈업(위·정면, 몸 단독)에서 butt가 제거됨(주먹 앞뒤의 짧은 자루 토막은 사용자 결정에 따라 G2.16에서 처리) | 렌더 |

### G2 RETOPO-STATIC (Retopo Final Gate는 G3 이후 이 Gate를 재실행한 것)
| ID | 조건 | 검증 |
|---|---|---|
| G2.1 | `rig/gob_r01_retopo.blend`의 `GOB_mesh`(단일 오브젝트)에 face int 속성 `part_id`가 있음. 매핑은 `rig/data/parts.json`: body, head, hand_l, hand_r, shoe_l, shoe_r, belt. 몽둥이는 별도 오브젝트 `GOB_club` | check_g2_static |
| G2.2 | 각 part 셸이 닫힌 manifold(non-manifold edge 0). body는 튜브 끝이 강체 셸 안쪽에서 막혀 있음 | check_g2_static |
| G2.3 | 뒤집힌 노멀 0(각 셸 외향 일관성 검사) | check_g2_static |
| G2.4 | 불필요한 내부 면 0: 다른 셸 내부 2 mm보다 깊이 완전히 묻힌 면은 설계된 overlap(손목·발목·머리 이음매·벨트)을 제외하고 0 | check_g2_static |
| G2.5 | 관절 링이 `rig/data/retopo_loops.json`에 있고, 각 링이 닫힌 edge loop이며 개수가 맞음: shoulder 링 2~3, elbow 3(center 1), wrist 2, hip 링 2~3, knee 3(center 1), ankle 2, spine_01·spine_02 몸통 루프 각 1, belt_top·belt_bot 각 1 | check_g2_static |
| G2.6 | center 루프 중심과 해당 피벗의 거리 ≤ 3 mm(elbow, knee, spine_01, spine_02) | check_g2_static |
| G2.7 | Pole 규칙: body의 shoulder·hip 링 영역(가장 바깥 링에서 몸통 쪽 30 mm까지)에서 valence≠4인 정점이 굴곡대에 0개. 어깨 굴곡대는 팔 축 기준 위(+Z)·아래(−Z) 각 ±30° 섹터, 골반 굴곡대는 다리 축 기준 앞(−Y)·뒤(+Y) 각 ±30° 섹터 | check_g2_static |
| G2.8 | 삼각형 수: `GOB_mesh` 6,000~**9,500**(2026-09-25 사용자 결정: G3 관절 루프 보강분으로 8,500. **2026-09-26 사용자 결정: 머리 재토폴로지 + 입 셰이프 키 때문에 9,500으로 올림, d-29 HR**), `GOB_club` < 1,000 | check_g2_static |
| G2.9 | UV: 모든 face에 UV, 모든 좌표 ∈ [0,1], UV 섬 겹침 0(2048² 래스터 검사, 좌우 포함), 머리 정면 texel density ≥ body 평균 × 1.5 **2026-09-26 HR2:** 입 face(입 안·이·혀·입술 패치, material slot 1~3 포함)도 같은 규칙으로 판정한다. material slot별 face 수, [0,1] 밖 loop 수, UV 섬 수는 보고만 한다. | check_g2_static |
| G2.10 | 커스텀 노멀이 있고 NaN 없음 | check_g2_static |
| G2.11 | 원본 대비 표면 편차(양방향: retopo→SRC, SRC→retopo, 각 표면 샘플 ≥ 50k): 평균 ≤ 1 mm, p95 ≤ 2 mm, max ≤ 3 mm. part별로도 보고. 비교는 합집합(SRC_hi ∪ SRC_club_hi vs GOB_mesh ∪ GOB_club) 기준이다. 같은 합집합 안의 다른 닫힌 셸 내부로 1 mm 넘게 들어간 점(가려진 점)과, 다른 닫힌 셸 표면에서 0.1 mm 이내인 점(interface: 원본 몽둥이 절단 캡처럼 겹친 내부 면)은 판정에서 제외한다. 제외한 점의 수는 보고한다. (2026-09-23 T7 리뷰 반영) **설계 변경 영역도 제외한다:** grip_r 축 기준 반경 ≤ r_grip_front + 3 mm(r_grip_front는 G2.16과 같은 방식으로 구한 빈 구간 앞쪽 경계의 자루 반경), 축방향 s ∈ [gap_lo, gap_hi]인 영역(주먹 속 자루 토막과 빈 주먹 구멍 벽)의 점은 양방향 모두 제외하고 개수를 보고한다. (2026-09-24 빈 주먹 결정 반영) **사용자 예외(2026-09-25):** 손목·발목 이음매 띠(wrist·ankle 피벗 기준 축방향 **±40 mm**, 튜브 반경 **+ 20 mm** 이내. G2 실행에서 실측한 현상 범위(피벗에서 35 mm, 튜브 + 18 mm)를 덮도록 ±20 → ±30 → ±40 mm로 확정했다(사용자 결정 2026-09-25))의 편차는 현 상태를 수용한다(T10e 기준: 손목 뒤쪽 최대 약 6 mm, 신발 약 3.7 mm). checker는 이 띠의 값을 그대로 보고하고, main은 이 띠 밖의 값으로 판정한다. **공 주먹 설계 변경(2026-09-25):** retopo 쪽은 hand_l·hand_r 셸 전체, 원본 쪽은 손 전체 영역(wrist 피벗보다 손끝 쪽(s > 0)이면서 wrist 축에서 200 mm 이내인 SRC 점. 원본 손은 손가락까지 약 310 mm였다)을 판정에서 제외하고 개수를 보고한다. (2026-09-25 정정: 처음 정의한 '공 반경 + 30 mm'는 원본 손가락 끝을 덮지 못했다) **2026-09-26 HR2:** GOB_mesh는 Basis 형태(입 닫힘)로 비교한다. 설계 변경 영역 두 곳은 양방향 모두 판정에서 제외하고, 개수와 영역 안 편차 통계를 보고한다. (1) 입 영역: `rig/data/mouth.json` region의 삼각형(output_faces, material slot ≥ 1, region 정점으로 이루어진 삼각형)과, 경계 루프 다각형의 (x, z) 안이면서 입 정점 y 범위 ± 5 mm 안인 점. (2) `rig/data/head_design_zones.json`에서 use가 exclude인 항목(eye_gap_xneg): polygons_xz 안, y_range 안, depth_rule이 있으면 국소 SRC 앞면보다 exclude_depth_min_m 넘게 깊은 점. mouth.json의 output sha256이 검사 blend와 다르거나 두 파일 중 하나가 없으면 G2.11·G2.12는 이유와 함께 blocked. | check_g2_shape |
| G2.12 | 정면·측면·¾ ortho 실루엣(**2048 px** 높이) 윤곽 최대 편차 ≤ 3 mm + 1 px(래스터 양자화 허용, 2026-09-25 사용자 결정). 이음매 띠(G2.11 정의) 투영 영역은 제외하고 판정한다. (2026-09-23 수정: 1024 px에서는 3 mm가 1.4~2 px로 래스터 오차와 비슷함) 공 주먹 변경 뒤에는 손 영역을 판정에서 빼고 보고만 한다. 손 영역은 G2.11의 원본 손 영역과 공 셸을 투영한 윤곽점이다. **2026-09-26 HR2:** 입 영역과 eye_gap_xneg 영역(G2.11 정의의 RETOPO·SRC 삼각형을 투영하고 2 px 팽창)도 판정 윤곽에서 빼고, 영역별 값은 보고만 한다(design_regions_2d). 2026-09-26 HR2: G9.6와 같은 규칙(2 px closing 뒤 500 px 미만 닫힌 구멍 채움)을 윤곽 전에 적용한다. 근거: T121c/T121d — 몽둥이–턱 틈이 raster 위상에 따라 3.087↔4.63 mm로 흔들림(형상 차이 아님). 2026-09-26 HR2(T121f): 몸과 몽둥이를 따로 판정한다(캐릭터 GOB_mesh vs SRC_hi, 몽둥이 GOB_club vs SRC_club_hi). 두 물체 사이 틈은 형상이 아니다(T121c–e: 열린 몽둥이–턱 틈이 raster 위상에 따라 3.09/3.94/4.63 mm로 흔들림). 합친 윤곽 값은 보고만 한다. | check_g2_shape |
| G2.13 [U] | 실루엣 overlay 렌더 3장 + 머리·눈·송곳니 클로즈업 | 렌더 |
| G2.14 | 경계(정적): 손목·발목에서 body 튜브 끝 테두리 정점이 모두 강체 셸 안쪽 ≥ 3 mm. 머리 셸 하단 테두리가 body 안쪽 ≥ 3 mm. body 정점 중 벨트 z 범위의 것이 벨트 셸 바깥면 안쪽 ≥ 0.5 mm | check_g2_static |
| G2.15 [U] | 관절 루프를 색으로 표시한 와이어프레임 렌더(어깨·골반 클로즈업 포함) | 렌더 |
| G2.16 | (2026-09-25 공 주먹으로 재정의) 오른손 공이 그립을 감싼다: grip_r 축 기준 s ∈ [−20, +20] mm 구간의 몽둥이 자루 정점이 모두 hand_r 셸 안쪽 ≥ 1 mm에 있다. 자루는 공 밖으로 양쪽(주먹 앞·뒤)에서 나온다: 공 표면과 자루 축이 만나는 두 교점 사이 거리가 공 지름의 80% 이상. 양손 공의 반경과 중심을 보고한다. [U] 손 클로즈업 | check_g2_static + 렌더 |
| G2.17 | (2026-09-26 HR2, `rig/data/hr2_contract.md`) 메시 계약: `GOB_mesh`의 material slot이 순서대로 GOB_skin, GOB_mouth_inner, GOB_teeth, GOB_tongue(4개)이고 모든 face의 material_index < 4. shape key는 정확히 Basis + mouth_open이고 relative다(use_relative, reference key = Basis, mouth_open의 relative key = Basis). slot별 face·삼각형 수와 key별 이동 정점 수·최대 변위는 보고 | check_g2_static |

### G3 RETOPO-DEFORM: 임시 뼈 변형 테스트
| ID | 조건 | 검증 |
|---|---|---|
| G3.1 | 임시 리그 `rig/gob_r01t_temprig.blend`: pivots.json의 좌표로 만든 임시 DEF. body는 heat 자동 웨이트, 강체 part는 해당 본 100% | check_g3_deform |
| G3.2 | 포즈 세트 좌우 각각: 어깨 들기 30/60/90°, 어깨 앞 30/60/90°, 팔꿈치 30/60/90°, 골반 굴곡 30/60/90°, 무릎 30/60/90°, 손목 twist 90°, 발목 30° | check_g3_deform |
| G3.3 | 30°·60° 포즈에서 관절 링 영역 튜브의 최소 단면 반경 / rest ≥ 0.8(shoulder, elbow, wrist, hip, knee, ankle). 90° 포즈 값은 보고만 하고 G3.6에서 육안 판정한다. (2026-09-25 개정: 자동 웨이트 임시 리그에서 90° LBS 볼륨 손실은 토폴로지가 아니라 웨이트 품질 문제라 분리함) (2026-09-25 사용자 결정: T16a/b 보강 뒤에도 elbow·knee·hip 60°와 wrist twist 90°가 0.66~0.79로 남음. 최저점은 루프 수와 무관해 LBS·heat 웨이트 원인으로 판단 → 리토폴은 확정하고 이 미달분은 G5.5에서 판정) | check_g3_deform |
| G3.4 | (보고 전용, G5.5로 이관) 손목·발목 튜브 끝 테두리 정점의 강체 셸 내부 깊이. 튜브 끝 웨이트가 강체 본을 따라가느냐에 달린 웨이트 문제라 판정은 G5.5에서 한다. | check_g3_deform |
| G3.5 | (a) rest 포즈에서 body 자기교차 0. (b) 30°·60° 포즈에서 관절 영역 body 면의 자기교차 0(설계된 overlap 제외). 90°는 보고만 한다. (c) 모든 포즈에서 면적이 rest의 10% 미만으로 붕괴한 면 0 | check_g3_deform |
| G3.6 [U] | 포즈별 관절 클로즈업 시트(와이어 겹침 포함) | 렌더 |
| 후속 | 루프 간격을 조정했다면 G2 전체와 G3를 재실행하고, 둘 다 PASS면 **Retopo Final** | — |

### G4 SKEL: canonical skeleton
| ID | 조건 | 검증 |
|---|---|---|
| G4.1 | `rig/gob_r02_skeleton.blend`의 `GOB_rig`에 스펙 §2의 24본. 이름과 부모가 정확히 일치하고 다른 본은 없음(이 단계에서는 DEF만) | check_g4_skeleton |
| G4.2 | use_deform: root, weapon_socket_r = False, 나머지 22 = True | check_g4_skeleton |
| G4.3 | 본 head 위치: upperarm=shoulder, lowerarm=elbow center 루프 중심 + preferred bend 오프셋, hand=wrist 링 중심, thigh=hip, calf=knee center 루프 중심 + preferred bend 오프셋, foot=ankle 링 중심. 모두 ≤ 2 mm. shoulder_x head=shoulder_root ≤ 2 mm | check_g4_skeleton |
| G4.4 | 롤 규약: 각 팔·다리 체인(upperarm·lowerarm·hand / thigh·calf)의 로컬 X축이 체인 안에서 평행(≤ 1°). lowerarm_l/r을 로컬 X +30° 회전하면 손이 −Y로 이동. calf_l/r을 로컬 X +30° 회전하면 발이 +Y로 이동 | check_g4_skeleton |
| G4.5 | twist 본: upperarm_twist는 head=upperarm head, 길이=upperarm의 50%, 같은 축과 롤(≤ 0.1°). lowerarm_twist는 head=lowerarm의 50% 지점, tail=lowerarm tail, 같은 축과 롤 | check_g4_skeleton |
| G4.6 | weapon_socket_r: head=grip_r ≤ 2 mm, 로컬 Y축 = 몽둥이 자루 축(몽둥이 머리 방향) ≤ 1°, 부모 hand_r | check_g4_skeleton |
| G4.7 | `rig/data/canonical_skeleton.json`(name, parent, head, tail, roll, use_deform, rest matrix)으로 새 아마추어를 생성하면 모든 본의 rest matrix 차이가 ≤ 1e-5 | check_g4_skeleton |
| G4.8 | root head=(0,0,0), tail=(0,0,0.2), 롤 0 | check_g4_skeleton |
| G4.9 | Preferred bend: 팔(upperarm·lowerarm)과 다리(thigh·calf) 체인마다 두 세그먼트 사이 각도가 0.5°~3°(1차 기준). 구부림 방향이 elbow는 +Y, knee는 −Y. 구부림 평면의 법선과 체인 본들의 로컬 X축이 평행(≤ 1°). 튜브 축에서 본 head까지의 오프셋은 elbow 4 ±0.5 mm, knee 1.5 ±0.5 mm. (2026-09-25 수정: 다리가 150 mm라 4 mm를 옮기면 6°가 되어 각도 기준과 모순이었음. T13에서 발견) | check_g4_skeleton |

### G5 SKIN
| ID | 조건 | 검증 |
|---|---|---|
| G5.1 | `rig/gob_r03_skinned.blend`: 정점당 0이 아닌 웨이트(>1e-4) ≤ 4, 합 = 1 ±1e-4, root·weapon_socket_r 웨이트 0 | check_g5_skin |
| G5.2 | 강체 part는 단일 본 100%: head→head, hand_x→hand_x, shoe_x→foot_x, belt→(Option A) pelvis 또는 spine_01 / (Option B) 2본. 선택을 `rig/data/skin_decisions.json`에 기록 **2026-09-26 HR2:** 입 정점(GOB_mouth_inner·GOB_teeth·GOB_tongue slot face의 정점과 mouth_open key가 움직이는 정점)은 모두 head part 정점이고 DEF head 100%여야 한다. head.mouth에 보고하고, 입이 없으면 0개라 판정할 것이 없다. | check_g5_skin |
| G5.3 | twist 제약: upperarm_twist·lowerarm_twist가 각 세그먼트의 axial twist를 분산. 비율과 방향을 skin_decisions.json에 기록(시작값 0.5). (2026-09-25 사용자 결정: Copy Rotation의 Euler 추출은 대각 스윙 90°에서 twist 본이 15° 따라 돌아서 Transformation 제약 SWING_TWIST_Y로 교체) (b) twist 없는 대각 스윙 90°에서 twist 본의 부모 기준 축 회전 ≤ 2° | check_g5_skin |
| G5.4 | ROM(`rig/data/rom_poses.json`, 스펙 §4의 전 범위 + 스윙·이동·피격·사망·점프 대표 포즈 각 ≥ 1): 강체 잔차 < 1 mm | check_g5_skin |
| G5.5 | ROM 판정 대상(judged: shoulder·elbow·knee·hip 30/60°, wrist twist 90°, ankle 30°): **30° 단면 비 ≥ 0.8, 60° 단면 비 ≥ 0.75**, 붕괴 면 0, 자기교차는 모든 교차 쌍을 덮는 최소 면 집합(vertex cover) ≤ 1개, 즉 면 하나가 튀어나와 이웃을 뚫는 경우까지만 허용하고 [U]로 육안 확인한다(2026-09-25 정정: 처음에는 '고유 면 수'로 적어 상대 면까지 셌다). 경계 노출: wrist/ankle `_end` 깊이 ≥ 0, `_1` 링 깊이 ≥ rest 깊이 − 0.1 mm(rest보다 나빠지지 않음). 그 외 ROM 포즈는 보고만 하고 G5.7 육안 판정. (2026-09-25 실측 후 사용자 확정: 60°는 LBS 한계가 elbow 0.79·knee 0.77이다. 발목 `_1` 링은 rest에서 이미 신발 밖 0.25~0.43 mm인 리토폴 형상이라 포즈에 따른 악화만 판정한다) | check_g5_skin |
| G5.6 | ROM 전 포즈: body 체적 변화 ≤ 5% | check_g5_skin |
| G5.7 [U] | ROM 시트(전신 + 어깨·팔꿈치·손목·골반·무릎·발목 클로즈업) | 렌더 |

### G6 CTRL
| ID | 조건 | 검증 |
|---|---|---|
| G6.1 | `rig/gob_r04_ctrl.blend`: 스펙 §5의 CTRL 본이 모두 있음(목록은 `rig/data/ctrl_manifest.json`). 본 컬렉션 CTRL/MCH/DEF가 있고 기본 가시성은 CTRL만. 색은 L/R/C 규약 | check_g6_ctrl |
| G6.2 | 모든 CTRL 기본값(identity, 속성 기본값)에서 DEF pose matrix = canonical rest ≤ 1e-4 | check_g6_ctrl |
| G6.3 | IK↔FK 스냅(팔·다리 각각, 무작위 포즈 20개): 스냅 전후 DEF 월드 차이 ≤ 1 mm, ≤ 1° | check_g6_ctrl |
| G6.4 | 공간 전환(head chest↔world, 손 IK world/root/torso(+왼손 weapon), 무기 hand_r/hand_l/torso/world, 무릎 pole): 전환 프레임 전후 대상 월드 차이 ≤ 1 mm, ≤ 1° | check_g6_ctrl |
| G6.5 | 각 CTRL을 사용 범위(ctrl_manifest) 안에서 1° 또는 1 cm 간격으로 sweep한다. 인접 샘플 사이 DEF 본 회전 변화가 샘플 간격 × 3 + 1°를 넘는 구간은 튐 후보로 보고, 그 구간을 이분해 최대 변화 쪽으로 4단계(간격/16)까지 좁힌다. 간격/16에서의 변화가 원래 변화의 0.5배를 넘으면 불연속(튐)이고, 튐이 0개여야 통과한다. NaN도 없어야 한다. 사용 범위는 캐릭터 치수에 맞는 물리적으로 의미 있는 범위다(다리 150 mm). (2026-09-25 실측 후 정정: 거의 곧은 관절 근처의 IK는 연속이지만 기울기가 커서(무릎 약 35°/mm) 고정 간격 기준이 오판했고, 다리 길이를 넘는 sweep 범위는 실제 접힘 튐을 만들었다) (b) 팔·다리 IK/FK 체인마다 굽힘 부호 θ = atan2(dot(cross(u,l), X_upper), dot(u,l))를 기록한다(u, l은 윗·아랫 세그먼트 방향, X_upper는 윗 본의 월드 로컬 X). 인접 샘플에서 부호가 바뀌고 두 샘플 모두 |θ| > 2°이면 branch flip(무릎·팔꿈치 반전)이며, 0개여야 한다. (2026-09-25 외부 리뷰 반영) | check_g6_ctrl |
| G6.6 | sweep 전체에서 DEF 본 스케일 = 1 ±1e-4(비균등 스케일 없음). IK 체인의 ik_stretch = 0 | check_g6_ctrl |
| G6.7 | 준비 자세(CTRL_torso를 월드 Z −15 mm로 내려 무릎 약 51° 굽힘, 다리 IK, 발 IK는 rest)에서 foot_roll·bank·heel_twist·toe_twist를 각각 ctrl_manifest clamps 범위 안에서 1° 간격으로 sweep한다. 모든 값에서 **planted** 조건을 만족해야 한다: (1) IK 도달 오차(MCH_calf_ik tail ↔ MCH_roll_foot head) ≤ 1 mm, (2) 신발 최저 정점 z ∈ [−1, +3] mm, (3) roll·bank에 한해 누적 slip ≤ 5 mm. 0에서 각 방향으로 한 스텝씩 갈 때, 스텝 k의 접지 정점 집합(z ≤ 그 스텝 최저 z + 1 mm)이 스텝 k+1까지 수평으로 움직인 평균 거리를 더한 값이다. 구르는 접점은 멈춰 있고 미끄러지는 접점만 누적되며, 최저 정점이 바뀌어도 값이 튀지 않는다. (2026-09-25 외부 리뷰 반영, 접지 영역 중심 이동은 정상적인 뒤꿈치 구름도 잡아서 쓰지 않음) (2026-09-25 실측 후 확정: 처음 기준인 z ≥ −1 mm만으로는 발이 뜨는 것과 미끄러지는 것을 잡지 못했다. 사용 가능 범위는 roll 약 −30~+27°, bank 최소 ±13°이고, 발끝 쪽이 +40°에 못 미치는 것은 신발 앞코가 위로 휜 형상 때문이라 수용했다) | check_g6_ctrl |
| G6.8 | 오른손은 weapon 공간을 가질 수 없음. 의존 그래프에 순환 없음(Blender 의존성 경고 0) | check_g6_ctrl |
| G6.9 | 스냅·공간 전환·리셋·Ready pose 연산자가 blend 내 텍스트에 등록되고, 같은 코드의 애드온 `rig/scripts/addon/goblin_rig_ui.py`가 import됨 | check_g6_ctrl |
| G6.10 [U] | 사용자가 Blender GUI에서 조작성 확인(main이 MCP로 파일을 열어줌) | 사용자 |
| G6.11 | `goblin.ready_pose` 연산자(작업 시작 자세): CTRL_torso를 월드 Z −15 mm로 내리고, 팔은 현재 모드(FK/IK) 그대로 팔꿈치를 약간 굽힌다. 다리가 FK이면 IK 해에 FK 컨트롤을 맞춰 발을 지면에 유지한다. 팔 FK/IK × 다리 FK/IK의 4가지 조합 모두에서 판정한다. 결과에서 무릎 굽힘 40~60°, 팔꿈치 굽힘 10~30°, 양발 planted(G6.7 (1)(2)), DEF 스케일 1이어야 한다. 오토키가 켜져 있으면 키를 넣는다. (2026-09-25 외부 리뷰 반영: 거의 곧은 rest 근처에서 IK가 민감한 문제의 대책. soft IK는 rest가 이미 99.96% 펴져 있어 G6.2와 충돌하므로 채택하지 않음) | check_g6_ctrl |
| G6.12 | (2026-09-26 HR2, `rig/data/hr2_contract.md`) PROPS `mouth_open`: float, UI 범위 [0, 1], UI 기본값 0, props 계약 기본값 0. `GOB_mesh` shape key `key_blocks["mouth_open"].value`에 driver가 정확히 1개: AVERAGE, 유효, SINGLE_PROP 변수 1개가 OBJECT `GOB_rig`의 `pose.bones["PROPS"]["mouth_open"]`를 읽는다. 속성을 0, 0.25, 0.5, 0.75, 1, 0으로 바꿀 때 평가된 key 값 = 속성 값(≤ 1e-6). `goblin.reset_rig`와 `goblin.ready_pose`(mouth_open 0.7에서 호출) 뒤 속성과 평가된 key 값이 0(≤ 1e-6)이고 연산자 오류가 없다 | check_g6_ctrl |

### G7 BAKE
| ID | 조건 | 검증 |
|---|---|---|
| G7.1 | 테스트 클립 `rigtest`(CTRL 키, `rig/data/rigtest_manifest.json`에 구간 정의)가 다음을 모두 포함: root 이동·회전, COG, **골반 독립 회전·이동(CTRL_pelvis)**, 팔 FK ROM, IK/FK blend·스냅, head 공간 전환, 왼손 weapon 공간, 무기 world 공간(socket offset)과 복귀, foot roll/bank sweep, 다리 FK | check_g7_bake |
| G7.2 | `GOB_export`: 본 24개, 이름·부모 = canonical, rest matrix = canonical ≤ 1e-5, 제약 0, 모든 본 use_connect False, CTRL/MCH 오브젝트·본 없음 | check_g7_bake |
| G7.3 | 서로 다른 두 클립(rigtest, rest 1프레임)을 각각 생성했을 때 GOB_export의 rest가 동일 ≤ 1e-6 | check_g7_bake |
| G7.4 | rigtest 전 프레임, 24본 월드 transform 비교(작업 리그 DEF vs 베이크된 GOB_export): 위치 ≤ 1 mm, 회전 ≤ 1° | check_g7_bake |
| G7.5 | 구간별 오차를 보고: twist, IK, 공간 전환, 무기 offset, 발 고정(발 IK 고정 구간 foot 월드 이동 ≤ 1 mm) | check_g7_bake |
| G7.6 | (2026-09-26 HR2) rigtest 전 프레임: stage `goblin_mesh`의 평가된 shape key 값 mouth_open = 작업 리그 `GOB_mesh`의 평가된 값(≤ 1e-6). 두 stage(rigtest, rest) 모두 key mouth_open을 유지한다(relative). stage Key 액션·driver·modifier는 보고 | check_g7_bake |

### G8 FBX
| ID | 조건 | 검증 |
|---|---|---|
| G8.1 | `rig/export/goblin.fbx`, `goblin@rigtest.fbx`, `goblin_club.fbx` 생성. 설정은 `rig/data/export_preset.json`(Forward −Z, Up Y, leaf bone OFF, bake sampling 1, simplify 0, force start/end keying ON, apply_unit·apply_transform 값과 Blender→Unity 축 매핑 행렬 기록) | check_g8_fbx |
| G8.2 | factory-startup Blender로 재임포트: 아마추어 `goblin`과 메시 `goblin_mesh` 외에 transform 노드 없음, 둘 다 스케일 1 ±1e-6 | check_g8_fbx |
| G8.3 | 재임포트 본 = 24개, 이름·계층 일치, `_end` leaf 없음, rest가 canonical과 일치 ≤ 0.5 mm / 0.5°(축 변환 보정 후) | check_g8_fbx |
| G8.4 | 커스텀 노멀 유지: 재임포트 노멀과 원본 노멀의 각도 차 p99 ≤ 1° | check_g8_fbx |
| G8.5 | 재임포트 rigtest 프레임 수가 같고, 24본 월드 transform이 GOB_export와 일치 ≤ 1 mm, ≤ 1° 재임포트 뒤 비교 전에 모든 본을 use_connect False로 둔다. Blender FBX 임포터는 부모 tail이 자식 head에 닿으면 자식을 연결해 위치 키를 무시하게 만드는데, 이는 FBX 내용과 무관한 임포터 동작이다. Unity에는 연결 개념이 없다. (2026-09-25 T35c 진단: FBX 파일의 pelvis Lcl Translation 곡선은 정상이었다) | check_g8_fbx |
| G8.6 | goblin_club.fbx: 삼각형 < 1,000. 메시 데이터가 **weapon_socket_r rest 프레임** 기준이다. 즉 소켓 rest 월드 행렬로 변환하면 Blender의 rest 몽둥이와 정점 위치가 ≤ 0.5 mm로 일치한다(원점 = grip, 로컬 +Y = 자루 축(몽둥이 머리 방향) ≤ 1°, 롤 포함). (2026-09-25 정정: 원점과 +Y만 보면 자루 축 둘레 롤 89° 차이를 놓쳤다. T35에서 발견) | check_g8_fbx |
| G8.7 | (2026-09-26 HR2) goblin.fbx와 모든 goblin@*.fbx를 재임포트: goblin_mesh에 shape key mouth_open(BlendShape/Channel, relative)과 계약 이름의 material 4개가 있다. 클립마다 재임포트한 mouth_open 값(DeformPercent 곡선 / 100, 곡선이 없으면 정적 값 0)이 stage goblin_mesh의 평가된 값과 매 프레임 ×100 기준 ≤ 0.01(anim_offset 보정, stage = `rig/export/stage_<clip>.blend`) | check_g8_fbx |

### G9 UNITY (`unity/AvatarCheck`)
| ID | 조건 | 검증 |
|---|---|---|
| G9.1 | 임포트 설정이 에디터 스크립트로 적용됨. goblin.fbx: Generic, Create From This Model, Root node `root`. goblin@rigtest.fbx: Generic, Copy From Other Avatar(goblin Avatar). 압축 Off | GoblinRigCheck.Run → `goblin_report.json` |
| G9.2 | 계층: `goblin` 아래 24본(이름·계층 일치), 예상 외 노드 0, 모든 transform의 lossyScale = 1 ±1e-4 | goblin_report |
| G9.3 | 방향: rest에서 root→head 벡터 = Unity +Y ≤ 1°, hand_r→hand_l 벡터 = Unity −X ≤ 1°(정면 +Z) | goblin_report |
| G9.4 | rigtest 샘플 프레임(rigtest_manifest의 구간마다 ≥ 3프레임)에서 24본 월드 transform(축 매핑 적용) vs Blender GOB_export: 위치 ≤ 1 mm, 회전 ≤ 1° | compare_g9.py |
| G9.5 | 몽둥이 prefab을 weapon_socket_r에 local 0으로 붙였을 때, 샘플 프레임의 몽둥이 월드 transform vs Blender 몽둥이: ≤ 1 mm, ≤ 1° | compare_g9.py |
| G9.6 | 같은 카메라(`rig/data/compare_cams.json`)로 렌더한 Blender vs Unity 실루엣(정면·¾, 샘플 3프레임) 윤곽 최대 편차 ≤ 3 px(1024 px 높이) 두 마스크 모두 2 px 반경 morphological closing을 적용하고, 500 px 미만의 작은 구멍은 채운 뒤 비교한다(closing은 거의 닫힌 1~3 px 틈이 한쪽에서만 트이는 픽셀 경계 차이를 없앤다. front_58에서 4.1 px로 나왔던 경우다)(2026-09-25: 주먹-머리 틈에 생긴 1~4 px 구멍의 모양 차이가 윤곽 거리 54 px로 튀었다. IoU 0.997). | compare_g9.py |
| G9.7 [U] | Blender/Unity 렌더 나란히 비교(셰이딩·노멀) | 렌더 |
| G9.8 | 프레임 사이 비교: G9.4 샘플 구간마다 N+0.25/0.5/0.75에서 Unity AnimationClip.SampleAnimation 결과를 선형 보간 참조와 비교한다. 참조는 Blender 정수 프레임 N, N+1을 부모 로컬 lerp + slerp로 보간한 뒤 월드 체인을 재구성해 만든다. 전 본·전 샘플에서 위치 p95 ≤ 2 mm, 회전 p95 ≤ 1°(쿼터니언 각거리)이고, 최대 위치 ≤ 20 mm, 최대 회전 ≤ 15°여야 한다. 최대값 상한은 쿼터니언/오일러 뒤집힘이나 시간 오프셋 같은 왜곡을 잡기 위한 것이다(이런 왜곡은 수십 도 또는 수 cm로 나타난다). Blender 베지어 참조는 보고만 한다. (2026-09-25 실측 후 확정, 사용자에게 보고함. 실측값: 선형 참조 대비 p95 1.69 mm / 0.72°, max 15.9 mm / 7.4°(f371, 왼손 IK 목표가 팔 길이 1.7배 밖에서 돌아오는 급변 구간). 처음 초안인 '이동량 × 0.25 + 1 mm'는 정지 본에서 Unity 곡선 탄젠트 차이 1~2 mm를 걸러내지 못했다. 이 기준은 측정을 본 뒤 정한 것이다) | compare_g9.py |
| G9.9 | Unity 임포트 본의 local rest(바인드) 행렬 = canonical(축 매핑 적용) ≤ 1e-4, goblin.fbx와 goblin@rigtest.fbx 모두 | goblin_report |
| G9.10 | SkinnedMeshRenderer bounds: rigtest 전 프레임에서 스킨 정점(베이크 메시) AABB가 renderer localBounds 안에 있다(여유 포함). 실패하면 bounds를 넓혀 prefab에 기록한다 | goblin_report |
| G9.11 | (2026-09-26 HR2) goblin.fbx SkinnedMeshRenderer에 blend shape `mouth_open`이 있고 subMeshCount = 4, material 4개(이름과 순서는 보고) | GoblinRigCheck mesh_contract → compare_g9 |

### G10 FINAL
- (2026-09-25 외부 리뷰 반영) 출시 설정에서 애니메이션 압축을 켤 계획이면 G10 전에 압축 ON 상태로 G9.4/G9.8을 한 번 더 실행하고 오차를 기록한다. 기준은 압축 오차 허용치를 실측해 정한다.
| ID | 조건 | 검증 |
|---|---|---|
| G10.1 | `rig/scripts/run_all_gates.ps1`로 G1~G9 checker를 최종 파일에서 재실행하고, 모든 evidence의 inputs sha256이 현재 파일과 일치 | run_all_gates |
| G10.2 | 합의문서(§0~§7)와 최종 산출물 대조 체크리스트 완료 | main |
| G10.3 | 범위 밖 변경 0: 저장소 `git status`에서 기준선(T0 스냅샷) 대비 바뀐 경로가 `work/goblin_swing/rig/**`, `unity/AvatarCheck/Assets/Editor/Goblin/**`, `unity/AvatarCheck/Assets/Goblin/**`(+ .meta)뿐 | run_all_gates |

## 8. 확정되지 않은 것 (구현 중 결정, 사용자 확인)
- jump 루트 모션 규칙(animated vs gameplay-driven): jump 클립 제작 전에 결정한다.
- 벨트 스키닝 Option A/B: Skin Gate 테스트 결과로 결정한다.
- twist 비율·방향, 관절 루프 최종 간격: 변형 테스트 결과로 결정한다.
- Apply Transform preset: FBX 왕복 테스트로 결정한다.
- 팔꿈치·무릎 위치: 원본 튜브에 관절 형상이 없으므로 기본값을 shoulder→wrist, hip→ankle의 50% 지점으로 둔다. G1.7 / G3.6에서 사용자가 조정할 수 있다.
