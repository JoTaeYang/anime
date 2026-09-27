# Unity 스프링 본 설계 — 부속물 2차 모션 + 발 IK

> 작성일: 2026-07-26 · 상태: 승인됨 (사용자)
> 상위: [PLAN.md](../../../PLAN.md) Phase 2 · 선행: [Mixamo 하이브리드 설계](2026-07-22-phase2-mixamo-hybrid-design.md)

## 0. 배경 — 왜 지금인가

2a-1 리타게팅 검증 결과, 관절 데이터는 레퍼런스 리그와 1~2° 이내로 일치함이 수치로
증명됐다 (RefAttack 동일 클립 비교: 무릎 55.5°/55.4° 등). 남은 사용자 가시 결함의
지배 요인은 **부속물 정지**: Unity Humanoid가 비인간 본을 구동하지 않아 치마가
엉덩이에 고정된 통짜로 남고, 걷기·공격마다 다리가 치마를 관통한다. 파일럿 때
"후속 작업"으로 유보한 스프링 본이 실기동 판정의 선결 조건이 됐다.

부차 결함: 접지 시 발이 바닥을 수 cm 관통 (Foot IK 미사용).
비결함(대응 유보): 치비 체형에서 성인 모캡이 약하게 읽히는 현상 — 각도는 동일함이
증명됐으므로 버그가 아니라 체형 특성. 스프링 본 후 재판정에서 결정.

## 1. 확정 결정 (사용자)

| 항목 | 결정 |
|---|---|
| 구현 방식 | **A안: 자체 스프링 본 스크립트** (UniVRM 도입·Unity Cloth 기각) |
| 대상 | 꼬리 5본 체인×1, 치마 2본 체인×8, 목도리 2본 체인×2, 후드 귀 1본×2 |
| 본 목록 원천 | `exports/character/<name>_meta.json`의 `appendageBones` (단일 원천) |
| 충돌 | 허벅지·종아리 구체 콜라이더로 치마 관통 **완화** (완전 방지는 비목표) |
| 발 | Animator 상태 Foot IK 토글 시도 — 효과 없으면 정직 보고 후 제거 |

## 2. 컴포넌트 (파일당 1책임)

### SpringBoneChain.cs (런타임, Assets/Play/)
체인 하나의 물리. 각 본의 자식 방향 끝점을 관성 추적(감쇠 진자):
매 틱 끝점 목표 = 부모 애니메이션이 만드는 레스트 방향, 현재 추적점은 관성+중력으로
지연, 본 회전 = 레스트 방향→추적 방향 회전(각도 클램프). 파라미터:
`stiffness`(복원), `damping`(감쇠), `gravity`, `maxAngleDeg`(클램프), `colliders`.

**핵심 설계 결정 — 시뮬 틱을 `public void Step(float dt)`로 분리.**
- 런타임: `LateUpdate()`에서 `Step(Time.deltaTime)` (애니메이터가 본을 쓴 뒤 덮어씀)
- 캡처(에디트 모드, LateUpdate 없음): ClipCapture가 `animator.Update(DT)` 뒤
  같은 `Step(DT)`를 수동 호출 → **물리 포함 프레임을 렌더**할 수 있어
  에이전트 튜닝 루프가 성립한다. 이 심(seam)이 없으면 검증 불가.

### SpringCollider.cs (런타임, Assets/Play/)
`radius`만 갖는 구체 마커. SpringBoneChain.Step이 추적점을 구 밖으로 밀어냄.
허벅지(LeftUpperLeg/RightUpperLeg)·종아리(LeftLowerLeg/RightLowerLeg)에 부착.

### SpringBoneSetup.cs (에디터, Assets/Editor/)
meta `appendageBones`를 이름 규칙으로 체인 그룹핑해 프리팹 인스턴스에
SpringBoneChain들과 다리 SpringCollider를 부착하는 정적 유틸.
PlaySceneBuild(씬 생성 시)와 ClipCapture(캡처 시) 양쪽에서 호출 — 씬 재생성·캡처
재현성 유지. 그룹별 튜닝값은 이 파일의 데이터 테이블로 노출:

| 그룹 | stiffness | damping | gravity | maxAngle | 콜라이더 |
|---|---|---|---|---|---|
| 꼬리 (Tail*) | 낮음(출렁) | 중 | 소 | 45° | 없음 |
| 치마 (Skirt*) | 높음(단단) | 높음 | 소 | 30° | 다리 4구 |
| 목도리 (Scarf*) | 낮음 | 중 | 중 | 50° | 없음 |
| 귀 (HoodEar*) | 중 | 중 | 소 | 25° | 없음 |

(수치 초깃값은 구현 계획에서 확정, 캡처 판독 루프로 수렴 — walk_poses 패턴)

### Foot IK
PlaySceneBuild에서 이동·공격 상태 `iKOnFeet = true`. 캡처 프레임에서 발 관통
개선이 확인되지 않으면 제거하고 보고서에 기록 (침묵 유지 금지).

## 3. 검증 3층

- **기계**: playscene_report에 `springs_attached` 단언 추가 — 부착 체인 수 ==
  meta 기대치(꼬리1+치마8+목도리2+귀2=13). 캡처 JSON에 꼬리 끝 본 world 위치를
  위상별 기록(정지 상태와 달라야 함 — 물리 작동 증명). 기존 단언 불변.
- **에이전트**: ClipCapture(물리 포함)로 Walk·RefAttack 캡처 → 치마가 다리를 따라
  벌어지는지, 관통 완화됐는지, 꼬리·목도리 출렁임이 자연스러운지 프레임 판독.
  판독 통과 후에만 사용자 호출.
- **사용자**: 실기동 재판정 — **2a-1 최종 게이트**. 판정 재료가 오염되지 않도록
  스프링 튜닝 수렴 전에는 호출하지 않는다.

## 4. 하지 않는 것 (YAGNI)

Unity Cloth · UniVRM · 정점 단위 충돌 · Blender 팔로우스루 재수출(Blender 프리뷰
전용 존치) · 치비 리딩 대응(재판정 후 별도) · dummy 프로필 스프링(부속물 없음)

## 5. 완료 기준

1. `run.ps1 playscene` 초록 (springs_attached 13 포함)
2. 캡처 프레임에서: 걷기 중 치마가 다리 스윙을 따라 벌어짐, 다리-치마 관통이
   현저히 감소, 꼬리가 위상 지연을 갖고 출렁임 (에이전트 판독)
3. 발 IK 효과 확인 또는 제거 기록
4. 사용자 실기동 재판정 (긍정 시 2a-1 마감 → 브랜치 정리)
5. 기존 단언·스테이지 전부 초록 유지

## 6. 리스크

| 리스크 | 완화 |
|---|---|
| 에디트 모드 캡처에서 물리 재현 불가 | Step(dt) 심으로 해소 (설계 핵심) |
| 구체 콜라이더로 관통 완전 방지 불가 | 목표를 "완화"로 명시, 재판정에서 수용도 확인 |
| 튜닝 발산 | walk_poses 패턴(데이터 테이블 + 캡처 판독 루프), 상한 3회 후 사용자 상의 |
| 스프링이 콤보 급가속에서 폭주 | maxAngleDeg 클램프 + damping, 캡처에 RefAttack 포함 |
