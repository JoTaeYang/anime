# Mixamo 다운로드 목록 (Phase 2a-1)

mixamo.com 로그인(Adobe 계정) → 캐릭터는 기본 Y Bot 그대로 두고 애니메이션만 검색.

## 공통 다운로드 설정
- Format: **FBX Binary (.fbx)**
- Skin: **Without Skin**
- Frames per Second: **30**
- Keyframe Reduction: **none**

## 클립 (저장 파일명을 정확히 맞춰야 함 → assets/mocap/에 배치)

| 저장 파일명 | Mixamo 검색어 | 다운로드 옵션 | 용도 |
|---|---|---|---|
| Idle.fbx | Idle (또는 Sword And Shield Idle) | 기본 | 대기 |
| Walking.fbx | Walking | **In Place 체크** | 걷기 |
| Running.fbx | Running | **In Place 체크** | 달리기 |
| Roll.fbx | Stand To Roll | 기본 | 구르기 |
| Attack1.fbx | Sword And Shield Slash | 기본 | 콤보 1타 |
| Attack2.fbx | Sword And Shield Attack (또는 다른 Slash 변형) | 기본 | 콤보 2타 |
| Attack3.fbx | Stable Sword Outward Slash (또는 다른 변형) | 기본 | 콤보 3타 |
| Hit.fbx | Sword And Shield Impact | 기본 | 피격 |
| Death.fbx | Sword And Shield Death | 기본 | 사망 |
| (선택) DashSlashBase.fbx | Running Slash 계열 | 기본 | 돌진 베기 베이스 (후속 계획) |

- 검색 결과에서 미리보기로 마음에 드는 변형을 고르면 됩니다 — 표의 검색어는 출발점.
- 콤보 3타는 서로 다른 동작 3개면 됩니다 (같은 클립 3번 아님).
- In Place 체크가 없는 클립(Walking/Running에만 있음)은 그대로 받으면 됩니다.