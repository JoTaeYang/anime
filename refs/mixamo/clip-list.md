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
| Walking.fbx | Walking | 기본 (In Place 불필요) | 걷기 |
| Running.fbx | Running | 기본 (In Place 불필요) | 달리기 |
| **YBotWalking.fbx (필수)** | Walking | **With Skin** | 아바타 원천 + 레퍼런스 비교 |
| Roll.fbx | Stand To Roll | 기본 | 구르기 |
| Attack1.fbx | Sword And Shield Slash | 기본 | 콤보 1타 |
| Attack2.fbx | Sword And Shield Attack (또는 다른 Slash 변형) | 기본 | 콤보 2타 |
| Attack3.fbx | Stable Sword Outward Slash (또는 다른 변형) | 기본 | 콤보 3타 |
| Hit.fbx | Sword And Shield Impact | 기본 | 피격 |
| Death.fbx | Sword And Shield Death | 기본 | 사망 |
| (선택) DashSlashBase.fbx | Running Slash 계열 | 기본 | 돌진 베기 베이스 (후속 계획) |

- 검색 결과에서 미리보기로 마음에 드는 변형을 고르면 됩니다 — 표의 검색어는 출발점.
- 콤보 3타는 서로 다른 동작 3개면 됩니다 (같은 클립 3번 아님).
- **In Place 불필요** (2026-07-27 정정): 인게임 제자리 재생은 Unity에서 `applyRootMotion=false`로
  처리하므로 일반본이면 됩니다. 초기 가이드의 "In Place 체크" 지시는 오류였음.
- **YBotWalking.fbx(With Skin)는 클립 임포트의 필수 입력**: Without-Skin FBX의 자체 아바타는
  T포즈 캘리브레이션이 부실해 리타게팅이 왜곡되므로(무릎 얕아짐 등, 실측으로 규명),
  ClipImport가 모든 클립의 아바타를 이 파일의 스킨 기반 아바타로 복사한다(CopyFromOther).