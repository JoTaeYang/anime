# Phase 0 설계 — 더미 마네킹으로 Blender→Unity 파이프라인 관통

> 작성일: 2026-07-17 · 상태: 승인됨 (사용자)
> 상위 문서: [PLAN.md](../../../PLAN.md) — 이 문서는 PLAN의 Phase 0을 구체화한 설계다.

## 1. 목적과 범위

더미 마네킹으로 **메시 생성 → Rigify 리깅 → 애니메이션 → 베이크 → FBX 익스포트 → Unity Humanoid 검증** 전 구간을 캐릭터 없이 관통한다. 목적은 두 가지다.

1. Blender→Unity의 알려진 함정(축 방향, 스케일 100배, 본 롤, 본 네이밍)을 실제 캐릭터가 오기 전에 해소한다.
2. Blender 5.1 + Rigify의 현재 API로 파이프라인 코드를 조기 검증한다.

**이것은 배관 검사이지 품질 검사가 아니다.** 웨이트 품질, 애니메이션 품질은 Phase 1/2의 일이다.

## 2. 역할 분담 (품질 판정의 경계)

- **에이전트**: 기계적 오류 검출(Unity 단언), 볼 것을 차려내기(프리뷰 렌더), 파이프라인 자동화.
- **사용자**: 품질 판정. 웨이트가 충분한지, 애니메이션이 좋은지는 최종적으로 사용자가 결정한다.
- 에이전트의 컨택트 시트는 에이전트 눈용 곁검사다. **사용자 눈용 고해상도 프리뷰(턴테이블 등)는 Phase 1에서 만든다** — 더미 박스는 고화질로 봐도 판정할 품질이 없다.

## 3. 확정된 결정 사항

| 결정 | 내용 | 근거 |
|---|---|---|
| 검증 방식 | Unity batchmode 자가 검증 (에이전트가 스스로 루프를 돎) | Blender 쪽 사전 검사만으로는 "Unity가 Humanoid로 받아주는가"에 답할 수 없음 |
| Unity 버전 | **6000.3.20f1 (Unity 6.3 LTS)** — 설치 완료, `C:\Program Files\Unity\Hub\Editor\6000.3.20f1` | 6.0 LTS는 지원 종료 임박(2026-10), 6.3 LTS는 2027-12까지 |
| 스테이지 연결 | 단계별 `.blend` 스냅샷 체인 | 스테이지 경계가 Phase 1의 캐릭터 교체 지점. MCP 인터랙티브 보정은 중간 `.blend`가 파일로 있어야 성립 |
| 본 네이밍 | 익스포트 직전 Rigify 이름 → Unity 관례 리네임 (`DEF-upper_arm.L` → `LeftUpperArm`) | FBX가 자기 완결적이어야 함. Humanoid 규격 충족 책임은 FBX를 만드는 쪽에 있음 |
| 뼈대 구성 | Rigify human 메타리그 실전 그대로 — 팔다리 2세그먼트(트위스트 본), 손가락 전부 포함 | Phase 0의 격리 목적("문제 변수를 캐릭터로 한정")은 뼈대가 실전과 같을 때만 성립 |
| 더미 키 | **1.7m** | 스케일 단언 기대치. Phase 1 캐릭터도 이 값에 맞춤 |
| 더미 재질/토폴로지 | 체커 무늬 재질 + 관절 주변 세분화 | 무지 회색 박스는 변형이 보이지 않고, 꼭짓점 없는 관절은 파손이 일어날 수 없음 |
| idle | 절차적 생성 (호흡 루프, 30~60프레임) | 클립이 파이프라인을 통과하는지 보는 운반체. 예술적 가치 없음 |

## 4. 파이프라인 — 5개 스테이지

각 스테이지의 계약: **입력 `.blend` 하나, 출력 `.blend` 하나, 단독 실행 가능.**
실행: `blender --background --python scripts/stages/<stage>.py` (래퍼 `scripts/run.ps1`이 감쌈).

| 스테이지 | 하는 일 | 출력 |
|---|---|---|
| `00_mesh` | 더미 마네킹 절차 생성 (1.7m, 체커 재질, 관절 세분화) | `build/00_mesh.blend` |
| `01_rig` | Rigify 메타리그 배치 → 컨트롤 리그 생성 → 자동 웨이트 | `build/01_rigged.blend` |
| `02_anim` | 절차적 idle 액션 생성 | `build/02_animated.blend` |
| `03_bake` | 컨트롤 리그 → deform 본만 베이크 + Unity 관례 리네임 | `build/03_baked.blend` |
| `04_export` | FBX 익스포트 (스케일·축 설정 스크립트에 고정) | `exports/dummy.fbx` |

- `03_bake`와 `04_export`를 나눈 이유: 리네임된 순수 deform 리그를 FBX로 나가기 **전에** 열어볼 수 있어야, FBX 문제의 원인이 베이크인지 익스포트 설정인지 가려진다.
- **Phase 1 확장 지점은 `00_mesh` 하나**: 실제 캐릭터가 오면 이 스테이지만 "사용자 `.blend` 반입 + 사전 점검"으로 교체된다.
- 본 매핑 테이블(Rigify 이름 → Unity 이름, 손가락·트위스트 포함)은 `scripts/lib/`에 명시적 데이터로 둔다. 트위스트 본은 Unity가 매핑하지 않는 여분 본으로 계층에 남는다 — 이것이 문제를 일으키지 않는지 확인하는 것도 Phase 0의 검증 항목이다.

## 5. 검증

### 5a. 주력 — Unity batchmode 단언 (기계 판정)

최소 Unity 프로젝트 `unity/AvatarCheck/`의 에디터 스크립트가 batchmode로 FBX를 임포트하고 아래를 단언한다. 결과는 JSON 리포트 + 종료 코드.

1. `Avatar.isValid` && `Avatar.isHuman`. 임포트 에러·경고 0건.
2. Humanoid 필수 본 15개 매핑 완비 + 각 본이 **의도한** 본에 매핑됐는지 이름 대조. (이 항목은 이름 매핑의 완비성·일관성을 본다. 실제 좌우 정합성은 `faces_plus_z` — Unity 왼손좌표계 기준 `leftHand.x < rightHand.x` — 와 비대칭 마커 기반 `laterality` 단언이 검증한다. **2026-07-18 확정**: 한때 도입했던 L/R 이름 스왑은 FBX 미러 보상이 아니라 좌표계 부호 오류의 보상이었음이 비대칭 마커 실험으로 판명되어 폐기됨. FBX 변환은 카이랄리티를 보존하며 미러는 없다. 증거: `.superpowers/sdd/lr-swap-investigation.md`)
3. 힙 본 월드 높이 ≈ 0.9m ±10% (스케일 100배 함정 검출).
4. 캐릭터가 +Z를 향하고 루트 회전 ≈ 항등 (축 함정 검출).
5. idle 클립 존재, 길이 > 0, t=0과 t=중간의 본 월드 좌표가 실제로 다름 (키 소실 검출).

실행: `Unity.exe -batchmode -nographics -quit -projectPath unity/AvatarCheck -executeMethod <검증 메서드> -logFile <log>`.

### 5b. 곁검사 — 컨택트 시트 (에이전트 눈)

`scripts/preview/contact_sheet.py`가 EEVEE로 렌더해 **PNG 한 장**(9칸 3×3 격자, 타일 512px → 긴 변 1536px)으로 합성한다.

- 레스트 포즈 4각도 (정면·측면·3/4·후면) — 4칸: 스케일·축·기본 스키닝.
- 극단 포즈 3개 (팔 위로, 쪼그리기, 상체 비틀기) — 3칸: 리그-메시 결합 확인.
- idle 2칸 (루프 0%·50%) — 곁눈질. 움직임 판정은 5a-5번 단언이 담당.

**이 시트의 판정 범위는 "리그가 붙긴 했나" 수준이다.** 박스 더미에서 웨이트 품질을 논하지 않는다. 극단 포즈 렌더 코드는 Phase 1의 완료 기준("극단 포즈에서 메시 파손 없음") 검증에 그대로 재사용된다.

## 6. 저장소 구조

```
anime/
├── PLAN.md
├── docs/superpowers/specs/   # 설계 문서 (이 파일)
├── scripts/
│   ├── run.ps1               # 래퍼: 스테이지 하나 또는 all
│   ├── stages/               # 00_mesh.py … 04_export.py
│   ├── preview/              # contact_sheet.py
│   └── lib/                  # 본 매핑 테이블, blend I/O 헬퍼
├── assets/                   # 소스 .blend (Phase 1부터 사용)
├── refs/                     # 키 포즈 레퍼런스 (Phase 2부터 사용)
├── build/                    # 중간 .blend (gitignore)
├── exports/                  # FBX 산출물 (당분간 gitignore, Phase 3에서 재검토)
├── previews/                 # 컨택트 시트 (gitignore)
└── unity/AvatarCheck/        # 검증 전용 최소 Unity 프로젝트 (Library/ 등 gitignore)
```

PLAN 대비 `build/`, `unity/`, `scripts/lib/`, `docs/`가 추가됐다.

## 7. 완료 기준

1. `scripts/run.ps1 all`이 종료 코드 0으로 완료된다 (00→04 + Unity 검증까지).
2. Unity 검증 리포트의 단언(5a)이 전부 통과한다.
3. 에이전트가 컨택트 시트를 읽고 사람 형상·정상 포즈로 보임을 확인한다.
4. 사용자가 Unity 에디터에서 최종 1회 눈으로 확인한다 (PLAN의 승인 게이트).

## 8. 하지 않는 것 (YAGNI)

Mixamo 리타게팅, 실제 캐릭터 처리, NLA 정리, 사용자 눈용 고해상도 프리뷰, 웨이트 품질 보정, MCP 인터랙티브 조정. 전부 Phase 1 이후로 미룬다.

## 9. 환경 (검증 완료)

- Blender 5.1 — `C:\Program Files\Blender Foundation\Blender 5.1\blender.exe`. Rigify·FBX 익스포터 번들 확인.
- Unity 6000.3.20f1 (6.3 LTS) — `C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe`. 설치·라이선스 활성화 완료.
- (참고) 6000.5.4f1도 Hub에 설치되어 있으나 파이프라인은 6000.3.20f1을 사용한다.
