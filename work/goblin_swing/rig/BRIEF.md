# Goblin Rig BRIEF (인수인계 요약)

새 에이전트는 이 파일과 task 프롬프트에 지정된 문서 절만 읽는다. main이 Phase 교체 때 갱신한다.
갱신일: 2026-09-25 (G1~G8 PASS, G6.10 사용자 확인 대기. 다음: G9 Unity)

## 문서
- 스펙(합의문서): `work/goblin_swing/d-23-goblin-rig-design.md`. Gate 정의는 §7이고, 절 번호로 참조한다.
- 계획서: `work/goblin_swing/d-23-goblin-rig-plan.md`. 공용 계약, task 블록, 실행 기록이 있다.

## 실행
- 모든 명령은 저장소 루트 `C:\Users\whxod\orca\anime`에서 실행한다.
- Blender: `powershell -NoProfile -File work/goblin_swing/rig/scripts/bl.ps1 -Script <py> [-Blend <rig 기준 상대경로>] [-- <args>]`
  - factory-startup, headless.
  - 없는 blend를 넘기면 Blender가 exit 1로 먼저 끝난다(스크립트는 실행되지 않음).
- Unity: `powershell -NoProfile -File work/goblin_swing/rig/scripts/unity_goblin.ps1 -Method <Class.Method>`
- 스크립트 첫머리: `sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import goblib`
- 공용 라이브러리 `rig/scripts/goblib.py`:
  - 경로 상수(RIG/DATA/INSPECT/WORK/EXPORT), Evidence, ortho_render, silhouette_mask, contour_max_dev_px, quat_angle_deg, bone_world, run_main.
  - `_view_basis(view)`는 렌더 뷰 방향 계산에 쓸 수 있다.

## 좌표·크기 (실측)
- 정면 −Y, 위 +Z, 캐릭터 왼쪽(`_l`) = +X, 지면 z=0.
- 키 1.3878 m. 몸통 알 중심 (0,0). 원본 FBX를 이동만 해서 쓰고 스케일은 1이다.
- 원본: `rig/gob_r00_source.blend`에 SRC_hi(91k verts, 한 덩어리)와 SRC_club_hi(몽둥이 2조각)가 있다.

## 피벗 (data/pivots.json, 좌우 비대칭 있음 — 미러 금지)
| 피벗 | 값 |
|---|---|
| 팔 튜브 | 반경 50.5 mm, z ≈ 0.92 |
| shoulder_root | x +0.151 / −0.145 |
| shoulder (접합부) | x +0.302 / −0.290 |
| elbow | x +0.483 / −0.471 |
| wrist | x +0.664 / −0.652 |
| 다리 튜브 | 반경 57 mm |
| hip | z 0.274 |
| knee | z 0.199 |
| ankle | z 0.124 (hip→ankle 150 mm) |
| pelvis | z 0.304 |
| spine_01 (= 벨트 상단) | z 0.5777 |
| spine_02 | z 0.7659 |
| head 이음매 | z 0.954 (어깨보다 약 3 cm 위) |
| 벨트 | z 0.430~0.578 |

- grip_r: co (−0.758, 0.043, 0.903), axis (−0.076, −0.997, 0.026). axis는 몽둥이 머리 쪽, 즉 −Y를 가리킨다.
- (폐기) 원본의 손바닥 구 반경 100.8 mm는 참고값으로만 남긴다. 현재 손은 공 주먹이다. 아래 'G2 변경: 공 주먹' 참조.
- 몽둥이 빈 구간: grip 기준 s −108 ~ +160 mm. 자루 반경은 뒤 36.6 mm, 앞 44.0 mm.

## 확정 결정 (사용자)
- 리토폴을 먼저 하고, DEF 위치는 최종 리토폴 링 중심으로 정한다.
- shoulder 본: head = shoulder_root, tail = 접합부. spine_02 = spine_01과 head의 z 중간.
- preferred bend: elbow는 +Y로, knee는 −Y로 4 mm. 본에만 적용하고 메시는 그대로다.
- (폐기) 오른손 자루 토막 캡은 공 주먹으로 대체되었다.
- 몽둥이는 16각 회전체로 재구성한다. 원점 = grip_r, 로컬 +Y = axis.
- 얼굴·손가락 리깅은 범위 밖이다.

## 데이터 계약 (계획서 "리토폴 데이터 계약"이 원본)
- `GOB_mesh`(단일)의 face int 속성 `part_id` = `data/parts.json`: {body 0, head 1, hand_l 2, hand_r 3, shoe_l 4, shoe_r 5, belt 6}. `GOB_club`은 별도 오브젝트다.
- `data/retopo_loops.json`의 링 이름: `shoulder_x_k`, `hip_x_k`(k=0이 몸통 쪽), `elbow_x_0..2`·`knee_x_0..2`(k=1이 center), `wrist_x_0..1`, `ankle_x_0..1`, `wrist_x_end`, `ankle_x_end`, `spine_01_0`, `spine_02_0`, `belt_top_0`, `belt_bot_0`.
- 설계된 overlap 쌍: body와 {hand_l, hand_r, shoe_l, shoe_r, head, belt}.

## 현재 산출물 (2026-09-25: G1·G2·G3 PASS, Retopo Final)
- **최종 리토폴 `rig/gob_r01_retopo.blend`:**
  - `GOB_mesh`: 8,198 tris. body 4,446(T16b 팔꿈치 jmid 2·무릎 jmid 1 포함) + 강체 6파트 3,752. face INT `part_id`. 커스텀 노멀, UV 적용. transform identity.
  - `GOB_club`: 992 tris. **오브젝트 transform 유지**(원점 = grip_r, 로컬 +Y = axis). 메시 데이터는 로컬 프레임이다.
  - 파트는 서로 정점을 공유하지 않는 별도 셸이다. body는 손·신발·머리·벨트 셸 안으로 들어가 숨는다(설계된 overlap).
- `data/parts.json`, `data/retopo_loops.json`(링 40개, `GOB_mesh` 인덱스 기준. body 정점이 앞쪽 0..2080).
- 링 변경점:
  - `spine_01_0`(z 0.5777)과 `belt_top_0`(z 0.57175)는 이제 **서로 다른 행**이다.
  - `belt_bot_0`은 z 0.4363이다.
  - shoulder_x_1 링은 16/17 정점이고, 나머지 관절 링은 12 정점이다.
- 손목·발목: 튜브는 wrist_0 → end까지 튜브 반경으로 곧다. `_1` 링은 원본 표면 안쪽 ≥ 0.5 mm이고, 벌어진 flare는 손·신발 셸이 덮는다.
- 스크립트:
  - 제작: `s00_source_prep`, `s01_pivots`, `s02a_body`, `s02b_limbs`, `s02c_rigid`, `s02d_finish`
  - 검사: `check_g1_source`, `check_g2_static`, `check_g2_shape`

## 사용자 예외·결정 (G2)
- 손목·발목 이음매 띠(피벗 기준 축방향 ±40 mm, 튜브 반경 + 20 mm 이내)의 원본 편차는 현 상태를 수용한다(띠 안 최대 약 7 mm).
- (폐기) 빈 주먹 구멍은 공 주먹으로 대체되었다.
- 머리 눈 링의 다각형 품질은 수용한다.

## 알려진 함정
- 원본 몽둥이 절단 캡은 SRC_hi와 SRC_club_hi에 같은 위치로 겹쳐 있다. 형상 비교 때는 interface로 제외한다(G2.11).
- 원본 손목·발목 단면(56~59 / 68 mm)이 튜브(50.5 / 57 mm)보다 크다. 강체 셸을 그대로 자르면 납작한 고리 캡이 겉으로 드러난다. 절단부를 튜브 반경 − 1.5 mm로 줄여 숨긴다.
- 벨트 밑 body 표면은 원본 벨트 표면보다 6~16 mm 안쪽이다. 벨트 셸 안쪽면은 body보다 1 mm 이상 안쪽이어야 한다.
- `spine_01_0`과 `belt_top_0`은 같은 정점 루프다(같은 높이).
- 추적 중인 위험: 어깨 뒤·위 원본 편차가 최대 약 7.8 mm(G2.11), 어깨 링 구역 25 mm(G3), 손목 `_end`가 wrist_1에서 약 10 mm(G3).
- 면적 샘플링 기반 편차 검사는 가는 바늘형 돌출을 놓칠 수 있다. 필요하면 정점 단위 검사를 함께 하라.
- 와이어 렌더(goblib ortho_render wire)의 pole 마커 곡선이 긴 선처럼 보일 수 있다. 메시 결함이 아니다.
- body 상단 돔(z ≈ 0.99)은 머리 속에 숨은 막이라 원본과 최대 약 168 mm 떨어져 있다(정상).
- Blender 5.1: 입력 blend를 덮어쓰지 말고 `save_as_mainfile(copy=True)`로 새 파일에 저장한다. `.blend1` 방지를 위해 `save_version = 0`.

## G3 1차 결과와 결정 (2026-09-25)
- 임시 리그 `rig/gob_r01t_temprig.blend`(s03_temprig.py): 24본, 롤 규약 OK, body는 heat 자동 웨이트, 강체는 100%. knee bend는 4 mm로 만들어졌다(스펙은 이후 1.5 mm로 개정, 테스트 전용이라 유지).
- G3 1차 FAIL 내용:
  - 어깨 들기·앞 90°에서 몸통-팔 사이가 얇은 막처럼 구겨지고 자기교차 43~46쌍. 어깨 링 구역 25 mm가 좁고 위쪽은 머리다.
  - 무릎 90°에서 단면 비 0.26(다리 150 mm, 3루프). 팔꿈치 90°는 0.49.
  - rest 자세에서도 body 자기교차 좌우 6쌍(어깨 위 돔 가장자리, z 0.97~0.98).
  - 발목 30°에서 튜브 끝 노출 −4.3 mm(웨이트 문제라 G5로 이관).
- 사용자 결정:
  - 토폴로지를 보강한다(어깨 구역 확대, 팔꿈치·무릎 루프 추가).
  - G3 기준을 분리한다: 30·60° 단면 ≥ 0.8, 30·60° 자기교차 0, rest 자기교차 0. 90°는 육안 판정. 노출은 G5.
  - GOB_mesh 삼각형 상한은 8,500이다.
- 관절 링 이름 계약(retopo_loops.json의 elbow/knee 3개, center k=1)은 **유지한다**. 추가 루프는 이름 없는 루프로 넣는다.

## G3 2차 진행 (2026-09-25)
- T16a(어깨 구역 확대, s02a `RING_K0_GAP`) 적용:
  - 어깨 30/60° 단면 비 0.84~0.97, 자기교차 0.
  - rest 자기교차 0.
  - body tris 3,078(limbs 전)로 변화 없음.
- T16c(골반 토폴로지) 결과: **변경 없음**(T16a 상태 그대로 복귀).
  - k0 간격 확대 → 악화. 중간 링 추가 → 변화 없음.
  - 원인: heat 웨이트가 thigh 영향을 몸통 k0까지 0.5~0.75로 퍼뜨림 → **hip_flex_60(좌 0.74 / 우 0.66)은 G5 웨이트로 이관.**
- 현재 60° 단면 비(T16a 기준):
  - elbow 0.78 / 0.77
  - knee 0.64 / 0.72
  - → T16b(이름 없는 루프 추가)로 보강했다.
- T16b 결과(적용):
  - s02b `JOINT_MID_LOOPS = {"elbow": 2, "knee": 1}`(kind "jmid", 이름 없음). s02c `GOB_MESH_MAX = 8450`.
  - GOB_mesh 8,198 tris(body 4,446).
  - 60° 단면 비: elbow 0.785 / 0.786, knee 0.68 / 0.74.
  - 최저점(|s| ≈ 25~35 mm)은 루프 수·서포트 간격과 무관했다. LBS와 heat 웨이트 문제로 본다.
  - 변형별 부작용: knee jmid 2개나 support 0.4에서는 60° 자기교차가 생긴다.
- 손목 twist 약 0.80은 임시 리그 twist 본에 제약이 없어서다. G5/G6에서 다시 본다.

## G4 기준점 (main 실측, 2026-09-25)
- 링 중심 = `retopo_loops.json` 해당 링 정점의 `GOB_mesh` 좌표 평균.
- elbow_x_1, wrist_x_1, knee_x_1, ankle_x_1 링 중심은 피벗과 ≤ 0.9 mm라 이 링 중심을 쓴다.
- 어깨·골반은 피벗을 쓴다. `_0`/`_1` 링은 몸통 쪽 구역이라 피벗과 22~46 mm 떨어져 있다.
- 임시 리그 `s03_temprig.py`의 본 배치·롤(roll_from_x, chain_x) 코드는 검증된 참고 구현이다. 단 knee BEND 4 mm는 구버전이다.

## G4 결과 (PASS, 2026-09-25)
- `rig/gob_r02_skeleton.blend`의 `GOB_rig`(컬렉션 RIG, identity)는 24본이다. 메시·몽둥이는 부모·웨이트가 없다.
- `data/canonical_skeleton.json`은 TREE 순서로 name, parent, use_connect, head, tail, roll(rad), use_deform, rest_matrix를 담는다.
- 생성 스크립트는 `s04_skeleton.py`, 검사는 `check_g4_skeleton.py`다.
- 본 규격:
  - hand 본은 손목에서 lowerarm 방향으로 hand 파트 최대 투영 길이만큼 뻗는다. 공 주먹으로 바꾼 뒤 약 176 mm이고, 값은 재생성된 canonical을 따른다.
  - foot 본은 ankle 높이에서 수평으로 foot_tip을 향한다. 로컬 X가 다리 체인과 약 16° 차이 난다.
  - pelvis 본은 z 0.304 → 0.5777이다. 벨트(z 0.43~0.578)는 pelvis 본 구간 안에 있다.
- 롤 규약:
  - 팔 체인 X(shoulder·upperarm·twist·lowerarm·twist·hand 공통): lowerarm을 +X로 돌리면 손이 −Y로 간다.
  - 다리 체인 X(thigh·calf·foot 공통): calf를 +X로 돌리면 발이 +Y로 간다.

## G5 결과 (PASS, 2026-09-25)
- 산출물: `rig/gob_r03_skinned.blend`
  - GOB_mesh에는 Armature modifier(GOB_rig)만 있고 object 부모는 없다.
  - GOB_club은 weapon_socket_r의 본 부모다.
- `data/skin_decisions.json`:
  - 벨트 본 = pelvis.
  - twist는 Transformation `SWING_TWIST_Y`, LOCAL/LOCAL로 둔다.
    - upperarm_twist ← upperarm: sign −1, ratio 0.5 (counter-twist)
    - lowerarm_twist ← hand: sign +1, ratio 0.5
  - **twist 본은 제약이 구동하므로 CTRL로 구동하지 않는다.**
- `data/rom_poses.json`의 `_doc`에 부호 규약이 있다(실측):
  - 팔 X+ = 앞으로. 팔 들기는 upperarm_l Z+, upperarm_r Z−.
  - thigh X− = 골반 굴곡(다리 앞으로 들기). calf X+ = 무릎 굽힘.
  - 외전(다리 벌리기)은 좌 Z−, 우 Z+. foot X− = 발끝 들기.
  - spine/head X+ = 앞으로 숙이기. Z+ = 캐릭터 오른쪽으로 기울이기.
- 스키닝 한계(LBS): 극단 각도에서 볼륨이 빠지고 접힌다(어깨 130~170°, 팔꿈치 140°, 무릎 130°, 손목 꺾기 60°). 사용자가 수용했다.

## G6 결과 (G6.1~G6.9, G6.11 PASS; G6.10 [U] 대기, 2026-09-25)
- 산출물:
  - `rig/gob_r04_ctrl.blend`: GOB_rig 83본(DEF 24 + CTRL 30 + MCH 29), 컬렉션 CTRL(보임) / MCH / DEF / WGT(숨김).
  - 매니페스트: `data/ctrl_manifest.json`(controls·spaces·ikfk·clamps). blend 텍스트 `goblin_manifest.json`에 같은 사본이 들어 있다.
  - 체인: s06a → s06b → s06c → s06d → s06e. 입력은 gob_r03_skinned, 중간 결과는 work/r04a~d.
- DEF 구동:
  - root·pelvis·spine·head·shoulder·weapon_socket_r: Copy Transforms(WORLD/WORLD) ← CTRL.
  - upperarm·lowerarm·hand·thigh·calf·foot: `fk_copy`(FK CTRL) 뒤에 `ik_copy`(MCH IK)를 두고, `ik_copy` influence = 드라이버 `*_ik_fk`로 섞는다.
  - twist 본은 G5의 Transformation 제약을 그대로 쓴다.
- PROPS(본 `PROPS`의 커스텀 속성):
  - IK/FK: `arm_ik_fk_l/r` 기본 0(FK), `leg_ik_fk_l/r` 기본 1(IK).
  - 공간(int): `head_space`, `hand_ik_space_l/r`, `weapon_space`, `knee_pole_space_l/r`.
  - 발: `foot_roll/bank`, `heel/toe_twist_l/r`. 범위는 clamps를 따른다.
- 연산자(`rig/scripts/addon/goblin_rig_ui.py`와 blend 텍스트 모듈이 같은 코드):
  - `goblin.snap_ikfk(chain, direction='TO_FK'|'TO_IK')`
  - `goblin.switch_space(prop, value)`: 월드 행렬을 유지하며 전환한다. requires를 위반하면 CANCELLED.
  - `goblin.reset_rig()`
  - `goblin.ready_pose()`: COG −15 mm, 무릎 51°, 팔꿈치 17°, 발 고정.
  - 세 연산자 모두 오토키가 켜져 있으면 키를 넣는다.
- 무기:
  - CTRL_weapon의 공간: hand_r(DEF hand_r) / hand_l(CTRL_hand_fk_l, 순환 방지용) / torso / world(MCH_world).
  - hand_l 공간은 왼팔이 FK일 때만 허용된다.
- 머리 world 공간은 회전만 월드에 고정하고, 위치는 chest를 따라간다.
- **함정:**
  - Python으로 PROPS를 쓴 뒤에는 `arm.update_tag()`와 `view_layer.update()`를 호출해야 드라이버가 재평가된다.
  - 헤드리스에서 연산자를 쓰려면 blend 텍스트 `goblin_rig_ui.py`를 모듈로 실행해 등록한다(`Text.as_module()` 또는 exec).
  - 정수 공간 속성을 키로 넣을 때는 CONSTANT 보간이어야 전환 프레임 사이가 섞이지 않는다.
- 한계(수용함):
  - rest가 거의 곧아서 IK가 민감하다(무릎 약 35°/mm). 작업은 ready_pose에서 시작하도록 안내한다.
  - foot_roll 범위: 뒤꿈치 쪽 −24/−27°, 발끝 쪽 +26/+29°.

## G6 추가 변경 (T24c, 사용자 결정)
- CTRL_pelvis의 피벗은 **허리**(DEF pelvis tail, z 0.5778)다. 본 방향은 월드 −Z.
- CTRL_pelvis의 자식 MCH_pelvis(rest = DEF pelvis)를 DEF pelvis가 Copy Transforms로 따라간다.
- 축: rot X+는 척추 숙이기와 같은 방향(엉덩이가 뒤로 감), rot Y/Z는 척추 컨트롤과 반대, loc Y는 아래.

## G7 결과 (PASS, 2026-09-25)
- 작업 클립은 `rig/gob_r05_rigtest.blend`의 action `rigtest`(583프레임, 24fps, 12구간)다.
  - 키는 CTRL과 PROPS에만 있다. Blender 5.1 slot은 'OBGOB_rig'.
  - 구간 정의는 `data/rigtest_manifest.json`에 있다(segments, covers, foot_lock, events).
- export 스테이지는 `rig/scripts/s07b_export_rig.py`의 `build_export(src_blend, action_name|None, out_blend)`로 만든다. CLI는 `--action rigtest --out ...` 또는 `--rest --out ...`.
  - 산출물: `rig/export/stage_rigtest.blend`, `stage_rest.blend`.
  - 내용: `GOB_export`(24본, canonical에서만 생성, **모든 본 use_connect False**, 제약 0, 쿼터니언, 매 프레임 loc/rot/scale 키, slot 'OBGOB_export'), `goblin_mesh`(Armature → GOB_export, 커스텀 노멀 유지), `goblin_club`(weapon_socket_r 본 부모).
  - 액션 이름은 `rigtest` 또는 `rest`다.
- 베이크 오차: 583프레임 × 24본에서 위치 ≤ 0.0006 mm, 회전 ≤ 0.107°.

## G8 결과 (PASS, 2026-09-25)
- 출력 파일(`rig/scripts/s08_export_fbx.py`, `-Blend gob_r05_rigtest.blend`로 실행):
  - `rig/export/goblin.fbx`: 아마추어 오브젝트 `goblin` + `goblin_mesh`, 애니메이션 없음.
  - `goblin@rigtest.fbx`: AnimStack `rigtest`, 583프레임(Blender 프레임 1..583), 24fps.
  - `goblin_club.fbx`: 메시가 **weapon_socket_r rest 프레임** 기준이다. 오브젝트는 identity이고, 소켓 아래 local 0에 두면 제자리에 붙는다.
- `rig/data/export_preset.json`(후보 A):
  - Forward −Z, Up Y, bake_space_transform False, FBX_SCALE_ALL, apply_unit_scale True, leaf off.
  - FBX 헤더 UnitScaleFactor 100. 루트 노드마다 lcl_rotation (−90, 0, 0)이 붙는다.
  - `axis_mapping.blender_to_unity` = [[-1,0,0],[0,0,1],[0,-1,0]]. 즉 unity.x = −blender.x, unity.y = blender.z, unity.z = −blender.y. 실제 Unity 동작은 G9.3에서 확인한다.
- **함정:**
  - Blender FBX 임포터는 부모 tail이 자식 head에 닿으면 자식을 connected로 만든다(pelvis, spine_02, upperarm_l/r, foot_l). 그러면 위치 키가 무시되므로, Blender에서 재임포트해 비교할 때는 모든 본을 unconnect한다.
  - 재임포트하면 키가 1프레임 밀린다(anim_offset 1).
  - FBX 헤더에 생성 시각이 들어가서, export를 다시 하면 sha256이 바뀐다.

## G2 변경: 공 주먹 (2026-09-25 사용자 결정, T60/T61)
- 양손은 닫힌 quad sphere다. 반경 97.86 mm로, 레퍼런스 '주먹 지름 / 머리 폭' 0.346 × 머리 폭 565.5 mm / 2로 정했다. 손마다 432 tris.
- 중심은 wrist 축에서 d = 77.86 mm 떨어진 곳이다.
  - hand_l: (0.7418, 0.0407, 0.9163)
  - hand_r: (−0.7298, 0.0414, 0.9160)
  - wrist `_1` 링은 공 안쪽 약 4.4 mm, `_end` 링은 약 12 mm에 있다.
- 오른손: 몽둥이 자루가 공을 관통한다(설계된 overlap). grip_r은 공 중심에서 31 mm 떨어져 있다. 자루 축이 공을 지나는 현 길이는 지름의 0.94배다.
- GOB_mesh는 8,122 tris다.
- G2.11/G2.12는 손 영역을 설계 변경으로 보고 제외한다. G2.16은 그립 구간 자루가 공 안쪽 ≥ 1 mm에 있는지, 그리고 현/지름 ≥ 0.8인지로 판정한다.

## 애니메이션 클립 작업 (idle / hit 경험, 2026-09-25)
- 도구 (playbook §4):
  - `s12_clip_anim.py --clip <name>`: gob_r04_ctrl.blend + data/clips/<name>.json → gob_r08_<name>.blend, inspect/<name>/*. 반복 실행은 `--no-render --no-save`로 한다.
  - `check_anim_clip.py --action <name> --json rig/data/clips/<name>.json --gate 1|2|both`
  - 작성 예시: data/clips/idle.json, hit.json.
- 클립 JSON에서 작성자가 쓰는 부분: poses(ctrl_overrides / props_overrides, 기준은 goblin.reset_rig + ready_pose), breakdowns(copy / blend [fa, fb, t] + overrides), contact_ranges, sections, targets, gate_id 매핑(report 플래그), reference, match_first_frame(attack_swing f1). f1은 idle.json poses[frame 1]을 그대로 복사한다.
- 실측 제약 (hit T90/T91):
  - 발을 딛고 있을 때 COG는 −0.036 m까지만 내릴 수 있다. pelvis 8°에서 −0.040이면 다리 접힘이 7쌍 보이고 hip_flex_l이 60.7이 된다.
  - 팔을 크게 다른 포즈 사이에서 blend하면 겨드랑이 접힘이 보인다. 이때는 팔 값을 한쪽 포즈에 고정한다.
  - shoulder_raise는 정확히 25.0이면 C2 경계에 걸린다(부동소수). 24.5로 쓴다.
  - 오른 위팔 raise는 100° 이하로 둔다. 105° 이상이면 몽둥이가 머리를 관통한다.
  - 오른 어깨를 들면 DEF 손목 bend가 40을 넘기 쉽다. 오른손 FK로 보정한다.
- 인접 프레임 머리 회전 18°/frame(hit f6–7)은 사용자가 승인했다.

## 입 셰이프 키와 새 머리 (HR2, 2026-09-26)
- GOB_mesh: 고른 quad 머리. 셰이프 키는 Basis와 mouth_open(relative)이다. 재질은 4개다(GOB_skin, GOB_mouth_inner, GOB_teeth, GOB_tongue). 9,168 tris(G2.8 ≤ 9,500).
- PROPS `mouth_open` 범위는 [0,1], 기본값 0이다. 드라이버로 셰이프 키에 연결된다. ready_pose와 reset_rig는 이 값을 0으로 되돌린다.
- s12에서는 clip JSON의 `interpolation.props_overrides {"mouth_open": "BEZIER"}`로 입만 부드럽게 보간한다(다른 PROPS는 CONSTANT). 렌더는 재질 색이다(입 안이 검게 보인다).
- mouth_open = 1일 때:
  - 머리가 X 0.88, Z 1.15로 늘어난다.
  - 눈이 약 +121 mm 올라간다. head 본은 움직이지 않으므로 M2 눈 기준 진폭은 셰이프 키 효과를 포함한다.
  - 몽둥이 간격은 늘어난 머리 기준으로 측정된다(evaluated mesh).
- export stage는 Key 액션 `<action>_shapekeys`를 쓴다. Unity blendShape.mouth_open은 0–100이다.
- Unity 렌더에는 submesh 수만큼 재질이 필요하다(GoblinRigCheck에서 수정함).
