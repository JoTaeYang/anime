import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import ensure_rigify, open_blend, save_as
from lib.profiles import get_profile

PROFILE = get_profile()
P = PROFILE.LANDMARKS

open_blend(paths.blend_path("00_mesh"))
ensure_rigify()

# 1) 메타리그 추가 + 1.7m 스케일
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.armature_human_metarig_add()
meta = bpy.context.active_object
if PROFILE.METARIG_SCALE:
    meta.scale = (PROFILE.METARIG_SCALE,) * 3
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

# 2b) 랜드마크 배치 + 부속물 체인 (character 프로필: 스케일 대신 좌표 직접 배치)
bpy.ops.object.mode_set(mode='EDIT')
eb = meta.data.edit_bones
# 손가락 리매핑용 "배치 전" 손목 스냅샷 (side별 실측). 아래 랜드마크 루프가 hand.L/R을
# 덮어쓰기 전에 캡처해둔다 — lib.proportions.P는 dummy 전용 스케일 테이블이라 character
# 메타리그(스케일 미적용)의 실제 기준과 최대 ~16% 어긋난다 (원인: Fix 1, task-10).
_old_hand = {side: (eb[f"hand.{side}"].head.copy(),
                     (eb[f"hand.{side}"].tail - eb[f"hand.{side}"].head).length)
             for side in ("L", "R")}
# 1) 랜드마크 배치 (양측 미러)
for name, ht in PROFILE.LANDMARKS.items():
    for side_name, mirror in ((name, 1.0), (name.replace(".L", ".R"), -1.0)):
        if side_name in eb:
            eb[side_name].head = (ht["head"][0] * mirror, ht["head"][1], ht["head"][2])
            eb[side_name].tail = (ht["tail"][0] * mirror, ht["tail"][1], ht["tail"][2])
# 1b) 무릎/팔꿈치 최소 굽힘 보정 (브리프 코드 블록 외 추가 — 아래 사유)
#     character_landmarks.py는 T포즈 실루엣 추정값이라 무릎·팔꿈치에 깊이(bend) 정보가 없어
#     thigh/shin, upper_arm/forearm이 완전 일직선이 된다. Rigify(5.1.2) limb_rigs.py의
#     compute_pole_angle은 이 경우 elbow_vector가 길이 0이 되어
#     "Vector.angle(other): zero length vectors have no valid angle" 로 rigify_generate 자체가
#     죽는다(실측 확인, Task 5). dummy 프로필의 proportions.P는 이미 자연스러운 굽힘이 있어
#     영향 없음(교차곱 근사-평행 검사로 게이트). 실제 좌표 수렴은 Task 6의 몫 — 여기서는 생성이
#     죽지 않을 최소한의 강제 굽힘(1.5cm)만 부여한다.
#     부호: 캐릭터는 -Y를 바라본다. 무릎은 앞(-Y)으로, 팔꿈치는 뒤(+Y)로 굽는다 — 기본 메타리그
#     (lib/proportions.py)로 교차검증: upper_arm.tail.y(0.0760) > shoulder.y(0.0229)/wrist.y(0.0423)
#     → 팔꿈치는 +Y로 튀어나옴. thigh.tail.y(-0.0246) < hip/ankle.y → 무릎은 -Y로 튀어나옴.
#     본 쓰기: use_connect인 자식 본의 head는 부모 tail과 라이브 동기화될 수 있어 두 번 따로
#     쓰면 겹쳐 써지거나(2배) 덮어써질(무효화) 위험이 있다 — proximal.tail만 쓰고 distal.head는
#     그 값을 그대로 복사해 단일 소스로 만든다.
_BEND = 0.015
for _side in ("L", "R"):
    for _prox, _dist, _sign in ((f"thigh.{_side}", f"shin.{_side}", -1.0),
                                 (f"upper_arm.{_side}", f"forearm.{_side}", +1.0)):
        if _prox in eb and _dist in eb:
            _vp = eb[_prox].tail - eb[_prox].head
            _vd = eb[_dist].tail - eb[_dist].head
            if _vp.length > 1e-6 and _vd.length > 1e-6 and _vp.normalized().cross(_vd.normalized()).length < 1e-4:
                _landmark_y = eb[_prox].tail.y
                eb[_prox].tail.y = _landmark_y + _sign * _BEND
                eb[_dist].head = eb[_prox].tail  # 단일 소스 복사 — 겹쳐쓰기/무효화 방지
                print("NUDGE", _prox, eb[_prox].tail.y, _dist, eb[_dist].head.y,
                      "landmark_y=", _landmark_y, "expected=", _landmark_y + _sign * _BEND)

# 2) 손가락: 손 본 변화에 맞춰 이동+스케일 (wrist 기준 상대 변환)
#    랜드마크 배치 직전에 캡처한 실제 old wrist(_old_hand, side별)를 기준으로 새 hand.L/R의
#    (이동, 길이비)을 손가락·팜 전체에 적용한다. _old_hand는 이미 side별로 캡처했으므로
#    mirror를 곱하지 않는다 (mirror는 new_wrist에만 적용).
from mathutils import Vector as _V
for side, mirror in (("L", 1.0), ("R", -1.0)):
    old_wrist, old_len = _old_hand[side]
    new_wrist = _V(PROFILE.LANDMARKS["hand.L"]["head"]); new_wrist.x *= mirror
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

# 3) 스케일 검증 (proportions 테이블과 일치해야 함)
if PROFILE.METARIG_SCALE:
    hips_z = meta.data.bones["spine"].head_local.z
    assert abs(hips_z - P["spine"]["head"][2]) < 0.005, f"metarig scale drift: hips at {hips_z}"
else:
    hips_z = meta.data.bones["spine"].head_local.z
    assert abs(hips_z - PROFILE.HIPS_HEIGHT) < 0.005, f"landmark hips mismatch: hips at {hips_z}"

# 4) 컨트롤 리그 생성 → "rig" 오브젝트
bpy.ops.pose.rigify_generate()
rig = bpy.context.active_object
assert rig.name == "rig", f"unexpected rig name {rig.name}"

# 5) 자동 웨이트 바인딩 (DEF 본만 use_deform=True이므로 DEF에만 붙는다)
dummy = bpy.data.objects[PROFILE.MESH_OBJECT]
bpy.ops.object.select_all(action='DESELECT')
dummy.select_set(True)
rig.select_set(True)
bpy.context.view_layer.objects.active = rig
bpy.ops.object.parent_set(type='ARMATURE_AUTO')

# 6) 웨이트 품질 보정 (character 프로필에서만 동작, dummy는 no-op)
from tools.fix_weights import apply as _fix_weights  # scripts/가 sys.path 루트
_fix_weights(dummy)

save_as(paths.blend_path("01_rigged"))
print("STAGE 01_rig OK")
