import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile

PROFILE = get_profile()

open_blend(paths.blend_path("02_animated"))

scene = bpy.context.scene
assert scene.render.fps == 24, f"fps {scene.render.fps}"
assert (scene.frame_start, scene.frame_end) == (1, PROFILE.FRAME_END)

rig = bpy.data.objects["rig"]
act = rig.animation_data.action if rig.animation_data else None
assert act is not None and act.name == PROFILE.ACTION_NAME, f"action: {act and act.name}"

def torso_matrix(frame):
    scene.frame_set(frame)
    return rig.pose.bones["torso"].matrix.copy()

# 이동 단언: 사이클 중 torso가 f1 대비 최대로 벌어지는 지점을 찾아 확인한다
# (carrier의 호흡 딥은 f_end/2 부근, walk의 이중 bob 피크는 f_end/4, 3*f_end/4
# 부근이라 프로필별 특정 프레임을 고정하면 서로 어긋난다).
m1 = torso_matrix(1)
max_delta, max_f = 0.0, 1
for f in range(2, PROFILE.FRAME_END):
    d = (m1.translation - torso_matrix(f).translation).length
    if d > max_delta:
        max_delta, max_f = d, f
assert max_delta > 0.005, f"torso barely moves across cycle (max at f{max_f}): {max_delta}"

# 루프 폐쇄 프레임: carrier는 frame_end 자체에 동일 포즈를 키잉하지만, walk_build는
# 보간 탄젠트를 위해 frame_end+1에 f1 복제 키를 넣는다(walk_poses 주석 "f25 = f1 복제").
loop_frame = PROFILE.FRAME_END if PROFILE.ANIM_ACTION == "carrier" else PROFILE.FRAME_END + 1
m_end = torso_matrix(loop_frame)
assert (m1.translation - m_end.translation).length < 1e-4, f"loop not closed (f1 != f{loop_frame})"

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

print("CHECK_02 OK")
