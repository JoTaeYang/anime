import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
import numpy as np
from lib import paths
from lib.blender_utils import open_blend
from lib.profiles import get_profile

PROFILE = get_profile()
open_blend(paths.blend_path("00_mesh"))
ob = bpy.data.objects[PROFILE.MESH_OBJECT]
n = len(ob.data.vertices)
co = np.empty(n * 3, dtype=np.float32)
ob.data.vertices.foreach_get("co", co)
co = co.reshape(n, 3)
H = float(co[:, 2].max())


def slice_at(z, band=0.015):
    return co[(co[:, 2] > z - band) & (co[:, 2] < z + band)]


def width(z):
    s = slice_at(z)
    return float(s[:, 0].max() - s[:, 0].min()) if len(s) else 0.0


# --- 팔: x 극값에서 손끝, 몸통 경계에서 어깨 ---
hand_tip_x = float(co[:, 0].max())
arm_band = co[co[:, 0] > hand_tip_x * 0.55]          # 팔 영역 (몸통 밖)
arm_z = float(np.median(arm_band[:, 2]))              # 팔 높이 (T포즈 수평)
arm_y = float(np.median(arm_band[:, 1]))
# 몸통 반폭: 팔 높이에서 몸통만 남긴 슬라이스 (|x| 작은 클러스터의 최대 |x|)
torso_sl = slice_at(arm_z)
torso_half = float(np.percentile(np.abs(torso_sl[:, 0]), 35))
shoulder_x = torso_half * 1.05
wrist_x = hand_tip_x * 0.80                            # 손 길이를 팔 20%로 가정(초안)
elbow_x = (shoulder_x + wrist_x) / 2

# --- 다리/몸통 높이: 폭 프로파일 특징점 ---
zs = np.linspace(0.02 * H, 0.98 * H, 97)
ws = np.array([width(z) for z in zs])
# 가랑이: 하반신(15..45%H)에서 폭이 최소가 되는 높이 부근의 위쪽
low_i = (zs > 0.15 * H) & (zs < 0.45 * H)
crotch_z = float(zs[low_i][np.argmin(ws[low_i])])
hips_z = crotch_z + 0.06 * H
# 무릎/발목: 가랑이-바닥 구간의 등분(초안; 루프에서 실루엣 보고 보정)
knee_z = crotch_z * 0.55
ankle_z = 0.045 * H
# 다리 x: 하반신 슬라이스의 |x| 중앙값
leg_sl = slice_at(knee_z)
leg_x = float(np.median(np.abs(leg_sl[:, 0])))
# 목/머리: 상단에서 폭 최소(목) — 팔 위 영역만
top_i = (zs > arm_z + 0.03 * H) & (zs < 0.93 * H)
neck_z = float(zs[top_i][np.argmin(ws[top_i])]) if top_i.any() else 0.85 * H
head_top = H
# 척추 분할: hips..neck을 P와 같은 비율로 4분할
spine_zs = np.linspace(hips_z, neck_z, 5)

LM = {
    "spine":       {"head": [0.0, 0.0, round(hips_z, 4)], "tail": [0.0, 0.0, round(float(spine_zs[1]), 4)]},
    "spine.001":   {"head": [0.0, 0.0, round(float(spine_zs[1]), 4)], "tail": [0.0, 0.0, round(float(spine_zs[2]), 4)]},
    "spine.002":   {"head": [0.0, 0.0, round(float(spine_zs[2]), 4)], "tail": [0.0, 0.0, round(float(spine_zs[3]), 4)]},
    "spine.003":   {"head": [0.0, 0.0, round(float(spine_zs[3]), 4)], "tail": [0.0, 0.0, round(neck_z, 4)]},
    "spine.004":   {"head": [0.0, 0.0, round(neck_z, 4)], "tail": [0.0, 0.0, round(neck_z + (head_top - neck_z) * 0.18, 4)]},
    "spine.005":   {"head": [0.0, 0.0, round(neck_z + (head_top - neck_z) * 0.18, 4)], "tail": [0.0, 0.0, round(neck_z + (head_top - neck_z) * 0.36, 4)]},
    "spine.006":   {"head": [0.0, 0.0, round(neck_z + (head_top - neck_z) * 0.36, 4)], "tail": [0.0, 0.0, round(head_top, 4)]},
    "shoulder.L":  {"head": [round(shoulder_x * 0.15, 4), round(arm_y, 4), round(arm_z + 0.02, 4)],
                    "tail": [round(shoulder_x, 4), round(arm_y, 4), round(arm_z + 0.02, 4)]},
    "upper_arm.L": {"head": [round(shoulder_x, 4), round(arm_y, 4), round(arm_z, 4)],
                    "tail": [round(elbow_x, 4), round(arm_y, 4), round(arm_z, 4)]},
    "forearm.L":   {"head": [round(elbow_x, 4), round(arm_y, 4), round(arm_z, 4)],
                    "tail": [round(wrist_x, 4), round(arm_y, 4), round(arm_z, 4)]},
    "hand.L":      {"head": [round(wrist_x, 4), round(arm_y, 4), round(arm_z, 4)],
                    "tail": [round(hand_tip_x * 0.97, 4), round(arm_y, 4), round(arm_z, 4)]},
    "thigh.L":     {"head": [round(leg_x, 4), 0.0, round(hips_z - 0.02, 4)],
                    "tail": [round(leg_x, 4), 0.0, round(knee_z, 4)]},
    "shin.L":      {"head": [round(leg_x, 4), 0.0, round(knee_z, 4)],
                    "tail": [round(leg_x, 4), 0.0, round(ankle_z, 4)]},
    "foot.L":      {"head": [round(leg_x, 4), 0.0, round(ankle_z, 4)],
                    "tail": [round(leg_x, 4), -0.09, round(0.015, 4)]},
    "toe.L":       {"head": [round(leg_x, 4), -0.09, 0.015], "tail": [round(leg_x, 4), -0.14, 0.015]},
}

out = paths.PROJECT_ROOT / "scripts" / "lib" / "profiles" / "character_landmarks.py"
with open(out, "w", encoding="utf-8") as f:
    f.write("# estimate_landmarks.py 초안 + 오버레이 루프 수동 보정. 좌표는 미터.\n")
    f.write("LANDMARKS = ")
    import pprint
    f.write(pprint.pformat(LM, width=100))
    f.write("\n")
print("LANDMARKS WRITTEN:", out)
for k, v in LM.items():
    print(" ", k, v)
