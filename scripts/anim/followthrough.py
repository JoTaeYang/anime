"""부속물 팔로스루: 몸통 움직임의 지연·감쇠 사본을 부속물 컨트롤에 키잉.
walk_build.build() 말미에서 호출된다. 24f 루프 전제(신호 샘플을 주기적으로 순환).

신호 2종 (2026-07-20 사용자 판정 반영 — 드래그/오버랩 원칙):
- 수직: hips 세계 z 속도 → local X 회전 (스텝 충격의 상하 반동)
- 측방: 골반 요잉 각의 지연 사본 → local Z 회전 (꼬리/목도리 좌우 스웨이,
  주기가 보행 1사이클이라 수직 신호의 2배 주기 메트로놈 느낌을 상쇄)
"""
import math

import bpy

from anim.walk_poses import FRAME_END, KEY_FRAMES, FOLLOWTHROUGH, PARAMS


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


def _yaw_signal():
    """프레임별 골반 요잉 각(도) — walk_build와 동일한 사인 위상."""
    return {f: PARAMS["pelvis_yaw_deg"] * math.sin(2 * math.pi * (f - 1) / FRAME_END)
            for f in range(1, FRAME_END + 1)}


def apply(rig, scene):
    delay = FOLLOWTHROUGH["delay_frames"]
    damp = FOLLOWTHROUGH["damp"]
    sig = _hips_signal(rig, scene)
    yaw = _yaw_signal()
    pbs = rig.pose.bones
    for chain in FOLLOWTHROUGH["chains"]:
        for i, name in enumerate(chain["controls"]):
            pb = pbs[name]
            if pb.rotation_mode == 'QUATERNION':
                pb.rotation_mode = 'XYZ'
            k = chain["gain"] * damp ** (i + 1)
            ky = chain["yaw_gain"] * damp ** (i + 1)
            d = delay * (i + 1)
            for f in KEY_FRAMES + [FRAME_END + 1]:
                src = ((1 if f == FRAME_END + 1 else f) - 1 - d) % FRAME_END + 1
                ang_x = math.radians(max(-25.0, min(25.0, sig[src] * k)))
                ang_z = math.radians(max(-25.0, min(25.0, yaw[src] * ky)))
                scene.frame_set(f)
                pb.rotation_euler = (ang_x, 0.0, ang_z)
                pb.keyframe_insert(data_path="rotation_euler", frame=f)
