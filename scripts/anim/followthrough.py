"""부속물 팔로스루: 몸통 움직임의 지연·감쇠 사본을 부속물 컨트롤에 키잉.
walk_build.build() 말미에서 호출된다. 24f 루프 전제(신호 샘플을 주기적으로 순환)."""
import math

import bpy

from anim.walk_poses import FRAME_END, KEY_FRAMES, FOLLOWTHROUGH


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


def apply(rig, scene):
    delay = FOLLOWTHROUGH["delay_frames"]
    damp = FOLLOWTHROUGH["damp"]
    sig = _hips_signal(rig, scene)
    pbs = rig.pose.bones
    for chain in FOLLOWTHROUGH["chains"]:
        for i, name in enumerate(chain):
            pb = pbs[name]
            if pb.rotation_mode == 'QUATERNION':
                pb.rotation_mode = 'XYZ'
            k = damp ** (i + 1)
            d = delay * (i + 1)
            for f in KEY_FRAMES + [FRAME_END + 1]:
                src = ((1 if f == FRAME_END + 1 else f) - 1 - d) % FRAME_END + 1
                ang = math.radians(max(-25.0, min(25.0, sig[src] * k)))
                scene.frame_set(f)
                pb.rotation_euler = (ang, 0.0, 0.0)
                pb.keyframe_insert(data_path="rotation_euler", frame=f)
