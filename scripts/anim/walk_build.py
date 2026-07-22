"""character walk 액션 빌더. 02_anim이 디스패치한다.
IK 다리(foot_ik 위치 키) + FK 팔(IK_FK=1 전환 후 스윙) + 몸통 키."""
import math

import bpy
from mathutils import Matrix

from anim.walk_poses import FPS, FRAME_END, KEY_FRAMES, PARAMS, OVERRIDES
from anim.followthrough import apply as _followthrough


def _rotate_world(rig, pb, axis, deg):
    bpy.context.view_layer.update()
    head = pb.matrix.to_translation()
    r = Matrix.Translation(head) @ Matrix.Rotation(math.radians(deg), 4, axis) @ Matrix.Translation(-head)
    pb.matrix = r @ pb.matrix
    bpy.context.view_layer.update()


def _key_all(pb, frame):
    pb.keyframe_insert(data_path="location", frame=frame)
    if pb.rotation_mode == 'QUATERNION':
        pb.keyframe_insert(data_path="rotation_quaternion", frame=frame)
    else:
        pb.keyframe_insert(data_path="rotation_euler", frame=frame)


def _reset(rig):
    for pb in rig.pose.bones:
        pb.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()


def _foot_pose(phase, side):
    """phase 0..1 (사이클 위상, 접지 시작=0). (y, z) 반환. 전방=-Y.
    접지 절반: y -stride/2 -> +stride/2 등속, z=0. 스윙 절반: y 복귀, z 아치."""
    s = PARAMS["stride"] / 2.0
    if phase < 0.5:                       # 접지
        t = phase / 0.5
        return (-s + PARAMS["stride"] * t, 0.0)
    t = (phase - 0.5) / 0.5               # 스윙
    y = s - PARAMS["stride"] * t
    z = PARAMS["foot_lift"] * math.sin(math.pi * t)
    return (y, z)


def build(rig, scene):
    scene.render.fps = FPS
    scene.frame_start = 1
    scene.frame_end = FRAME_END

    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode='POSE')
    _reset(rig)
    pbs = rig.pose.bones

    # 팔 FK 전환 (f1에 키 — 베이크가 상태를 굽도록)
    for side in ("L", "R"):
        par = pbs[f"upper_arm_parent.{side}"]
        par["IK_FK"] = 1.0
        par.keyframe_insert(data_path='["IK_FK"]', frame=1)

    rest_foot = {s: pbs[f"foot_ik.{s}"].matrix.to_translation().copy() for s in ("L", "R")}

    frames = KEY_FRAMES + [FRAME_END + 1]          # f25 = f1 복제로 루프 폐쇄
    for f in frames:
        src_f = 1 if f == FRAME_END + 1 else f
        cycle_t = (src_f - 1) / FRAME_END          # 0..~1
        scene.frame_set(f)
        _reset(rig)

        # --- 다리 (IK): 위상 L=0시작, R=반주기 offset ---
        for side, offset in (("L", 0.0), ("R", 0.5)):
            phase = (cycle_t + offset) % 1.0
            y, z = _foot_pose(phase, side)
            fik = pbs[f"foot_ik.{side}"]
            base = rest_foot[side]
            m = fik.matrix.copy()
            m.translation.x = base.x
            m.translation.y = base.y + y           # rest y 기준 전후
            m.translation.z = base.z + z
            fik.matrix = m
            bpy.context.view_layer.update()

        # --- 몸통: 상하 bob(접지 최저/통과 최고) + 숙임 + 골반/가슴 요잉 ---
        bob = -PARAMS["torso_bob"] * math.cos(4 * math.pi * cycle_t)   # f1 최저, f7 최고 (반주기 2회)
        torso = pbs["torso"]
        mt = torso.matrix.copy()
        mt.translation.z += bob
        torso.matrix = mt
        bpy.context.view_layer.update()
        _rotate_world(rig, torso, 'X', -PARAMS["torso_lean_deg"])      # 전방(-Y) 숙임
        yaw = PARAMS["pelvis_yaw_deg"] * math.sin(2 * math.pi * cycle_t)
        _rotate_world(rig, pbs["hips"], 'Z', yaw)
        _rotate_world(rig, pbs["chest"], 'Z', -PARAMS["chest_counter_deg"] * math.sin(2 * math.pi * cycle_t))

        # --- 팔 (FK): 내리고, 다리 반대 위상 스윙 ---
        # 팔꿈치는 고정각이 아니라 전방 스윙에서 더 굽고 후방에서 펴진다
        # (실보행 굴곡-신전 ROM ~30°; 굽힘 위상은 어깨보다 arm_drag_frames 지연 —
        # 오버랩). 손목은 그 2배 지연으로 스윙 반대 방향으로 끌린다.
        for side, sgn, leg_offset in (("L", +1.0, 0.5), ("R", -1.0, 0.0)):
            ua = pbs[f"upper_arm_fk.{side}"]
            _rotate_world(rig, ua, 'Y', sgn * PARAMS["arm_down_deg"])  # 팔 내림 (컨택트시트 부호 규약)
            swing = PARAMS["arm_swing_deg"] * math.sin(2 * math.pi * ((cycle_t + leg_offset) % 1.0))
            _rotate_world(rig, ua, 'X', swing)
            drag = PARAMS["arm_drag_frames"] / FRAME_END
            bend_ph = (cycle_t - drag + leg_offset) % 1.0
            bend = PARAMS["elbow_bend_deg"] + PARAMS["elbow_swing_deg"] * max(0.0, math.sin(2 * math.pi * bend_ph))
            _rotate_world(rig, pbs[f"forearm_fk.{side}"], 'X', -bend)
            hand = pbs.get(f"hand_fk.{side}")
            if hand is not None:
                hand_ph = (cycle_t - 2.0 * drag + leg_offset) % 1.0
                _rotate_world(rig, hand, 'X', -PARAMS["wrist_drag_deg"] * math.sin(2 * math.pi * hand_ph))

        # --- 키 오버라이드 (T5 루프가 채움) ---
        for ctrl, spec in OVERRIDES.get(src_f, {}).items():
            pb = pbs[ctrl]
            if "rot_world" in spec:
                _rotate_world(rig, pb, spec["rot_world"][0], spec["rot_world"][1])
            if "loc_delta" in spec:
                m = pb.matrix.copy()
                m.translation.x += spec["loc_delta"][0]
                m.translation.y += spec["loc_delta"][1]
                m.translation.z += spec["loc_delta"][2]
                pb.matrix = m
                bpy.context.view_layer.update()

        # --- 키 삽입 ---
        for name in ("torso", "hips", "chest",
                     "foot_ik.L", "foot_ik.R",
                     "upper_arm_fk.L", "upper_arm_fk.R",
                     "forearm_fk.L", "forearm_fk.R",
                     "hand_fk.L", "hand_fk.R"):
            if name in pbs:
                _key_all(pbs[name], f)

    _followthrough(rig, scene)
    bpy.ops.object.mode_set(mode='OBJECT')
    rig.animation_data.action.name = "Walk"
