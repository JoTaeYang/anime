"""walk 사이클 파라미터. 값 수정 = 애니메이션 수정 (T5 비교 루프의 대상).
전방 = -Y. 제자리 사이클: 접지한 발이 +Y로 등속 후퇴."""

FPS = 24
FRAME_END = 24                      # f25 = f1 (루프)
KEY_FRAMES = [1, 4, 7, 10, 13, 16, 19, 22]
# 접지 구간 (기계 검사용): 발별 (시작f, 끝f) — 이 구간 동안 발 z≈0, y 등속 후퇴
CONTACTS = {"L": (1, 12), "R": (13, 24)}

PARAMS = {
    "stride": 0.48,        # 보폭 (전후 총 이동량, m)
    "foot_lift": 0.12,     # 스윙 중 발 최고 높이
    "foot_x_L": 0.084,     # 발 좌우 폭 (랜드마크 다리 x)
    "torso_bob": 0.012,    # 몸통 상하 진폭 (접지 직후 최저, 통과 시 최고)
                           # 0.028→0.016→0.012 (2026-07-20 사용자 판정: 배/허리 상하 펌핑 과잉)
    "torso_lean_deg": 5.0, # 전방 숙임 (고정)
    "pelvis_yaw_deg": 6.0, # 골반 요잉 (왼발 전방일 때 왼골반 앞으로)
    "chest_counter_deg": 5.0,  # 가슴 반대 요잉
    "arm_down_deg": 72.0,  # T포즈에서 팔 내리는 기본 각 (rotate_world Y축: L +, R -)
    "arm_swing_deg": 38.0, # 팔 전후 스윙 진폭 (다리와 반대 위상, world X축 회전)
    # 팔꿈치: 실보행에서 굴곡-신전 ROM ~30° (고정각 아님). 전방 스윙에서 더 굽고
    # 후방에서 펴진다 (2026-07-20 웹리서치: cise 2023, rustyanimator/animationmentor).
    "elbow_bend_deg": 22.0,   # 기본 굽힘 (후방 스윙 시 잔여 굽힘)
    "elbow_swing_deg": 14.0,  # 전방 스윙 시 추가 굽힘 (합계 최대 36°)
    "arm_drag_frames": 2,     # 오버랩: 팔꿈치 굽힘 위상이 어깨 스윙보다 2f 지연
    "wrist_drag_deg": 9.0,    # 손목 드래그 진폭 (어깨보다 4f 지연, 스윙 반대로 끌림)
}

# 키프레임별 미세 오버라이드: {frame: {"<control>": {"rot_world": (axis, deg)} | {"loc_delta": (x,y,z)}}}
# T5 루프에서 레퍼런스와 어긋나는 관절을 여기에 추가한다.
OVERRIDES = {}

FOLLOWTHROUGH = {
    "delay_frames": 3,     # 체인 단계당 지연
    "damp": 0.6,           # 단계당 감쇠
    # 체인별 {gain, yaw_gain, controls} (실측 컨트롤명, Global Constraints 참조).
    # gain: 수직 신호(hips z 속도 → local X 회전) 배율.
    # yaw_gain: 측방 신호(골반 요잉 각의 지연 사본 → local Z 회전) 배율.
    # 2026-07-20 사용자 판정 2회 반영: (1) 치마 동위상 펄럭임 → 대폭 감쇠.
    # (2) 꼬리 수직 메트로놈 → 수직은 줄이고 골반 요잉을 끌림 사본으로 받아
    # 좌우 스웨이 주도 (드래그/오버랩 원칙 — 몸과 동기화 금지).
    "chains": [
        {"gain": 0.6, "yaw_gain": 2.0, "controls": ["tail", "tail.001", "tail.002", "tail.003", "tail.004"]},
        {"gain": 0.25, "yaw_gain": 0.0, "controls": ["skirt_f", "skirt_f.001"]},
        {"gain": 0.25, "yaw_gain": 0.0, "controls": ["skirt_fr", "skirt_fr.001"]},
        {"gain": 0.25, "yaw_gain": 0.0, "controls": ["skirt_r", "skirt_r.001"]},
        {"gain": 0.25, "yaw_gain": 0.0, "controls": ["skirt_br", "skirt_br.001"]},
        {"gain": 0.25, "yaw_gain": 0.0, "controls": ["skirt_b", "skirt_b.001"]},
        {"gain": 0.25, "yaw_gain": 0.0, "controls": ["skirt_bl", "skirt_bl.001"]},
        {"gain": 0.25, "yaw_gain": 0.0, "controls": ["skirt_l", "skirt_l.001"]},
        {"gain": 0.25, "yaw_gain": 0.0, "controls": ["skirt_fl", "skirt_fl.001"]},
        {"gain": 0.9, "yaw_gain": 0.4, "controls": ["scarf_l", "scarf_l.001"]},
        {"gain": 0.9, "yaw_gain": 0.4, "controls": ["scarf_r", "scarf_r.001"]},
        {"gain": 0.8, "yaw_gain": 0.0, "controls": ["hood_ear_l"]},
        {"gain": 0.8, "yaw_gain": 0.0, "controls": ["hood_ear_r"]},
    ],
}
