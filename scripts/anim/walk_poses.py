"""walk 사이클 파라미터. 값 수정 = 애니메이션 수정 (T5 비교 루프의 대상).
전방 = -Y. 제자리 사이클: 접지한 발이 +Y로 등속 후퇴."""

FPS = 24
FRAME_END = 24                      # f25 = f1 (루프)
KEY_FRAMES = [1, 4, 7, 10, 13, 16, 19, 22]
# 접지 구간 (기계 검사용): 발별 (시작f, 끝f) — 이 구간 동안 발 z≈0, y 등속 후퇴
CONTACTS = {"L": (1, 12), "R": (13, 24)}

PARAMS = {
    "stride": 0.34,        # 보폭 (전후 총 이동량, m)
    "foot_lift": 0.07,     # 스윙 중 발 최고 높이
    "foot_x_L": 0.084,     # 발 좌우 폭 (랜드마크 다리 x)
    "torso_bob": 0.020,    # 몸통 상하 진폭 (접지 직후 최저, 통과 시 최고)
    "torso_lean_deg": 5.0, # 전방 숙임 (고정)
    "pelvis_yaw_deg": 6.0, # 골반 요잉 (왼발 전방일 때 왼골반 앞으로)
    "chest_counter_deg": 5.0,  # 가슴 반대 요잉
    "arm_down_deg": 72.0,  # T포즈에서 팔 내리는 기본 각 (rotate_world Y축: L +, R -)
    "arm_swing_deg": 24.0, # 팔 전후 스윙 진폭 (다리와 반대 위상, world X축 회전)
    "elbow_bend_deg": 14.0,# 팔꿈치 상시 굽힘
}

# 키프레임별 미세 오버라이드: {frame: {"<control>": {"rot_world": (axis, deg)} | {"loc_delta": (x,y,z)}}}
# T5 루프에서 레퍼런스와 어긋나는 관절을 여기에 추가한다.
OVERRIDES = {}

FOLLOWTHROUGH = {
    "delay_frames": 3,     # 체인 단계당 지연
    "damp": 0.6,           # 단계당 감쇠
    # 체인 루트 컨트롤 → 체인 컨트롤 나열 (실측 컨트롤명, Global Constraints 참조)
    "chains": [
        ["tail", "tail.001", "tail.002", "tail.003", "tail.004"],
        ["skirt_f", "skirt_f.001"], ["skirt_fr", "skirt_fr.001"],
        ["skirt_r", "skirt_r.001"], ["skirt_br", "skirt_br.001"],
        ["skirt_b", "skirt_b.001"], ["skirt_bl", "skirt_bl.001"],
        ["skirt_l", "skirt_l.001"], ["skirt_fl", "skirt_fl.001"],
        ["scarf_l", "scarf_l.001"], ["scarf_r", "scarf_r.001"],
        ["hood_ear_l"], ["hood_ear_r"],
    ],
}
