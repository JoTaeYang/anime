"""발견된 웨이트 문제의 보정 누적 스크립트. 01_rig가 자동 웨이트 직후 호출한다.
새 문제를 발견하면 여기에 보정 호출을 추가하고 주석으로 증상을 기록한다."""
from lib import weight_tools
from lib.profiles import get_profile


def apply(obj):
    profile = get_profile()
    if profile.NAME != "character":
        return
    weight_tools.limit_total(obj, max_influences=4)

    # --- Iteration 1: 치마↔허벅지 영향 번짐 ---
    # 증상 (leg_spread, 행2 열2): 다리를 벌리면 치마 허리밴드가 허벅지에 끌려 가랑이 사이가
    # 늘어난다. 진단(_diag_weights): 스커트 정점(skirt weight>0.3)이 DEF-thigh.L/R 를 0.7까지
    # 물고 있음 (z0.75-0.92, |x|0.09-0.16). 그 z대엔 진짜 허벅지 정점(1094개)도 공존해서 위치
    # 박스로 thigh를 0으로 지우면 다리가 깨진다 → 두 갈래로 보정한다:
    #   (a) z<0.76 구간은 진짜 다리 정점이 0개(스커트 전용) → 안전하게 직접 축소로 확실히 떼어냄.
    #   (b) 다리와 겹치는 z0.75+ 구간은 thigh 그룹을 스무딩해 0.7 피크를 낮추고 경사를 부드럽게
    #       (게이트: "부드러운 경사 허용"). 균일한 실다리 영역(~1.0)은 스무딩에 거의 불변 → 안전.
    _skirt_band_L = weight_tools.select_verts_by_box(obj, (0.06, -0.30, 0.60), (0.30, 0.30, 0.76))
    weight_tools.reduce_influence(obj, "DEF-thigh.L", _skirt_band_L, 0.3)
    _skirt_band_R = weight_tools.select_verts_by_box(obj, (-0.30, -0.30, 0.60), (-0.06, 0.30, 0.76))
    weight_tools.reduce_influence(obj, "DEF-thigh.R", _skirt_band_R, 0.3)
    weight_tools.smooth_groups(obj, ["DEF-thigh.L", "DEF-thigh.R"], factor=0.5, repeat=4)
    weight_tools.limit_total(obj, max_influences=4)  # 축소·스무딩 후 재정규화
