import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import save_as
from lib.profiles import get_profile

PROFILE = get_profile()
assert PROFILE.SOURCE["kind"] == "blend", "00_intake는 blend 소스 프로필 전용"

src = paths.PROJECT_ROOT / PROFILE.SOURCE["path"]
bpy.ops.wm.open_mainfile(filepath=str(src))
paths.ensure_dirs()

ob = bpy.data.objects[PROFILE.SOURCE["object"]]
assert ob.type == 'MESH'

# 사전 점검: 트랜스폼 적용 상태, 대칭
assert all(abs(s - 1.0) < 1e-4 for s in ob.scale), f"scale not applied: {tuple(ob.scale)}"
assert all(abs(a) < 1e-4 for a in ob.rotation_euler), "rotation not applied"
import numpy as np
n = len(ob.data.vertices)
co = np.empty(n * 3, dtype=np.float32)
ob.data.vertices.foreach_get("co", co)
co = co.reshape(n, 3)
assert abs(float(co[:, 0].mean())) < 0.02, "mesh not X-symmetric"

# 정규화: 발 접지 (bbox min z -> 0), 원점 월드 0
ob.location.z -= float(co[:, 2].min()) + ob.location.z
bpy.ops.object.select_all(action='DESELECT')
ob.select_set(True)
bpy.context.view_layer.objects.active = ob
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

ob.name = PROFILE.MESH_OBJECT
# 정면 -Y 확인: 발끝(낮은 z 슬라이스)의 y 중심이 음수여야 함 (발이 앞으로 나옴)
low = co[co[:, 2] < co[:, 2].min() + 0.12]
assert float(low[:, 1].mean()) < 0.0, f"feet not pointing -Y: {float(low[:, 1].mean())}"

save_as(paths.blend_path("00_mesh"))
print("STAGE 00_intake OK height=", round(ob.dimensions.z, 3))
