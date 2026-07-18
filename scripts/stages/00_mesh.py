import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from mathutils import Vector
from lib import paths
from lib.blender_utils import clean_scene, save_as
from lib.proportions import P

clean_scene()
paths.ensure_dirs()


def box_along(name, head, tail, thickness, cuts=3):
    """head→tail을 잇는 세그먼트 박스. 로컬 +Z를 본 방향에 정렬."""
    head, tail = Vector(head), Vector(tail)
    vec = tail - head
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(head + tail) / 2)
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = (thickness, thickness, vec.length)
    ob.rotation_mode = 'QUATERNION'
    ob.rotation_quaternion = vec.to_track_quat('Z', 'Y')
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.subdivide(number_cuts=cuts)
    bpy.ops.object.mode_set(mode='OBJECT')
    return ob


def mirror(v):
    return [-v[0], v[1], v[2]]


def h(bone):
    return P[bone]["head"]


def t(bone):
    return P[bone]["tail"]


HAND_TIP_L = [0.70, 0.02, 1.00]  # 손끝(손가락 포함 미튼) 근사 — hand.L 헤드에서 연장

parts = []
# 몸통·머리 (중앙)
parts.append(box_along("torso", h("spine"), t("spine.003"), 0.30, cuts=6))
parts.append(box_along("neck", h("spine.004"), h("spine.006"), 0.09, cuts=2))
parts.append(box_along("head", h("spine.006"), t("spine.006"), 0.17, cuts=2))

# 사지 (좌우)
for side, m in (("L", lambda v: v), ("R", mirror)):
    parts.append(box_along(f"upper_arm.{side}", m(h("upper_arm.L")), m(t("upper_arm.L")), 0.075, cuts=4))
    parts.append(box_along(f"forearm.{side}", m(h("forearm.L")), m(t("forearm.L")), 0.065, cuts=4))
    parts.append(box_along(f"hand.{side}", m(h("hand.L")), m(HAND_TIP_L), 0.075, cuts=3))
    parts.append(box_along(f"thigh.{side}", m(h("thigh.L")), m(t("thigh.L")), 0.12, cuts=4))
    parts.append(box_along(f"shin.{side}", m(h("shin.L")), m(t("shin.L")), 0.10, cuts=4))
    parts.append(box_along(f"foot.{side}", m(h("foot.L")), m(t("toe.L")), 0.08, cuts=3))

# 비대칭 laterality 마커 (영구 tripwire): 해부학적 왼팔(Blender +X)에만 부착.
# 대칭 더미의 유일한 비대칭 요소이므로 Unity mesh 무게중심 x의 부호가 "해부학적 왼쪽
# 질량이 어느 월드 X면에 안착했는가"(=미러 여부)를 드러낸다. forearm.L↔hand.L 관절을
# 감싸 배치 → DEF-forearm.L/DEF-hand.L 이 관통하므로 ARMATURE_AUTO가 전 정점을 왼팔에
# 웨이트한다(check_01의 >98% 통과). 전부 x>0 이라 무게중심을 확실히 +X로 이동시킨다.
_MK_C = t("forearm.L")  # forearm.L tail = hand.L head, 관절점
bpy.ops.mesh.primitive_cube_add(size=0.09, location=(_MK_C[0], _MK_C[1], _MK_C[2]))
_marker = bpy.context.active_object
_marker.name = "marker_L"
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.subdivide(number_cuts=3)
bpy.ops.object.mode_set(mode='OBJECT')
parts.append(_marker)

# 하나로 합침
bpy.ops.object.select_all(action='DESELECT')
for ob in parts:
    ob.select_set(True)
bpy.context.view_layer.objects.active = parts[0]
bpy.ops.object.join()
dummy = bpy.context.active_object
dummy.name = "Dummy"
# join 후 원점은 첫 파트(몸통) 중심에 남는다 — 월드 0으로 정리
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# 체커 재질
mat = bpy.data.materials.new("Checker")
mat.use_nodes = True
bsdf = mat.node_tree.nodes["Principled BSDF"]
tex = mat.node_tree.nodes.new("ShaderNodeTexChecker")
tex.inputs["Scale"].default_value = 40.0
mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
dummy.data.materials.append(mat)

save_as(paths.blend_path("00_mesh"))
print("STAGE 00_mesh OK")
