import bpy


def select_verts_by_box(obj, min_co, max_co):
    """월드 AABB 안 정점 인덱스 (트랜스폼 적용된 메시 전제)."""
    out = []
    for v in obj.data.vertices:
        c = v.co
        if all(min_co[i] <= c[i] <= max_co[i] for i in range(3)):
            out.append(v.index)
    return out


def reduce_influence(obj, group_name, vert_indices, factor):
    """지정 정점에서 특정 그룹 웨이트를 factor배로 (0이면 제거)."""
    vg = obj.vertex_groups.get(group_name)
    if vg is None:
        return
    idx = vg.index
    for vi in vert_indices:
        for g in obj.data.vertices[vi].groups:
            if g.group == idx:
                if factor <= 0.0:
                    vg.remove([vi])
                else:
                    vg.add([vi], g.weight * factor, 'REPLACE')
                break


def smooth_groups(obj, group_names, factor=0.5, repeat=3):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    prev_mode = obj.mode
    bpy.ops.object.mode_set(mode='WEIGHT_PAINT')
    for name in group_names:
        vg = obj.vertex_groups.get(name)
        if vg is None:
            continue
        obj.vertex_groups.active_index = vg.index
        bpy.ops.object.vertex_group_smooth(group_select_mode='ACTIVE', factor=factor, repeat=repeat)
    bpy.ops.object.mode_set(mode=prev_mode if prev_mode != 'WEIGHT_PAINT' else 'OBJECT')


def limit_total(obj, max_influences=4):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.vertex_group_limit_total(limit=max_influences)
    bpy.ops.object.vertex_group_normalize_all(lock_active=False)
