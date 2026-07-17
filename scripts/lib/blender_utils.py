import bpy


def ensure_rigify() -> None:
    """factory-startup에서 rigify를 켠다. addon_utils.enable은 5.1.2에서 깨지므로 금지."""
    try:
        bpy.ops.object.armature_human_metarig_add.get_rna_type()
        return
    except Exception:
        pass
    bpy.ops.preferences.addon_enable(module="rigify")
    bpy.ops.object.armature_human_metarig_add.get_rna_type()  # 실패 시 여기서 예외


def clean_scene() -> None:
    bpy.ops.wm.read_homefile(use_empty=True)


def open_blend(path) -> None:
    bpy.ops.wm.open_mainfile(filepath=str(path))


def save_as(path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(path))


def op_kwargs(op, **kwargs) -> dict:
    """연산자가 실제로 받는 파라미터만 남긴다. Blender 버전 간 시그니처 차이 방어."""
    props = {p.identifier for p in op.get_rna_type().properties}
    return {k: v for k, v in kwargs.items() if k in props}
