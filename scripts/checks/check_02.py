import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bpy
from lib import paths
from lib.blender_utils import open_blend

open_blend(paths.blend_path("02_animated"))

scene = bpy.context.scene
assert scene.render.fps == 24, f"fps {scene.render.fps}"
assert (scene.frame_start, scene.frame_end) == (1, 48)

rig = bpy.data.objects["rig"]
act = rig.animation_data.action if rig.animation_data else None
assert act is not None and act.name == "Idle", f"action: {act and act.name}"

def torso_matrix(frame):
    scene.frame_set(frame)
    return rig.pose.bones["torso"].matrix.copy()

m1, m24 = torso_matrix(1), torso_matrix(24)
delta = (m1.translation - m24.translation).length
assert delta > 0.005, f"torso barely moves between f1/f24: {delta}"

m48 = torso_matrix(48)
assert (m1.translation - m48.translation).length < 1e-4, "loop not closed (f1 != f48)"
print("CHECK_02 OK")
