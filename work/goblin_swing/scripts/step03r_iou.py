"""Silhouette IoU of the v05 render through REF_CAM vs reference frame 1,
using step04_reference_setup.py's OWN mask pipeline, for both GOB meshes."""
import bpy, json, os, sys
import numpy as np

ROOT = r"C:\Users\whxod\orca\anime\work\goblin_swing"
sys.path.append(os.path.join(ROOT, "scripts"))
import step04_reference_setup as S4

sc = bpy.context.scene
cam = bpy.data.objects["REF_CAM"]
sc.camera = cam
sc.render.resolution_x, sc.render.resolution_y = 448, 576
sc.render.resolution_percentage = 100
sc.render.engine = "BLENDER_WORKBENCH"
sc.render.film_transparent = True
sc.render.image_settings.file_format = "PNG"
sc.render.image_settings.color_mode = "RGBA"
sc.display.shading.show_shadows = False
sc.display.shading.show_cavity = False
sc.display.render_aa = "8"
g = bpy.data.objects.get("REF_ground")
if g:
    g.hide_render = True

mask = S4.reference_mask(os.path.join(ROOT, "ref"), 1)
out = {"ref_mask_px": int(mask.sum())}
tmp = os.path.join(ROOT, "inspect", "step03r", "iou_tmp.png")
for name in ("GOB_body_lo", "GOB_body"):
    for o in bpy.data.objects:
        if o.name.startswith("GOB_body"):
            hid = (o.name != name)
            o.hide_render = hid
            for c in o.users_collection:
                c.hide_render = hid
    sc.render.filepath = tmp
    bpy.ops.render.render(write_still=True)
    rm = S4._load_rgba(tmp)[:, :, 3] > 0.5
    out[name] = {"px": int(rm.sum()),
                 "iou": round(float((rm & mask).sum()) / max(float((rm | mask).sum()), 1.0), 4)}
print("@@@JSON_START@@@")
print(json.dumps(out, indent=1))
print("@@@JSON_END@@@")
