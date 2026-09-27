import bpy, sys, json, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import profile_lib as PL

ob = bpy.data.objects["GOB_body"]
mw = ob.matrix_world
P = [tuple(mw @ v.co) for v in ob.data.vertices]
prof = PL.core_profile(P)
lm = PL.landmarks(prof)
out = {"object": ob.name, "verts": len(P), "tris": sum(len(p.vertices) - 2 for p in ob.data.polygons),
       "matrix_world": [list(r) for r in mw], "landmarks": lm,
       "profile": [{k: round(v, 6) if isinstance(v, float) else v for k, v in s.items()} for s in prof]}
# camera
cam = bpy.data.objects.get("REF_CAM")
if cam:
    out["cam"] = {"loc": list(cam.location), "rot": list(cam.rotation_euler),
                  "lens": cam.data.lens, "sensor": cam.data.sensor_width}
p = r"C:\Users\whxod\orca\anime\work\goblin_swing\t\inspect\step01\old_profile.json"
json.dump(out, open(p, "w"), indent=1)
print("WROTE", p)
print(json.dumps(lm, indent=1))
