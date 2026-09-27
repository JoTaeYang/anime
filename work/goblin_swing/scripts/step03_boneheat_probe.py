"""STEP 03 probe - try automatic (bone-heat) weights and measure how bad they are.
Run: blender -b goblin_v03_controls.blend --python step03_boneheat_probe.py
Writes a report only (and a throwaway blend in TEMP, not the deliverable).
"""
import bpy, json, math, os, time
from mathutils import Vector

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect"
rig = bpy.data.objects["GOB_rig"]
body = bpy.data.objects["GOB_body"]
me = body.data
MW = body.matrix_world
co = [MW @ v.co for v in me.vertices]
N = len(co)

R = {}

AX = Vector((-0.10609, 0.03071, 0.99388)).normalized()
GP = Vector((-0.6661, -0.3447, 0.9910))
HC_R = Vector((-0.676, -0.364, 0.992))
HC_L = Vector((0.772, 0.056, 0.720))

s_arr, d_arr = [], []
for p in co:
    v = p - GP
    s = v.dot(AX)
    s_arr.append(s)
    d_arr.append((v - AX * s).length)

# club set (same rule as the final script): near the axis, outside the ball band
CLUB = set()
for i in range(N):
    s, d = s_arr[i], d_arr[i]
    if -0.36 < s < 0.90 and abs(s) > 0.1134 and d < 0.22:
        CLUB.add(i)
HANDR = set(i for i in range(N)
            if abs(s_arr[i]) <= 0.1134 and (co[i] - HC_R).length < 0.155)
HEADSET = set(i for i in range(N) if co[i].z > 1.36 and abs(co[i].x) < 0.5)

for o in bpy.context.view_layer.objects:
    o.select_set(False)
bpy.context.view_layer.objects.active = rig
rig.select_set(True)
body.select_set(True)
t0 = time.time()
err = None
try:
    bpy.ops.object.parent_set(type='ARMATURE_AUTO')
except Exception as e:
    err = str(e)
R["bone_heat_seconds"] = round(time.time() - t0, 1)
R["bone_heat_error"] = err
R["n_vgroups"] = len(body.vertex_groups)
R["vgroup_names"] = [g.name for g in body.vertex_groups]
R["modifiers"] = [(m.type, getattr(m, 'object', None) and m.object.name) for m in body.modifiers]

if body.vertex_groups:
    gi = {g.index: g.name for g in body.vertex_groups}
    counts = {n: 0 for n in gi.values()}
    zerow = 0
    infl_hist = [0] * 12
    for v in me.vertices:
        gs = [g for g in v.groups if g.weight > 1e-4]
        infl_hist[min(11, len(gs))] += 1
        if not gs:
            zerow += 1
        for g in gs:
            counts[gi[g.group]] += 1
    R["verts_per_group"] = counts
    R["zero_weight_verts"] = zerow
    R["influence_count_hist"] = infl_hist

    def bleed(vset, label, forbidden):
        worst = {}
        for i in vset:
            for g in me.vertices[i].groups:
                n = gi[g.group]
                if n in forbidden and g.weight > worst.get(n, 0):
                    worst[n] = round(g.weight, 4)
        return {label: worst}

    bad = {}
    bad.update(bleed(CLUB, "club_gets", {"HEAD", "NECK", "CHEST", "SPINE_01", "HIPS",
                                         "UPPERARM_R", "FOREARM_R", "HAND_R",
                                         "UPPERARM_L", "FOREARM_L", "HAND_L"}))
    bad.update(bleed(HEADSET, "head_gets", {"WEAPON", "HAND_R", "FOREARM_R",
                                            "UPPERARM_R", "HIPS"}))
    bad.update(bleed(HANDR, "ringhand_gets", {"WEAPON", "HEAD", "CHEST", "HIPS"}))
    R["bone_heat_bleed"] = bad
    R["set_sizes"] = {"club": len(CLUB), "handR": len(HANDR), "head": len(HEADSET)}

    # weight on non-deform bones?
    nd = [n for n in R["vgroup_names"]
          if n not in [b.name for b in rig.data.bones if b.use_deform]]
    R["nondeform_groups"] = nd

print("@@@JSON_START@@@")
print(json.dumps(R, indent=1, default=str))
print("@@@JSON_END@@@")
with open(os.path.join(OUT, "step03_boneheat_probe.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
print("DONE")
