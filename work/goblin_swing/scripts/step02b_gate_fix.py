"""GATE 02 addendum: corrected head-clearance metric (T4) + IK-stretch check
for the reach failures found in T1/T2/T6.  Run on goblin_v03_controls.blend.
NEVER saves.
"""
import bpy, json, math, os
from mathutils import Vector, Matrix, Quaternion
from mathutils.bvhtree import BVHTree

OUT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step02b"
REPORT = r"C:\Users\whxod\orca\anime\work\goblin_swing\inspect\step02b_gate_fix.json"

sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
body = bpy.data.objects["GOB_body"]
PB = rig.pose.bones
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
LEN = {n: arm.bones[n].length for n in arm.bones.keys()}
R = {}


def upd():
    rig.update_tag()
    bpy.context.view_layer.update()
    return rig.evaluated_get(bpy.context.evaluated_depsgraph_get())


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0
        PB["FOOT_IK_" + S]["ik_stretch"] = 0.0


def set_world_loc(b, d):
    PB[b].location = REST[b].to_3x3().inverted() @ Vector(d)


def set_world_rot(b, axis, deg):
    M = REST[b].to_3x3()
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    ql = (M.inverted() @ q.to_matrix() @ M).to_quaternion()
    pb = PB[b]
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = ql
    else:
        pb.rotation_euler = ql.to_euler(pb.rotation_mode)


def bhead(ev, n):
    return ev.pose.bones[n].matrix.to_translation()


def btail(ev, n):
    return ev.pose.bones[n].matrix @ Vector((0, LEN[n], 0))


def foot_error(ev):
    return {S: round((bhead(ev, "FOOT_" + S) - bhead(ev, "FOOT_IK_" + S)).length, 6)
            for S in ("L", "R")}


# ------------------------------------------------ isolate the HEAD component
me = body.data
ZC = 1.34
sel = {i for i, v in enumerate(me.vertices) if v.co.z > ZC}
adj = {}
for e in me.edges:
    a, b = e.vertices
    if a in sel and b in sel:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
seen, comps = set(), []
for s in sel:
    if s in seen:
        continue
    st, c = [s], []
    seen.add(s)
    while st:
        u = st.pop()
        c.append(u)
        for w in adj.get(u, ()):
            if w not in seen:
                seen.add(w)
                st.append(w)
    comps.append(set(c))
comps.sort(key=len, reverse=True)
HEADSET = comps[0]
CLUBSET = comps[1] if len(comps) > 1 else set()
hv, hf, idx = [], [], {}
for p in me.polygons:
    vs = list(p.vertices)
    if all(i in HEADSET for i in vs):
        f = []
        for i in vs:
            if i not in idx:
                idx[i] = len(hv)
                hv.append(me.vertices[i].co.copy())
            f.append(idx[i])
        hf.append(tuple(f))
HB = BVHTree.FromPolygons(hv, hf, all_triangles=False, epsilon=0.0)
hb_min = Vector((min(v.x for v in hv), min(v.y for v in hv), min(v.z for v in hv)))
hb_max = Vector((max(v.x for v in hv), max(v.y for v in hv), max(v.z for v in hv)))
R["head_component"] = {"verts": len(hv), "faces": len(hf),
                       "bbox_min": [round(c, 3) for c in hb_min],
                       "bbox_max": [round(c, 3) for c in hb_max],
                       "club_component_verts": len(CLUBSET),
                       "cut_plane_z": ZC}

RAYS = [Vector((1, 0, 0)), Vector((-1, 0, 0)), Vector((0, 1, 0)),
        Vector((0, -1, 0)), Vector((0, 0, 1))]


def inside_head(p):
    """parity test; only rays that exit through closed faces (side/top)."""
    if p.z <= ZC + 1e-4:
        return False
    if not (hb_min.x - 1e-6 <= p.x <= hb_max.x + 1e-6 and
            hb_min.y - 1e-6 <= p.y <= hb_max.y + 1e-6 and
            p.z <= hb_max.z + 1e-6):
        return False
    votes = 0
    for d in RAYS:
        cur = p + d * 1e-5
        n = 0
        for _ in range(40):
            hit = HB.ray_cast(cur, d)
            if hit[0] is None:
                break
            n += 1
            cur = hit[0] + d * 1e-5
        if n % 2 == 1:
            votes += 1
    return votes >= 3


def seg_signed(a, b, n=30):
    """(min signed clearance, max penetration depth, fraction of samples inside)"""
    best = 1e9
    deep = 0.0
    ins = 0
    for i in range(n + 1):
        p = a.lerp(b, i / n)
        loc, nor, fi, dist = HB.find_nearest(p)
        if loc is None:
            continue
        if inside_head(p):
            ins += 1
            deep = max(deep, dist)
            best = min(best, -dist)
        else:
            best = min(best, dist)
    return round(best, 4), round(deep, 4), round(ins / (n + 1), 3)


def clearance(ev):
    segs = {"upperarm": (bhead(ev, "UPPERARM_R"), bhead(ev, "FOREARM_R")),
            "forearm": (bhead(ev, "FOREARM_R"), bhead(ev, "HAND_R")),
            "hand": (bhead(ev, "HAND_R"), btail(ev, "HAND_R")),
            "weapon": (bhead(ev, "WEAPON"), btail(ev, "WEAPON"))}
    o = {}
    for k, (a, b) in segs.items():
        s, d, f = seg_signed(a, b)
        o[k] = {"clr": s, "pen": d, "frac_in": f}
    return o


# ------------------------------------------------ T4 corrected
clear_pose()
ev = upd()
R["rest_clearance"] = clearance(ev)

clear_pose()
set_world_rot("UPPERARM_FK_L", (0, -1, 0), 90)
set_world_rot("UPPERARM_FK_R", (0, 1, 0), 90)
ev = upd()
R["both_arms_90"] = {"clearance": clearance(ev),
                     "handR": [round(c, 3) for c in bhead(ev, "HAND_R")],
                     "club_tip": [round(c, 3) for c in btail(ev, "WEAPON")]}

sweep = []
for a in range(0, 181, 5):
    clear_pose()
    set_world_rot("UPPERARM_FK_R", (0, 1, 0), a)
    ev = upd()
    c = clearance(ev)
    sweep.append({"deg": a, "c": c,
                  "min_clr": round(min(v["clr"] for v in c.values()), 4),
                  "any_in": any(v["frac_in"] > 0 for v in c.values()),
                  "club_tip": [round(x, 3) for x in btail(ev, "WEAPON")]})
R["right_raise_sweep"] = sweep
first = next((s["deg"] for s in sweep if s["any_in"]), None)
R["first_penetration_deg"] = first
R["max_clean_raise_deg"] = (sweep[[s["deg"] for s in sweep].index(first) - 1]["deg"]
                            if first not in (None, 0) else None)

# wind-up over the right shoulder: raise + swing back (rotate about world X too)
wind = []
for a in range(0, 181, 15):
    for back in (0, 20, 40):
        clear_pose()
        M = REST["UPPERARM_FK_R"].to_3x3()
        q = (Quaternion(Vector((0, 0, 1)), math.radians(-back)) @
             Quaternion(Vector((0, 1, 0)), math.radians(a)))
        PB["UPPERARM_FK_R"].rotation_quaternion = (
            M.inverted() @ q.to_matrix() @ M).to_quaternion()
        ev = upd()
        c = clearance(ev)
        wind.append({"raise": a, "yaw_back": back,
                     "min_clr": round(min(v["clr"] for v in c.values()), 4),
                     "any_in": any(v["frac_in"] > 0 for v in c.values()),
                     "club_tip": [round(x, 3) for x in btail(ev, "WEAPON")]})
R["windup_grid"] = wind
R["windup_clean"] = [w for w in wind if not w["any_in"]]

# ------------------------------------------------ IK stretch verification
st = []
for case, fn in [
        ("COG +0.15 X", lambda: set_world_loc("COG_CTRL", (0.15, 0, 0))),
        ("COG -0.15 X", lambda: set_world_loc("COG_CTRL", (-0.15, 0, 0))),
        ("COG +0.05 Z", lambda: set_world_loc("COG_CTRL", (0, 0, 0.05))),
        ("HIPS yaw30 + COG", lambda: (set_world_rot("HIPS_CTRL", (0, 0, 1), 30),
                                      set_world_loc("COG_CTRL", (0.05, 0, -0.04)))),
]:
    for stretch in (0.0, 1.0):
        clear_pose()
        for S in ("L", "R"):
            PB["FOOT_IK_" + S]["ik_stretch"] = stretch
        fn()
        ev = upd()
        st.append({"case": case, "ik_stretch": stretch,
                   "foot_err_m": foot_error(ev),
                   "thigh_len": round((bhead(ev, "SHIN_L") - bhead(ev, "THIGH_L")).length, 5),
                   "shin_len": round((bhead(ev, "FOOT_L") - bhead(ev, "SHIN_L")).length, 5)})
R["ik_stretch_check"] = st

# rest invariance with stretch enabled
clear_pose()
for S in ("L", "R"):
    PB["FOOT_IK_" + S]["ik_stretch"] = 1.0
ev = upd()
rows = []
for n in REST:
    if n.endswith("_CTRL") or n.startswith("MCH") or "_IK_" in n or "POLE" in n:
        continue
    m = ev.pose.bones[n].matrix
    dp = (m.to_translation() - REST[n].to_translation()).length
    dr = math.degrees(abs(m.to_quaternion().rotation_difference(
        REST[n].to_quaternion()).angle))
    rows.append([n, round(dp, 8), round(dr, 6)])
R["rest_invariance_stretch_on"] = {"max_pos_m": max(r[1] for r in rows),
                                   "max_rot_deg": max(r[2] for r in rows)}
clear_pose()

# ------------------------------------------------ head rotation axis in WORLD
t8 = []
for label, axis, deg in [("yaw+40", (0, 0, 1), 40), ("pitch+30", (1, 0, 0), 30)]:
    clear_pose()
    set_world_rot("HEAD_CTRL", axis, deg)
    ev = upd()
    Rw = ev.pose.bones["HEAD"].matrix.to_3x3() @ REST["HEAD"].to_3x3().inverted()
    q = Rw.to_quaternion()
    t8.append({"case": label, "world_axis": [round(c, 4) for c in q.axis],
               "world_angle_deg": round(math.degrees(q.angle), 3)})
    Rw2 = ev.pose.bones["CHEST"].matrix.to_3x3() @ REST["CHEST"].to_3x3().inverted()
    t8[-1]["chest_rot_deg"] = round(math.degrees(Rw2.to_quaternion().angle), 5)
R["head_axis_world"] = t8
clear_pose()

with open(REPORT, "w") as f:
    json.dump(R, f, indent=1, default=str)
print("@@@FIX_DONE@@@")
