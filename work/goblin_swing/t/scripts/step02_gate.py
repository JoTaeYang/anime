"""GATE 02 - stress test for gobT_v02_rig.blend.  NEVER saves.
blender -b gobT_v02_rig.blend --python step02_gate.py -- [--res 768] [--norender]
"""
import bpy, json, math, os, sys, time
import numpy as np
from mathutils import Vector, Quaternion, Matrix
from mathutils.bvhtree import BVHTree

T = r"C:\Users\whxod\orca\anime\work\goblin_swing\t"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
RES = int(argv[argv.index("--res") + 1]) if "--res" in argv else 768
NORENDER = "--norender" in argv
OUT = os.path.join(T, "inspect", "step02")
os.makedirs(OUT, exist_ok=True)
sc = bpy.context.scene
rig = bpy.data.objects["GOB_rig"]
arm = rig.data
PB = rig.pose.bones
lo = bpy.data.objects["GOB_body_lo"]
club = bpy.data.objects["GOB_club"]
REST = {n: arm.bones[n].matrix_local.copy() for n in arm.bones.keys()}
LEN = {n: arm.bones[n].length for n in arm.bones.keys()}
CM0 = club.matrix_world.copy()
R = {"res": RES}
DEF = [n for n in arm.bones.keys() if arm.bones[n].use_deform]

# ---------------------------------------------------------------- helpers
def upd():
    rig.update_tag()
    bpy.context.view_layer.update()
    return bpy.context.evaluated_depsgraph_get()


def clear_pose():
    for pb in PB:
        pb.location = (0, 0, 0); pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
        pb.rotation_euler = (0, 0, 0)
        pb.rotation_axis_angle = (0, 0, 1, 0)
    for S in ("L", "R"):
        PB["HAND_IK_" + S]["ik_fk"] = 0.0
        PB["FOOT_IK_" + S]["leg_roll"] = 0.0


def wloc(bone, delta, add=False):
    v = REST[bone].to_3x3().inverted() @ Vector(delta)
    PB[bone].location = (PB[bone].location + v) if add else v


def _apply(pb, q, add):
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (pb.rotation_quaternion @ q) if add else q
    else:
        cur = pb.rotation_euler.to_quaternion() if add else Quaternion()
        pb.rotation_euler = (cur @ q).to_euler(pb.rotation_mode)


def wrot(bone, axis, deg, add=False):
    M = REST[bone].to_3x3()
    q = Quaternion(Vector(axis).normalized(), math.radians(deg))
    _apply(PB[bone], (M.inverted() @ q.to_matrix() @ M).to_quaternion(), add)


def lrot(bone, axis, deg, add=False):
    _apply(PB[bone], Quaternion(Vector(axis).normalized(), math.radians(deg)), add)


def ev_rig(dg):
    return rig.evaluated_get(dg)


# rest world mesh of the low-poly body + BVH (body is unskinned, so it never moves)
me = lo.data
NV = len(me.vertices)
LP = np.empty(NV * 3); me.vertices.foreach_get("co", LP); LP = LP.reshape(NV, 3)
LP = (np.c_[LP, np.ones(NV)] @ np.array(lo.matrix_world).T)[:, :3]
me.calc_loop_triangles()
LTRI = [tuple(t.vertices) for t in me.loop_triangles]
# The body is NOT skinned yet, so for clearance it is moved RIGIDLY: everything above
# the belt by the CHEST delta, everything below by the HIPS delta.  The character's
# right arm + hand is excluded entirely - in reality it moves with the club.
_A = np.array((-0.42, 0.145, 1.265)); _B = np.array((-1.33, 0.167, 1.256))
_ab = _B - _A
_t = np.clip(((LP - _A) @ _ab) / float(_ab @ _ab), 0.0, 1.0)
_d = np.linalg.norm(LP - (_A + _t[:, None] * _ab), axis=1)
RARM = (LP[:, 0] < -0.42) & (_d < 0.17)
KEEP = ~RARM
UPPER = LP[:, 2] > 0.55
KTRI = [t for t in LTRI if KEEP[t[0]] and KEEP[t[1]] and KEEP[t[2]]]
KHTRI = [t for t in KTRI if LP[t[0], 2] > 1.315 and LP[t[1], 2] > 1.315 and LP[t[2], 2] > 1.315]


def body_trees(dg):
    ev = ev_rig(dg)
    Mu = np.array(ev.pose.bones["CHEST"].matrix @ REST["CHEST"].inverted())
    Ml = np.array(ev.pose.bones["HIPS"].matrix @ REST["HIPS"].inverted())
    H = np.c_[LP, np.ones(len(LP))]
    Q = np.where(UPPER[:, None], (H @ Mu.T)[:, :3], (H @ Ml.T)[:, :3])
    pts = [Vector(q) for q in Q]
    return (BVHTree.FromPolygons(pts, KTRI, all_triangles=True, epsilon=0.0),
            BVHTree.FromPolygons(pts, KHTRI, all_triangles=True, epsilon=0.0))
cme = club.data
NC = len(cme.vertices)
CP = np.empty(NC * 3); cme.vertices.foreach_get("co", CP); CP = CP.reshape(NC, 3)
cme.calc_loop_triangles()
CTRI = [tuple(t.vertices) for t in cme.loop_triangles]

# club rest offset relative to the WEAPON bone
WR = REST["WEAPON"]
K_CLUB = WR.inverted() @ CM0                       # club world = WEAPON pose @ K_CLUB
K_HAND = REST["HAND_R"].inverted() @ REST["WEAPON"]   # WEAPON rest rel. HAND_R


def club_world(dg):
    return club.evaluated_get(dg).matrix_world.copy()


def club_metrics(dg):
    M = club_world(dg)
    WM = ev_rig(dg).pose.bones["WEAPON"].matrix
    resid = max(abs((WM @ K_CLUB)[i][j] - M[i][j]) for i in range(4) for j in range(4))
    BODY, HEADBVH = body_trees(dg)
    Q = (np.c_[CP, np.ones(NC)] @ np.array(M).T)[:, :3]
    dmin, where, hmin = 1e9, None, 1e9
    for i in range(0, NC, 2):
        loc, nor, fi, d = BODY.find_nearest(Vector(Q[i]), 1.5)
        if loc is not None and d < dmin:
            dmin = d; where = [round(float(v), 3) for v in loc]
        lh, _, _, dh = HEADBVH.find_nearest(Vector(Q[i]), 1.5)
        if lh is not None and dh < hmin:
            hmin = dh
    ov = BVHTree.FromPolygons([Vector(p) for p in Q], CTRI,
                              all_triangles=True, epsilon=0.0).overlap(BODY)
    return {"attach_resid": round(float(resid), 9),
            "club_body_min_mm": round(dmin * 1000, 1),
            "club_body_at": where,
            "club_head_min_mm": round(hmin * 1000, 1),
            "club_body_face_overlaps": len(ov),
            "club_tip_world": [round(v, 4) for v in (M @ Vector((0, 0.97269, 0)))],
            "grip_world": [round(v, 4) for v in M.to_translation()]}


def local_rot(ev, bone):
    """effective parent-relative bone-space rotation of a DEF bone"""
    b = arm.bones[bone]
    par = b.parent
    if par is None:
        return (REST[bone].inverted() @ ev.pose.bones[bone].matrix)
    Mp = ev.pose.bones[par.name].matrix
    rel = REST[par.name].inverted() @ REST[bone]
    return (Mp @ rel).inverted() @ ev.pose.bones[bone].matrix


def euler_of(ev, bone, ctrl):
    m = local_rot(ev, bone).to_quaternion().to_matrix().to_4x4()
    e = m.to_euler(PB[ctrl].rotation_mode)
    return [round(math.degrees(v), 2) for v in e]


# ---------------------------------------------------------------- renderer
PROX = []
if not NORENDER:
    try:
        sc.render.engine = 'BLENDER_EEVEE_NEXT'
    except TypeError:
        sc.render.engine = 'BLENDER_EEVEE'
    sc.render.film_transparent = False
    sc.render.image_settings.file_format = 'PNG'
    sc.render.resolution_percentage = 100
    try:
        sc.eevee.taa_render_samples = 16
    except Exception:
        pass
    w = bpy.data.worlds.new("gw"); sc.world = w
    w.use_nodes = True
    nt = w.node_tree; nt.nodes.clear()
    o = nt.nodes.new("ShaderNodeOutputWorld"); bg = nt.nodes.new("ShaderNodeBackground")
    bg.inputs[0].default_value = (0.07, 0.07, 0.08, 1); bg.inputs[1].default_value = 1.0
    nt.links.new(bg.outputs[0], o.inputs[0])

    def emis(name, rgb, s=2.2):
        m = bpy.data.materials.new(name); m.use_nodes = True
        t = m.node_tree; t.nodes.clear()
        oo = t.nodes.new("ShaderNodeOutputMaterial"); e = t.nodes.new("ShaderNodeEmission")
        e.inputs[0].default_value = (*rgb, 1); e.inputs[1].default_value = s
        t.links.new(e.outputs[0], oo.inputs[0])
        return m

    COL = {"spine": emis("cs", (1.0, 0.85, 0.10)), "L": emis("cl", (0.15, 0.55, 1.0)),
           "R": emis("cr", (1.0, 0.18, 0.18)), "weap": emis("cw", (0.15, 1.0, 0.35)),
           "root": emis("cro", (1.0, 1.0, 1.0), 1.4)}
    bm = bpy.data.materials.new("xray"); bm.use_nodes = True
    t = bm.node_tree; t.nodes.clear()
    oo = t.nodes.new("ShaderNodeOutputMaterial"); mix = t.nodes.new("ShaderNodeMixShader")
    dif = t.nodes.new("ShaderNodeBsdfDiffuse"); dif.inputs[0].default_value = (0.7, 0.72, 0.62, 1)
    tra = t.nodes.new("ShaderNodeBsdfTransparent"); fr = t.nodes.new("ShaderNodeFresnel")
    fr.inputs[0].default_value = 1.3
    t.links.new(fr.outputs[0], mix.inputs[0]); t.links.new(tra.outputs[0], mix.inputs[1])
    t.links.new(dif.outputs[0], mix.inputs[2]); t.links.new(mix.outputs[0], oo.inputs[0])
    if hasattr(bm, "surface_render_method"):
        bm.surface_render_method = 'BLENDED'
    if hasattr(bm, "show_transparent_back"):
        bm.show_transparent_back = False
    lo.data.materials.clear(); lo.data.materials.append(bm)

    tmp = bpy.data.collections.new("TMP_PROXY"); sc.collection.children.link(tmp)

    def cyl(name, r, mat):
        m = bpy.data.meshes.new(name)
        N = 8
        v = [(r * math.cos(2 * math.pi * i / N), k, r * math.sin(2 * math.pi * i / N))
             for k in (0, 1) for i in range(N)]
        f = [(i, (i + 1) % N, N + (i + 1) % N, N + i) for i in range(N)]
        f.append(tuple(range(N - 1, -1, -1))); f.append(tuple(range(N, 2 * N)))
        m.from_pydata(v, [], f); m.update(); m.materials.append(mat)
        ob = bpy.data.objects.new(name, m); tmp.objects.link(ob)
        return ob

    def grp(n):
        if n.startswith("WEAPON"):
            return "weap"
        if n in ("ROOT", "COG"):
            return "root"
        return "L" if n.endswith("_L") else ("R" if n.endswith("_R") else "spine")

    DRAW = [n for n in arm.bones.keys() if not n.startswith(("MCH_", "WGT"))
            and not n.endswith("_CTRL") and "_FK_" not in n and "_IK_" not in n
            and "_POLE_" not in n]
    for n in DRAW:
        PROX.append((n, cyl("BX_" + n, max(0.010, min(0.022, LEN[n] * 0.08)), COL[grp(n)])))

    lamp = bpy.data.lights.new("gl", 'SUN'); lamp.energy = 3.0
    lob = bpy.data.objects.new("gl", lamp); sc.collection.objects.link(lob)
    lob.rotation_euler = (math.radians(55), 0, math.radians(30))
    cdat = bpy.data.cameras.new("gc"); cdat.type = 'ORTHO'
    gcam = bpy.data.objects.new("gc", cdat); sc.collection.objects.link(gcam)
    REFCAM = bpy.data.objects.get("REF_CAM")

SHOTS = []


def place_proxies(dg):
    ev = ev_rig(dg)
    for n, ob in PROX:
        pb = ev.pose.bones[n]
        a = rig.matrix_world @ pb.head
        b = rig.matrix_world @ pb.tail
        d = b - a
        q = d.to_track_quat('Y', 'Z')
        ob.matrix_world = (Matrix.Translation(a) @ q.to_matrix().to_4x4()
                           @ Matrix.Diagonal((1.0, d.length, 1.0, 1.0)))


def shoot(name, dg, view="ref", target=(0, 0.05, 0.95), scale=2.3, az=0, el=0):
    if NORENDER:
        return
    place_proxies(dg)
    if view == "ref" and REFCAM is not None:
        sc.camera = REFCAM
        sc.render.resolution_x = int(448 * RES / 576.0); sc.render.resolution_y = RES
    else:
        sc.camera = gcam
        sc.render.resolution_x = RES; sc.render.resolution_y = RES
        a, e = math.radians(az), math.radians(el)
        d = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)))
        ctr = Vector(target)
        gcam.location = ctr + d * 9.0
        cdat.ortho_scale = scale
        gcam.rotation_mode = 'XYZ'
        gcam.rotation_euler = (ctr - gcam.location).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = os.path.join(OUT, "g_%s.png" % name)
    bpy.ops.render.render(write_still=True)
    SHOTS.append(sc.render.filepath)


# ---------------------------------------------------------------- leg measure
def leg_row(dg):
    ev = ev_rig(dg)
    out = {}
    for S in ("L", "R"):
        aim = ev.pose.bones["MCH_LEGAIM_" + S].matrix
        top = ev.pose.bones["MCH_LEGTOP_" + S].matrix.to_translation()
        ft = ev.pose.bones["FOOT_" + S].matrix
        fi = ev.pose.bones["FOOT_IK_" + S].matrix
        th = ev.pose.bones["THIGH_" + S].matrix
        shn = ev.pose.bones["SHIN_" + S].matrix
        knee = math.degrees((shn.to_translation() - th.to_translation()).angle(
            (shn @ Vector((0, LEN["SHIN_" + S], 0))) - shn.to_translation()))
        out[S] = {"foot_err_m": round((ft.to_translation() - fi.to_translation()).length, 9),
                  "stretch": round(aim.to_scale()[1], 6),
                  "d_hip_ankle_m": round((top - aim.to_translation()).length, 6),
                  "knee_deg": round(knee, 4),
                  "hip_z": round(th.to_translation().z, 5)}
    return out


TESTS = {}
t0 = time.time()


def run(key, fn, render=None, legs=False, note=""):
    clear_pose()
    extra = fn() or {}
    dg = upd()
    row = {"note": note}
    row.update(club_metrics(dg))
    if legs:
        row["legs"] = leg_row(dg)
    row.update(extra if isinstance(extra, dict) else {})
    TESTS[key] = row
    if render:
        for v in render:
            shoot(key + "_" + v[0], dg, *v[1:])
    print("### %s %s" % (key, json.dumps(row)[:400]))
    return row


# ================================================================ 0 rest
run("rest", lambda: None, render=[("ref", "ref"), ("side", "aux", (0, 0.05, 0.95), 2.4, 90, 0)])

# ================================================================ 1 COG lateral
for tag, dx in (("cog_latP10", 0.10), ("cog_latM10", -0.10)):
    run(tag, (lambda dx=dx: wloc("COG_CTRL", (dx, 0, 0))), legs=True)

# ================================================================ 2 COG vertical
for tag, dz in (("cog_dn05", -0.05), ("cog_dn10", -0.10), ("cog_dn15", -0.15),
                ("cog_dn20", -0.20), ("cog_up03", 0.03)):
    run(tag, (lambda dz=dz: wloc("COG_CTRL", (0, 0, dz))), legs=True,
        render=[("legs", "aux", (0.0, 0.05, 0.32), 1.1, 28, -12)] if tag in
        ("cog_dn20", "cog_up03") else None)

# leg roll sweep (20 samples)
sweep = []
clear_pose(); dg = upd(); ev = ev_rig(dg)
u0 = {S: (ev.pose.bones["MCH_LEGAIM_" + S].matrix.to_3x3() @ Vector((0, 1, 0))).normalized()
      for S in ("L", "R")}
x0 = {S: (ev.pose.bones["MCH_LEGAIM_" + S].matrix.to_3x3() @ Vector((1, 0, 0))).normalized()
      for S in ("L", "R")}
for k in range(21):
    t = k / 20.0
    clear_pose()
    wloc("FOOT_IK_R", (-0.15 * t, 0, 0))
    wloc("COG_CTRL", (-0.05 * t, 0, -0.15 * t))
    wrot("CHEST_CTRL", (1, 0, 0), 25 * t)
    ev = ev_rig(upd())
    rowk = {"t": round(t, 3)}
    for S in ("L", "R"):
        m = ev.pose.bones["MCH_LEGAIM_" + S].matrix
        u = (m.to_3x3() @ Vector((0, 1, 0))).normalized()
        x = (m.to_3x3() @ Vector((1, 0, 0))).normalized()
        xp = (u0[S].rotation_difference(u) @ x0[S]).normalized()
        rowk["tilt_" + S] = round(math.degrees(u0[S].angle(u)), 3)
        rowk["twist_" + S] = round(math.degrees(math.atan2(xp.cross(x).dot(u), xp.dot(x))), 5)
    sweep.append(rowk)
R["roll_sweep"] = sweep
R["roll_max_twist_deg"] = {S: round(max(abs(r["twist_" + S]) for r in sweep), 5) for S in "LR"}
R["roll_max_step_deg"] = {S: round(max(abs(sweep[i + 1]["twist_" + S] - sweep[i]["twist_" + S])
                                       for i in range(20)), 5) for S in "LR"}

# ================================================================ 3 CHEST
for tag, ax, dg_ in (("chest_twistP45", (0, 0, 1), 45), ("chest_twistM45", (0, 0, 1), -45),
                     ("chest_fwd30", (1, 0, 0), -30), ("chest_side25", (0, 1, 0), 25)):
    run(tag, (lambda a=ax, d=dg_: wrot("CHEST_CTRL", a, d)),
        render=[("ref", "ref")] if tag in ("chest_twistP45", "chest_fwd30") else None)

# ================================================================ 4 ARM POSES
S_SH = REST["UPPERARM_R"].to_translation()
LU = LEN["UPPERARM_R"]; LF = LEN["FOREARM_R"]
GRIP_OFF = (REST["WEAPON"].to_translation() - REST["HAND_R"].to_translation()).length


def weapon_matrix(G, u, roll_deg):
    u = Vector(u).normalized()
    hint = Vector((0, 0, 1)) if abs(u.z) < 0.9 else Vector((0, 1, 0))
    z = (hint - u * hint.dot(u)).normalized()
    z = (Quaternion(u, math.radians(roll_deg)) @ z).normalized()
    x = u.cross(z).normalized()
    M = Matrix(((x.x, u.x, z.x, G[0]), (x.y, u.y, z.y, G[1]),
                (x.z, u.z, z.z, G[2]), (0, 0, 0, 1)))
    return M


ARM_TARGETS = {
    # idle (f_0001): upper arm down/out 45 deg, elbow flexed ~45, forearm horizontal out,
    # club vertical head-up on the character's RIGHT
    "idle": (None, (0, 0, 1)),
    "windup": ((-0.48, 0.19, 1.55), (0.10, 0.87, 0.48)),
    "impact": ((-0.21, -0.36, 0.90), (0.73, 0.06, -0.65)),
}
E_IDLE = S_SH + LU * Vector((-0.7071, 0, -0.7071))
W_IDLE = E_IDLE + LF * Vector((-1, 0, 0))
ARM_TARGETS["idle"] = (tuple(W_IDLE + GRIP_OFF * Vector((-1, 0, 0))), (0, 0, 1))

ARM = {}
for name, (G, u) in ARM_TARGETS.items():
    best = None
    for roll in range(0, 360, 5):
        clear_pose()
        PB["HAND_IK_R"]["ik_fk"] = 1.0
        PB["HAND_IK_R"].matrix = weapon_matrix(G, u, roll) @ K_HAND.inverted()
        dg = upd(); ev = ev_rig(dg)
        reach = (ev.pose.bones["HAND_R"].matrix.to_translation()
                 - ev.pose.bones["HAND_IK_R"].matrix.to_translation()).length
        fy = Vector(ev.pose.bones["FOREARM_R"].matrix.to_3x3().col[1]).normalized()
        hy = Vector(ev.pose.bones["HAND_R"].matrix.to_3x3().col[1]).normalized()
        wdev = math.degrees(fy.angle(hy))
        hl = local_rot(ev, "HAND_R").to_quaternion()
        tw = abs(math.degrees(2 * math.atan2(hl.y * (1 if hl.w >= 0 else -1), abs(hl.w))))
        score = wdev + 0.4 * tw + reach * 2000
        if best is None or score < best[0]:
            best = (score, roll, reach, wdev)
    score, roll, reach, wdev = best
    clear_pose()
    PB["HAND_IK_R"]["ik_fk"] = 1.0
    PB["HAND_IK_R"].matrix = weapon_matrix(G, u, roll) @ K_HAND.inverted()
    dg = upd(); ev = ev_rig(dg)
    eul = {b: euler_of(ev, "%s_R" % b, "%s_FK_R" % b) for b in
           ("UPPERARM", "FOREARM", "HAND")}
    fy = Vector(ev.pose.bones["FOREARM_R"].matrix.to_3x3().col[1]).normalized()
    hy = Vector(ev.pose.bones["HAND_R"].matrix.to_3x3().col[1]).normalized()
    wbend = math.degrees(fy.angle(hy))
    hl = local_rot(ev, "HAND_R").to_quaternion()
    wtwist = math.degrees(2 * math.atan2(hl.y * (1 if hl.w >= 0 else -1), abs(hl.w)))
    flex = math.degrees(Vector(ev.pose.bones["UPPERARM_R"].matrix.to_3x3().col[1]).angle(
        Vector(ev.pose.bones["FOREARM_R"].matrix.to_3x3().col[1])))
    ik_club = club_world(dg).copy()
    ik_metrics = club_metrics(dg)
    # --- replay as pure FK
    clear_pose()
    for b in ("UPPERARM", "FOREARM", "HAND"):
        PB["%s_FK_R" % b].rotation_euler = [math.radians(v) for v in eul[b]]
    dg = upd()
    fk_club = club_world(dg)
    fkdev = max(abs(fk_club[i][j] - ik_club[i][j]) for i in range(4) for j in range(4))
    row = {"note": "IK-solved then replayed as FK", "roll_deg": roll,
           "ik_reach_err_m": round(reach, 6),
           "wrist_bend_deg": round(wbend, 2), "wrist_twist_deg": round(wtwist, 2),
           "elbow_flex_deg": round(flex, 2), "fk_euler_deg": eul,
           "fk_vs_ik_club_dev": round(float(fkdev), 7)}
    row.update(club_metrics(dg))
    ARM[name] = row
    TESTS["arm_" + name] = row
    shoot("arm_" + name + "_ref", dg, "ref")
    shoot("arm_" + name + "_34", dg, "aux", (-0.25, 0.05, 1.15), 2.6, 40, 8)
    print("### arm_%s %s" % (name, json.dumps(row)[:400]))

# arms down (pure FK)
def _down():
    wrot("UPPERARM_FK_L", (0, 1, 0), 70)
    wrot("UPPERARM_FK_R", (0, 1, 0), -70)
    ev = ev_rig(upd())
    return {"fk_euler_deg": {"UPPERARM_L": [round(math.degrees(v), 2) for v in
                                            PB["UPPERARM_FK_L"].rotation_euler],
                             "UPPERARM_R": [round(math.degrees(v), 2) for v in
                                            PB["UPPERARM_FK_R"].rotation_euler]},
            "handL_z": round(ev.pose.bones["HAND_L"].matrix.to_translation().z, 4),
            "handR_z": round(ev.pose.bones["HAND_R"].matrix.to_translation().z, 4)}


run("arms_down", _down, render=[("ref", "ref")], note="both arms down ~70 deg from T")

# ================================================================ 5 elbow sign + IK sweep
def _elbow():
    lrot("FOREARM_FK_L", (1, 0, 0), 90)
    lrot("FOREARM_FK_R", (1, 0, 0), 90)
    ev = ev_rig(upd())
    o = {}
    for S in ("L", "R"):
        w = ev.pose.bones["HAND_" + S].matrix.to_translation()
        o["wrist_" + S + "_dy"] = round(w.y - REST["HAND_" + S].to_translation().y, 4)
        o["flex_" + S] = round(math.degrees(
            Vector(ev.pose.bones["UPPERARM_" + S].matrix.to_3x3().col[1]).angle(
                Vector(ev.pose.bones["FOREARM_" + S].matrix.to_3x3().col[1]))), 2)
    o["sign_ok"] = o["wrist_L_dy"] < -0.05 and o["wrist_R_dy"] < -0.05
    return o


run("elbow_fk90", _elbow, render=[("top", "aux", (0, 0.05, 1.26), 2.2, 0, 85)],
    note="+90 deg local X on both forearms must flex forward (-Y) on both sides")

# IK sweep windup -> impact
Gw, uw = ARM_TARGETS["windup"]
Gi, ui = ARM_TARGETS["impact"]
Mw = weapon_matrix(Gw, uw, ARM["windup"]["roll_deg"]) @ K_HAND.inverted()
Mi = weapon_matrix(Gi, ui, ARM["impact"]["roll_deg"]) @ K_HAND.inverted()
qw, qi = Mw.to_quaternion(), Mi.to_quaternion()
pw, pi = Mw.to_translation(), Mi.to_translation()
sw = []
prev_el = None
for k in range(20):
    t = k / 19.0
    clear_pose()
    PB["HAND_IK_R"]["ik_fk"] = 1.0
    PB["HAND_IK_R"].matrix = (Matrix.Translation(pw.lerp(pi, t))
                              @ qw.slerp(qi, t).to_matrix().to_4x4())
    ev = ev_rig(upd())
    el = ev.pose.bones["FOREARM_R"].matrix.to_translation()
    rerr = (ev.pose.bones["HAND_R"].matrix.to_translation()
            - ev.pose.bones["HAND_IK_R"].matrix.to_translation()).length
    jump = 0.0 if prev_el is None else (el - prev_el).length
    prev_el = el
    sw.append({"t": round(t, 3), "reach_err_m": round(rerr, 6),
               "elbow": [round(v, 4) for v in el], "elbow_step_m": round(jump, 5)})
R["ik_sweep"] = sw
R["ik_sweep_max_reach_err_m"] = max(s["reach_err_m"] for s in sw)
R["ik_sweep_max_elbow_step_m"] = max(s["elbow_step_m"] for s in sw)

# ================================================================ 6 HIPS
for tag, ax, d in (("hips_yawP30", (0, 0, 1), 30), ("hips_yawM30", (0, 0, 1), -30),
                   ("hips_rollP10", (0, 1, 0), 10), ("hips_rollM10", (0, 1, 0), -10)):
    run(tag, (lambda a=ax, dd=d: wrot("HIPS_CTRL", a, dd)), legs=True,
        render=[("legs", "aux", (0, 0.05, 0.35), 1.2, 28, -12)] if tag == "hips_yawP30" else None)

# ================================================================ 7 right foot step
run("stepR_air", lambda: wloc("FOOT_IK_R", (-0.15, 0, 0.10)), legs=True,
    render=[("legs", "aux", (0, 0.05, 0.32), 1.25, 20, -10)], note="R foot up .10 out .15")


def _wide():
    wloc("FOOT_IK_R", (-0.15, 0, 0))
    wloc("COG_CTRL", (0, 0, -0.12))


run("stanceR_wide", _wide, legs=True,
    render=[("legs", "aux", (0, 0.05, 0.32), 1.25, 20, -10)], note="wide plant + COG -0.12")

# ================================================================ 8 HEAD
def _head(a, dd):
    def f():
        wrot("HEAD_CTRL", a, dd)
        ev = ev_rig(upd())
        o = {}
        for b in ("NECK", "HEAD"):
            q = ev.pose.bones[b].matrix.to_quaternion().rotation_difference(
                REST[b].to_quaternion())
            o[b + "_world_rot_deg"] = round(math.degrees(abs(q.angle)), 3)
        o["head_top_world"] = [round(v, 4) for v in
                               (ev.pose.bones["HEAD"].matrix @ Vector((0, LEN["HEAD"], 0)))]
        return o
    return f


for tag, ax, d in (("head_yawP40", (0, 0, 1), 40), ("head_yawM40", (0, 0, 1), -40),
                   ("head_pitchP30", (1, 0, 0), 30), ("head_pitchM30", (1, 0, 0), -30)):
    run(tag, _head(ax, d), render=[("ref", "ref")] if tag == "head_pitchP30" else None)

# ================================================================ 9 WEAPON_CTRL
clear_pose(); dg = upd()
hand0 = ev_rig(dg).pose.bones["HAND_R"].matrix.to_translation().copy()
tip0 = Vector(club_metrics(dg)["club_tip_world"])
wc = {}
for ax, i in (("X", 0), ("Y", 1), ("Z", 2)):
    for d in (20, -20):
        clear_pose()
        e = [0, 0, 0]; e[i] = math.radians(d)
        PB["WEAPON_CTRL"].rotation_euler = e
        dg = upd()
        m = club_metrics(dg)
        hand = ev_rig(dg).pose.bones["HAND_R"].matrix.to_translation()
        wc["%s%+d" % (ax, d)] = {"tip_move_m": round((Vector(m["club_tip_world"]) - tip0).length, 5),
                                 "hand_move_m": round((hand - hand0).length, 9),
                                 "attach_resid": m["attach_resid"],
                                 "club_body_min_mm": m["club_body_min_mm"]}
R["weapon_ctrl"] = wc

# ================================================================ restore
clear_pose()
dg = upd(); ev = ev_rig(dg)
rows = [(n, (ev.pose.bones[n].matrix.to_translation() - REST[n].to_translation()).length,
         math.degrees(abs(ev.pose.bones[n].matrix.to_quaternion().rotation_difference(
             REST[n].to_quaternion()).angle))) for n in DEF]
R["final_rest_max_pos_m"] = max(r[1] for r in rows)
R["final_rest_max_rot_deg"] = max(r[2] for r in rows)
R["final_club_dev"] = float(max(abs(club_world(dg)[i][j] - CM0[i][j])
                                for i in range(4) for j in range(4)))
R["residual_posed_bones"] = [pb.name for pb in PB if pb.location.length > 1e-9 or
                             (Vector(pb.scale) - Vector((1, 1, 1))).length > 1e-9]
R["has_action"] = bool(rig.animation_data and rig.animation_data.action)
R["n_actions"] = len(bpy.data.actions)
R["tests"] = TESTS
R["arm_poses"] = ARM
R["renders"] = SHOTS
R["seconds"] = round(time.time() - t0, 1)
with open(os.path.join(T, "inspect", "step02_gate.json"), "w") as f:
    json.dump(R, f, indent=1, default=str)
print("@@@GATE_DONE@@@", R["seconds"])
