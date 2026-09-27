"""s08_export_fbx.py - T35 (G8.1-G8.6, G9.3 axes): FBX export of the goblin rig (preset candidate A).

Input : the work blend given with -Blend (rig/gob_r05_rigtest.blend; opened by s07b.build_export, never saved),
        rig/data/canonical_skeleton.json (read only)
Output: rig/export/stage_rest.blend, rig/export/stage_rigtest.blend (rebuilt with s07b_export_rig.build_export),
        rig/export/goblin.fbx          stage_rest    : armature `goblin` + goblin_mesh, no animation
        rig/export/goblin@rigtest.fbx  stage_rigtest : armature `goblin` + goblin_mesh + the active action `rigtest`
        rig/export/goblin_club.fbx     goblin_club alone, object transform identity (mesh data = grip frame)
        rig/data/export_preset.json    export settings, Blender -> FBX / Unity axis mapping and its derivation

CLI:  bl.ps1 -Script s08_export_fbx.py -Blend gob_r05_rigtest.blend

- Renames happen in memory on the opened stage (never saved): GOB_export -> `goblin`; the scene -> `rigtest` for the
  clip file, since the active-action path (bake_anim_use_all_actions False, bake_anim_use_nla_strips False) names its
  single AnimStack after the scene.
- goblin_club: unparented, matrix_parent_inverse / matrix_basis identity, so the file carries the mesh in its local
  (grip) frame; the log shows the stage club world matrix vs the weapon_socket_r rest frame it must match in Unity.
- Self-check (stdout only, not a gate checker): file sizes, FBX header / node summary, and one reimport per file with
  factory settings (automatic_bone_orientation False, manual orientation = the preset axes): node list, armature and
  mesh scale, bone count / names, `_end` leaves, rigtest keyed frames.
"""
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402
import s07b_export_rig as s07b  # noqa: E402

import bpy  # noqa: E402
from bpy_extras.io_utils import axis_conversion  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

STAGE_REST = goblib.EXPORT / "stage_rest.blend"
STAGE_RIGTEST = goblib.EXPORT / "stage_rigtest.blend"
FBX_BODY = goblib.EXPORT / "goblin.fbx"
FBX_CLIP = goblib.EXPORT / "goblin@rigtest.fbx"
FBX_CLUB = goblib.EXPORT / "goblin_club.fbx"
PRESET_JSON = "export_preset.json"
SRC_NAME = "gob_r05_rigtest.blend"
CLIP = "rigtest"
PELVIS_FRAMES = (1, 49, 100, 300, 583)

EXPORT = "GOB_export"
ARM_OUT = "goblin"
MESH_OUT = "goblin_mesh"
CLUB_OUT = "goblin_club"
SOCKET = "weapon_socket_r"

CANDIDATE = "A"
AXIS_FORWARD, AXIS_UP = "-Z", "Y"
PRESET = {  # candidate A (plan T35); candidate B = bake_space_transform True
    "bake_space_transform": False,
    "apply_scale_options": "FBX_SCALE_ALL",
    "apply_unit_scale": True,
    "global_scale": 1.0,
}
COMMON = {
    "use_selection": True,
    "object_types": ["ARMATURE", "MESH"],
    "use_space_transform": True,
    "axis_forward": AXIS_FORWARD,
    "axis_up": AXIS_UP,
    "add_leaf_bones": False,
    "primary_bone_axis": "Y",
    "secondary_bone_axis": "X",
    "armature_nodetype": "NULL",
    "use_armature_deform_only": False,
    "mesh_smooth_type": "OFF",
    "use_mesh_modifiers": True,
    "use_tspace": False,
    "use_triangles": False,
    "use_custom_props": False,
    "path_mode": "AUTO",
    "embed_textures": False,
    "bake_anim_use_all_bones": True,
    "bake_anim_use_nla_strips": False,
    "bake_anim_use_all_actions": False,
    "bake_anim_force_startend_keying": True,
    "bake_anim_step": 1.0,
    "bake_anim_simplify_factor": 0.0,
}
IMPORT = {  # self-check reimport (axes mapped back from the preset)
    "use_manual_orientation": True,
    "axis_forward": AXIS_FORWARD,
    "axis_up": AXIS_UP,
    "global_scale": 1.0,
    "bake_space_transform": PRESET["bake_space_transform"],
    "automatic_bone_orientation": False,
    "ignore_leaf_bones": False,
    "use_custom_normals": True,
    "use_anim": True,
    "use_custom_props": False,
}
# Unity's FBX importer: right-handed Y-up FBX space -> left-handed Unity space by negating X.
FBX_TO_UNITY = Matrix(((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))


def log(msg):
    print(f"[s08] {msg}")
    sys.stdout.flush()


def m3(m):
    return [[round(float(m[i][j]), 9) + 0.0 for j in range(3)] for i in range(3)]


def v3(v, nd=6):
    return [round(float(x), nd) + 0.0 for x in v]


def select_only(objs):
    vl = bpy.context.view_layer
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for o in vl.objects:
        o.select_set(False)
    for o in objs:
        o.select_set(True)
    vl.objects.active = objs[0]
    return sorted(o.name for o in bpy.context.selected_objects)


def export_fbx(path, objs, bake_anim):
    sel = select_only(objs)
    kw = dict(COMMON, **PRESET)
    kw["object_types"] = set(kw["object_types"])
    kw["bake_anim"] = bake_anim
    res = bpy.ops.export_scene.fbx(filepath=str(path), **kw)
    if res != {"FINISHED"}:
        raise RuntimeError(f"export_scene.fbx {path.name} returned {res}")
    log(f"EXPORT {path.name}: selected {sel}, bake_anim {bake_anim}, scene {bpy.context.scene.name!r} frames "
        f"{bpy.context.scene.frame_start}..{bpy.context.scene.frame_end} -> {res}")


def open_stage(path):
    bpy.ops.wm.open_mainfile(filepath=str(path), load_ui=False)
    arm = bpy.data.objects[EXPORT]
    arm.name = ARM_OUT
    if arm.name != ARM_OUT:
        raise RuntimeError(f"rename {EXPORT} -> {ARM_OUT} gave {arm.name!r}")
    mesh = bpy.data.objects[MESH_OUT]
    return arm, mesh


def scene_units():
    us = bpy.context.scene.unit_settings
    return {"system": us.system, "scale_length": us.scale_length,
            "fbx_unit_scale_factor_expected": 100.0 * us.scale_length if us.system != "NONE" else 100.0}


# ---------------------------------------------------------------- FBX file summary (raw header / nodes)
def _child(e, id_):
    return next((c for c in e.elems if c.id == id_), None) if e is not None else None


def _p70(e):
    p = _child(e, b"Properties70")
    out = {}
    if p is not None:
        for c in p.elems:
            if c.id == b"P":
                out[c.props[0].decode()] = list(c.props[4:])
    return out


def _name(e):
    return e.props[1].split(b"\x00\x01")[0].decode()


def fbx_summary(path):
    from io_scene_fbx import parse_fbx
    root, ver = parse_fbx.parse(str(path))
    gs = _p70(_child(root, b"GlobalSettings"))
    hdr = {k: (gs[k][0] if k in gs and gs[k] else None) for k in
           ("UpAxis", "UpAxisSign", "FrontAxis", "FrontAxisSign", "CoordAxis", "CoordAxisSign",
            "UnitScaleFactor", "OriginalUnitScaleFactor", "TimeMode", "CustomFrameRate")}
    objs = _child(root, b"Objects")
    nodes, limbs, stacks = [], [], []
    for e in objs.elems if objs is not None else []:
        if e.id == b"Model":
            kind = e.props[2].decode()
            if kind == "LimbNode":
                limbs.append(_name(e))
            else:
                p = _p70(e)
                nodes.append({"name": _name(e), "type": kind,
                              "lcl_translation": v3(p.get("Lcl Translation", [0, 0, 0])[:3]),
                              "lcl_rotation_deg": v3(p.get("Lcl Rotation", [0, 0, 0])[:3]),
                              "lcl_scaling": v3(p.get("Lcl Scaling", [1, 1, 1])[:3])})
        elif e.id == b"AnimationStack":
            stacks.append(_name(e))
    return {"fbx_version": ver, "header": hdr, "nodes": nodes, "limb_nodes": len(limbs),
            "limb_end_nodes": [n for n in limbs if n.endswith("_end")], "anim_stacks": stacks}


# ---------------------------------------------------------------- self-check reimport
def action_fcurves(act):
    out = []
    for layer in act.layers:
        for strip in layer.strips:
            for cb in strip.channelbags:
                out.extend(cb.fcurves)
    return out


def reimport(path, canon_names, canon_parent):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    res = bpy.ops.import_scene.fbx(filepath=str(path), **IMPORT)
    log(f"REIMPORT {path.name}: import_scene.fbx {res}")
    for o in sorted(bpy.data.objects, key=lambda o: o.name):
        loc, rot, sc = o.matrix_world.decompose()
        log(f"  node {o.name!r} ({o.type}) parent {o.parent.name if o.parent else None!r}: world loc "
            f"{v3(loc * 1000.0, 4)} mm, rot {v3([math.degrees(a) for a in rot.to_euler()], 4)} deg, scale "
            f"{v3(sc, 7)}; local scale {v3(o.scale, 7)}")
    for o in bpy.data.objects:
        if o.type == "ARMATURE":
            names = [b.name for b in o.data.bones]
            ends = [n for n in names if n.endswith("_end")]
            par_ok = all((o.data.bones[n].parent.name if o.data.bones[n].parent else None) == canon_parent.get(n)
                         for n in names if n in canon_parent)
            log(f"  armature {o.name!r}: {len(names)} bones, name set == canonical {set(names) == set(canon_names)}, "
                f"parents == canonical {par_ok}, missing {sorted(set(canon_names) - set(names))}, extra "
                f"{sorted(set(names) - set(canon_names))}, _end leaves {ends}")
            ad = o.animation_data
            log(f"  armature {o.name!r} animation_data.action "
                f"{(ad.action.name if ad and ad.action else None)!r}, slot "
                f"{(ad.action_slot.identifier if ad and ad.action_slot else None)!r}")
        elif o.type == "MESH":
            me = o.data
            tris = sum(len(p.vertices) - 2 for p in me.polygons)
            log(f"  mesh {o.name!r}: verts {len(me.vertices)}, polys {len(me.polygons)}, tris {tris}, custom normals "
                f"{me.has_custom_normals}, vertex groups {len(o.vertex_groups)}, modifiers "
                f"{[(m.type, m.object.name if getattr(m, 'object', None) else None) for m in o.modifiers]}; HR2 shape "
                f"keys {[kb.name for kb in me.shape_keys.key_blocks] if me.shape_keys else []}, materials "
                f"{[m.name if m else None for m in me.materials]}")
    for a in bpy.data.actions:
        fcs = action_fcurves(a)
        keys = sorted({round(k.co.x, 3) for fc in fcs for k in fc.keyframe_points})
        log(f"  action {a.name!r}: slots {[s.identifier for s in a.slots]}, fcurves {len(fcs)}, frame_range "
            f"{v3(a.frame_range, 3)}, distinct keyed frames {len(keys)}"
            + (f" ({keys[0]}..{keys[-1]})" if keys else ""))
    log(f"  actions {len(bpy.data.actions)}; scene fps {bpy.context.scene.render.fps}, frames "
        f"{bpy.context.scene.frame_start}..{bpy.context.scene.frame_end}")


def pelvis_check(stage_pelvis, stage_f0):
    """T35c: reimported rigtest pelvis world position vs stage, as imported and after use_connect False.

    The Blender FBX importer sizes each bone to the mean distance of its children and connects a child whose head
    lands on that tail (import_fbx.build_skeleton / child_connect); a connected bone ignores its location keys.
    """
    g = bpy.data.objects[ARM_OUT]
    act = g.animation_data.action
    off = int(round(act.frame_range[0])) - stage_f0
    scene = bpy.context.scene

    def errs():
        out = []
        for f, w in stage_pelvis.items():
            scene.frame_set(f + off)
            out.append((f, round(((g.matrix_world @ g.pose.bones["pelvis"].matrix).translation - w).length * 1000.0, 4)))
        return out

    connected = [b.name for b in g.data.bones if b.use_connect]
    e_imp = errs()
    bpy.context.view_layer.objects.active = g
    bpy.ops.object.mode_set(mode="EDIT")
    for eb in g.data.edit_bones:
        eb.use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")
    e_dis = errs()
    log(f"PELVIS SELFCHECK {FBX_CLIP.name} (reimport frame = stage frame + {off}): pelvis world error mm "
        f"[stage frame, mm] as imported {e_imp} (reimported use_connect True: {connected}); after use_connect False "
        f"on reimported bones {e_dis}")


# ---------------------------------------------------------------- main
def main():
    if not bpy.data.filepath:
        raise RuntimeError("no source blend open (use bl.ps1 -Blend gob_r05_rigtest.blend)")
    src = Path(bpy.data.filepath).resolve()
    if src.name != SRC_NAME:
        raise RuntimeError(f"expected -Blend {SRC_NAME}, got {src.name}")
    canon = goblib.load_json("canonical_skeleton.json")
    canon_names = [b["name"] for b in canon["bones"]]
    canon_parent = {b["name"]: b["parent"] for b in canon["bones"]}
    canon_head = {b["name"]: Vector(b["head"]) for b in canon["bones"]}

    # 1. fresh stage blends (the work blend is only opened by build_export, never saved)
    s07b.build_export(src, None, STAGE_REST)
    s07b.build_export(src, CLIP, STAGE_RIGTEST)

    # 2. goblin.fbx + goblin_club.fbx from stage_rest
    arm, mesh = open_stage(STAGE_REST)
    units = scene_units()
    log(f"STAGE {STAGE_REST.name}: objects {[(o.name, o.type) for o in bpy.data.objects]}, scene units {units}, "
        f"{ARM_OUT} matrix_world identity {arm.matrix_world == Matrix.Identity(4)}, {MESH_OUT} modifiers "
        f"{[(m.type, getattr(getattr(m, 'object', None), 'name', None)) for m in mesh.modifiers]}, custom normals "
        f"{mesh.data.has_custom_normals}")
    export_fbx(FBX_BODY, [arm, mesh], bake_anim=False)

    club = bpy.data.objects[CLUB_OUT]
    bpy.context.view_layer.update()
    sock_w = arm.matrix_world @ arm.pose.bones[SOCKET].matrix
    sock_rest = arm.matrix_world @ arm.data.bones[SOCKET].matrix_local
    cp, cr = s07b.mdiff(club.matrix_world, sock_w)
    rp, rr = s07b.mdiff(sock_w, sock_rest)
    cs = club.matrix_world.to_scale()
    log(f"CLUB stage {CLUB_OUT} world vs {SOCKET} pose (head frame): {cp:.4f} mm / {cr:.4f} deg; socket pose vs rest "
        f"{rp:.4f} mm / {rr:.4f} deg; club world scale {v3(cs, 7)}; parent {club.parent.name if club.parent else None} "
        f"{club.parent_type} {club.parent_bone}; modifiers {[m.type for m in club.modifiers]}, vertex groups "
        f"{len(club.vertex_groups)}")
    # T35b (G8.6 rev.): mesh data re-expressed in the weapon_socket_r rest frame (in memory, stage never saved)
    club_w = club.matrix_world.copy()
    me = club.data
    club_rest_world_co = [club_w @ v.co for v in me.vertices]  # Blender rest club, world
    to_sock = sock_rest.inverted() @ club_w
    nmat = to_sock.to_3x3().inverted().transposed()
    want_n = [(nmat @ Vector(cn.vector)).normalized() for cn in me.corner_normals]
    club.parent = None
    club.matrix_parent_inverse = Matrix.Identity(4)
    club.matrix_basis = Matrix.Identity(4)
    me.transform(to_sock)
    me.normals_split_custom_set([tuple(n) for n in want_n])
    me.update()
    bpy.context.view_layer.update()
    n_err = max(math.degrees(n.angle(Vector(cn.vector), 0.0)) for n, cn in zip(want_n, me.corner_normals))
    v_err = max(((sock_rest @ v.co) - w).length for v, w in zip(me.vertices, club_rest_world_co)) * 1000.0
    log(f"CLUB for export: frame weapon_socket_r rest; parent {club.parent}, matrix_world identity "
        f"{club.matrix_world == Matrix.Identity(4)}; mesh transform inv(socket_rest_world) @ club_world; "
        f"socket_rest @ v vs rest club world max {v_err:.6f} mm; custom normals {me.has_custom_normals}, corner "
        f"normals vs transformed originals max {n_err:.4f} deg ({len(want_n)} corners)")
    export_fbx(FBX_CLUB, [club], bake_anim=False)

    # 3. goblin@rigtest.fbx from stage_rigtest (active action only)
    arm, mesh = open_stage(STAGE_RIGTEST)
    scene = bpy.context.scene
    old_scene = scene.name
    scene.name = CLIP
    ad = arm.animation_data
    act = ad.action if ad else None
    log(f"STAGE {STAGE_RIGTEST.name}: scene {old_scene!r} -> {scene.name!r} (AnimStack name), fps {scene.render.fps}, "
        f"frames {scene.frame_start}..{scene.frame_end}; {ARM_OUT} action {act.name if act else None!r} slot "
        f"{ad.action_slot.identifier if ad and ad.action_slot else None!r} frame_range "
        f"{v3(act.frame_range, 3) if act else None}; actions in file {[a.name for a in bpy.data.actions]}; NLA tracks "
        f"{len(ad.nla_tracks) if ad else 0}")
    if act is None or act.name != CLIP:
        raise RuntimeError(f"stage_rigtest: {ARM_OUT} active action is {act.name if act else None!r}, expected {CLIP!r}")
    f_cur = scene.frame_current
    stage_pelvis = {}  # T35c self-check reference: stage pelvis world position
    for f in PELVIS_FRAMES:
        scene.frame_set(f)
        stage_pelvis[f] = (arm.matrix_world @ arm.pose.bones["pelvis"].matrix).translation.copy()
    scene.frame_set(f_cur)
    stage_f0 = scene.frame_start
    export_fbx(FBX_CLIP, [arm, mesh], bake_anim=True)

    # 4. raw file summaries + export_preset.json
    files = {FBX_BODY: ("export/stage_rest.blend", [ARM_OUT, MESH_OUT], False),
             FBX_CLUB: ("export/stage_rest.blend", [CLUB_OUT], False),
             FBX_CLIP: ("export/stage_rigtest.blend", [ARM_OUT, MESH_OUT], True)}
    summ = {}
    for p in (FBX_BODY, FBX_CLIP, FBX_CLUB):
        summ[p.name] = fbx_summary(p)
        s = summ[p.name]
        log(f"FILE {p.name}: {p.stat().st_size} bytes; FBX {s['fbx_version']}; header {s['header']}; nodes "
            f"{s['nodes']}; limb nodes {s['limb_nodes']}, _end {s['limb_end_nodes']}; anim stacks {s['anim_stacks']}")

    b2f = axis_conversion(from_forward="Y", from_up="Z", to_forward=AXIS_FORWARD, to_up=AXIS_UP)
    b2u = FBX_TO_UNITY @ b2f
    expected = Matrix(((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0)))
    if max(abs(b2u[i][j] - expected[i][j]) for i in range(3) for j in range(3)) > 1e-9:
        raise RuntimeError(f"blender_to_unity {m3(b2u)} != expected {m3(expected)}")

    def ang(a, b):
        return math.degrees(a.normalized().angle(b.normalized()))

    up_v = canon_head["head"] - canon_head["root"]
    lr_v = canon_head["hand_l"] - canon_head["hand_r"]
    front_v = Vector((0.0, -1.0, 0.0))
    p = Vector((1.0, 2.0, 3.0))
    checks = [
        {"what": "arbitrary point (1,2,3) m", "blender": v3(p), "fbx": v3(b2f @ p), "unity": v3(b2u @ p)},
        {"what": "character front (Blender -Y)", "blender": v3(front_v), "unity": v3(b2u @ front_v),
         "expected_unity": [0, 0, 1], "angle_deg": round(ang(b2u @ front_v, Vector((0, 0, 1))), 6)},
        {"what": "root->head bone heads (canonical), G9.3", "blender": v3(up_v), "unity": v3(b2u @ up_v),
         "expected_unity_dir": [0, 1, 0], "angle_deg": round(ang(b2u @ up_v, Vector((0, 1, 0))), 4)},
        {"what": "hand_r->hand_l bone heads (canonical), G9.3", "blender": v3(lr_v), "unity": v3(b2u @ lr_v),
         "expected_unity_dir": [-1, 0, 0], "angle_deg": round(ang(b2u @ lr_v, Vector((-1, 0, 0))), 4)},
    ]
    for c in checks:
        log(f"AXIS check {c}")
    rha = {"up_front_coord (axis, sign)": None}
    try:
        from io_scene_fbx.fbx_utils import RIGHT_HAND_AXES
        rha = {"up_front_coord (axis, sign)": [list(t) for t in RIGHT_HAND_AXES[(AXIS_UP, AXIS_FORWARD)]]}
    except Exception as e:  # noqa: BLE001
        rha["error"] = repr(e)

    preset = {
        "_doc": ("T35 FBX export preset (candidate A). export_scene_fbx = keyword arguments passed to "
                 "bpy.ops.export_scene.fbx for every file (bake_anim per file). axis_mapping matrices act on column "
                 "vectors: v_out = M @ v_blender (rows = output axes)."),
        "candidate": CANDIDATE,
        "generated_by": "scripts/s08_export_fbx.py",
        "blender_version": bpy.app.version_string,
        "export_scene_fbx": dict(COMMON, **PRESET),
        "files": {p.name: {"source_stage": src_, "objects": objs, "bake_anim": ba}
                  for p, (src_, objs, ba) in files.items()},
        "renames_in_memory": {"armature_object": f"{EXPORT} -> {ARM_OUT}", "mesh_object": MESH_OUT,
                              "scene_for_clip": f"-> {CLIP} (AnimStack name of the active-action export)",
                              "club": (f"{CLUB_OUT}: unparented, object matrix identity, mesh data and custom normals "
                                       f"transformed by inv(socket_rest_world) @ club_world")},
        "club_frame": "weapon_socket_r rest",
        "scene_units": units,
        "axis_mapping": {
            "blender_to_fbx": m3(b2f),
            "blender_to_unity": m3(b2u),
            "unity_from_blender": "unity.x = -blender.x, unity.y = blender.z, unity.z = -blender.y",
            "derivation": [
                ("blender_to_fbx = bpy_extras.io_utils.axis_conversion(from_forward='Y', from_up='Z', "
                 f"to_forward='{AXIS_FORWARD}', to_up='{AXIS_UP}'): the global_matrix io_scene_fbx ExportFBX.execute "
                 "builds when use_space_transform is True. With bake_space_transform False it is applied to root "
                 "object transforms only (mesh / bone data stay in Blender axes) and the header records the axis "
                 "system (see fbx_files.*.header)."),
                ("blender_to_unity = diag(-1, 1, 1) @ blender_to_fbx: Unity converts the right-handed Y-up FBX space "
                 "to its left-handed space by negating X. Not measurable in Blender; G9.3 confirms it."),
            ],
            "fbx_header_expected": rha,
            "check_points": checks,
        },
        "fbx_files": summ,
        "blender_reimport": {"import_scene_fbx": IMPORT,
                             "note": "reimport maps the preset axes back (Blender space); factory settings"},
    }
    out = goblib.save_json(PRESET_JSON, preset)
    log(f"PRESET {out}")

    # 5. self-check reimport (factory settings, once per file)
    for p in (FBX_BODY, FBX_CLIP, FBX_CLUB):
        log(f"SIZE {p.name} {p.stat().st_size} bytes")
        reimport(p, canon_names, canon_parent)
        if p == FBX_CLIP:
            pelvis_check(stage_pelvis, stage_f0)
    ob = bpy.data.objects[CLUB_OUT]  # last reimport = goblin_club.fbx
    got = [sock_rest @ (ob.matrix_world @ v.co) for v in ob.data.vertices]
    from mathutils.kdtree import KDTree
    kd = KDTree(len(club_rest_world_co))
    for i, co in enumerate(club_rest_world_co):
        kd.insert(co, i)
    kd.balance()
    near = max(kd.find(g)[2] for g in got) * 1000.0
    idx = (max((g - w).length for g, w in zip(got, club_rest_world_co)) * 1000.0
           if len(got) == len(club_rest_world_co) else None)
    log(f"CLUB SELFCHECK reimport {CLUB_OUT}: socket_rest_world @ (matrix_world @ v) vs Blender rest club world "
        f"vertices: count {len(got)} / {len(club_rest_world_co)}, max index-wise {idx if idx is None else round(idx, 6)} "
        f"mm, max nearest {near:.6f} mm (threshold 0.5 mm)")
    log("DONE")


if __name__ == "__main__":
    goblib.run_main(main)
