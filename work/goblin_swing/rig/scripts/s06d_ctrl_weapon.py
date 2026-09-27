"""s06d_ctrl_weapon.py - T27: weapon control + left hand IK weapon slot (G6.1, G6.4 weapon / left hand weapon, G6.8).

Input : rig/work/r04c.blend (s06a torso/head/PROPS, s06b arms, s06c legs)
Output: rig/work/r04d.blend (save copy, save_version 0), rig/data/ctrl_manifest.json (adds owner "s06d" entries and
        fills the weapon slot of spaces.hand_ik_space_l)

Run:  bl.ps1 -Script s06d_ctrl_weapon.py -Blend work/r04c.blend

New bones (use_deform False):
  MCH_CTRL_weapon_space (no parent, rest = DEF weapon_socket_r) > CTRL_weapon (rest = DEF weapon_socket_r)
  MCH_CTRL_weapon_space: Armature constraint per weapon_space value, influence = driver PROPS["weapon_space"] == k:
     space_hand_r -> DEF hand_r (0), space_hand_l -> CTRL_hand_fk_l (1, spec 5: DEF hand_l would close a cycle with
     the left hand IK weapon space), space_torso -> CTRL_torso (2), space_world -> MCH_world (3).
DEF weapon_socket_r: parent stays hand_r; Copy Transforms WORLD/WORLD <- CTRL_weapon (GOB_club stays bone-parented).
Left hand IK: MCH_CTRL_hand_ik_l_space gets space_weapon (Armature -> CTRL_weapon, influence = driver
  PROPS["hand_ik_space_l"] == 3), filling the slot T25 left empty.
Self-checks: static bone dependency graph (parents, constraint targets, IK chains, driver variables) has no cycle;
  depsgraph "Dependency cycle" output captured during the first evaluation; G6.2 DEF vs canonical rest.
"""
import ctypes
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goblib  # noqa: E402

import bpy  # noqa: E402

OUT_BLEND = goblib.WORK / "r04d.blend"
MANIFEST = "ctrl_manifest.json"
OWNER = "s06d"
ARM = "GOB_rig"
CLUB = "GOB_club"
CTRL = "CTRL_weapon"
SPACE_MCH = "MCH_CTRL_weapon_space"
SOCKET = "weapon_socket_r"
PROP = "weapon_space"
WEAPON_SPACES = [("hand_r", "space_hand_r", "hand_r"), ("hand_l", "space_hand_l", "CTRL_hand_fk_l"),
                 ("torso", "space_torso", "CTRL_torso"), ("world", "space_world", "MCH_world")]
HAND_L_SPACE_MCH = "MCH_CTRL_hand_ik_l_space"
HAND_L_PROP = "hand_ik_space_l"
HAND_L_WEAPON_VALUE = 3
HAND_L_WEAPON_CON = "space_weapon"
SIDE = "R"

DOC = {
    "s06d_weapon": "CTRL_weapon (s06d, T27): rest = DEF weapon_socket_r rest (head = grip_r, local Y = club axis), "
                   "child of MCH_CTRL_weapon_space (no parent) whose Armature constraints space_hand_r -> DEF hand_r "
                   "(actual IK/FK hand; the right hand IK has no weapon space, so no cycle), space_hand_l -> "
                   "CTRL_hand_fk_l (spec 5), space_torso -> CTRL_torso, space_world -> MCH_world (same target as the "
                   "head and hand IK world spaces) have influence = driver PROPS[\"weapon_space\"] == k. DEF "
                   "weapon_socket_r keeps parent hand_r and copies CTRL_weapon WORLD/WORLD (weapon socket offset "
                   "animation after bake); GOB_club stays bone-parented to weapon_socket_r. controls.CTRL_weapon.side "
                   "= R (the right hand's weapon; R colour).",
    "s06d_ui_conditions": "For the T28 UI / switch operator: weapon_space = hand_l follows CTRL_hand_fk_l, so it only "
                          "makes sense with the left arm in FK (arm_ik_fk_l = 0); combining it with hand_ik_space_l = "
                          "weapon (left hand IK following CTRL_weapon) is meaningless (the weapon would follow the "
                          "left FK hand while the IK hand follows the weapon).",
}


# ---------------------------------------------------------------- build
def edit_bones(arm):
    vl = bpy.context.view_layer
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    for n in (CTRL, SPACE_MCH):
        if ebs.get(n) is not None:
            raise RuntimeError(f"bone {n} already exists (expected input work/r04c.blend)")
    for _, _, tgt in WEAPON_SPACES:
        if ebs.get(tgt) is None:
            raise RuntimeError(f"space target {tgt} missing")
    s = ebs[SOCKET]
    for name, parent in ((SPACE_MCH, None), (CTRL, SPACE_MCH)):
        eb = ebs.new(name)
        eb.head, eb.tail, eb.roll = s.head.copy(), s.tail.copy(), s.roll
        eb.use_deform = False
        eb.use_connect = False
        eb.parent = ebs[parent] if parent else None
    bpy.ops.object.mode_set(mode="OBJECT")


def add_driver(arm, con, prop, expr):
    fc = con.driver_add("influence")
    for m in list(fc.modifiers):
        fc.modifiers.remove(m)
    drv = fc.driver
    drv.type = "SCRIPTED"
    v = drv.variables.new()
    v.name = "v"
    v.type = "SINGLE_PROP"
    v.targets[0].id_type = "OBJECT"
    v.targets[0].id = arm
    v.targets[0].data_path = f'pose.bones["PROPS"]["{prop}"]'
    drv.expression = expr
    return drv


def armature_con(arm, pb, name, target, influence):
    c = pb.constraints.new("ARMATURE")
    c.name = name
    c.use_deform_preserve_volume = False
    c.use_bone_envelopes = False
    c.use_current_location = False
    t = c.targets.new()
    t.target = arm
    t.subtarget = target
    t.weight = 1.0
    c.influence = influence
    return c


def pose_setup(arm):
    pbs = arm.pose.bones
    props = pbs["PROPS"]
    drivers = {}
    pb = pbs[CTRL]
    pb.rotation_mode = "XYZ"
    pb.lock_scale = (True, True, True)
    cur = props[PROP]
    sp = pbs[SPACE_MCH]
    for k, (_, cname, tgt) in enumerate(WEAPON_SPACES):
        c = armature_con(arm, sp, cname, tgt, 1.0 if cur == k else 0.0)
        drivers[f"{SPACE_MCH}.{cname}"] = add_driver(arm, c, PROP, f"1.0 if v == {k} else 0.0")
    c = pbs[SOCKET].constraints.new("COPY_TRANSFORMS")
    c.name = "CTRL_copy"
    c.target = arm
    c.subtarget = CTRL
    c.target_space = "WORLD"
    c.owner_space = "WORLD"
    c.mix_mode = "REPLACE"
    hl = pbs[HAND_L_SPACE_MCH]
    if hl.constraints.get(HAND_L_WEAPON_CON) is not None:
        raise RuntimeError(f"{HAND_L_SPACE_MCH}.{HAND_L_WEAPON_CON} already exists")
    c = armature_con(arm, hl, HAND_L_WEAPON_CON, CTRL, 1.0 if props[HAND_L_PROP] == HAND_L_WEAPON_VALUE else 0.0)
    drivers[f"{HAND_L_SPACE_MCH}.{HAND_L_WEAPON_CON}"] = add_driver(arm, c, HAND_L_PROP,
                                                                    f"1.0 if v == {HAND_L_WEAPON_VALUE} else 0.0")
    return drivers


def assign_collections(arm):
    ad = arm.data
    colls = {c.name: c for c in ad.collections_all}
    for name, coll in ((CTRL, "CTRL"), (SPACE_MCH, "MCH")):
        b = ad.bones[name]
        for c in list(b.collections):
            c.unassign(b)
        colls[coll].assign(b)


# ---------------------------------------------------------------- cycle checks
def bone_graph(arm):
    """bone -> set of bones it depends on (parent, constraint targets incl. Armature targets and IK pole, IK chain
    members depend on the IK target / pole, driver owners depend on the driver variable bones)."""
    pbs = arm.pose.bones
    g = {pb.name: set() for pb in pbs}
    for pb in pbs:
        if pb.parent is not None:
            g[pb.name].add(pb.parent.name)
        for c in pb.constraints:
            deps = set()
            if getattr(c, "target", None) == arm and getattr(c, "subtarget", ""):
                deps.add(c.subtarget)
            if c.type == "ARMATURE":
                deps |= {t.subtarget for t in c.targets if t.target == arm and t.subtarget}
            if c.type == "IK" and c.pole_target == arm and c.pole_subtarget:
                deps.add(c.pole_subtarget)
            g[pb.name] |= deps
            if c.type == "IK":
                chain, b = [], pb
                for _ in range(c.chain_count or 1):
                    if b is None:
                        break
                    chain.append(b.name)
                    b = b.parent
                for m in chain:
                    g[m] |= deps
    ad = arm.animation_data
    ndrv = 0
    if ad is not None:
        for fc in ad.drivers:
            path = fc.data_path
            if not path.startswith('pose.bones["'):
                continue
            owner = path.split('"')[1]
            for v in fc.driver.variables:
                for t in v.targets:
                    tp = t.data_path or ""
                    if t.id == arm and tp.startswith('pose.bones["'):
                        g[owner].add(tp.split('"')[1])
                        ndrv += 1
    return g, ndrv


def find_cycle(g):
    color = {n: 0 for n in g}
    stack = []

    def dfs(n):
        color[n] = 1
        stack.append(n)
        for m in g[n]:
            if m not in color:
                continue
            if color[m] == 1:
                return stack[stack.index(m):] + [m]
            if color[m] == 0:
                r = dfs(m)
                if r:
                    return r
        stack.pop()
        color[n] = 2
        return None

    sys.setrecursionlimit(10000)
    for n in g:
        if color[n] == 0:
            r = dfs(n)
            if r:
                return r
    return None


def _crt_flush():
    for lib in ("ucrtbase", "msvcrt"):
        try:
            getattr(ctypes.cdll, lib).fflush(None)
        except Exception:
            pass


def evaluate_capturing():
    """First depsgraph evaluation after the new relations, with C-level stdout/stderr captured."""
    sys.stdout.flush()
    sys.stderr.flush()
    _crt_flush()
    tmp = tempfile.TemporaryFile()
    saved = {fd: os.dup(fd) for fd in (1, 2)}
    try:
        for fd in (1, 2):
            os.dup2(tmp.fileno(), fd)
        bpy.context.view_layer.update()
        bpy.context.evaluated_depsgraph_get().update()
        _crt_flush()
    finally:
        for fd, s in saved.items():
            os.dup2(s, fd)
            os.close(s)
    tmp.seek(0)
    text = tmp.read().decode("utf-8", "replace")
    tmp.close()
    return text


# ---------------------------------------------------------------- manifest
def write_manifest():
    p = goblib.DATA / MANIFEST
    with open(p, "r", encoding="utf-8") as f:
        old = json.load(f)
    man = dict(old)
    man["_doc"] = dict(old.get("_doc", {}))
    man["_doc"].update(DOC)
    for sec in ("controls", "spaces"):
        man[sec] = {k: v for k, v in old.get(sec, {}).items()
                    if not (isinstance(v, dict) and v.get("owner") == OWNER)}
    man["controls"][CTRL] = {"owner": OWNER, "side": SIDE, "part": "weapon", "rotation_mode": "XYZ",
                             "sweep": [{"channel": "loc", "axis": a, "min": -0.3, "max": 0.3} for a in "XYZ"]
                             + [{"channel": "rot", "axis": a, "min": -90, "max": 90} for a in "XYZ"]}
    cons = [s[1] for s in WEAPON_SPACES]
    man["spaces"][PROP] = {
        "owner": OWNER, "control": CTRL, "prop": PROP, "values": [s[0] for s in WEAPON_SPACES], "mch": SPACE_MCH,
        "constraints": cons, "constraint_type": {c: "ARMATURE" for c in cons},
        "targets": {s[1]: s[2] for s in WEAPON_SPACES},
        "influence": {s[1]: f"driver PROPS[prop] == {k}" for k, s in enumerate(WEAPON_SPACES)},
        "value_constraint": {s[0]: s[1] for s in WEAPON_SPACES},
        "value_effect": {s[0]: f"full follow of {s[2]}" for s in WEAPON_SPACES},
        "requires": {"hand_l": {"arm_ik_fk_l": 0}},
        "def_follow": {"bone": SOCKET, "constraint": "CTRL_copy", "type": "COPY_TRANSFORMS",
                       "space": "WORLD/WORLD", "parent": "hand_r"}}
    hl = man["spaces"].get(HAND_L_PROP)
    if hl is None:
        raise RuntimeError(f"manifest spaces.{HAND_L_PROP} (s06b) missing")
    hl = dict(hl)
    if HAND_L_WEAPON_CON not in hl["constraints"]:
        hl["constraints"] = list(hl["constraints"]) + [HAND_L_WEAPON_CON]
    hl["constraint_type"] = dict(hl["constraint_type"], **{HAND_L_WEAPON_CON: "ARMATURE"})
    hl["targets"] = dict(hl["targets"], **{HAND_L_WEAPON_CON: CTRL})
    hl["influence"] = dict(hl["influence"], **{HAND_L_WEAPON_CON: f"driver PROPS[prop] == {HAND_L_WEAPON_VALUE}"})
    hl["value_constraint"] = dict(hl["value_constraint"], weapon=HAND_L_WEAPON_CON)
    hl["value_effect"] = dict(hl["value_effect"], weapon=f"full follow of {CTRL}")
    slots = dict(hl.get("empty_slots", {}))
    slots.pop("weapon", None)
    if slots:
        hl["empty_slots"] = slots
    else:
        hl.pop("empty_slots", None)
    hl["filled_slots"] = {"weapon": {"value": HAND_L_WEAPON_VALUE, "filled": True, "by": OWNER,
                                     "constraint": HAND_L_WEAPON_CON, "target": CTRL}}
    hl["requires"] = {"weapon": {"not": {PROP: 1}}}
    man["spaces"][HAND_L_PROP] = hl
    return goblib.save_json(MANIFEST, man)


# ---------------------------------------------------------------- self-checks / report
def fmt_v(v):
    return "(" + ", ".join(f"{a:+.4f}" for a in v) + ")"


def g62(arm, canon):
    bpy.context.view_layer.update()
    worst, wn = 0.0, None
    for b in canon["bones"]:
        m = arm.pose.bones[b["name"]].matrix
        r = b["rest_matrix"]
        e = max(abs(m[i][j] - r[i][j]) for i in range(4) for j in range(4))
        if e > worst:
            worst, wn = e, b["name"]
    sm = arm.pose.bones[SOCKET].matrix
    sr = next(b["rest_matrix"] for b in canon["bones"] if b["name"] == SOCKET)
    se = max(abs(sm[i][j] - sr[i][j]) for i in range(4) for j in range(4))
    return worst, wn, se


def report(arm, drivers):
    ad = arm.data
    for name in (SPACE_MCH, CTRL, SOCKET, HAND_L_SPACE_MCH):
        b = ad.bones[name]
        pb = arm.pose.bones[name]
        cons = []
        for c in pb.constraints:
            if c.type == "ARMATURE":
                cons.append(f"ARMATURE:{c.name}->{','.join(t.subtarget for t in c.targets)} infl {c.influence:.3f}")
            else:
                cons.append(f"{c.type}:{c.name}->{getattr(c, 'subtarget', '')} infl {c.influence:.3f}")
        print(f"[s06d] BONE {name}: parent={b.parent.name if b.parent else None}, head={fmt_v(b.head_local)}, "
              f"tail={fmt_v(b.tail_local)}, roll-matrix z_axis={fmt_v(b.matrix_local.col[2][:3])}, "
              f"deform={b.use_deform}, coll={[c.name for c in b.collections]}, rot_mode={pb.rotation_mode}, "
              f"lock_scale={tuple(pb.lock_scale)}, constraints={'; '.join(cons) or '-'}")
    for tag, drv in drivers.items():
        print(f"[s06d] DRIVER {tag}: expr '{drv.expression}', valid={drv.is_valid}, simple={drv.is_simple_expression}")
    club = bpy.data.objects[CLUB]
    print(f"[s06d] CLUB {CLUB}: parent={club.parent.name if club.parent else None}/{club.parent_type}/"
          f"{club.parent_bone}")


def main():
    print(f"[s06d] Blender {bpy.app.version_string}")
    arm = bpy.data.objects[ARM]
    canon = goblib.load_json("canonical_skeleton.json")
    edit_bones(arm)
    drivers = pose_setup(arm)
    assign_collections(arm)
    arm.data.pose_position = "POSE"
    captured = evaluate_capturing()
    cyc_lines = [ln for ln in captured.splitlines() if "cycle" in ln.lower()]
    print(f"[s06d] DEPSGRAPH captured output during first evaluation: {len(captured.splitlines())} lines, lines with "
          f"'cycle': {len(cyc_lines)}" + (f" -> {cyc_lines[:5]}" if cyc_lines else ""))
    g, ndrv = bone_graph(arm)
    cyc = find_cycle(g)
    print(f"[s06d] CYCLE STATIC CHECK bone graph: {len(g)} bones, {sum(len(v) for v in g.values())} dependency edges "
          f"(parents, constraint / Armature / IK pole targets, IK chains, {ndrv} driver variable edges): "
          + ("no cycle" if cyc is None else f"CYCLE {' -> '.join(cyc)}"))
    report(arm, drivers)
    worst, wn, se = g62(arm, canon)
    props = arm.pose.bones["PROPS"]
    print(f"[s06d] G6.2 SELFCHECK (defaults: weapon_space={props[PROP]}, hand_ik_space_l={props[HAND_L_PROP]}, "
          f"arm_ik_fk_l/r={props['arm_ik_fk_l']}/{props['arm_ik_fk_r']}, leg_ik_fk_l/r={props['leg_ik_fk_l']}/"
          f"{props['leg_ik_fk_r']}, all CTRL identity): DEF vs canonical rest max abs diff = {worst:.3e} (bone {wn}); "
          f"weapon_socket_r = {se:.3e}; threshold 1e-4")
    nondef = [n for n in (CTRL, SPACE_MCH) if arm.data.bones[n].use_deform]
    print(f"[s06d] new bones with use_deform True: {nondef}; total bones={len(arm.data.bones)}; collections: "
          + ", ".join(f"{c.name}(visible={c.is_visible}, bones={len(c.bones)})" for c in arm.data.collections_all))
    out = write_manifest()
    print(f"[s06d] MANIFEST {out}")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    print(f"[s06d] OUTPUT {OUT_BLEND}")
    print(f"[s06d] G6.2 max_err={worst:.3e}; cycle static={'none' if cyc is None else 'FOUND'}, depsgraph cycle "
          f"lines={len(cyc_lines)}")


if __name__ == "__main__":
    goblib.run_main(main)
