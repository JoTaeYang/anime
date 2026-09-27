"""p24d_ctrl_weapon.py - T240 (P2.5): weapon controls (right + left) and the left hand IK weapon slot.

Input : work/player/rig/work/pl_r04c.blend (p24a-c)
Output: work/player/rig/work/pl_r04d.blend (save copy), work/player/rig/data/ctrl_manifest.json (owner "p24d" entries,
        fills the weapon slot of spaces.hand_ik_space_l)

CLI:  blender --background --factory-startup work/player/rig/work/pl_r04c.blend --python p24d_ctrl_weapon.py

Copied / adapted from work/goblin_swing/rig/scripts/s06d_ctrl_weapon.py (T27: weapon space MCH + Armature space
constraints, DEF socket Copy Transforms, static bone-graph cycle check, depsgraph output capture), goblin files unchanged.
Per side X (R, L): MCH_CTRL_weapon_x_space (no parent, rest = DEF WeaponSocket_X) > CTRL_weapon_x (rest = the socket).
  Space constraints on the MCH (Armature, influence = driver PROPS["weapon_space_x"] == k):
    space_hand -> DEF Hand_X (0; the actual FK / IK hand), space_torso -> CTRL_torso (1), space_world -> MCH_world (2).
  DEF WeaponSocket_X keeps parent Hand_X and copies CTRL_weapon_x (Copy Transforms WORLD/WORLD).
Left hand IK: MCH_CTRL_hand_ik_l_space gets space_weapon (Armature -> CTRL_weapon_r, influence = driver
  PROPS["hand_ik_space_l"] == 3). The right hand IK has no weapon space and CTRL_weapon_l follows DEF Hand_L, so the
  graph has no cycle (relations exist even at influence 0, playbook pitfall).
HeadEquipmentSocket, BackSocket, BackWeaponSocket: plain DEF children (no control).
Self-checks: static bone graph cycle search; depsgraph output captured during the first evaluation ("cycle" lines);
  G6.2 DEF vs canonical rest.
"""
import ctypes
import json
import os
import sys
import tempfile
from pathlib import Path

import bpy

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
IN_BLEND = RIG / "work" / "pl_r04c.blend"
OUT_BLEND = RIG / "work" / "pl_r04d.blend"
MANIFEST = RIG / "data" / "ctrl_manifest.json"
CANON = RIG / "data" / "canonical_skeleton.json"
OWNER = "p24d"
ARM = "PL_rig"
SIDES = ("r", "l")
HAND_L_SPACE_MCH = "MCH_CTRL_hand_ik_l_space"
HAND_L_PROP = "hand_ik_space_l"
HAND_L_WEAPON_VALUE = 3
HAND_L_WEAPON_CON = "space_weapon"
HAND_L_WEAPON_TARGET = "CTRL_weapon_r"


def names(x):
    S = x.upper()
    return {"ctrl": f"CTRL_weapon_{x}", "mch": f"MCH_CTRL_weapon_{x}_space", "socket": f"WeaponSocket_{S}",
            "prop": f"weapon_space_{x}",
            "spaces": [("hand", "space_hand", f"Hand_{S}"), ("torso", "space_torso", "CTRL_torso"),
                       ("world", "space_world", "MCH_world")]}


DOC = {
    "p24d_weapons": "CTRL_weapon_r / CTRL_weapon_l (p24d): rest = DEF WeaponSocket_R / _L (head = fist centre, local "
                    "Y along the hand), child of MCH_CTRL_weapon_x_space (no parent) whose Armature constraints "
                    "space_hand -> DEF Hand_X, space_torso -> CTRL_torso, space_world -> MCH_world have influence = "
                    "driver PROPS[\"weapon_space_x\"] == k. DEF WeaponSocket_X keeps parent Hand_X and copies "
                    "CTRL_weapon_x WORLD/WORLD (socket offset animation). Left hand IK weapon space follows "
                    "CTRL_weapon_r (two-handed grip on the right weapon); the right hand IK has no weapon space (cycle "
                    "free). HeadEquipmentSocket / BackSocket / BackWeaponSocket have no controls.",
}


def edit_bones(arm):
    vl = bpy.context.view_layer
    vl.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    ebs = arm.data.edit_bones
    for x in SIDES:
        n = names(x)
        for b in (n["ctrl"], n["mch"]):
            if ebs.get(b) is not None:
                raise RuntimeError(f"bone {b} already exists (expected input pl_r04c.blend)")
        s = ebs[n["socket"]]
        for name, parent in ((n["mch"], None), (n["ctrl"], n["mch"])):
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
    for x in SIDES:
        n = names(x)
        pb = pbs[n["ctrl"]]
        pb.rotation_mode = "XYZ"
        pb.lock_scale = (True, True, True)
        cur = props[n["prop"]]
        for k, (_, cname, tgt) in enumerate(n["spaces"]):
            c = armature_con(arm, pbs[n["mch"]], cname, tgt, 1.0 if cur == k else 0.0)
            drivers[f"{n['mch']}.{cname}"] = add_driver(arm, c, n["prop"], f"1.0 if v == {k} else 0.0")
        c = pbs[n["socket"]].constraints.new("COPY_TRANSFORMS")
        c.name = "CTRL_copy"
        c.target = arm
        c.subtarget = n["ctrl"]
        c.target_space = "WORLD"
        c.owner_space = "WORLD"
        c.mix_mode = "REPLACE"
    hl = pbs[HAND_L_SPACE_MCH]
    if hl.constraints.get(HAND_L_WEAPON_CON) is not None:
        raise RuntimeError(f"{HAND_L_SPACE_MCH}.{HAND_L_WEAPON_CON} already exists")
    c = armature_con(arm, hl, HAND_L_WEAPON_CON, HAND_L_WEAPON_TARGET,
                     1.0 if props[HAND_L_PROP] == HAND_L_WEAPON_VALUE else 0.0)
    drivers[f"{HAND_L_SPACE_MCH}.{HAND_L_WEAPON_CON}"] = add_driver(arm, c, HAND_L_PROP,
                                                                    f"1.0 if v == {HAND_L_WEAPON_VALUE} else 0.0")
    return drivers


def assign_collections(arm):
    ad = arm.data
    colls = {c.name: c for c in ad.collections_all}
    for x in SIDES:
        n = names(x)
        for name, coll in ((n["ctrl"], "CTRL"), (n["mch"], "MCH")):
            b = ad.bones[name]
            for c in list(b.collections):
                c.unassign(b)
            colls[coll].assign(b)


# ---------------------------------------------------------------- cycle checks (s06d)
def bone_graph(arm):
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


def write_manifest():
    old = json.loads(MANIFEST.read_text(encoding="utf-8"))
    man = dict(old)
    man["_doc"] = dict(old.get("_doc", {}))
    man["_doc"].update(DOC)
    for sec in ("controls", "spaces"):
        man[sec] = {k: v for k, v in old.get(sec, {}).items() if not (isinstance(v, dict) and v.get("owner") == OWNER)}
    for x in SIDES:
        n = names(x)
        man["controls"][n["ctrl"]] = {
            "owner": OWNER, "side": x.upper(), "part": "weapon", "rotation_mode": "XYZ", "drives": n["socket"],
            "sweep": [{"channel": "loc", "axis": a, "min": -0.3, "max": 0.3} for a in "XYZ"]
            + [{"channel": "rot", "axis": a, "min": -90, "max": 90} for a in "XYZ"]}
        cons = [s[1] for s in n["spaces"]]
        man["spaces"][n["prop"]] = {
            "owner": OWNER, "control": n["ctrl"], "prop": n["prop"], "values": [s[0] for s in n["spaces"]],
            "mch": n["mch"], "constraints": cons, "constraint_type": {c: "ARMATURE" for c in cons},
            "targets": {s[1]: s[2] for s in n["spaces"]},
            "influence": {s[1]: f"driver PROPS[prop] == {k}" for k, s in enumerate(n["spaces"])},
            "value_constraint": {s[0]: s[1] for s in n["spaces"]},
            "value_effect": {s[0]: f"full follow of {s[2]}" for s in n["spaces"]},
            "def_follow": {"bone": n["socket"], "constraint": "CTRL_copy", "type": "COPY_TRANSFORMS",
                           "space": "WORLD/WORLD", "parent": f"Hand_{x.upper()}"}}
    hl = dict(man["spaces"][HAND_L_PROP])
    if HAND_L_WEAPON_CON not in hl["constraints"]:
        hl["constraints"] = list(hl["constraints"]) + [HAND_L_WEAPON_CON]
    hl["constraint_type"] = dict(hl["constraint_type"], **{HAND_L_WEAPON_CON: "ARMATURE"})
    hl["targets"] = dict(hl["targets"], **{HAND_L_WEAPON_CON: HAND_L_WEAPON_TARGET})
    hl["influence"] = dict(hl["influence"], **{HAND_L_WEAPON_CON: f"driver PROPS[prop] == {HAND_L_WEAPON_VALUE}"})
    hl["value_constraint"] = dict(hl["value_constraint"], weapon=HAND_L_WEAPON_CON)
    hl["value_effect"] = dict(hl["value_effect"], weapon=f"full follow of {HAND_L_WEAPON_TARGET}")
    hl.pop("empty_slots", None)
    hl["filled_slots"] = {"weapon": {"value": HAND_L_WEAPON_VALUE, "filled": True, "by": OWNER,
                                     "constraint": HAND_L_WEAPON_CON, "target": HAND_L_WEAPON_TARGET}}
    man["spaces"][HAND_L_PROP] = hl
    MANIFEST.write_text(json.dumps(man, indent=1) + "\n", encoding="utf-8")
    return MANIFEST


def def_vs_canon(arm, canon):
    bpy.context.view_layer.update()
    worst, wn = 0.0, None
    for b in canon["bones"]:
        m = arm.pose.bones[b["name"]].matrix
        r = b["rest_matrix"]
        e = max(abs(m[i][j] - r[i][j]) for i in range(4) for j in range(4))
        if e > worst:
            worst, wn = e, b["name"]
    return worst, wn


def log(msg):
    print(f"[p24d] {msg}")
    sys.stdout.flush()


def main():
    if not bpy.data.filepath or Path(bpy.data.filepath).resolve() != IN_BLEND.resolve():
        raise RuntimeError(f"open {IN_BLEND} as the main file (got {bpy.data.filepath!r})")
    arm = bpy.data.objects[ARM]
    canon = json.loads(CANON.read_text(encoding="utf-8"))
    edit_bones(arm)
    drivers = pose_setup(arm)
    assign_collections(arm)
    arm.data.pose_position = "POSE"
    captured = evaluate_capturing()
    cyc_lines = [ln for ln in captured.splitlines() if "cycle" in ln.lower()]
    log(f"DEPSGRAPH captured output during the first evaluation: {len(captured.splitlines())} lines, lines with "
        f"'cycle': {len(cyc_lines)}" + (f" -> {cyc_lines[:5]}" if cyc_lines else ""))
    g, ndrv = bone_graph(arm)
    cyc = find_cycle(g)
    log(f"CYCLE STATIC CHECK bone graph: {len(g)} bones, {sum(len(v) for v in g.values())} edges ({ndrv} driver "
        f"variable edges): " + ("no cycle" if cyc is None else f"CYCLE {' -> '.join(cyc)}"))
    for tag, drv in drivers.items():
        log(f"DRIVER {tag}: expr '{drv.expression}', valid={drv.is_valid}")
    worst, wn = def_vs_canon(arm, canon)
    log(f"G6.2 SELFCHECK (defaults) DEF vs canonical rest {worst:.3e} ({wn}), threshold 1e-4")
    out = write_manifest()
    log(f"MANIFEST {out}")
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT_BLEND), copy=True)
    log(f"OUTPUT {OUT_BLEND}; bones {len(arm.data.bones)}; collections " + ", ".join(
        f"{c.name}(visible={c.is_visible}, bones={len(c.bones)})" for c in arm.data.collections_all))
    log("DONE")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
