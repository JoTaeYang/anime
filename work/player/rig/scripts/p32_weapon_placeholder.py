"""p32_weapon_placeholder.py - T310 (Sword_Idle SI1, spec work/player/d-04-player-sword-idle.md section 1): a preview-only
placeholder sword for the weapon socket contract (work/player/rig/data/weapon_socket_contract.json).

Output: work/player/rig/weapons/placeholder_sword.blend with one object WPN_sword_placeholder (mesh, 4 flat materials).
        Never part of the stage / FBX (p26 / p31 do not read it); p30 appends it for the Blender sheets only.

CLI:  blender -b --factory-startup --python p32_weapon_placeholder.py

Object space = the contract's socket space (pivot = grip centre = socket head, identity object transform under a
Child Of constraint without inverse): +Z = blade direction (grip -> tip), Y = guard span, X = normal of the blade flat.
Parts (low poly, flat shaded; sizes in m): pommel sphere r 0.014 at z -0.088; grip octagonal cylinder r 0.014 from z -0.080
to +0.080 (the ball fist has r ~0.075, so the grip shows below it and the guard sits on it); guard box 0.13 (Y) x 0.024 (X)
x 0.022 (Z) centred at z +0.088; blade 0.060 (Y) x 0.009 (X) from z +0.099 to +0.284, tapering to the tip at z +0.338.
Total length (pommel bottom -> tip) = 0.440.
"""
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
OUT = RIG / "weapons" / "placeholder_sword.blend"
NAME = "WPN_sword_placeholder"
COLOURS = {"WPN_blade": (0.78, 0.80, 0.84, 1.0), "WPN_guard": (0.28, 0.26, 0.26, 1.0),
           "WPN_grip": (0.42, 0.25, 0.12, 1.0), "WPN_pommel": (0.22, 0.20, 0.20, 1.0)}


def log(msg):
    print(f"[p32] {msg}")
    sys.stdout.flush()


def add_box(bm, center, size, mat):
    r = bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation(Vector(center)) @ Matrix.Diagonal(Vector(size + (1.0,))))
    for f in {f for v in r["verts"] for f in v.link_faces}:
        f.material_index = mat


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    me = bpy.data.meshes.new(NAME)
    ob = bpy.data.objects.new(NAME, me)
    bpy.context.scene.collection.objects.link(ob)
    mats = list(COLOURS)
    for n in mats:
        m = bpy.data.materials.new(n)
        m.diffuse_color = COLOURS[n]
        if m.node_tree is not None:   # T313: the FBX exporter reads the Principled Base Color (Unity preview colours)
            for nd in m.node_tree.nodes:
                if nd.type == "BSDF_PRINCIPLED":
                    nd.inputs["Base Color"].default_value = COLOURS[n]
        me.materials.append(m)
    bm = bmesh.new()
    # pommel
    r = bmesh.ops.create_uvsphere(bm, u_segments=8, v_segments=6, radius=0.014,
                                  matrix=Matrix.Translation((0.0, 0.0, -0.088)))
    for f in {f for v in r["verts"] for f in v.link_faces}:
        f.material_index = mats.index("WPN_pommel")
    # grip
    r = bmesh.ops.create_cone(bm, cap_ends=True, segments=8, radius1=0.014, radius2=0.014, depth=0.160,
                              matrix=Matrix.Translation((0.0, 0.0, 0.0)))
    for f in {f for v in r["verts"] for f in v.link_faces}:
        f.material_index = mats.index("WPN_grip")
    # guard (spans Y)
    add_box(bm, (0.0, 0.0, 0.088), (0.024, 0.130, 0.022), mats.index("WPN_guard"))
    # blade: flat hexagonal section (edges along +-Y), straight to z 0.275, tip at 0.324
    hw, ht = 0.030, 0.0045
    sec = [(0.0, -hw), (ht, -hw * 0.5), (ht, hw * 0.5), (0.0, hw), (-ht, hw * 0.5), (-ht, -hw * 0.5)]
    lo = [bm.verts.new((x, y, 0.099)) for x, y in sec]
    hi = [bm.verts.new((x, y, 0.284)) for x, y in sec]
    tip = bm.verts.new((0.0, 0.0, 0.338))
    faces = [bm.faces.new(lo[::-1])]
    for i in range(6):
        j = (i + 1) % 6
        faces.append(bm.faces.new((lo[i], lo[j], hi[j], hi[i])))
        faces.append(bm.faces.new((hi[i], hi[j], tip)))
    for f in faces:
        f.material_index = mats.index("WPN_blade")
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.to_mesh(me)
    bm.free()
    for p in me.polygons:
        p.use_smooth = False
    zs = [v.co.z for v in me.vertices]
    ys = [v.co.y for v in me.vertices]
    xs = [v.co.x for v in me.vertices]
    log(f"{NAME}: {len(me.vertices)} verts, {len(me.polygons)} faces, tris "
        f"{sum(len(p.vertices) - 2 for p in me.polygons)}; z {min(zs):.4f}..{max(zs):.4f} (length {max(zs) - min(zs):.4f} m), "
        f"y {min(ys):.4f}..{max(ys):.4f}, x {min(xs):.4f}..{max(xs):.4f}; materials {mats}")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT))
    log(f"OUTPUT {OUT}")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.exit(1)
