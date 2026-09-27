"""Convert a kimodo.cpp SOMA-30 animation into a BVH clip.

kimodo writes two raw streams beside each generated animation:
  root_positions.f32        frames * 3 float32, metres, Y-up
  local_rotations_xyzw.f32  frames * 30 * 4 float32, parent-local quaternions

BVH is the shortest path into Blender (native importer) and from there onto a
Mixamo-style rig, so this rewrites those streams against the published SOMA
skeleton rather than going through the mesh-less demo GLB.
"""

import argparse
import math
import struct
from pathlib import Path

FPS = 30

# Copied from kimodo.cpp demo/skeletons_extra.go, itself NVIDIA's Apache-2.0
# SOMA definition. Note LeftLeg/RightLeg are the thighs and LeftShin/RightShin
# the calves -- the names do not line up with Mixamo's LeftUpLeg/LeftLeg.
NAMES = [
    "Hips", "Spine1", "Spine2", "Chest", "Neck1", "Neck2", "Head", "Jaw",
    "LeftEye", "RightEye", "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "LeftHandThumbEnd", "LeftHandMiddleEnd", "RightShoulder", "RightArm",
    "RightForeArm", "RightHand", "RightHandThumbEnd", "RightHandMiddleEnd",
    "LeftLeg", "LeftShin", "LeftFoot", "LeftToeBase",
    "RightLeg", "RightShin", "RightFoot", "RightToeBase",
]
PARENTS = [-1, 0, 1, 2, 3, 4, 5, 6, 6, 6, 3, 10, 11, 12, 13, 13, 3, 16, 17, 18,
           19, 19, 0, 22, 23, 24, 0, 26, 27, 28]
OFFSETS = [
    (0, 0, 0), (-.00013727, .0500376256, -.00053726669),
    (-1.86574103e-9, .0712530139, -.000298248546),
    (-5.75188398e-9, .0755006305, -.00815970992),
    (-.00181676517, .263112953, -.00553348292),
    (-2.85102231e-8, .0770939664, .0230258546),
    (-4.5975437e-8, .0612891595, .0195370861),
    (2.63687901e-5, .0047559225, .0309494062),
    (.0320638079, .0538020513, .0758688308),
    (-.0322244017, .05361869, .0755823359),
    (.0162165175, .232371641, .0511341324),
    (.149198457, 2.19397873e-8, -.0550232576),
    (.287393078, 2.50268389e-9, -2.58787737e-5),
    (.270939812, -7.06625108e-9, 2.60897248e-5),
    (.122686267, -.0322017573, .0483306876),
    (.190119595, -.00312878387, -.000339570373),
    (-.0138011824, .231803086, .0521415786),
    (-.150371962, 1.17387901e-7, -.0554560437),
    (-.287366393, 1.87628082e-8, -2.59709359e-5),
    (-.271336198, -1.16767401e-9, 2.61269368e-5),
    (-.122642483, -.0321145448, .0480403904),
    (-.190005945, -.00306615542, -.0003157343),
    (.10043214, -.0843452671, .0259565473),
    (-1e-8, -.432217537, -.00802912805),
    (1e-8, -.421550959, -.0348152298),
    (0, -.0505947206, .132315294),
    (-.10047278, -.0829525995, .0262031695),
    (1e-8, -.433622059, -.00805555828),
    (2e-8, -.421173943, -.0347839785),
    (-3.42907669e-9, -.0507960932, .132841956),
]


def children_of(index):
    return [i for i, parent in enumerate(PARENTS) if parent == index]


def quat_to_zxy_degrees(x, y, z, w):
    """Decompose an XYZW quaternion into BVH's Zrotation Xrotation Yrotation."""
    # Rotation matrix entries needed for the M = Rz @ Rx @ Ry decomposition.
    m01 = 2 * (x * y - z * w)
    m11 = 1 - 2 * (x * x + z * z)
    m20 = 2 * (x * z - y * w)
    m21 = 2 * (y * z + x * w)
    m22 = 1 - 2 * (x * x + y * y)
    rx = math.asin(max(-1.0, min(1.0, m21)))
    if abs(m21) < 0.9999:
        ry = math.atan2(-m20, m22)
        rz = math.atan2(-m01, m11)
    else:  # gimbal lock: fold the free rotation into Z
        ry = 0.0
        rz = math.atan2(2 * (x * y + z * w), 1 - 2 * (y * y + z * z))
    return math.degrees(rz), math.degrees(rx), math.degrees(ry)


def write_hierarchy(out, index, depth, scale):
    pad = "\t" * depth
    offset = tuple(component * scale for component in OFFSETS[index])
    kids = children_of(index)
    if index == 0:
        out.append(f"ROOT {NAMES[index]}")
        out.append("{")
        out.append(f"\tOFFSET {offset[0]:.6f} {offset[1]:.6f} {offset[2]:.6f}")
        out.append("\tCHANNELS 6 Xposition Yposition Zposition "
                   "Zrotation Xrotation Yrotation")
    else:
        out.append(f"{pad}JOINT {NAMES[index]}")
        out.append(f"{pad}{{")
        out.append(f"{pad}\tOFFSET {offset[0]:.6f} {offset[1]:.6f} {offset[2]:.6f}")
        out.append(f"{pad}\tCHANNELS 3 Zrotation Xrotation Yrotation")
    for kid in kids:
        write_hierarchy(out, kid, depth + 1, scale)
    if not kids:
        # BVH needs a terminating End Site; give it a short stub along the
        # bone's own direction so Blender draws a sane final bone.
        length = math.dist((0, 0, 0), offset) or 0.05 * scale
        out.append(f"{pad}\tEnd Site")
        out.append(f"{pad}\t{{")
        out.append(f"{pad}\t\tOFFSET 0.000000 {-length * 0.5:.6f} 0.000000"
                   if index in (25, 29) else
                   f"{pad}\t\tOFFSET 0.000000 {length * 0.5:.6f} 0.000000")
        out.append(f"{pad}\t}}")
    out.append(f"{pad}}}" if index else "}")


def channel_order(index, order):
    """Depth-first joint order, matching how BVH lays out motion columns."""
    order.append(index)
    for kid in children_of(index):
        channel_order(kid, order)
    return order


def convert(source: Path, destination: Path, scale: float) -> None:
    roots = source / "root_positions.f32"
    rotations = source / "local_rotations_xyzw.f32"
    root_data = roots.read_bytes()
    rotation_data = rotations.read_bytes()
    frames = len(root_data) // 12
    joints = len(NAMES)
    expected = frames * joints * 16
    if len(rotation_data) != expected:
        raise SystemExit(
            f"{rotations.name} holds {len(rotation_data)} bytes, expected "
            f"{expected} for {frames} frames x {joints} joints")

    lines = ["HIERARCHY"]
    write_hierarchy(lines, 0, 0, scale)
    lines.append("MOTION")
    lines.append(f"Frames: {frames}")
    lines.append(f"Frame Time: {1.0 / FPS:.8f}")

    order = channel_order(0, [])
    for frame in range(frames):
        px, py, pz = struct.unpack_from("<3f", root_data, frame * 12)
        row = [f"{px * scale:.6f}", f"{py * scale:.6f}", f"{pz * scale:.6f}"]
        for joint in order:
            quat = struct.unpack_from(
                "<4f", rotation_data, ((frame * joints) + joint) * 16)
            row.extend(f"{angle:.6f}" for angle in quat_to_zxy_degrees(*quat))
        lines.append(" ".join(row))

    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {destination} ({frames} frames, {joints} joints, "
          f"{frames / FPS:.2f}s @ {FPS}fps)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path,
                        help="kimodo demo-output/<animation-id> directory")
    parser.add_argument("destination", type=Path, help="output .bvh path")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="metres multiplier (100 for centimetre rigs)")
    args = parser.parse_args()
    convert(args.source, args.destination, args.scale)


if __name__ == "__main__":
    main()
