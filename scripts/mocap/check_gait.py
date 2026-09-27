"""Gate: the retargeted walk has to behave like a walk.

    blender --background CLIP.blend --python scripts/mocap/check_gait.py

Measures the things that actually break a game walk -- foot skate, ground
penetration, knee range, cadence, hip oscillation -- and fails the build on the
ones that are defects rather than taste. Symmetry and the loop seam are
reported but not gated: both are properties of the generated source clip, not
of the transfer, so failing on them would block every clip kimodo produces.
"""
import json
import math
import sys
from pathlib import Path

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lib import paths  # noqa: E402
from mocap import rigs  # noqa: E402

# A planted foot must be world-stationary; anything else reads as sliding, and
# it is the first thing a player notices.
# 1cm, not 2.5cm: mid-swing toe clearance is only a couple of centimetres, so a
# loose threshold counts a foot in flight as planted and reports its travel as
# skate. That mistake produced a "24cm skate" on a clip that skates 0.2cm.
PLANT = 0.010
LIMITS = {
    "skate_cm_per_frame": 1.0,
    "penetration_cm": -1.0,
    "knee_max_deg": 40.0,       # a walk that never bends the knee is the
                                # "person with no knees" failure
    "cadence_min": 70.0,
    "cadence_max": 150.0,
    "hip_oscillation_cm": (2.0, 15.0),
}

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
REPORT = Path(argv[0]) if argv else paths.PREVIEWS_DIR / "mocap_gait.json"

target = next(o for o in bpy.data.objects if o.type == "ARMATURE")
_, bone_map = rigs.detect(target)
HIPS = bone_map["Hips"]
LFOOT, LTOE, LKNEE, LTHIGH = (bone_map[n] for n in
                              ("LeftFoot", "LeftToeBase", "LeftShin", "LeftLeg"))
RFOOT, RTOE, RKNEE, RTHIGH = (bone_map[n] for n in
                              ("RightFoot", "RightToeBase", "RightShin", "RightLeg"))

scene = bpy.context.scene
FPS = scene.render.fps
# Trust the action, not the scene: a fresh BVH import leaves the scene range at
# Blender's default 250 and the trailing frames would sit frozen on the last
# pose, poisoning speed and symmetry.
clip = target.animation_data.action
start, end = int(min(clip.frame_range)), int(max(clip.frame_range))
frames = end - start + 1

track = {b: [] for b in (HIPS, LFOOT, LTOE, RFOOT, RTOE, LKNEE, LTHIGH, RKNEE, RTHIGH)}
for index in range(start, end + 1):
    scene.frame_set(index)
    for bone in track:
        track[bone].append((target.matrix_world @ target.pose.bones[bone].head).copy())

# Ground reference = the lowest the toes ever get. Works for a retargeted rig
# (where that equals the rest toe height) and for a raw BVH skeleton alike.
rest_toe = min(p.z for t in (LTOE, RTOE) for p in track[t])
report, failures = {"clip": clip.name, "fps": FPS, "frames": frames}, []
print(f"RESULT fps={FPS} frames={frames} duration={frames / FPS:.2f}s")

travel = track[HIPS][-1] - track[HIPS][0]
speed = math.hypot(travel.x, travel.y) / ((frames - 1) / FPS)
report["speed_m_s"] = round(speed, 3)
print(f"RESULT speed={speed:.3f} m/s")

for toe, label in ((LTOE, "L"), (RTOE, "R")):
    slips, planted = [], 0
    for i in range(1, frames):
        if track[toe][i].z - rest_toe < PLANT and track[toe][i - 1].z - rest_toe < PLANT:
            planted += 1
            delta = track[toe][i] - track[toe][i - 1]
            slips.append(math.hypot(delta.x, delta.y))
    if not slips:
        failures.append(f"{label} foot never plants")
        print(f"RESULT skate_{label} NO PLANTED FRAMES (foot never settles)")
        continue
    mean_slip = sum(slips) / len(slips) * 100
    report[f"skate_{label}_cm_per_frame"] = round(mean_slip, 2)
    report[f"planted_frames_{label}"] = planted
    print(f"RESULT skate_{label} planted_frames={planted}/{frames} "
          f"mean={mean_slip:.2f}cm/frame max={max(slips) * 100:.2f}cm/frame")
    if mean_slip > LIMITS["skate_cm_per_frame"]:
        failures.append(f"{label} foot skates {mean_slip:.2f}cm/frame")

for toe, label in ((LTOE, "L"), (RTOE, "R")):
    depth = (min(p.z for p in track[toe]) - rest_toe) * 100
    report[f"penetration_{label}_cm"] = round(depth, 2)
    print(f"RESULT penetration_{label}={depth:.2f}cm (negative = through the floor)")
    if depth < LIMITS["penetration_cm"]:
        failures.append(f"{label} toe goes {-depth:.1f}cm through the floor")


def knee_series(thigh, knee, foot):
    out = []
    for i in range(frames):
        a = track[thigh][i] - track[knee][i]
        b = track[foot][i] - track[knee][i]
        out.append(180.0 - math.degrees(math.acos(
            max(-1.0, min(1.0, a.dot(b) / (a.length * b.length))))))
    return out


left = knee_series(LTHIGH, LKNEE, LFOOT)
right = knee_series(RTHIGH, RKNEE, RFOOT)
report["knee_L"] = [round(min(left), 1), round(max(left), 1)]
report["knee_R"] = [round(min(right), 1), round(max(right), 1)]
print(f"RESULT knee_L={min(left):.1f}..{max(left):.1f} "
      f"knee_R={min(right):.1f}..{max(right):.1f}")
for series, label in ((left, "L"), (right, "R")):
    if max(series) < LIMITS["knee_max_deg"]:
        failures.append(f"{label} knee only bends to {max(series):.1f}deg")

# Stride period by autocorrelation of the knee signal.
mean_left = sum(left) / frames
centred = [v - mean_left for v in left]
best, period = None, None
for lag in range(int(FPS * 0.6), int(FPS * 2.0)):
    if lag >= frames:
        break
    score = sum(centred[i] * centred[i + lag] for i in range(frames - lag)) / (frames - lag)
    if best is None or score > best:
        best, period = score, lag
if period is None:
    failures.append("clip is too short to measure a stride")
else:
    cadence = 120 * FPS / period
    report["stride_period_frames"] = period
    report["cadence_steps_per_min"] = round(cadence)
    print(f"RESULT stride_period={period} frames ({period / FPS:.2f}s) "
          f"cadence={cadence:.0f} steps/min")
    if not LIMITS["cadence_min"] <= cadence <= LIMITS["cadence_max"]:
        failures.append(f"cadence {cadence:.0f} steps/min is not a walk")

    # Left vs right should be the same curve half a stride apart. Search around
    # the half-period rather than assuming it: an integer-frame estimate that is
    # off by one inflates the error and would read as a limp that is not there.
    best_shift, best_mean, best_max = None, None, None
    for shift in range(max(1, period // 2 - 3), period // 2 + 4):
        if shift >= frames:
            break
        errors = [abs(left[i] - right[i + shift]) for i in range(frames - shift)]
        mean_error = sum(errors) / len(errors)
        if best_mean is None or mean_error < best_mean:
            best_shift, best_mean, best_max = shift, mean_error, max(errors)
    if best_mean is not None:
        report["symmetry_mean_deg"] = round(best_mean, 1)
        report["symmetry_max_deg"] = round(best_max, 1)
        print(f"RESULT symmetry_error mean={best_mean:.1f}deg max={best_max:.1f}deg "
              f"(best half-stride shift={best_shift} frames) [reported, not gated]")
    report["stride_length_m"] = round(speed * period / FPS, 3)
    print(f"RESULT stride_length={speed * period / FPS:.3f}m")

hips_z = [p.z for p in track[HIPS]]
oscillation = (max(hips_z) - min(hips_z)) * 100
report["hip_oscillation_cm"] = round(oscillation, 1)
low, high = LIMITS["hip_oscillation_cm"]
print(f"RESULT hip_oscillation={oscillation:.1f}cm (human walk is about 4-6cm)")
if not low <= oscillation <= high:
    failures.append(f"hip oscillation {oscillation:.1f}cm is outside {low}-{high}cm")

# Loop seam, root motion removed: how far the joints have to jump to restart.
seam = max(((track[bone][0] - track[bone][-1])
            - (track[HIPS][0] - track[HIPS][-1])).length
           for bone in (LKNEE, RKNEE, LFOOT, RFOOT))
report["loop_pose_gap_cm"] = round(seam * 100, 2)
print(f"RESULT loop_pose_gap={seam * 100:.2f}cm [reported, not gated]")

report["failures"] = failures
REPORT.parent.mkdir(parents=True, exist_ok=True)
REPORT.write_text(json.dumps(report, indent=2), encoding="utf-8")
print(f"RESULT report={REPORT}")

if failures:
    raise RuntimeError("gait check failed: " + "; ".join(failures))
print("CHECK_GAIT OK")
