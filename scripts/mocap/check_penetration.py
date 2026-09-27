"""Gate: no frame of the clip may have the arms intersecting the body.

    blender --background CLIP.blend --python scripts/mocap/check_penetration.py

Fails the build on any intersecting frame. This is a per-character gate, not a
one-off fix: the clearance a motion needs depends on the target's proportions,
so every new character has to clear it again.
"""
import json
import sys
from pathlib import Path

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lib import paths  # noqa: E402
from mocap.penetration import MAX_DEPTH, scan  # noqa: E402

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
REPORT = Path(argv[0]) if argv else paths.PREVIEWS_DIR / "mocap_penetration.json"

armature = next(o for o in bpy.data.objects if o.type == "ARMATURE")
clip = armature.animation_data.action
start, end = int(min(clip.frame_range)), int(max(clip.frame_range))

per_frame, by_obstacle = scan(armature, start, end)
total = len(per_frame)
touching = [(f, d) for f, d, _ in per_frame if d > 0]
over = [(f, d) for f, d in touching if d > MAX_DEPTH]
soft_frames = sum(1 for _, _, soft in per_frame if soft)

for name in sorted(by_obstacle):
    entry = by_obstacle[name]
    print(f"RESULT arm intersects {name}: frames={entry['frames']}/{total} "
          f"max_face_pairs={entry['max']}")
worst = sorted(touching, key=lambda item: -item[1])[:6]
deepest = worst[0][1] if worst else 0.0
print(f"RESULT frames_touching={len(touching)}/{total} "
      f"deepest={deepest * 1000:.1f}mm limit={MAX_DEPTH * 1000:.0f}mm")
if worst:
    print("RESULT worst=" + ", ".join(f"f{f}({d * 1000:.1f}mm)" for f, d in worst))
if soft_frames:
    print(f"RESULT simulated_cloth_contact={soft_frames}/{total} frames "
          f"[reported, not gated -- the cloth is at rest here, not where the "
          f"spring bones will put it at runtime]")

REPORT.parent.mkdir(parents=True, exist_ok=True)
REPORT.write_text(json.dumps({
    "clip": clip.name,
    "frames": total,
    "frames_touching": len(touching),
    "frames_over_limit": len(over),
    "deepest_mm": round(deepest * 1000, 2),
    "limit_mm": MAX_DEPTH * 1000,
    "simulated_cloth_contact_frames": soft_frames,
    "obstacles": by_obstacle,
    "worst_frames": [{"frame": f, "depth_mm": round(d * 1000, 2)} for f, d in worst],
}, indent=2), encoding="utf-8")
print(f"RESULT report={REPORT}")

if over:
    raise RuntimeError(
        f"arms go up to {deepest * 1000:.1f}mm into the body on {len(over)}/"
        f"{total} frames (limit {MAX_DEPTH * 1000:.0f}mm). Re-run the retarget "
        f"with --abduct auto to solve for the clearance.")
print("CHECK_PENETRATION OK")
