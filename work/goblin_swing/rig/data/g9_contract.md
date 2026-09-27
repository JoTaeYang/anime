# G9 data contract (main, 2026-09-25)

Shared by the Unity agent (GoblinImport.cs / GoblinRigCheck.cs) and the Blender agent (s09_blender_ref.py / compare_g9.py). Neither agent changes this file. If a change is needed, report it to main.

## Frame and time mapping
- Blender stage frame f (1..583, 24 fps) ↔ Unity clip time t = (f − 1) / 24 s.
  - The FBX take starts at frame 1. If the Unity clip's firstFrame differs, record it in the report and correct for it.
- **Sample frames (integer, G9.4):** for every segment in rigtest_manifest, take start, mid = floor((start+end)/2), and end. Drop duplicates.
- **Subframes (G9.8):** for each G9.4 sample f that satisfies f+1 ≤ 583, add f+0.25, f+0.5, f+0.75.
  - Blender evaluates these with `scene.frame_set(int(f), subframe=frac)`.
  - Unity evaluates them with `AnimationClip.SampleAnimation(go, t)`.
- **Bounds (G9.10):** every integer frame 1..583.

## Coordinates
- Unity values are written as-is in Unity world coordinates (left-handed, Y-up, meters).
- The Blender dump is written as-is in Blender world coordinates (Z-up, meters).
- The comparison (compare_g9.py) converts Blender to Unity with `export_preset.json` `axis_mapping.blender_to_unity` M:
  - Position: p_u = M p_b.
  - Rotation: R_u = M R_b Mᵀ, which is a reflection conjugation (det M = −1).
- Rotation comparison uses the bone's world rotation matrix, compared as a quaternion angular distance.
- The bone axis conventions differ between Unity and Blender (in FBX, Blender bone Y is the bone direction). A **rest-relative comparison** removes that difference:
  - For each bone, compute ΔR = R_pose · R_restᵀ on the Blender side and on the Unity side.
  - Compare ΔR_u to M ΔR_b Mᵀ.
- Position is compared as the world position of the bone head (Unity Transform.position).

## goblin_report.json (written by Unity, `unity/AvatarCheck/goblin_report.json`)
```json
{
 "unity_version": "...",
 "fps": 24, "clip": {"name": "...", "length_s": 0.0, "frameRate": 24, "firstFrame": 0.0},
 "importers": {"goblin.fbx": {...}, "goblin@rigtest.fbx": {...}, "goblin_club.fbx": {...}},
 "hierarchy": [{"path": "goblin/root/pelvis", "name": "pelvis", "parent": "root", "localPosition": [x,y,z], "localRotation": [x,y,z,w], "lossyScale": [x,y,z]}],
 "unexpected_nodes": ["..."],
 "rest": {"<bone>": {"pos": [x,y,z], "rot": [x,y,z,w], "localPos": [...], "localRot": [...]}},
 "bindposes": {"<bone>": [[4x4 row-major]]},
 "samples": [{"frame": 49.0, "time_s": 2.0, "bones": {"<bone>": {"pos": [..], "rot": [x,y,z,w]}}, "club": {"pos": [..], "rot": [..]}}],
 "subsamples": [ same format as samples, frame is fractional ],
 "bounds": {"localBounds": {"center": [..], "extents": [..]}, "rootBone": "...", "frames_checked": 583, "max_outside_m": 0.0, "worst_frame": 1, "per_frame_outside": [[frame, outside_m], ...only frames with outside_m > 0]},
 "renders": ["rig/inspect/G9/unity_<cam>_<frame>.png"],
 "errors": []
}
```
- All 24 bones are included. The club is the goblin_club instance attached at local 0 under weapon_socket_r.

## blender_dump.json (written by Blender, `rig/inspect/G9/blender_dump.json`)
- Same keys as the Unity report:
  - `rest`
  - `samples` and `subsamples`, each with `bones` and `club`, in Blender world coordinates
  - rotation as a quaternion `[x,y,z,w]`, plus a 3×3 matrix field `rotm`
- `canonical_bind`: for each bone, the canonical `rest_matrix` (4×4).

## compare_cams.json (written by Blender, `rig/data/compare_cams.json`)
- Cameras `front` and `three_quarter`, orthographic, 1024 px height, square aspect.
- Each camera records its Blender world matrix and its Unity world position, rotation and orthographicSize (M applied).
- Render frames: 3 frames from the G9.4 sample set. The Blender agent chooses them and records them in the file.
