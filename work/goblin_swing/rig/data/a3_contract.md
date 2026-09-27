# A3 data contract (main, 2026-09-25)

Shared by the A3 production agent (s11_export_clip.py + Unity GoblinClipCheck) and the checker agent (check_a3_clip.py). Neither agent changes this file; report any change needed to main.

## Inputs and outputs
- Work animation: `rig/gob_r07_swing_anim.blend` (GOB_rig, action `attack_swing`, frames 1–39, 24 fps). Approved in A2.
- Export stage: `s07b_export_rig.build_export(gob_r07_swing_anim.blend, "attack_swing", export/stage_attack_swing.blend)`.
- FBX: `rig/export/goblin@attack_swing.fbx`, exported with exactly the same settings as `data/export_preset.json` (armature `goblin`, mesh `goblin_mesh`, one AnimStack `attack_swing`, bake_anim_use_all_actions False). goblin.fbx and goblin_club.fbx are reused unchanged.
- Unity: copy to `unity/AvatarCheck/Assets/Goblin/goblin@attack_swing.fbx`. GoblinImport picks it up through the `goblin@*.fbx` pattern (Generic, CopyFromOther goblinAvatar, compression Off).
- Unity report: `unity/AvatarCheck/goblin_clip_report.json`, written by the editor method `GoblinClipCheck.Run`. Clip name comes from the command line arg `-clip attack_swing`, defaulting to attack_swing.

## Frame and time
- Blender stage frame f (1..39) ↔ Unity clip time t = (f − firstFrame)/24, where firstFrame is from the importer clip settings (expected 1). Every integer frame 1..39 is sampled.

## goblin_clip_report.json
```json
{
 "unity_version": "...", "clip": {"name": "attack_swing", "length_s": 0.0, "frameRate": 24, "firstFrame": 1, "lastFrame": 39,
   "hasRootCurves": false, "hasMotionCurves": false, "hasGenericRootTransform": false},
 "importer": {"animationType": "Generic", "avatarSetup": "CopyFromOther", "sourceAvatar": {...}, "animationCompression": "Off", "resampleCurves": true},
 "samples": [{"frame": 1, "time_s": 0.0, "bones": {"<bone>": {"pos": [x,y,z], "rot": [x,y,z,w]}}, "club": {"pos": [..], "rot": [..]},
              "root_local": {"pos": [..], "rot": [..]}, "instance_root": {"pos": [..], "rot": [..]}}],
 "bounds": {"prefab": "Assets/Goblin/goblin.prefab", "localBounds": {"center": [..], "extents": [..]},
            "frames_checked": 39, "max_outside_m": 0.0, "per_frame_outside": [[frame, m], ...]},
 "errors": []
}
```
- Bones: all 24, world coordinates (Unity). The club is a goblin_club.fbx instance under weapon_socket_r at local 0 (G9 convention: root localRotation identity).
- Bounds: use the SkinnedMeshRenderer localBounds of a **goblin.prefab** instance (rig G9.10), and compare the BakeMesh AABB against them in rootBone-local space. If a frame is outside, only record it (do not widen the bounds automatically).
- root_local: the `root` bone Transform local values; instance_root: the instance's top-level Transform. These are for A3.3 root motion 0.

## Blender-side comparison (check_a3_clip.py)
- **A3.1 bake:** work DEF vs stage GOB_export world for all 39 frames × 24 bones: ≤ 1 mm, ≤ 1°.
- **A3.2 FBX:** reimport goblin@attack_swing.fbx with the export_preset reimport options, and set all bones to use_connect False (G8 pitfall). The anim_offset shifts frames by 1; correct it. Compare against stage GOB_export on all frames: ≤ 1 mm, ≤ 1°. 24 bones, names and hierarchy equal to canonical, no `_end`.
- **A3.3 Unity:**
  - The report's integer samples vs stage GOB_export, after mapping into Unity coordinates with export_preset `axis_mapping.blender_to_unity` M. Position p_u = M·p_b. Rotation is compared as rest-relative ΔR, the same way as compare_g9. Both ≤ 1 mm and ≤ 1°.
  - Club ≤ 1 mm / 1°.
  - Bounds: 0 frames outside.
  - Root motion: the root bone local position and rotation, and instance_root, stay at their frame-1 values within ≤ 1e-5 on every frame. clip hasRootCurves / hasMotionCurves are reported.
  - Importer settings match the conditions above.
- Evidence: `rig/inspect/A3/check_a3_clip.json`. Its inputs are the work blend, the stage, the FBX, the Unity report, export_preset, canonical and the checker itself.
