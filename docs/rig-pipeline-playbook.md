# Game character rig → animation playbook

The procedure and principles to follow as-is for new characters and new clips. It is based on the goblin work (2026-09-25). The background is in `docs/retro/2026-09-25-goblin-rig-anim-retro.md`.

## 1. Rig-ready model spec (for Meshy or a modeler)
Check these before starting to rig. A model that fails them costs much more in retopo, rigging and animation.

| Item | Requirement | Why |
|---|---|---|
| Pose | A-pose (arms 30–45° down); elbows and knees bent 15–20° | Keeps IK away from its straight-limb sensitivity. Defines the bend direction |
| Props | Weapons and accessories as separate objects, not merged with the body. Record the grip position | Separating them and rebuilding the join is expensive |
| Hands, feet | Shape decided in advance (to match the reference). Fingers only if they will be rigged | Changing the hand shape later forces a full rerun |
| Proportions | Measure leg/arm length and joint positions and write them down | Short legs limit foot roll and crouching |
| Joints | Shoulder and hip joints shaped (fillets). Some thickness at elbows and knees | Needed for topology and for keeping volume |
| Scale / axes | Real-world size, front −Y, up +Z, ground z=0 | Convention for the whole pipeline |
| Reference comparison | Compare silhouette, hand and prop style with the reference animation and fix mismatches **before rigging** | The goblin's hands were caught late and replaced |

## 2. Gate operating rules
- **Measure first:** every new numeric criterion is report-only on its first run (calibration run). Look at the distribution, then fix the threshold and record the basis in the spec.
- **Blocking vs report-only:** block only on problems that are visible or break things (penetration, pops, foot sliding, export mismatch). Refinement metrics such as pixel-level silhouette or between-frame interpolation stay report-only.
- **Visible vs hidden:** self-intersection counts only intersections that can be seen. Intersections hidden inside rigid shells are excluded.
- **main re-verifies:** do not trust a checker's ok as-is. main runs it itself, compares input sha256, and looks at the renders directly.
- **User approval [U]:** visual quality (key poses, animation, handling feel) is judged by the user. Show the side-by-side sheet or video.

## 3. Order
1. **Walking skeleton (first day):** crude rig → FBX → Unity round trip. Catch problems in axes, importer behavior (bone connection), bounds, batch rendering and the license first.
2. Source cleanup and pivot measurement (G1) → retopo (G2) → deformation test (G3).
3. DEF skeleton (G4): generated from canonical_skeleton.json. **Export bones are all unconnected.**
4. Skinning (G5): rigid parts 100%, twist bones via SWING_TWIST_Y.
5. Control rig (G6): IK/FK, space switching that preserves world matrices, foot roll (pivots near the contact plane, cumulative slip), Ready pose.
6. Bake → FBX → Unity (G7–G9).
7. Final full rerun (G10): `rig/scripts/run_all_gates.ps1`.

## 4. Adding a clip (current tools)
1. Key poses: redesign the reference within the rig's safe range (do not copy it) → [U] approval.
2. In-betweens: fill to the reference timing. Add breakdown keys at Euler jumps → A2 per-frame checks → [U] approval of the side-by-side video.
3. Export:
   ```
   bl.ps1 -Script s11_export_clip.py -Blend <anim.blend> -- --action <clip>
   unity_goblin.ps1 -Method GoblinRigCheck.Run      # prefab bounds = union of all goblin@* clips + 0.1 m
   unity_goblin.ps1 -Method GoblinClipCheck.Run
   bl.ps1 -Script check_a3_clip.py -- --action <clip>   # per-clip Unity report goblin_clip_report_<clip>.json
   ```
   After a new clip, re-run check_a3_clip for every existing clip and compare_g9: the prefab bounds and goblin_report.json are rewritten.
   New gate prefixes go into check_a3_clip DEFAULT_GATE / DEFAULT_WORK (A3 attack_swing, I3 idle, H3 hit).
   A pose outside the rig's sweep ranges (for example lying down, where G6 tested the feet planted) goes in the clip JSON `sweep_overrides` with a `_reason`. It only widens, only in memory, and only for that clip, and the clip's own per-frame checks validate it. Clips that touch the ground with the body declare `ground_rest_ranges` + `ground` (C11: penetration 1 mm, float 3 mm).
   After any change to s11, s12 or the checkers, re-run the animation checkers of the existing clips too: their evidence hashes those scripts.
4. Set the root-motion policy per clip: attack is locked; walk and run move; decide jump before making it.

## 5. Known pitfalls
- Setting PROPS values from Python does not re-evaluate drivers. Call `arm.update_tag()` and then `view_layer.update()`.
- Blender's FBX importer auto-connects bones when a parent's tail touches a child's head, so translation keys are ignored. Unconnect before comparing.
- In Unity batch mode, SkinnedMeshRenderer skinning is not reflected in renders. Render from a BakeMesh instead.
- Blender includes constraint relations in the dependency graph even at influence 0. Design space switches so they cannot form cycles.
- The FBX header contains a creation timestamp, so the hash changes on every re-export. Run checkers after the last export.
- Waiting on Unity with `Start-Process -Wait` also waits for its leftover dotnet child process (about +10 min). `unity_goblin.ps1` waits only for Unity.exe (WaitForExit) and caches `$p.Handle` so the exit code is kept. Fixed 2026-09-25: 11.8 s / 7.4 s per run.
- Close the Unity editor before batch runs. An open editor on the same project blocks them.

- **Blender Workbench renders back faces; Unity culls them.** Check face-winding bugs with a `show_backface_culling` render (s02e culled_*). A Unity capture needs **one material per submesh** (`sharedMaterials`): a single `sharedMaterial` draws only submesh 0 (HR2, the mouth looked see-through).
- **Silhouette contours:** `goblib._contour` also counts the edges of holes and slits. Compare separate objects (character vs club) separately, or a gap between two objects gets measured by raster phase (G2.12: 3.09 / 3.94 / 4.63 mm, T121c–f).
- **Meshy retopo:** a collapse-decimated and fitted head is uneven (matcap zigzags) even when the source is smooth. Measure a matcap and an MLS reference first (HR0). An even-quad shell with loops around the eyes and fangs plus a crease rim loop fixes it (HR1).
- **Facial shape keys:** R0 proved the round trip (a driver is baked by the FBX exporter; the only modifier must be Armature). Keep them relative, and keep the key value on the stage mesh as a Key action.
- PowerShell: do not name helper functions `R`, `H`, `r`… — they are built-in aliases (Invoke-History, …) and silently swallow the call.

- **Unity bind poses at a gimbal singularity:** a bone whose parent-relative rest rotation sits near ±90° pitch (FBX Euler XYZ) gets a wrong bind pose on Unity import. The sampled poses stay exact, but the skin is off at rest (player Skirt_L/R: 1.15 mm). Change the roll so every bone has ≥ 30° margin; check `bindposes_ok` (player P2.8a).
- **SpringBoneChain resets to its Init rest every tick**, so any animation keyed on chain bones (e.g. a skirt that follows the thigh) is thrown away. Set `followAnimation = true` when the clip keys the chain bones every frame (player P2e).
- **Byte-level hashes of regenerated artifacts** (.blend saves, FBX, Unity reports) differ run to run. For a full rerun, judge identical row values plus identical source/data inputs.
- **PowerShell functions: `Write-Output` inside a function becomes part of its return value.** Use `Write-Host` for logs.
- **Locate the poking vertices (rest position, part) before choosing a fix.** The player f477 patch looked like the thigh top but was the knee; the thinning attempt (P2f) was wasted.
- **Self-intersection rows must cover every limb-part × body-part pair.** On Sword_Attack_01, C7 checked fist vs torso (0) but had no sleeve vs tunic or arm vs scarf pair, so the arm sinking into the chest passed. The user caught it. When reading sheets, check the top view for limbs inside the body outline (player C11).

## 6. Improvements to make (backlog)
- [x] Unity wrapper: wait only for the Unity process (done 2026-09-25)
- [ ] Unity: batch several checks per launch; check the license and open editor beforehand
- [ ] Shared metric library (clearance, visible intersection, contact, branch flip, silhouette comparison)
- [x] Clip-agnostic animation tools: s12_clip_anim.py + rig/data/clips/<clip>.json, check_anim_clip.py, check_a3_clip.py --action (done with idle, 2026-09-25)
