# HR2 data contract: new head + mouth shape key through the chain (2026-09-26)

Production (T120) and the checkers (T121) both follow this file. Spec: d-29 §3 HR and R2.

## Chain order
```
s02c → s02d → s02e → check_g2_static → check_g2_shape → s03 → … (rest unchanged)
```
- s02d writes `rig/work/r01d_merged.blend`. It no longer writes gob_r01_retopo.blend.
- s02e reads `work/r01d_merged.blend` and writes `rig/gob_r01_retopo.blend` (the canonical retopo final) plus `rig/data/mouth.json`.
- run_all_gates.ps1 gets `BL s02e_mouth.py work/r01d_merged.blend`.

## Mesh (GOB_mesh → goblin_mesh in the export)
- **Shape keys:** `Basis` plus one relative key `mouth_open`. 0 = closed, 1 = roar.
- **Material slots, in this order:**
  0. `GOB_skin`
  1. `GOB_mouth_inner`
  2. `GOB_teeth`
  3. `GOB_tongue`

  All four stay on the export mesh and in the FBX.
- **Modifiers:** the only modifier on the exported mesh is Armature. R0 found that any other active modifier drops the shape keys.
- **part_id:** every mouth face (cavity, tongue, teeth, lip patch) is head.
- **Skin weights:** every head vertex, including all mouth vertices, is 100 % DEF head (G5.2).
- **Design-change regions for G2.11 / G2.12:** these are reported, not judged, in both directions. Production writes them and the checker reads them.
  - `mouth.json` → `region`:
    - the rebuilt mouth patch
    - the cavity, tongue and teeth
    - the head-face area inside the region boundary loop
    - the world-space region polygon / vertex list, for sampling SRC points
  - `rig/data/head_design_zones.json` (written by s02c):
    - `eye_gap_xneg`: the x<0 eye area whose source lump is detached. It is defined in the (x, z) plane of the face with a y range, from the SRC ray-hit gap map (T113b).
    - Both directions of G2.11 are excluded inside this zone (user decision 2026-09-26, corrected by main after T113d: head→SRC also leaves the source there).

## Rig
- PROPS gets a float property `mouth_open` with range [0, 1] and default 0.
  - It is added to the props contract (check_g6 CONTRACT_DEFAULTS, check_anim_clip CONTRACT_DEFAULTS).
  - s06 adds it to the UI panel.
- A driver on `GOB_mesh.data.shape_keys.key_blocks["mouth_open"].value` reads `PROPS["mouth_open"]`. It uses a SINGLE_PROP variable with an AVERAGE expression, so no Python expression is needed.
- goblin.reset_rig and goblin.ready_pose set mouth_open to 0.
- **Interpolation:**
  - s12 clip JSON `interpolation.props_overrides`, for example `{"mouth_open": "BEZIER"}`, may give a PROPS key its own interpolation.
  - Every other PROPS key stays CONSTANT.

## Export stage (s07b build_export) and FBX
- The stage's goblin_mesh keeps the shape key `mouth_open`.
- When an action is baked, the evaluated value of the work rig's shape key is keyed on the stage's `goblin_mesh` Key datablock at every baked frame (action on the Key; relative key). The value is thereby written as keys, so the stage needs no driver.
- Clips that never animate mouth_open have value 0 on every frame. The key is still present, so Unity always has the blend shape.
- FBX (preset unchanged):
  - BlendShape and BlendShapeChannel `mouth_open` on goblin_mesh.
  - A DeformPercent curve when the value is animated.
  - 4 materials.

## Unity (Assets/Goblin)
- **Import:** importBlendShapes = true (the default; GoblinImport keeps it and states it).
- **Model:** the SkinnedMeshRenderer of goblin.fbx has blend shape `mouth_open` and 4 submeshes / materials.
- **Clips:** a `blendShape.mouth_open` curve on goblin_mesh when the value is animated. GoblinClipCheck reports it per clip; absent or constant 0 counts as 0.
- **Thresholds** (fixed from R0):
  - Unity weight vs Blender ×100 ≤ 0.01.
  - BakeMesh vertex vs Blender ≤ 0.01 mm, at the mouth vertices, on frames where the value > 0.

## Unchanged
- 24 DEF bones, canonical skeleton, bone names and parents, and the export preset.
- The existing clips' motion.
