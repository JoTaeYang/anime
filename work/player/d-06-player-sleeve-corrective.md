# Player P2g: sleeve corrective (2026-09-27, user "소매 보정")

## 0. Problem
- The sleeves are rigid (100 % on Arm_L/R, P2.4). When the arm is raised forward or up, the sleeve top rotates into the scarf, the tunic shoulder and, at high raises, the head.
- **Known since:** P1 finding 3; the P2.2 report "the sleeve pushes into the scarf when the arm goes forward, up to 42 mm".
- **Measured on the clips** (C11, baseline = the Sword_Idle f1 pose):

  | Clip | Pair | Excess |
  |---|---|---|
  | Sword_Attack_01 | sleeve_r~scarf | 69 pairs / 42 mm (f15) |
  | Sword_Attack_01 | sleeve_l~scarf | 47 pairs / 29 mm (f11) |
  | Sword_Attack_01 | sleeve_r~head | 22 pairs / 12.5 mm (f11) |
  | Sword_Attack_01 | torso volume | sleeve_r 21 newly inside, 48 mm |
  | Sword_Idle | all | ≤ 4 verts / 3 mm (breathing) |

- Every future weapon clip raises the arms, so this is fixed once at the rig level.

## 1. Step A: investigation (T330, production)
- **Pose grid:** the arm alone, the rest of the rig at the ready pose.
  - Upper-arm pitch forward 0 / 30 / 60 / 90 / 120°, raise to the side 0 / 45 / 90°, and forward + across the body. Both sides.
  - Also the two clips' worst frames.
- **Per pose:** C11-style excess vs the ready pose, for each sleeve and arm-tube pair against scarf, scarf_tail, tunic, belt and head (visible pairs, max depth, newly-inside vertices).
- **Prototype at least two approaches in scratch** (do not touch the canonical rig):
  1. **Corrective shape keys** on the sleeve (and/or the scarf) driven by the upper-arm angle relative to Spine_02. Same pattern as the goblin mouth: driver in the rig, value baked on the stage as a Key action, FBX blend shape, Unity weight.
  2. **Weights / helper bone:** the sleeve's inner-top ring partly follows Clavicle / Spine_02 (or a new helper bone that is not exported and is baked into DEF weights), so it lags behind the arm.
  3. **Scarf lift (optional):** the scarf edge over the shoulder rises with the clavicle or arm raise, as real cloth does.
- **Report per approach:**
  - residual excess on the grid
  - sleeve shape distortion: edge-length stretch, visible creases
  - the rest pose changes by exactly 0 (the corrective is 0 at the ready pose)
  - export and Unity impact: blend-shape count, bone count; the canonical 41-bone skeleton must not change unless main approves
  - render sheet: grid poses, before / after, front / 3/4 / top
- **Main then picks the approach** and shows the user the before/after sheet [U].

## 2. Step B: implementation gates (draft; report first, then fixed)
| ID | Criterion | Status |
|---|---|---|
| P2.10a | Grid and clips: sleeve vs scarf / tunic / head excess over the ready pose ≤ *tolerance* | report → fixed after the first measurement |
| P2.10b | At the ready pose and the rest pose the corrective contributes exactly 0 (≤ 1e-6 m) | blocking |
| P2.10c | No new problems: arm tube vs sleeve, sleeve self-overlap, sleeve edge stretch ≤ *x* %, scarf silhouette at rest unchanged | report → fixed |
| P2.10d | Export: the canonical skeleton is unchanged (41 bones). Blend shapes (if used): Unity weight vs Blender ≤ 0.01, BakeMesh vs Blender ≤ 0.01 mm (goblin HR2 thresholds) | blocking |
| P2.10e | Re-run P2 gates (run_player_gates) and both clips (run_player_clip). All previously passing rows still pass; C11 drops | blocking |
| [U] | Grid before / after sheet, plus both clips' videos | user |
