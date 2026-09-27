# Goblin death animation: design and Gates (2026-09-25)

## 0. Goal
- Make one game death clip, `death`, with the finished rig (`rig/gob_r04_ctrl.blend`), and export it to Unity with the A3 pipeline.
- Reference: `work/goblin_swing/ref_death/`.
  - reference.mp4 is a copy of the source Downloads/orc-death.mp4 (same md5 7a46c269…).
  - frames/f_0001–0056.png: 56 frames, 24 fps, 448×576, static camera.
  - The framing is the same as swing, idle and hit, so `rig/data/swing_cam.json` is reused.
  - Its f1 matches idle reference f1 (mean pixel diff 2.0 / 255).
- Procedure: `docs/rig-pipeline-playbook.md` §4, with the same tools as idle and hit, plus one new check (ground contact, §4 C11). Key poses (D1) → animation (D2) → export (D3).

## 1. Reference measurement
main's pixel tracking: `ref_death/measure_ref.py` → `ref_death/measure.json`. 1 px ≈ 3.7 mm. Image y points down. After f30, the belt, club and feet segmentation mixes up the lying body, so only the silhouette box is used there.

| Frames | Phase | Measured |
|---|---|---|
| 1–9 | still | = f1 (ready) |
| 10–22 | slow loss of strength | Head top 108 → 124 (**−16 px ≈ −59 mm**). Eyes shift to screen left (the character's right) by 12 px ≈ 44 mm. Silhouette area −13 % (leaning back, away from the camera). Left fist moves in 22 px. The club tilts slightly. Feet planted |
| 23–27 | sag speeds up, balance lost | Head top 124 → 176 (−52 px). Knees bend, head tips back. Feet planted to f26; at f27 the feet start to lift (feet box y 448 → 442) |
| 28–30 | falls backward | Motion blur. The body rotates back about 90°, and the feet swing up toward the camera |
| 31 | impact | Lying on the back. Head away from the camera, belly up, soles facing the camera. Silhouette top 302, box height 159 px (standing: 343 px) |
| 32–38 | small bounce and settle | Top 302 → 294 → 306. The legs spread a little (feet box x 126–329 → 48–367) |
| 39–56 | still, lying | Arms out to the sides on the ground. **The club stays in the right hand**, lying on the ground at the character's right |

## 2. Design decisions (main defaults; the user can override at D1.3)
- **f1 = attack_swing f1 exactly** (DEF ≤ 1e-4). The clip starts from the shared ready pose.
- **Not a loop.** The last frame holds the lying pose; in game the clip plays once and holds the end.
- **Root motion: locked** (CTRL_root identity; the standard for a death clip). The body falls through the COG/pelvis. The lying body ends behind the start position, and the prefab bounds grow to cover it.
- **Timing follows the reference:** still to f9, sag f10–22, balance lost f23–27, fall f28–30, impact f31, settle f32–38, still to f56.
  - As with hit, the long still head and sag can later be trimmed in the Unity importer if game response needs it.
- **Club stays in the right hand.** weapon_space stays 0. The club ends on the ground at the character's right.
- **Legs:** planted IK for f1–26. From f27 the feet leave the ground by animating the foot IK controls. Switching IK to FK would pop, because PROPS use CONSTANT interpolation.
- **Clip-only range extension (user decision 2026-09-25, after T101 hit the limit):** the rig's sweep ranges (G6: tested with the feet planted) do not allow the lie (CTRL_torso rot X ±45, loc Y ≤ +0.01, foot IK loc Z ≤ 0.08; the best in-range attempt still floated 82–93 mm). death.json declares `sweep_overrides` with a reason, and s12 widens the ranges in memory only (widening only; ctrl_manifest.json, s10a and the rig are unchanged). The widened poses are validated by this clip's per-frame checks (C11 ground, C8 clearance/cover, C9 flips).
- **No scale.** The body is rigid; the fall is shown with CTRL_torso rotation and translation, spine, head and limbs.
- **Measured limits carried over from hit (T90):**
  - While the feet are planted, the COG drop is ≥ −0.036 (below that the legs show a visible crease and hip_flex exceeds 60).
  - Blending the arms between very different poses opens visible armpit creases. Where needed, hold the arm values of one pose.
  - The right upperarm raise must stay ≤ 100° (the club passes through the head above about 105°).
- **Lying pose:**
  - On the back, the torso about 90° back, the back resting on the ground.
  - The head tipped back toward the ground; it must not pass through the ground.
  - Legs extended toward the camera and slightly up (small hip flex relative to the torso, within safe range).
  - Arms out to the sides, resting on the ground.

## 3. Motion amplitude (report-only; the user judges the feel [U])
- Head drop by f22: 40–80 mm (reference 59 mm), from check_anim_clip M2 `head_rise_mm` per frame.
- The fall and the lying pose are judged visually; there are no amplitude targets for them.

## 4. Gates (main judges; [U] needs user approval)
`gate_id` mapping in `rig/data/clips/death.json`:

| C-id | Gate id | Report-only? |
|---|---|---|
| C1 | D1.1 | |
| C2, C3, C4 | D1.2 | C2 is report-only |
| C5 | D2.1 | |
| C6 | D2.2 | **report-only** (not a loop) |
| C7, M1 | D2.2 | |
| C8, C9, C10 | D2.3 | |
| C11 | D2.4 | report-only on the first run, then fixed after measurement |
| M2 | D2.5 | report-only |

**New criterion C11 (ground contact).** It is added to check_anim_clip by a separate checker task (T100).
- For every frame, the lowest evaluated vertex z of the character mesh and the club.
- Penetration: lowest z ≥ −P mm.
- In the JSON `ground_rest_ranges` (the lying frames), the body rests on the ground: lowest body z ≤ +R mm, so it does not float.
- P and R are set after the first measured run, and the basis is recorded here.
- **Fixed 2026-09-26 (after the T102 calibration run): P = 1 mm, R = 3 mm.** Measured lowest body z: −0.56 mm standing (shoe), −0.85 mm lying (belt, f31 and f34–56). Club ≥ +3.1 mm. The bounce frames f32–33 are +19 / +14 mm and are outside the rest ranges ([[31,31],[34,56]]). This is the same band as the G6.7 planted rule [−1, +3] mm. From now on C11 blocks (`gate_id` C11 = D2.4, no report flag).
- M2 (D2.5) is emitted only when the JSON has `targets`; death has none. The amplitude is read from the s12 anim_measure instead: the eye point is −59 mm at f21, and the reference measures 59 mm.

| ID | Condition | Verification |
|---|---|---|
| D1.1 | Key poses recorded in `rig/data/clips/death.json` (idle/hit schema): f1 (= attack f1), sag (around f22), tip (around f27), impact (f31), rest (around f38) | check_anim_clip |
| D1.2 | Per pose: clearance ≥ 10 mm, visible self-intersection cover ≤ 1; contact feet planted where contact is given (f1, f22); safe range reported | check_anim_clip |
| D1.3 [U] | Reference vs ours key-pose sheet (front = swing_cam + ¾) | render |
| D2.1 | Action `death`: 24 fps, 1–56, CTRL/PROPS keys only; key poses match (≤ 1e-4) | check_anim_clip |
| D2.2 | f1 = attack_swing f1 DEF (≤ 1e-4); CTRL_root motion 0; loop reported (not required) | check_anim_clip |
| D2.3 | Every frame: clearance ≥ 10 mm, visible cover ≤ 1, branch flip 0. Feet planted (≤ 1 mm) for their contact ranges (both feet 1–26) | check_anim_clip |
| D2.4 | Ground contact (C11): no penetration every frame; resting on the ground in the lying frames (thresholds fixed after measurement) | check_anim_clip |
| D2.5 | Motion amplitude report (§3) | check_anim_clip |
| D2.6 [U] | Reference side-by-side video (24 fps) and contact sheet | render |
| D3 | Same as A3.1–A3.3 for `goblin@death.fbx`. The prefab bounds are recomputed over the union of all clips; the lying pose will enlarge them, so re-run A3, I3, H3 and G9 | check_a3_clip --action death (gate prefix D3) |
