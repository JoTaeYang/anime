# Goblin hit animation: design and Gates (2026-09-25)

## 0. Goal
- Make one game hit-reaction clip, `hit`, with the finished rig (`rig/gob_r04_ctrl.blend`), and export it to Unity with the A3 pipeline.
- Reference: `work/goblin_swing/ref_hit/` (reference.mp4 is a copy of the source KakaoTalk_20260920_211530723.mp4, same md5 7c6c7a26…; frames/f_0001–0039.png). 39 frames, 24 fps, 448×576, static camera, the same framing as swing and idle, so `rig/data/swing_cam.json` is reused. Its f1 is the idle reference f1 (mean pixel diff 1.6 / 255).
- Procedure: `docs/rig-pipeline-playbook.md` §4, the same tools as idle (s12_clip_anim.py + `rig/data/clips/hit.json`, check_anim_clip.py, s11, Unity, check_a3_clip.py). Key poses (H1) → animation (H2) → export (H3).

## 1. Reference measurement
main's pixel tracking (`ref_hit/measure_ref.py` → `ref_hit/measure.json`); 1 px ≈ 3.7 mm with swing_cam 271 px/m. Image y points down, so a y decrease means up.

| Frames | Phase | Measured |
|---|---|---|
| 1–5 | still | = f1 (ready) |
| 6–9 | flinch (hit arrives at f6) | head top 110 → 128 px (**−18 px ≈ −66 mm**, lowest f7–8). Belt 371 → 395 (**−24 px ≈ −89 mm**, lowest f8). Knees bend; head pitched down; eyes half closed. Club top moves out/down (70,114) → (50,120). Eye centroid 161 → 216 (lids cover the eyes, so not usable as a head height) |
| 10–14 | recoil | Belt 350 (**+21 px ≈ +78 mm above rest**, f13, a squash/stretch body effect). Head top ≈ rest while the eyes rise to 139.5 (+22 px): the head is pitched back, face up. **Left fist** (399,328) → (437,270): **out 38 px ≈ 140 mm, up 58 px ≈ 214 mm** (f13–14). The right arm swings out to the character's right: silhouette x min 66 → 28 (38 px ≈ 140 mm). Club top rises 114 → 80 (**≈ 120 mm**). Silhouette area −4 % (leaning back) |
| 15–20 | return | Arms come down; the club is back at f20 |
| 20–26 | settle | eyes 2–5 px from f1 |
| 26–39 | still | residual ≈ 2 px vs f1 (not a perfect loop) |

Feet planted for the whole clip (feet bbox shifts by ≤ 2 px).

## 2. Design decisions
- **f1 = attack_swing f1 = idle f1 exactly** (DEF ≤ 1e-4), so transitions from idle or attack into hit and back do not pop. **f39 = f1.** The clip ends in the ready pose.
- **Timing follows the reference.** f1–5 still, flinch f6–9, recoil f10–14, return f15–20, settle to f26, still f26–39.
  - For game response, the still head (f1–5) and tail (f27–39) can later be trimmed in the Unity importer (clip start/end frames) without re-authoring. That is not done now; the user decides after seeing it in Unity.
- **Redesign within the rig's safe range; do not copy.**
  - The reference squashes and stretches the body (belt −89 / +78 mm). We use **no scale** (DEF scale 1, as in idle).
  - Instead, we show the flinch with a COG drop plus knee bend, a forward curl of spine and chest, the head pitched down, and hunched shoulders.
  - We show the recoil with a back bend of spine and chest, the head pitched back, and both arms flung out.
- **Flinch key (around f8):**
  - COG down 40–70 mm. The safe limit is cog_dz ≥ −80 mm; legs stay IK and planted.
  - Spine and chest curl forward. Head pitched down. Both shoulders raised (protective hunch).
  - Arms drawn in slightly. Club top tilts outward/down as in the reference, with club clearance ≥ 10 mm.
- **Recoil key (around f12–13):**
  - COG back to about the ready height. There is almost no room above: the ready pose is COG −15 mm, and ik_ratio must stay ≤ 0.97.
  - Spine and chest bent back. Head pitched back, face up.
  - Left arm flung out and up: fist rise about 120–220 mm, out about 80–150 mm.
  - The right arm swings out to the right and raises the club (about +120 mm at the club head). The right upperarm raise must stay ≤ 100°; the club passes through the head above about 105°.
  - Club clearance ≥ 10 mm to the body and head.
- **Eyes** are head geometry (no blink shapes), so the half-closed eyes of the flinch are not reproduced.
- **Root motion: locked** (CTRL_root identity). The reference does not travel.
- Safe range, clearance, visible self-intersection and contact follow the d-25 §2 conventions.

## 3. Motion amplitude (report-only; the user judges the feel [U])
Relative to f1 (check_anim_clip M2 summary: `max` and `min` over the clip):

| Item | Target |
|---|---|
| Eye-point rise at recoil (`head_rise_mm.max`) | 10–40 |
| Eye-point drop at flinch (`head_rise_mm.min`) | −50 to −100 |
| Left fist rise (`fist_center_rise_mm.max`) | 120–220 |
| Left fist out (`fist_center_out_mm.max`) | 80–150 |
| COG drop (C2 `cog_dz_m` at the flinch key) | ~~−0.04 to −0.07~~ → **≥ −0.036 (measured limit, T90 2026-09-25)**: with pelvis 8°, −0.040 gives a visible left calf/thigh crease (7 pairs) and hip_flex_l 60.7 (> 60); without the pelvis tilt, −0.030 already gives 29 visible pairs. So the flinch drop is 21 mm below ready (ready = −0.015), not the reference's 66–89 mm |
| Club tip movement (`club_tip_move_mm.max`) | reported, no target |

The JSON `targets` holds only the ranges that check_anim_clip checks against `max`: head_rise_mm, fist_rise_mm, fist_out_mm. The `min` items are read from the summary. All of these are report-only. They are a first calibration and will be revised after measuring our clip, with the basis recorded here.

## 4. Gates (main judges; [U] needs user approval)
Gate ids reach check_anim_clip through a `gate_id` mapping in `rig/data/clips/hit.json`:

| C-id | Gate id |
|---|---|
| C1 | H1.1 |
| C2, C3, C4 | H1.2 |
| C5 | H2.1 |
| C6, C7, M1 | H2.2 |
| C8, C9, C10 | H2.3 |
| M2 | H2.4 |

C2 and M2 are report-only.

| ID | Condition | Verification |
|---|---|---|
| H1.1 | Key poses f1 (= attack f1), flinch (around f8) and recoil (around f12–13) are recorded in `rig/data/clips/hit.json` (idle.json schema) | check_anim_clip |
| H1.2 | Per pose: clearance ≥ 10 mm, visible self-intersection cover ≤ 1, contact feet planted, safe range reported | check_anim_clip |
| H1.3 [U] | Reference vs ours key-pose sheet (front = swing_cam + ¾) | render |
| H2.1 | action `hit`: 24 fps, 1–39, CTRL/PROPS keys only; key poses match (≤ 1e-4) | check_anim_clip |
| H2.2 | f39 = f1 (≤ 1e-4); **f1 = attack_swing f1 DEF (≤ 1e-4)**; CTRL_root motion 0 | check_anim_clip |
| H2.3 | Every frame: clearance ≥ 10 mm, visible cover ≤ 1, branch flip 0, both feet planted the whole clip (≤ 1 mm) | check_anim_clip |
| H2.4 | Motion amplitude report (§3) | check_anim_clip |
| H2.5 [U] | Reference side-by-side video (24 fps) and contact sheet | render |
| H3 | Same as A3.1–A3.3 for `goblin@hit.fbx` (bake, FBX, Unity bones/club/bounds/root/importer). The prefab bounds are recomputed over the union of all clips; the flung arms may enlarge them, so re-run the A3, I3 and G9 checks afterwards | check_a3_clip --action hit (gate prefix H3) |
