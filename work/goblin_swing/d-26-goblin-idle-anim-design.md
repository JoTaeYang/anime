# Goblin idle animation: design and Gates (2026-09-25)

## 0. Goal
- Make one game idle loop clip, `idle`, with the finished rig (`rig/gob_r04_ctrl.blend`), and export it to Unity with the A3 pipeline.
- Reference: `work/goblin_swing/ref_idle/` (reference.mp4 is a copy of the source KakaoTalk_20260920_201614811.mp4, same md5; frames/f_0001–0039.png). 39 frames, 24 fps, 448×576, static camera. This is the same framing as the swing reference, so `rig/data/swing_cam.json` can be reused.
- Procedure: `docs/rig-pipeline-playbook.md` §4. Key poses (I1) → animation (I2) → export (I3).

## 1. Reference measurement (main, pixel tracking; 1 px ≈ 3.7 mm with swing_cam 271 px/m)
| Item | Measured |
|---|---|
| Timing | f1–15 inhale (rise), f13–17 peak hold, f17–26 exhale (return), f26–39 still |
| Head / eyes | up 5–6 px ≈ 18–22 mm (eye y 168.2 → 163.0, head top 110 → 104) |
| Body | olive silhouette area +2.8% at the peak (inhale expansion) |
| Left fist (free hand) | up about 5 px and out about 2.5 px ≈ 18 mm / 9 mm |
| Right hand and club | club tip y constant (114), x change ≤ 1 px → effectively still |
| Loop | the source is not a perfect loop (f39 vs f1 has a ~2 px residual). Our clip enforces f39 = f1 |

## 2. Design decisions
- **f1 = attack_swing f1 exactly** (the same CTRL/PROPS values, DEF difference ≤ 1e-4). This keeps idle ↔ attack transitions and blends free of pops. The ready pose is the stance.
- **Inhale peak (around f15):**
  - Lift the chest a little and bend it slightly back (spine/chest X small negative).
  - Raise the shoulders slightly.
  - Pitch the head slightly up and move it up.
  - Lift the COG a few mm; the legs stay planted and within reach.
  - The left arm rises with the shoulder and opens slightly.
  - **The right hand and club keep their world position** (≤ 5 mm; compensate with the arm).
  - Scale is not used: DEF scale stays 1. The belly expansion is conveyed with chest lift and shoulders instead.
- **Exhale:** return to the f1 pose by f26. f26–39 is essentially still, and may include a very small settle (≤ 2 mm).
- **Motion amplitude targets:** head rise 15–22 mm (**measured at the eye point**, the same basis as the reference measurement: the world z rise of the mean position of the head part's eye-ring vertices. The DEF head bone head sits at the neck joint and barely moves with rotation, so it is not used. 2026-09-25 correction), left fist rise 12–20 mm, club tip movement ≤ 5 mm. These are report-only; the user judges the feel [U].
- Safe range, clearance, visible self-intersection and contact follow the d-25 §2 conventions.

## 3. Gates (main judges; [U] needs user approval)
| ID | Condition | Verification |
|---|---|---|
| I1.1 | Key poses: f1 (= attack f1) and the inhale peak (the frame is set by the production side based on the reference, around f15) are recorded in `rig/data/clips/idle.json` (same schema as swing_keyposes: frame, name, contact, ctrl, props, def snapshot, plus contact_ranges) | check_anim_clip |
| I1.2 | Per pose: clearance ≥ 10 mm, visible self-intersection cover ≤ 1, contact feet planted, safe range reported | check_anim_clip |
| I1.3 [U] | Reference vs ours sheet (front camera = swing_cam + ¾) | render |
| I2.1 | action `idle`: 24 fps, 1–39, CTRL/PROPS keys only; key poses match (≤ 1e-4) | check_anim_clip |
| I2.2 | f39 = f1 (≤ 1e-4); **f1 = attack_swing f1 DEF (≤ 1e-4)**; CTRL_root motion 0 | check_anim_clip |
| I2.3 | Every frame: clearance ≥ 10 mm, visible cover ≤ 1, branch flip 0, both feet planted for the whole clip (movement ≤ 1 mm) | check_anim_clip |
| I2.4 | Motion amplitude report: head rise, left fist rise, club tip movement (the §2 targets, report-only) | check_anim_clip |
| I2.5 [U] | Reference side-by-side video (24 fps) and contact sheet | render |
| I3 | Same as A3.1–A3.3 (bake, FBX, Unity bones/club/bounds/root/importer) for `goblin@idle.fbx`. The prefab bounds are recomputed over the union of all clips | check_a3_clip (--action idle) |
