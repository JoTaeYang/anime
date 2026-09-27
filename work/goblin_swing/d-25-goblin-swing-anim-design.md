# Goblin club swing animation: design and Acceptance Gates (2026-09-25)

## 0. Goal
- Make one game attack clip `attack_swing` with the finished rig (`rig/gob_r04_ctrl.blend`, G1–G10 PASS), then export it to Unity through the G7–G9 pipeline.
- **Follow the reference's timing, flow of force and silhouette feel, and redesign the poses inside this rig's safe range** (user decision, 2026-09-25). Do not copy the reference frame by frame.
- Reference: `work/goblin_swing/ref/` (reference.mp4, 39 frames at 24 fps; REFERENCE_ANALYSIS.md; keyframes/key_00NN.png; metrics.json; club_track.json).
- Grip: one-handed right hand, unchanged throughout the clip. The current rig socket grip matches the reference: head 30–72 cm in front of the fist, butt 24 cm behind.

## 1. Kept from the reference (timing, frames 1:1)
| Section | Frames | Content |
|---|---|---|
| idle | 1–5 | Club held vertical beside the right hip, head up |
| wind-up | 6–12 | Club raised up and back. Body stretches; hips rise slightly. Right foot steps out. f9 is the anticipation peak, f12 the maximum wind-up |
| swing | 13–15 | 2-frame swing. f13 is the top of the arc |
| impact | 15 | Club head on the ground in front of the left foot. Deepest crouch, about 25° lean toward the left, head pitched down |
| follow-through hold | 15–23 | Hold the impact pose with small settling |
| recovery | 24–34 | f24 is the maximum lean at 27°. Right foot steps back (**a step, not a slide**). f29 is mid-recovery |
| settle / loop | 35–39 | f39 = f1 |

## 2. Adapted for our rig (safe range; draft, fixed after A1 measurement)
Numbers are based on G5/G6 measurements. Angles are the controls' local rotations using the sign convention in the `_doc` of `rig/data/rom_poses.json`.

| Item | Limit (draft) | Basis |
|---|---|---|
| Upperarm raise (FK) | ≤ 100° | G5 ROM: 90° is acceptable, 130–170° collapses |
| Extra height | CTRL_shoulder up to +25° | The shoulder bone takes the extra height |
| Upperarm forward/back | ≤ 90° / ≥ −45° | G5 ROM |
| Elbow flexion | ≤ 100° | 140° folds |
| Wrist bend / twist | ≤ 40° / ≤ 90° (actual DEF hand angle relative to lowerarm, check_a_swing definition) | 60° bend collapses; twist 90° is 0.88 |
| Hip flexion / knee | ≤ 60° / ≤ 90° | G5 ROM |
| COG drop | ≥ −0.08 m | Leg length 150 mm (G6 sweep) |
| Spine and chest controls | each ≤ 30° per axis | G6 sweep range |
| IK target distance | ≤ 0.97 × limb length | IK sensitivity near a straight limb (G6, G9 f371) |
| Club clearance | ≥ 10 mm from body, head, left arm and legs | Mesh-to-mesh minimum distance. The right hand gripping it and the fist hole are excluded |
| Foot roll / bank | Within ctrl_manifest clamps | G6.7 |

Adaptation principles:
- **f12 maximum wind-up:** do not lay the club fully behind the head. Build height and a sense of pulling back from these: upperarm raise ≤ 100°, shoulder bone up, torso leaning back and twisting to the right, head clearance ≥ 10 mm. Seen from the front camera, it should read as "the club is cocked high over the right shoulder".
- **f13:** the arm does not fully extend. Keep a small elbow bend, and animate in FK.
- **f15 impact / f24 follow-through (user decision after A1 round 1, 2026-09-25):** less hunched, like the reference. Hip→eye lean about 25–30°, face visible from the front camera. The club head does not need to touch the ground; it may stop within ~60 mm above it. The arm-crossing feel comes from the arm swing plus moderate torso twist (≤ 30°). Wrist bend ≤ 40° (DEF). Crouch COG ≥ −0.08 m.
- **Club path:** keep ≥ 10 mm clearance from body and head in every frame.

## 3. Production method
- Animation is built only from CTRL/PROPS keys (no direct DEF keys). Arms mainly in FK, legs in IK (feet planted); ready_pose is the base stance.
- A key pose script produces the keys (reproducible). Blender GUI is for the user's viewing only.
- The action name is `attack_swing`, 24 fps, frames 1–39. Root motion is **locked** (spec §6 clip list: attack_swing root translation locked).
- The rotation mode is the controls' own (XYZ). If a gimbal problem appears, report it.

## 4. Acceptance Gates (main judges; [U] needs user approval)
### A1 KEYPOSE
| ID | Condition | Verification |
|---|---|---|
| A1.1 | Key poses at reference frames 1, 9, 12, 13, 15, 24, 29 (7 poses) are defined as CTRL values in `rig/data/swing_keyposes.json` | check_a_swing |
| A1.2 | Report each pose's safe-range values (joint angles, COG, IK distance ratio). Values outside the range are listed with a reason | check_a_swing |
| A1.3 | For each pose: club clearance ≥ 10 mm (report the minimum and the location). **Visible** self-intersection cover ≤ 1: intersecting pairs whose intersection point lies inside a rigid shell (head, hands, shoes, belt) are excluded (report only); the total before exclusion is also reported. (User decision 2026-09-25: invisible intersections such as the hidden body dome inside the head were blocking head turns and arms hanging down) | check_a_swing |
| A1.4 | For each pose, feet in contact are planted: IK reach ≤ 1 mm, lowest shoe z ∈ [−1, +3] mm. Which foot is in contact in which pose is recorded in the JSON | check_a_swing |
| A1.5 [U] | Reference keyframe and our pose rendered side by side (front camera matched to the reference framing + ¾ view). The user approves the "feel" | render |

### A2 ANIM
| ID | Condition | Verification |
|---|---|---|
| A2.1 | Action `attack_swing`: 24 fps, frames 1–39, keys on CTRL/PROPS only. Key poses from A1 are at their frames (DEF difference ≤ 1e-4) | check_a_swing |
| A2.2 | Loop: DEF world at f39 = f1 (≤ 1e-4). CTRL_root has no motion in any frame (root motion locked) | check_a_swing |
| A2.3 | Every frame: club clearance ≥ 10 mm; visible self-intersection cover ≤ 1 (A1.3 definition) | check_a_swing |
| A2.4 | Every frame: 0 branch flips (G6.5b definition). Change in DEF rotation between adjacent frames is reported as the maximum per section (swing section 13–15 is expected to be large) | check_a_swing |
| A2.5 | In contact sections, the contacting foot's DEF world movement ≤ 1 mm. Contact sections are defined in swing_keyposes.json | check_a_swing |
| A2.6 [U] | Side-by-side video/contact sheet of the reference and our animation at 24 fps, same frame numbers. The user approves | render |

### A3 EXPORT
| ID | Condition | Verification |
|---|---|---|
| A3.1 | Bake: `build_export(..., "attack_swing", ...)`. G7.4 criteria on all frames (≤ 1 mm / 1°) | G7-type comparison |
| A3.2 | FBX `export/goblin@attack_swing.fbx`: reimported, G8.3/G8.5 criteria | G8-type comparison |
| A3.3 | Unity: GoblinImport applied (Generic, CopyFromOther). Integer-frame samples ≤ 1 mm / 1° (G9.4 criteria), club (G9.5 criteria). All 39 frames inside goblin.prefab bounds (widen the prefab if outside, then recheck). root translation = 0 in every frame | Unity + compare |

## 5. Undecided items
- The G6.10 control-feel check has been approved by the user (2026-09-25).
- jump root-motion rule: not needed for this clip.
