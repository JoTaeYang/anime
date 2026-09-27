# Player clip: Sword_Attack_01 (2026-09-27)

## 0. Reference
- **File:** `C:\Users\whxod\Downloads\player-sword3.mp4` (read-only)
  - sha256 6fd80563…f13aef
  - 512², 24 fps, 39 frames (1.625 s)
- **Extracted:** `work/player/refs/sword3/f001–f039.png`, with sheets `sheet_every2.png` and `sheet_action.png` (f6–f24).
- **Main's reading** (24 fps frame numbers; for the design, not to copy):

  | Ref frames | Phase | What happens |
  |---|---|---|
  | f1–5 | Idle | The Sword_Idle pose |
  | f6–10 | Anticipation / wind-up | The sword rises to the character's right and up. The body leans right. The left foot kicks back and up (weight onto the right leg). |
  | f11–13 | Strike | A horizontal slash at chest height from the character's right to left, with the arm extended. The left foot lands wide at f12. |
  | f13–19 | Follow-through hold | The arm is crossed in front of the body, the sword points to the character's left, the torso is twisted left, and the stance is wide with slightly bent knees. |
  | f20–26 | Recovery | The sword comes down and back across the body. The feet return to the idle width. |
  | f27–39 | Idle | Settled |

- **Time mapping to 30 fps** (f30 = 1 + 30 · (f24 − 1) / 24):

  | Ref frame (24 fps) | Clip frame (30 fps) |
  |---|---|
  | f6 | 7 |
  | f10 | 12 |
  | f11 | 13.5 |
  | f13 | 16 |
  | f19 | 23.5 |
  | f23 | 28.5 |
  | f27 | 33.5 |
  | f39 | 48.5 |

## 1. Decisions
- **User (2026-09-27):**
  - The clip name is **Sword_Attack_01**.
  - **Hit events:** `hit_start` / `hit_end` go in the clip and become Unity AnimationEvents.
- **Main defaults:**
  - 30 fps, frames 1–49, no loop.
  - root_motion `locked`: the character ends where it started; the step out and back is in place.
  - The first and last frames equal the Sword_Idle f1 pose, so that clips blend and chain cleanly.
  - skirt_follow 0.5, with the placeholder sword on R.
  - **Contacts:**
    - The right foot is planted for the whole clip.
    - The left foot is planted before the lift, while it stands wide, and after the return. Production states the exact ranges.
  - **Events:** a clip JSON field `events`: `[{name: "hit_start", frame}, {name: "hit_end", frame}]`. The draft window is the strike, where the blade sweeps across the front (about f13–f16). The exact frames come with the key poses, from where the blade tip actually crosses the front arc.

## 2. Gates (prefix SA1)
| ID | Row | Status |
|---|---|---|
| SA1.C1–C5 | d-03 §3 (C3 on the declared planted ranges; C5 locked) | blocking |
| SA1.C6, C7 | skirt poke; self-intersection vs rest | report (the user judges the sheets). The high-knee kick (C6) is the case d-02 §11 deferred to clips. |
| SA1.C8 | sword vs body | **blocking** (fixed after Sword_Idle: 0 pairs) |
| SA1.C9 | first and last frames equal the Sword_Idle f1 pose: every DEF bone within 0.01 mm / 0.01°, and the PROPS equal | blocking candidate; report on this first run |
| SA1.C10 | events: the names are known, the frames are inside the range, hit_start < hit_end, and the blade tip is moving (speed reported) inside the window | report |
| SA1.U1–U4 | d-03 §3 | U1–U3 blocking, U4 report |
| SA1.U5 | weapon Unity vs Blender | blocking, ≤ 0.1 mm |
| SA1.U6 | the Unity clip carries AnimationEvents `hit_start` / `hit_end` at time (frame − first) / fps, within half a frame | blocking candidate; report on the first run |
| [U] | SA1-1 key poses → SA1-2 side-by-side video → SA1-3 Unity capture | user |

## 3. SA1-1 round 1 (2026-09-27, T320)
- **Checks:** main re-ran check_player_clip `--prefix SA1`: 10/10 rows identical to production's.
  - C8: 0 pairs
  - C9: first and last = Sword_Idle f1, 0 / 0
  - C6: 0 (no knee poke at the kick)
  - C10: the window f13–16 contains the tip peak at f14
- **Main's view of the sheet:**
  - f12 wind-up: weaker than the reference. The sword stays beside the head, and the left-foot kick (0.10 m back, 0.08 m up) barely shows from the front.
  - f14: already in front of the body. At this point the reference still has the arm extended to the right with the blade trailing.
  - Follow-through and recovery match the reference's direction.
- **User:** "준비·베기 강하게 다시" (redo the wind-up and strike stronger). T322.

## 4. SA1-1 round 2 (2026-09-27, T322)
- **Changes:**
  - f12: the arm is up and out to the side, the sword above the shoulder, a clear lean (torso Z 8°, chest Z 10°).
  - f12 kick: back 0.15 m, out 0.06 m, heel up 0.18 m, foot pitch −30°.
  - f14: the arm is extended right with the blade trailing.
  - No sweep_overrides.
  - Events are now hit_start 14 / hit_end 16. The tip peaks at f15 (18.3 m/s, inside the window).
- **Main re-ran the check:** 10/10 rows identical, inputs identical.
  - C8: 0 pairs (head clearance 125 mm)
  - C6: 0
  - C9: 0 / 0
  - C7: sleeve vs scarf R 62 at f10 (rest 31)
- **Main's view of `SA1-1_r2_compare.png`:** the f12 silhouette (raised sword, lean, kick) and the f14 extended trailing arm now read like the reference. From the front, the f12 sword is seen nearly edge-on.
- **User approved SA1-1 round 2** (2026-09-27). Next: SA1-2 (T323).

## 5. SA1-2 round 1 (2026-09-27, T323)
- **Checks:** main re-ran them: 10/10 rows identical.
  - C8: 0 pairs on every frame
  - Events 14–16, peak f15 at 18.3 m/s
  - The blade path is an outward arc: up to 0.58 m outward of the chord, radius 0.53–0.73 m.
- **Main's view of the video frames f23–f32:** the recovery snaps from f27 (sword low in front) to f28 (already at the idle side). Tip speed 13.7 → 1.2 m/s.
- **User:** "복귀만 부드럽게" (smooth only the recovery). T324 spreads f24–f31 through the low-right arc with an ease-out; the rest stays unchanged.
- **T324 recovery rework:**
  - Right-arm keys f24–f32 are solved from one sword path; f19–f22 arm Y is re-keyed with the old values so f1–f23 stay exact.
  - Tip speed peaks at f27 (15.5 m/s), then falls by ≤ 2.07× per frame (was 11.8× at f28→29).
- **Main re-ran the check:** 10/10 rows identical. C8 0 pairs; C7 fist vs torso 0.
- **Main viewed video frames f23–f32:** down in front → low right with the blade down and out (f28) → rising → idle. This lines up with ref f19–f26.
- **User (SA1-2 round 2): not approved.** "팔을 휘두를 때 몸통을 관통하잖아" (the arm goes through the torso when it swings).
  - Main confirmed it in the video frames: at f15–f19 the right sleeve and arm sink inside the torso outline (top view) and into the scarf.
  - **Checker gap:** C7 covered only fist vs head, fist vs torso, sleeve vs scarf and arm vs tunic. It had no sleeve vs tunic and no arm vs scarf pair, so "fist vs torso 0" hid the problem. Main also missed it on the sheets.
- **Plan (user "진행", 2026-09-27):**
  1. Torso leads: more twist to the left, chest + spine ≈ 45–50° total, so the right shoulder comes forward.
  2. Keep the arm off the body: the fist ≥ ~0.3 m in front of the chest through the strike and the hold, with less elbow flexion.
  3. New row **C11 arm vs body**: all arm parts (sleeve, arm tube, fist, both sides) vs all body parts (tunic, belt, pouch, scarf, scarf tail, head).
     - Measured: visible intersecting pairs and max visible depth per frame, as the excess over the same pair at rest.
     - Report on this first measured run (Sword_Idle and Sword_Attack_01), then the threshold is fixed from the distribution. The intended rule is "no visible penetration beyond rest".

## 6. SA1-2 round 3 (2026-09-27, T325–T327 checker, T326 production)
- **Root cause of the arm through the torso:** a sign error. On CTRL_chest / spine_01 / torso, rot Y+ twists the character to its **left**, and production had used the opposite sign since T320. So the round-2 follow-through was twisted to the right and pulled the right arm into the chest.
- **Fix:**
  - Left twist at f16–f23: chest + spine ≈ 48–51°, plus hips 3–9°. The chest is at its sweep limit of 30.
  - Fist 0.34 m in front of the chest (was 0.15–0.17 m).
  - Shoulder_r shrug 10°.
  - The wind-up arm is kept outside the head.
  - The recovery was re-solved from the new hold.
  - f1, f33 and f49 are unchanged (C9 0).
- **C11** (new; baseline = the approved Sword_Idle f1; report this run) — newly-inside torso vertices:

  | Arm part | Round 2 | Round 3 | Sword_Idle's own breathing |
  |---|---|---|---|
  | sleeve_r | 57 / 136 mm | 21 / 48 mm (f22) | ≤ 4 verts / 3 mm |
  | arm_r | 134 / 131 mm | 24 / 35 mm (f18) | ≤ 4 verts / 3 mm |

  - Remaining visible pairs: rigid sleeve vs scarf when the arm is raised (sleeve_r~scarf 69 / 42 mm at f15; sleeve_l~scarf 47; sleeve_r~head 22 / 12.5 mm at f11). This is the known P1/P2 finding: the sleeve pushes into the scarf when the arm goes forward.
- **Main re-ran check_player_clip:** 11/11 rows identical. C8 0 pairs; events 14–16 with the peak f15 inside.
- **Main viewed `SA1-2_r3_compare.png`:** at f15–f19 the top view shows the torso turned left, with the arm in front of the body, not inside it. The front view reads like the reference. The sleeve tucks under the scarf edge.
- **User approved SA1-2 round 3** (2026-09-27). The sleeve vs scarf crossing is handled separately: a rig corrective, so that raising the arm makes the sleeve's inner edge give way. It applies to every clip and is in the backlog (§7).
- **C11 stays report-only** until the sleeve corrective exists. Then the threshold is fixed from the distributions of Sword_Idle and Sword_Attack_01. Intended rule: no visible penetration beyond the baseline, with a tolerance of about the idle's own ≤ 4 verts / 3 mm.

## 7. Backlog from this clip
- **Sleeve corrective:** when the arm pitches forward or up, the sleeve's inner edge must not cross into the scarf / tunic / head. Options: a shape key driven by the upper-arm angle, or sleeve weights on the clavicle. Rig-level (P2); re-run every clip afterwards.
- **Unity runtime:** AnimationEvents need a receiver component (hit_start / hit_end) on the player object before the clips play in a scene; otherwise Unity logs "AnimationEvent has no receiver".

## 8. SA1-3 result (2026-09-27, T328)
- **Events:** PlayerClipCheck writes the clip JSON events as AnimationEvents on the clip importer: hit_start 0.4333 s (f14), hit_end 0.5 s (f16). `SampleAnimation` does not fire them, and there was no missing-receiver warning.
- **Main ran `run_player_clip.ps1 -Clip Sword_Attack_01 -Prefix SA1`:** exit 0, 48 s. All 17 rows equal production's evidence (timing and render lists excluded).
  - U1 0.0014 mm / 0.0007°; U2 body 0.0005 mm; U3 0 outside (union 816); U5 0.001 mm.
  - **U6: events exactly on time (0.0 frames).** U6 is fixed as blocking at ≤ 0.5 frame (basis: 0.0 measured; the half-frame is the sampling resolution).
  - **C9 is fixed as blocking at 0.01 mm / 0.01°** (basis: 0 / 0 on both clip ends; needed for clean chaining).
  - Springs: sway p95 26 / max 58 mm (Skirt_F at f16); poke on / off 0 / 0.
- **Main viewed `SA1-3_unity_sheet.png`:** it matches the Blender motion, with the sword in the right hand throughout and no arm through the torso. The camera clips the raised sword at f12, and the f14 blade points at the camera.
- **Unity MP4:** main built `SA1-3_unity_side_by_side.mp4` (reference | Unity front | Unity 3/4, 3 plays with a pad, HIT band) and `SA1-3_unity_slow.mp4` (0.25×) from the Unity renders, at the user's request.
- **User approved SA1-3** (2026-09-27). **Sword_Attack_01 is done.**
- **Main's notes (not blocking):**
  - sleeve / scarf crossing (§7 backlog)
  - a 1-frame strike (a weapon-trail VFX is recommended in game)
  - the left-foot landing snaps at f13→f14 (no toe-first contact)
  - the placeholder sword is shorter than the reference: re-run C8 when the real sword model exists
