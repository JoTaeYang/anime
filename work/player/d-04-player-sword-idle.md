# Player clip: Sword_Idle (2026-09-27)

## 0. Reference
- File: `C:\Users\whxod\Downloads\player-idle.mp4` (read-only)
  - sha256 deb39cb3…77f8cd
  - 512², 24 fps, 73 frames (3.04 s)
- Frames extracted to `work/player/refs/idle/f001–f073.png`; contact sheet `sheet_every4.png`.
- **Main's reading (for the design, not to copy):**
  - **Pose:**
    - Right hand holds a one-handed sword upright; the blade points up and slightly outward.
    - Elbow bent about 90°, fist in front of the belt at hip height, a little to the side.
    - Left arm hangs relaxed, slightly away from the body.
    - Feet about shoulder width, knees straight, torso upright.
  - **Motion:** one breath per loop.
    - Head top rises about 4 px (≈ 1 % of the character height, ≈ 10 mm at 1.1 m) and the scarf about 7 px (≈ 17 mm, shoulders), measured by pixel tracking.
    - Inhale f1 → f35, exhale → f58, hold → f73 (24 fps).
    - A blink at f36–40.
  - Sword length ≈ 0.4 × the character height.

## 1. Decisions
- **User (2026-09-27):**
  - The clip is **Sword_Idle**: part of the sword set; hammer and dagger idles come later.
  - **No blink for now:** the eyes have no lids; blinking can be added later as face work.
- **main defaults:**
  - 30 fps, 91 frames (f91 = f1, loop), root_motion `in_place_cycle`, both feet planted on every frame.
  - Timing mapped from the reference (t = (f24 − 1) / 24 → f30 = 1 + 30t): inhale peak ≈ f43, back to rest ≈ f72, hold to f91.
- **Weapon for preview only:**
  - A placeholder sword (grip, guard, blade; ≈ 0.44 m) attached to `WeaponSocket_R` in the Blender sheets and the Unity renders. It is never part of Player.fbx.
  - This fixes the **weapon socket convention**: the pivot is at the grip centre, and the blade and grip axes follow the socket's local axes. Production proposes the convention in `rig/data/weapon_socket_contract.json`, and main approves it together with the key poses.

## 2. Steps and gates (prefix SI)
1. **Key poses:**
   - f1 (rest / exhale), f43 (inhale peak) and f72 (return; = f1 plus the settle), with the sword held.
   - Tools: the clip JSON and p30.
   - Sheet: the reference frame at the matching time next to our pose, front and 3/4.
   - **[U] SI1** approval.
2. **In-betweens and full timing:**
   - Overlap: the arms and the sword lag the torso by about 2–3 frames; the skirt follows with skirt_follow.
   - Checks C1–C7 (d-03 §3).
   - A side-by-side video (reference | Blender) → **[U] SI2**.
3. **Export and Unity:**
   - U1–U4.
   - Unity capture with springs and the placeholder sword → **[U] SI3**.
   - Then delete the smoke clip (d-03 §5).

| ID | Row | Status |
|---|---|---|
| SI.C1–C5 | d-03 §3; C3 planted: both feet on every frame; C5 loop seam | blocking |
| SI.C6, SI.C7 | skirt poke, self-intersection (sleeve vs scarf judged against rest) | report |
| SI.C8 | Sword vs body: the placeholder blade and guard must not intersect the head, torso or arm on any frame (the mesh-intersection rule, placeholder vs PL_mesh) | report on this first clip, then blocking |
| SI.U1–U4 | d-03 §3 | U1–U3 blocking, U4 report |

## 3. SI1 result (2026-09-27; production T310, checker T311)
- **Checker rows:** main re-ran check_player_clip `--prefix SI`: 8/8 rows identical to production's evidence, inputs identical.
  - C3 slip 0.016 mm
  - C5 seam 0
  - C6 poke 0
  - C8 sword vs body 0 pairs, min blade clearance 72 mm (arms)
  - C7 arm vs tunic 21 / 15 pairs, 27 mm under the sleeve at the armpit (rest 0), report
- **Main's view of `SI1_keypose_sheet.png`:**
  - The pose matches the reference layout.
  - The elbow is ~115° instead of ~90°: this arm length would put the fist above the belt at 90°.
  - The placeholder sword is shorter than the reference's.
  - Breathing: head +8.9 mm, shoulders +13.6 mm.
- **User approved SI1 and the weapon socket contract** (2026-09-27):
  - pivot = grip centre at the socket head
  - blade along socket +Z
  - guard along socket Y
  - Child Of the socket, identity offset
  - The contract's status → approved.

## 4. SI2 result (2026-09-27; production T312)
- **In-betweens:**
  - Breathing: BEZIER with AUTO_CLAMPED handles, plus a tiny drift f72→f91.
  - The arms are keyed 3 frames early, giving a measured lag: fist R +2.3 frames, fist L +2.0, sword tip +2.75, all behind Spine_02.
  - The head counters the chest (+1°, 3-frame lag).
  - Approved key values are unchanged except the evaluated drift: upper arm Z ±0.15° at f43, head X +0.98° at f43.
- **Seam velocity:** difference ≤ 0.0043 °/frame and 0.006 mm/frame. The seam is a near-zero-velocity turnover.
- **Main re-ran check_player_clip `--prefix SI`:** 8/8 rows identical to production's.
  - C3 slip 0.015 mm, C5 seam 0, C6 0, C8 0 pairs.
  - C7 arm vs tunic R 38.2 mm (f42).
  - **Main rendered the armpits at f1 / f42 from the back and the side:** the overlap is hidden under the sleeve, with no visible poke-through.
- **Main's p30 fix:** `weapon` is omitted from the resolved JSON when the clip has none. T312 found that smoke's C8 was blocked by `"weapon": {}`. After the fix, smoke re-ran exit 0: 7 C rows with no C8, U rows ok.
- **Media:**
  - `SI2_side_by_side.mp4` / `.gif`: reference | front | 3/4, 3 loops.
  - `SI2_upper_2x.mp4`: upper body at 2×. Main viewed the frames: the breathing is visible in the 2× video, and the layout matches the reference.
- **[U] SI2:** pending.
- **User approved SI2** (2026-09-27).

## 5. SI3 plan
- **Export and Unity checks:** `run_player_clip.ps1 -Clip Sword_Idle` (p31 → PlayerClipCheck → U1–U4).
- **Weapon in Unity (preview):**
  - The placeholder is exported as its own FBX (`Assets/Player/Preview/WPN_sword_placeholder.fbx`, never part of Player.fbx).
  - PlayerClipCheck attaches it under WeaponSocket_R with an identity local transform (per the approved contract) when the clip JSON has `weapon`.
  - It also reports per frame the world positions of the placeholder's blade tip, guard ends and pommel.
- **New row SI.U5:** weapon convention through the pipeline. Unity blade tip, guard ends and pommel vs Blender (socket world × contract), mapped by the preset axes. Report on the first run; the expected blocking value is ≤ 0.1 mm.
- **[U] SI3:** Unity capture sheet and a GIF (front and 3/4, springs r15 + follow, sword).
- **After SI3 passes:** delete the smoke clip (d-03 §5).

## 6. SI3 result (2026-09-27; production T313, checker T314)
- **Weapon compensation:**
  - Unity bone frames equal the axis-mapped Blender frames up to an x-flip (as axes), i.e. Rx(90°) as rotations. WeaponSocket_R and _L are the same within 5e-7.
  - Rx(+90°) is baked into the weapon FBX (bake_space_transform). The game attaches weapons as identity children of WeaponSocket_<side>.
  - Recorded in `weapon_socket_contract.json` `unity`; the approved Blender fields are unchanged.
- **main ran `run_player_clip.ps1 -Clip Sword_Idle`:** exit 0, 43 s. Every C1–C8 and U1–U5 row equals production's evidence, except render-list and timing fields.
  - U1: 0.0005 mm / 0.0007°
  - U2: body 0.0005 mm / 0.0001°
  - U3: 0 outside (union 797 frames)
  - U4: sway p95 3.5 / max 4.2 mm, poke 0 / 0
  - **U5: sword tip / guard / pommel in Unity vs Blender max 0.0005 mm.** This fixes U5 as blocking at ≤ 0.1 mm (basis: 0.0005 measured, 200× margin).
  - C8 is blocking from the next clip: 0 pairs (d-04 §2).
- **Main viewed `SI3_unity_sheet.png`:** the sword sits upright in the right fist as in Blender. Skirt, springs and materials look right.
- **Known:** the wrapper runs the checkers without `--prefix` (row IDs are plain). Left-socket attachment is untested in Unity.
- **[U] SI3:** pending.
- **User approved SI3** (2026-09-27). **Sword_Idle is done.**
- **Cleanup and final state:**
  - The smoke clip was moved (not deleted) to `rig/archive/smoke_2026-09-27/`: JSON, anim, stage, FBX, Unity FBX + meta, report, inspect.
  - `run_player_clip.ps1` gained `-Prefix` (main).
  - Final `run_player_clip.ps1 -Clip Sword_Idle -Prefix SI`: exit 0; SI.C1–C8 and SI.U1–U5 all ok; bounds union 767 frames (rigtest 676 + Sword_Idle 91).
  - `run_player_gates` run 8: 101/101 ok; the P2 report clips are rigtest and Sword_Idle.
