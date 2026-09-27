# Goblin roar: mouth shape key and the roar clip, design and Gates (2026-09-26)

## 0. Goal
- Make the game clip `roar` (reference `work/goblin_swing/ref_roar/`).
  - reference.mp4 is a copy of the source KakaoTalk file orc_roar.mp4 (md5 2383234a…): 39 frames, 24 fps, 448×576, the same camera as the other clips.
  - Its f1 matches the idle reference f1 (mean pixel diff 1.7 / 255).
- **The roar needs an open mouth, and the goblin has no mouth** (the head is one rigid part with fangs on the face).
- User decision (2026-09-26): **option B, a "mouth open" shape key.**
  - Model a mouth opening, a mouth interior and lower fangs on the head.
  - Add a shape key `mouth_open` (0 = closed, 1 = the roar mouth).
  - The skeleton stays at 24 bones, so the existing clips' motion does not change.
- Procedure: playbook §3/§4. The riskiest part is proven first with a throwaway round trip (walking skeleton, R0) before any production file changes.

## 1. Reference measurement
main's pixel tracking: `ref_roar/measure_mouth.json`; 1 px ≈ 3.7 mm.

| Frames | Phase | Measured |
|---|---|---|
| 1–4 | still | = f1 (ready). Closed mouth is a thin line between the fangs |
| 5–9 | wind-up | Head top 110 → 140 (−30 px ≈ −110 mm, lowest f7): crouch, head down, arms drawn in |
| 9–10 | mouth opens | Dark-mouth pixels 1.5k → 5.4k within 2 frames |
| 10–26 | roar held | Head top 98–100 (up ≈ 40 mm), head tilted up. Both arms flung up/out, club raised. Feet planted with the toes turned out. Mouth open about 80 % of the face width and about 55 % of the visible head height (≈ 61 px ≈ 225 mm at f15) |
| 27–29 | closes, return | Dark-mouth pixels 4.2k → 1.9k |
| 30–39 | still | Closed |

The first two rows (still, wind-up) are timing-ready for key poses. The mouth size will be measured again on close-ups by the modeling task (R1) against our head (width 565.5 mm, BRIEF).

## 2. Design decisions
- **Mouth = one shape key `mouth_open` on GOB_mesh.**
  - The basis (0) must look like the current face, optionally with a thin mouth line as in the reference. The mouth region is a design change, like the ball fists, so the G2 source-deviation checks exclude it.
  - At 1 it shows the roar mouth: opening, dark interior, upper and lower fangs.
  - All new vertices belong to the head part and are weighted 100 % to DEF head.
- **Control:** a PROPS property `mouth_open` [0, 1] drives the shape key, so s12 can key it like the other PROPS. Interpolation for this property is BEZIER, not CONSTANT; to be confirmed in R2.
  - For export, the stage bakes the driven value into a shape-key animation on the export mesh.
  - R0 tests whether FBX carries a driver or only keys.
- **Unity:** the model is imported with BlendShapes on. A clip carries the curve `blendShape.mouth_open` on goblin_mesh (Unity scale 0–100). Renders use BakeMesh, which includes blend shapes.
- **Existing clips:**
  - The mesh changes, so goblin.fbx and every goblin@*.fbx are re-exported, and the G2 (mouth region excluded), G5, G8, G9 and A3/I3/H3/D3 checks are re-run.
  - Existing clips keep mouth_open = 0.
- **Roar motion:** the same tools as hit (s12, check_anim_clip). f1 = attack f1; loop, f39 = f1; root locked.

## 3. Phases and Gates (main judges; [U] = user approval; numeric criteria marked *draft* are fixed after the first measured run)

### R0 — walking skeleton: shape-key round trip (throwaway, no production file changes)
| ID | Condition | Verification |
|---|---|---|
| R0.1 | On a temporary copy of the export stage, goblin_mesh gets a test shape key that moves a few head vertices, and its value is keyed 0 → 1 → 0. Exported with the project preset (export_preset.json), the FBX contains a BlendShape / BlendShapeChannel and an animation curve on it. A Blender reimport shows the shape key and the same values per frame | probe script output |
| R0.2 | The same test with the value **driven** by a custom property on the armature (no keys on the shape key) shows whether the exporter bakes it. Report only: this decides whether s07b must bake the value | probe script output |
| R0.3 | Unity, in a separate throwaway folder that GoblinImport / GoblinRigCheck / GoblinClipCheck do not pick up: the clip has a `blendShape.<name>` curve, the sampled weights match (×100) per frame, and a BakeMesh of the displaced vertices matches Blender within 0.1 mm (*draft*) | probe C# + JSON |
| R0.4 | Clean-up: every throwaway file (Blender temp, Unity folder, probe C#, .meta) is removed; Assets/Goblin, the prefab and the reports are unchanged (sha256 before = after) | listing + hashes |

**R0 result (T110, main re-verified 2026-09-26): PASS.**
- Keyed and driven shape-key values both reach the FBX as a 39-key DeformPercent curve: the exporter samples KeyBlock.value after every frame_set, so a driver is baked with no extra step.
- Unity gets `goblin_mesh | SkinnedMeshRenderer | blendShape.<name>`. The weight matches Blender ×100 exactly (max diff 0.0000 over 39 frames). BakeMesh vs Blender vertex max is 0.0004 mm.
- Clean-up: Assets/Goblin and goblin_*.json are identical before and after (22 hashes), and the Assets listing is identical.
- Conditions that matter:
  - Armature must be the only modifier on the mesh; any other active modifier makes the exporter evaluate the mesh and drop the keys.
  - Shape keys must be relative.
  - Unity importBlendShapes = true (the default).
- **Threshold fixed from this measurement:** Unity blend-shape weight vs Blender ×100 ≤ 0.01; BakeMesh vertex ≤ 0.01 mm (R4, and the re-verification in R2).

### R1 — mouth modeling [U]
- Head mesh with the mouth (basis = closed) plus the shape key `mouth_open`.
- Renders: closed / open, front and ¾, next to reference f7 / f15.
- Gates:
  - mesh validity: no non-manifold edges outside the designed openings, no zero-area faces
  - basis deviation outside the mouth region = 0
  - triangle budget (*draft*: + ≤ 800 tris)
  - no interior faces visible through the head at 0
  - [U] look

### HR — head retopology (user decision 2026-09-26, after R1 v6)
- Why: the matcap renders show the whole closed head (between the eyes, cheeks, forehead) is uneven, not only the mouth area. The current head is a collapse-decimated, fitted cut of SRC_hi (s02c). A flat, clay-like face around the mouth needs an even retopology of the head.
- **HR0 — measure first (no production change):**
  - matcap and faceted renders of SRC_hi's head vs the current GOB head
  - source roughness: deviation of SRC_hi's head from a smooth fitted surface, excluding eyes and fangs
  - current head triangle and valence statistics
  - the triangle budget: G2.8 allows at most 8,500 in total; the original is 8,122, and v6 adds 582
- HR1 (after HR0; criteria fixed from HR0's numbers):
  - A new head build in s02c: even quads, rounded box, eyes and fangs kept.
  - If the source itself is lumpy, the smoothed head is a design change. Like the ball fists, G2.11 / G2.12 then handle the head separately; the user decides.
  - The mouth (v6 method) is rebuilt on the new head in s02e.
  - The closed face must look like the current one apart from the smoothness.
- **HR0 result (T112, main re-ran it: all 1782 numbers identical):** the unevenness comes from decimation and fitting, not from the source.
  - Front deviation from the smooth reference (MLS R25): SRC 0.024 / 0.068 / 0.18 mm, current GOB 0.44 / 1.20 / 2.1 mm.
  - Shading p95: SRC 2.13°, GOB 3.22°.
  - The even-quad probe at 1600 tris (straddle) reaches 0.29 / 0.99 mm with a clean matcap.
  - Maxima over 3 mm occur only at the head/body crease fold.
  - Eye and fang patches cut from SRC are ragged.
- **Budget: user decision 2026-09-26, G2.8 GOB_mesh ≤ 9,500.**
- **HR1 Gates.** Thresholds come from the HR0 measurements; the production script is s02c (head) + s02e (mouth).

| ID | Condition | Basis |
|---|---|---|
| HR1.1 | Head shell topology: quads everywhere except designed transitions around the eye rings, fangs and crease (triangle count reported); visible edge-length CV ≤ 0.25 **measured outside the eye/fang zones** (clarified 2026-09-26: the basis was the probe shell without its eye/fang patches, 0.149 outside zones; the short eye/fang profile loops are designed) | probe 0.15–0.17, current 0.73; new head outside zones 0.221 |
| HR1.2 | Deviation to SRC_hi, both directions, per region (front / sides / top / bottom-crease / eye zones / fang zones): mean ≤ 1, p95 ≤ 2, max ≤ 3 mm. This is G2.11 with nothing excluded except the mouth opening, which is a design change reported like the ball fists | G2.11 |
| HR1.2 note | **User decision 2026-09-26:** the x<0 eye in SRC_hi is half detached from the face (air gap up to ≈ 36 mm over ≈ 6/16 azimuths, source defect; the x>0 eye is fused). The new head closes it. Like the ball fists, that eye zone is excluded from the SRC→head direction (reported only); head→SRC stays judged | T113b eye_gap_sheet |
| HR1.3 | Smoothness, front / sides / top: shading-angle p95 vs the SRC smooth reference ≤ 2.6°; deviation from the MLS R25 reference p95 ≤ 1.2 mm | SRC 2.13° / 0.07 mm; probe 1600 2.45° / ≤ 1.0 mm; current 3.22° / 1.2–1.5 mm |
| HR1.4 | Eyes and fangs: clean edge loops following the eye-ring and eyeball outlines and the fang plates; the matcap shows no ragged boundary | [U] |
| HR1.5 | G2 static rules on the new head: closed manifold, outward normals, custom normals without NaN, skirt inside the body ≥ 3 mm (G2.14), UV valid with head-front texel density ≥ 1.5 × body (G2.9) | G2 |
| HR1.6 | GOB_mesh triangles ≤ 9,500, including the mouth | user decision |
| HR1.7 [U] | Plain and matcap sheet: source / new head / new head + mouth closed / new head + mouth open (v6 method) | [U] |

- **HR1 PASS (2026-09-26):**
  - HR1.1–HR1.6: main re-ran the measurements (T113d).
  - HR1.7 [U]: the user approved hr1_mouth_sheet.png (T114).
  - GOB_mesh is 9,168 tris.
- **HR2 = R2 (integration), data contract `rig/data/hr2_contract.md`:**
  - Production T120 wires s02e into the chain, adds the PROPS mouth_open driver, bakes the shape-key value in the stage, and adds the s12 per-prop interpolation.
  - Checkers T121 cover:
    - G2: tris ≤ 9,500, design regions for the mouth and eye_gap_xneg.
    - G5: mouth vertices on DEF head.
    - G6: props contract.
    - G7: baked shape-key value = PROPS.
    - G8: FBX blend shape and 4 materials.
    - G9: Unity blend shape and 4 submeshes.
    - A3 family: the clip blend-shape curve vs Blender.
  - Then run_all_gates G1–G10, regeneration of the 4 existing clips (s10a/s10b, s12 ×3), their animation checkers, s11 ×4 + Unity + check_a3_clip ×4 + compare_g9. Existing clips keep mouth_open = 0.

### R2 — integration and re-verification
- PROPS `mouth_open` plus its driver; s07b / s11 bake the shape-key value.
- GoblinImport imports blend shapes; GoblinClipCheck reports the blend-shape curves.
- G2 (mouth region excluded), G5, G6 (props contract), G7–G9 and A3/I3/H3/D3 are re-run on the new mesh. Every existing clip keeps mouth_open = 0.

### R3 — roar clip (Q1 key poses [U], Q2 animation [U])
- Keys:
  - f1 ready
  - wind-up around f7
  - roar peak around f12 (mouth 1)
  - hold with a small shake around f20
  - close/return around f28
  - f39 = f1
- Mouth: 0 until f8, open by f10, held to f26, closed by f29.
- The same check_anim_clip gates as hit (clearance, cover, flips, feet planted for the whole clip).
- Plus mouth: keyed value range [0, 1], and 0 at f1 / f39.

**R3 reference body measurement** (main, `ref_roar/measure_ref.py` → `measure.json`; 1 px ≈ 3.7 mm):

| Frames | Phase | Measured |
|---|---|---|
| 1–3 | still | ready |
| 4–8 | wind-up | Head top 110 → 140 (**−111 mm**, lowest f7); belt −102 mm (squash); the club tilts back/down (top x 70 → 54) |
| 9–11 | burst | Head top 98–100 (**+40 mm** above rest; mostly covered by the mouth key's head stretch). **Left fist out 96 mm, up ≈ 570 mm** (arm raised high). **Club top up ≈ 330 mm** and 32 px inward, above the head |
| 11–23 | roar held | Near constant (±2 px) |
| 24–27 | arms lower | Mouth closes at f27–29 |
| 28–31 | return | back to ready |
| 31–39 | still | ready |

Feet stay planted, toes turned outward during the hold (feet box top y 448 → 441).

**R3 design:**
- Same tools and conventions as hit:
  - f1 = attack f1
  - loop, f39 = f1
  - root locked
  - no scale
- The squash and stretch are replaced by rig motion:
  - The crouch uses the upper body (COG ≥ −0.036 while planted, BRIEF).
  - The stretch comes from the mouth key's head stretch plus a chest and head lift.
- Limits:
  - Right upperarm raise ≤ 100° (the club passes through the head above ≈ 105°).
  - Shoulder raise ≤ 24.5°, wrist bend ≤ 40°, club clearance ≥ 10 mm including the stretched head.
- Toe-out: use the planted foot props (heel_twist / toe_twist) within the G6 clamps. Report them.
- PROPS `mouth_open` uses `interpolation.props_overrides {"mouth_open": "BEZIER"}`:
  - 0 until f8
  - 1 by f10
  - held to f26
  - 0 by f29

**R3 revision (user, 2026-09-26, after watching Q2):**
- In the roar the body rises on straightened legs.
- The feet splay outward with the toes lifted and the heels planted, as in reference f10–14 (ref_roar/feet_sheet.png).
- How it is built:
  - COG lift within the leg reach (ik_ratio ≤ 0.97).
  - foot_roll (toes up about the heel pivot) and heel_twist (splay) within the G6 clamps.
  - Both props use BEZIER via interpolation.props_overrides, so there is no snap.
- **Contact during that range:** C10's "foot DEF world movement ≤ 1 mm" cannot hold for a rolling or twisting foot.
  - The clip JSON marks the range as `"mode": "pivot"` in contact_ranges.
  - Such a range is judged with the G6.7 planted-roll rule: cumulative contact slip S ≤ 5 mm, and the lowest shoe z in [−1, +3] mm every frame (the s06c/G6.7 definition, ctrl_manifest _doc s06c_clamps).
  - Ranges without a mode keep the 1 mm rule.

**Q gates** (check_anim_clip `gate_id` mapping, same C-ids as hit; clip JSON `rig/data/clips/roar.json`, action `roar`):
- Q1.1 = C1. Key frames [1, 7, 12]: ready, wind-up, roar peak.
- Q1.2 = C2 (report), C3, C4.
- Q1.3 [U]: key-pose sheet, open mouth visible.
- Q2.1 = C5.
- Q2.2 = C6, C7, M1.
- Q2.3 = C8, C9, C10. Both feet planted for 1–39.
- Q2.4 = M2 (report).
- **C12 (report): the mouth_open per-frame curve.**
- Q2.5 [U]: side-by-side video.
- Q3 = A3 family for `goblin@roar.fbx` including .3f (Unity blend-shape weight vs stage ×100 ≤ 0.01; BakeMesh mouth vertices ≤ 0.01 mm on open frames). check_a3_clip gets the default Q3 / gob_r08_roar.blend.

### R4 — export (Q3)
- Same as A3 for `goblin@roar.fbx`.
- Plus the Unity blend-shape curve: it exists, and the sampled value per frame matches Blender ×100 within 0.5 (*draft*).
