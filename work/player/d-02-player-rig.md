# Player P2: rig-ready mesh → skeleton → skin → controls → export (2026-09-26)

## 0. Scope and approach
- Source: the A-pose FBX (P0b). The P1 walking skeleton is kept for its contracts:
  - skeleton v0 names
  - Unity folder `Assets/Player`
  - `unity_player.ps1`
  - `check_p1` methods
- **The goblin scripts stay untouched** so the goblin evidence stays valid. Player scripts live in `work/player/rig/scripts/` (prefix `p2x_`, `p3x_` …):
  - they import generic helpers only where these are already character-agnostic
  - they copy and adapt goblin logic otherwise
  - the goblin script that each copy comes from is noted in its docstring
  - a shared library is a later refactor, after the second character works (playbook backlog)
- Flow as on the goblin (playbook §3): retopo → deform test → skeleton → skin → controls → bake / FBX / Unity. Each phase gets a key-number measurement before its numeric criteria are fixed.

## 1. Defaults (main decisions; the user may override)
| Item | Default | Why |
|---|---|---|
| Triangle budget | GOB-like, **≤ 7,000 tris** for the character. Source 4,260; room for joint loops, the skirt inner surface and fist and foot cleanup | Same order as the goblin (9,168 incl. mouth); first measured run confirms |
| UVs | Make valid non-overlapping UVs now (head front density ≥ 1.5 × body, as goblin G2.9) even though colours are deferred | Adding UVs later means re-exporting every clip |
| Rigid parts | Kept as separate rigid shells on one bone: head, eyes, fists, shoes, cuffs, belt, pouch, scarf, scarf tail, sleeves | The LEGO principle of the design doc; weight work stays trivial |
| Deforming parts | Arm tubes, leg tubes and the tunic are rebuilt as even quad tubes with joint loops | The goblin G2/G3 lesson: straight decimated tubes collapse |
| Skirt | Cap removed, **inner surface** (thin double wall or back faces), rings between the belt bottom and the hem so every Skirt_*_01/_02 bone carries weights | P1 finding 1 |
| Shoulder | Sleeve roots kept rigid on Arm; the tunic/scarf intersection on arm raise is measured in the deform test and fixed there | P1 finding 3 |
| Skeleton | v0 (41 bones) plus fixes found in P2. No finger bones, no twist bones unless the deform test needs a forearm twist | Ball fists |
| Controls | Port the goblin control rig (IK/FK arms and legs, foot roll, COG, head, weapon space, PROPS, ready pose) | Proven on the goblin |
| Springs | Tuned in Unity against a single deflection metric; capsule colliders along the thighs and shins | P1 finding 2 |
| Export preset | A player-owned copy of the goblin preset | P1 finding 4 |

## 2. Gates
Numbers in *italics* are drafts: they are fixed after the first measured run, and the basis is recorded here.

**P2.1 Retopo static** (goblin G2 analogue)
- Each part is a closed manifold shell (or has an open rim hidden inside another part).
- Outward normals, no buried faces except the designed overlaps.
- Tris ≤ *7,000*.
- UVs valid: every face covered, coordinates in [0, 1], 0 overlap, head-front density ≥ 1.5 × body.
- Custom normals present.
- Deviation to the source for the unchanged parts: mean ≤ 1, p95 ≤ 2, max ≤ 3 mm.
- The skirt redesign region is reported.
- Joint loops present: elbow and knee 3 each, shoulder and hip rings.
- Skirt rings present between the belt and the hem.

**P2.2 Deform test** (goblin G3 analogue, temporary rig)
- Tube cross-section ratio at 30° / 60° bends ≥ *0.8* for elbow and knee (90° report).
- 0 visible self-intersections at 30° / 60°.
- Shoulder raise 30 / 60 / 90° with sleeve/tunic/scarf visible intersection reported.
- Hip flex / abduction / extension up to the P0b contact angles with the skirt on raw skin weights (report).
- [U] joint close-up sheet.

**P2.3 Skeleton** (G4)
- Canonical skeleton JSON; names, parents and counts equal to the contract.
- Positions from the landmarks, symmetric within *2 mm*.
- All export bones unconnected.

**P2.4 Skin** (G5)
- Rigid parts 100 % on one bone.
- Tubes smooth with max 4 influences.
- Skirt vertices on Skirt_* only (plus Pelvis at the belt).
- The P2.2 criteria hold on the final skin.

**P2.5 Controls** (G6)
- The goblin G6 set: FK/IK match, branch flip 0, planted foot roll clamps with slip ≤ 5 mm, space switch keeps world matrices, ready pose.
- [U] rig handling.

**P2.6 Bake / FBX / Unity** (G7–G9)
- Bake and FBX ≤ 0.01 mm / 0.01° (P1 measurement).
- Unity bones ≤ 0.01 mm / 0.01°.
- Bounds.
- Generic import, 0 errors.
- The Unity rest block (P1 finding 4).
- Springs: one agreed metric; tip deflection and leg–skirt penetration *report* in the first run, then thresholds.
- [U] Unity capture with springs.

**P2.7 Full rerun** (G10)
- `run_player_gates.ps1` twice: identical results and input hashes.

## 3. Order and agents
1. P2a retopo + UV (production) ∥ P2 checkers for P2.1–P2.2 (checker agent).
2. P2b skeleton + skin, then P2c controls, then P2d export + Unity (production continues per phase; checker per phase).
3. main re-runs every checker and looks at every sheet; the user judges the [U] items.

## 4. P2a result (2026-09-26; T220 production, T221 checkers; main re-ran both: static 12 / deform 23 rows identical, 15 input hashes match)
- **P2.1 static:**
  - All rows ok except **P2.1b**: 2 shoe-rim verts per side lie outside the cuff (L −1.81 / R −0.38 mm; source geometry). Fix in T222.
  - Tris 6,673 (≤ 7,000 draft).
  - UV overlap 0, head-front density 2.08.
  - Rigid parts vs source 0.0002 mm. Rebuilt arms / legs, visible surface: mean ≈ 0.37, max ≤ 2.94 mm.
  - Skirt: cap removed, 4 mm inner wall, 3 rings (z .318 / .348 / .378); every Skirt_* bone weighted.
  - Joint rings within 1.2 mm of the landmarks.
- **P2.2 deform (temp rig, auto weights):**
  - Section ratio at 60°: elbow .774 / .760, knee .793 / .803 (below the .8 draft on three joints, as on the goblin G3).
  - **Decision (main):** judge at P2.4 with the final skin; the threshold is set from that measurement (goblin precedent: G5.5 ≥ 0.75, user approved).
  - 0 visible self-intersections at 30 / 60°.
  - The shoulder-forward sleeve pushes into scarf / tunic (max 20.4 mm).
- **Checker limitation:** the open tunic neck and the designed skirt-ceiling piercing are counted as "visible". T223 adds a virtual neck cap and designed-crossing classification.
- **After T222 / T223 (main re-ran both checkers):**
  - **P2.1 all rows ok.** The shoe-rim tuck moves at most 2.85 mm, and the shoe stays within 2.23 mm of the source. Every other part's geometry is byte-identical; the UVs were re-packed (overlap 0, density 2.08).
  - **P2.2:** all rows ok (calibration); 23 rows identical to the agent's run; 15 input hashes match.
  - **Classification after T223:**
    - rest visible 0 (was 32)
    - the ceiling piercing is designed (16 per side)
    - hips: 0 visible and 0 poke-out up to the contact angle; beyond contact + 15° the leg pokes out 52–91 mm (the springs' job)
    - shoulder: the rest seam where the sleeve enters the tunic / scarf counts as "visible" at up to 35 mm (it is the source look); shoulder forward 60° deepens it to 40–42 mm → needs a visual judgment
- **P2a PASS** (main). The [U] joint sheet `inspect/P2/P2_joint_sheet.png` goes to the user with P2b.
- **P2b 시작(2026-09-26, 사용자 "진행"):** 어깨는 기본값(소매 강체 Arm)으로 두고 스킨 후 다시 측정한다. T230 제작(p22 skeleton v1 + p23 final skin; Neck/Clavicle 가중치 부여) ∥ T231 checker(check_p2_skel, check_p2_skin + 최종 스킨 기준 P2.2 metric 재측정).
- **T230 result (production):**
  - skeleton v1: 41 bones, symmetric, limbs moved ≤ 0.77 mm and skirt bones ≤ 2.8 mm vs v0
  - final skin: max 4 influences, rest deformation 0.00012 mm, Neck and Clavicle weighted
  - section ratio at 60°: elbow .796 / .792, knee .809 / .812 (probe)
  - **Open for P2c:**
    - no preferred bend: chains 0.62° / 0.42° (goblin used +4 mm at the elbow, −1.5 mm at the knee)
    - Calf roll sign opposite to goblin G4.4
    - the elbow sits at the rigid sleeve's lower edge
- **T231 checker (P2b):**
  - **P2.3 all ok.**
    - Symmetry 0.0 mm; joint heads 0.2–0.7 mm from the rings.
    - WeaponSocket 1 mm from the fist centre.
    - Chain bend: arm .616°, leg .420°. Calf +X moves the foot forward (goblin: backward).
  - **P2.4: 29 / 30 ok.** P2.4d fails on a Pelvis leak (≤ 0.0063) on the skirt_3 rows → T232.
  - Section ratio at 60°: elbow .796 / .792, knee .809 / .812. Twist 60°: .787 / .779.
  - 0 visible at 30 / 60°.
- **T232:**
  - Cause: the skirt height weights were interpolated from each vertex's actual z, which the straddle moved.
  - Fix: Pelvis = 0 on the skirt rows (50 verts, max change 0.0086).
- **P2b main judgment (2026-09-26, check_p2_skel / check_p2_skin re-run: 6 / 30 rows all ok, 21 input hashes match):**
  - **P2.3 PASS, P2.4 PASS.**
  - **Section-ratio threshold fixed from the measurement:** 60° ≥ 0.75 for elbow / knee (goblin G5.5 precedent, user-approved LBS limit; measured elbow .792–.796, knee .809–.812); 30° ≥ 0.9 (measured .909–.962); 90° report.
  - [U] joint sheet `inspect/P2/P2b_joint_sheet.png`.
- **Next P2c (controls):**
  - Preferred bend: goblin method (elbow +4 mm, knee −1.5 mm pole offsets) adapted to this skeleton.
  - Calf roll sign opposite to the goblin: the port adapts its sign in the control code. The skeleton contract stays unchanged.
- **P2c 시작(2026-09-26, 사용자 "진행"):** T240 제작(고블린 s06a–e 이식 → pl_r04_ctrl.blend, ctrl_manifest, player addon; 치마는 컨트롤 없음, 무기 컨트롤 R/L) ∥ T241 checker(check_p2_ctrl, G6 이식).
- **T240 control rig (production):**
  - p24a–e plus the player addon (4 operators); 30 controls; the ctrl_manifest.
  - Signs: knee Calf −X, hip Thigh +X, abduction thigh_l Z+.
  - The leg IK uses MCH helpers with a 0.94 mm knee offset.
  - Clamps at the ready stance (−4 mm): roll −45…60, bank ±30, twists within reach.
  - Ready pose: knee 17.5°, elbow 15.5°.
  - Probe: leg_r FK→IK snap 2.16° (> 1°?); flips 0; space switches 0.0001 mm; cycles 0.
  - Judged by T241.
- **T241 checker:** 14 / 15 ok.
  - **P2.5e fails:** leg FK→IK snap twists the Thigh 61.8° / 62.5° at random pose 13 (position 0.8 mm). Arms and IK→FK ≤ 0.05°.
  - P2.5i ran at the goblin −15 mm stance (the manifest key is `chosen_stance_m`) → checker fix T243.
  - Others: flips 0, space switch ≤ 0.0002 mm / 0.037°, cycles 0, reset / ready ok, skirt and sockets follow (3.4e-7), weapon L / R drive the sockets.
  - T242: production fixes the leg snap.
- **T242:**
  - Cause: the leg's rest bend plane is 61.1° off the knee hinge plane, so a near-straight FK leg was snapped with the wrong swivel.
  - Fix: the leg TO_IK snap matches the thigh local X axes.
  - Result: 0.875 mm / 0.55° (was 62.5°). The residual comes from the 0.94 mm MCH knee offset.
- **T243:** the checker reads `chosen_stance_m`.
- **P2c main judgment (2026-09-26):** main re-ran check_p2_ctrl: **15 / 15 ok**, 16 input hashes match.
  - P2.5i at the −4 mm stance.
  - P2.5 machine rows PASS.
  - **P2.5 [U] rig handling → user** (pl_r04_ctrl.blend, Sidebar > Player).

## 5. P2d plan (2026-09-26, user "진행"; P2.5 [U] handling feedback may come later and does not block P2d)
- **Rigtest action** (goblin s07a analogue):
  - every control through its sweep, the IK/FK switches, foot roll within the clamps, weapon spaces
  - a locomotion-like leg section to excite the skirt
  - root locked
- **Export stage** (goblin s07b analogue):
  - DEF-only armature `Player` with all 41 bones unconnected
  - per-frame loc/rot/scale bake
  - mesh `Player_mesh` with 20 material slots
- **Export:**
  - Player.fbx (rest) and Player@rigtest.fbx
  - a **player-owned** `data/export_preset.json` (a copy of the goblin preset; axis mapping unity.x = −x, unity.y = z, unity.z = −y)
- **Unity:**
  - PlayerImport (Generic, compression Off; the clip avatar is copied from Player.fbx)
  - PlayerRigCheck.cs (GoblinRigCheck analogue):
    - bones, bind poses, a **rest-rotation block**
    - bounds = union of all Player@ clips + 0.1 m
    - BakeMesh renders with one material per submesh
    - springs on
- **Spring metric (single definition for Unity and the checker):**
  - Tip = the tail point of each Skirt_*_02 bone.
  - `sway_mm(f)` = distance between the spring-simulated tip and the raw-animation tip at frame f (Unity world, same dt = 1/24, Init after sampling frame 1).
  - `penetration(f)` = leg-tube vertices outside the skirt wall while inside the skirt footprint (the check_p2_deform poke-out rule), with springs on vs off.
  - Both are *report* on the first run; thresholds are set from the distribution.
- **Colliders:**
  - SpringBoneChain / SpringCollider are not modified (Phase 2a code).
  - A capsule is approximated by 3 spheres along each thigh and shin (hip→knee, knee→ankle).
  - Initial spring params come from the P1 skirt row, then tuning.
- **Gates P2.6:**
  - bake (DEF vs stage) and FBX reimport ≤ 0.01 mm / 0.01°
  - Unity bones ≤ 0.01 mm / 0.01° (rest-relative)
  - Generic, 0 errors
  - bounds covering every frame
  - spring sway and penetration report → thresholds
  - [U] Unity capture sheet with springs
- **Metric reconciliation (T251, 2026-09-26).** The checker's independent tail cross-check on the P1 data reproduces Unity's per-frame tip values within 0.0005 mm on all 8 chains.
  - P1's 116 mm (Unity) and 41 mm (checker) were not a disagreement. The 41 mm was measured at the Skirt_*_02 **head**; 116 mm is at the **tail**.
  - The §5 metric (tail) is the agreed one.
- **Report contract additions:**
  - `bounds.space` = `unity_world_char_at_origin`.
  - A per-frame `skirt_raw` block, springs off, so the checker can judge the skirt bones.

## 6. P2d first run (2026-09-27, production T250, checker T251)
Main re-ran `check_p2_export`. Evidence: `inspect/P2/check_p2_export.json`.

- **Chain:** run_player_p2d.ps1 took 28.2 s. Every step exited 0 and Unity reported errors [].
- **P2.6a bake** (676 frames × 41 bones): 0.0006 mm / 0.0001°. ok.
- **P2.6b FBX:**
  - Preset ok.
  - 1 mesh, 20 slots, UV and custom normals present.
  - Reimport with the offset corrected: 0.0016 mm / 0.0013°. ok.
- **P2.6c Unity:**
  - Generic, 0 errors, 41 bones, rest block present.
  - Non-skirt bones: 0.0052 mm / 0.0008°. Skirt raw: 0.0032 mm / 0.0009°. ok.
  - **c1 fails on `bindposes_ok`.** Only Skirt_L/R_01/02 are off (max element 0.036), and their sampled poses are exact. Rest BakeMesh vs mesh is 1.15 mm, on M_tunic only.
  - Production's hypothesis: the ±90° pitch singularity in those bones' local rotation. T252 tests it with a roll experiment. The "inconsistent result" import warning is still present.
- **P2.6d bounds:** space `unity_world_char_at_origin`, 676 frames, springs on and off, 0 outside. ok.
- **P2.6e springs (report).** P1 parameters: stiffness .15, damping .25, gravity 1.5, maxAngle 40. Colliders are 3 spheres per thigh (r .045) and per calf (r .055).
  - Sway max 113–115 mm per chain, at the root segment and the locomotion yaw. p95 86–97, median 5.
  - Penetration, springs on: 21 verts / 47.3 mm. Springs off: 20 verts / 27.0 mm.
  - Worst frames: f57 (21 on / 0 off; spring-caused), f477 and f513 (20 / 20; rig-level, foot bank).
- **Renders:** `inspect/P2d/P2d_unity_sheet.png`. Main viewed it: no breakage; the f49 hem crumples from the spring.
- **Next (T252):**
  - the bindpose root-cause experiment
  - the spring parameter sweep
  - [U] captures: penetration frames, and locomotion GIFs for the current parameters plus 2 candidates

## 7. T252 results and decisions (2026-09-27), paused by the user; resume tomorrow
- **Bindpose root cause confirmed** (scratch roll experiment):
  - Rolling Skirt_L/R_01/02 off the ±90° singularity brings every bindpose to ≤ 4e-6. Rest BakeMesh drops to 0.0007 mm.
  - c2: L +90° / R −90° roll. Local Euler ≈ (0.85, −0.14, −170.76).
  - **Main decision:** adopt c2 in the canonical skeleton, as a v1 contract change on roll only.
    - Propagate it: p22 → p23 → p24 → checkers' canonical_skeleton.json → re-export.
    - Add a report row: minimum margin of every bone's local rotation from the ±90° pitch singularity.
- **"Inconsistent result" warning:** this is Unity's ConsistencyChecker, which found that re-importing Player.fbx gives a different artifact hash (non-deterministic import). It is not the bindpose problem: it did not reproduce in scratch. It names only the asset. Report-only (not visible, not breaking).
- **Spring sweep:** 24 runs, `inspect/P2d/spring_sweep.md`.
  - **User chose r15** (2026-09-27): stiffness 0.3, damping 0.5, maxAngle 25, gravity 1.5, colliders as T250.
  - r15 locomotion: pen on 0 / off 0. Cog: 8 verts / 7.2 mm on vs 11 / 7.2 off.
  - Main viewed the GIF frames: the current params flip the hem; r15 keeps it tidy.
- **Leg through the skirt front when the thigh is raised** (f477 / f513 / leg_fk, springs off 20 verts / 27 mm, springs do not fix it):
  - **User chose "skirt follows thigh"** (2026-09-27). The front and side skirt DEF bones inherit a fraction (≈0.5, to tune) of the adjacent thigh rotation in the rig. This is baked into the clips; springs add secondary motion on top.
- **Next, on resume:**
  1. Canonical roll fix (c2).
  2. Skirt-follows-thigh rig change, with its own gate on penetration springs-off in leg_fk / foot_roll (report first).
  3. Set r15 as the default spring parameters.
  4. Re-run the P2d chain and check_p2_export.
  5. P2.7 run_player_gates.ps1 twice.
  6. P2.5 [U] handling check (still pending).

## 8. P2e plan (resumed 2026-09-27, user "이어서 진행")
- **(a) Canonical roll fix, c2:**
  - Skirt_L_01/02 roll +90° and Skirt_R_01/02 roll −90° relative to the current values, in p22. canonical_skeleton.json → v1.1 (roll only; the change is recorded in the JSON).
  - Then run p23 → p24a–e → rebuild pl_r03 / pl_r04.
  - Skin weights are unaffected (roll only). Control bones that copy skirt axes, if any, follow.
- **(b) Skirt follows thigh:**
  - **MCH layer:** the skirt DEF bones are driven through an MCH layer. Each Skirt_*_01 base rotation = its rest rotation plus a push-only fraction of the adjacent thigh(s)' rotation:
    - F: both thighs' flex, the forward one wins.
    - FL / FR: that side's flex and abduction.
    - L / R: that side's abduction.
    - BL / BR / B: that side's extension (backward), the backward one wins.
  - **Push-only:** a panel moves outward with the leg and is never pulled inward by the opposite motion.
  - Skirt_*_02 stays a plain child (springs add secondary motion in Unity).
  - **Control:** a PROPS float `skirt_follow`, default 0.5, range [0, 1], in the UI panel and set by reset_rig / ready_pose. The exact per-panel fractions and the push-only mechanism are production's design; the rule above is the contract.
  - The result bakes into the DEF skirt bones, so the export needs no constraint.
- **(c) Spring defaults = r15:** stiffness 0.3, damping 0.5, maxAngle 25, gravity 1.5, colliders as T250.
- **Gates** (report on the first run, then thresholds):

| ID | Criterion | Status |
|---|---|---|
| P2.8a | No DEF bone's local rotation (FBX Euler XYZ, relative to its parent) closer than *30°* to ±90° pitch. Unity `bindposes_ok` true (max element ≤ 1e-4) | report → fixed after this run; bindposes_ok is blocking |
| P2.8b | Leg-through-skirt, **springs off** (the check_p2_deform poke-out rule on the Blender DEF mesh), per rigtest section, before vs after the follow | report; blocking candidate: after ≤ before in every section, and 0 visible poke in the [U] sheet at f477 / f513 / leg_fk |
| P2.8c | Skirt stays sane: hem ring edge-length ratio vs rest (max stretch, min compression) per frame; skirt vs opposite leg penetration; skirt vs itself (adjacent panels crossing) | report |
| P2.8d | `skirt_follow = 0` reproduces the pre-follow DEF skirt bones exactly (≤ 1e-5) | blocking |
| P2.8e | [U] | sheet of f477 / f513 / leg_fk / locomotion, before vs after, springs on (r15) and off; a locomotion GIF after |

- The existing P2.2–P2.6 rows are re-run on the new rig. P2.6 c1 bindposes becomes blocking.

## 9. P2e result (2026-09-27; production T253/T256, checker T254/T255; main ran run_player_gates.ps1 runs 1–3)
- **Main code changes:**
  - `Assets/Play/SpringBoneChain.cs`: new `followAnimation` flag, default false (Phase 2a behaviour unchanged). When it is true, Step keeps the animated local rotation instead of resetting to the Init rest.
    - Why: without it the spring discarded the baked skirt follow (T253 finding).
    - PlayerRigCheck sets it true.
    - Caveat: it is valid only when the clip keys the chain bones every frame, as the player export does.
    - Backup: `rig/backup_pre_P2e_2026-09-27/SpringBoneChain.cs.bak`.
  - `run_player_gates.ps1`: one `Write-Output` → `Write-Host` in Invoke-BlenderStep. It had polluted the return value.
- **P2.7, runs 2 and 3:**
  - Chain p20→p27→Unity: exit 0 in both runs, 134–135 s each.
  - All 7 checkers exit 0 with **101/101 rows ok** in both runs: static 12, deform 23, skel 6, skin 30, ctrl 15, export 11, skirt 4.
  - **Every row's measured-value hash is identical** between the runs.
  - Input sha256: 124/136 are identical. The 12 that differ are all files the chain regenerates: pl_r01…pl_r05 blends, both stages, both FBX files, player_report.json. They are not byte-deterministic (save and export timestamps; the report's timing fields). All source and data inputs are identical.
  - **Basis for P2.7 on this rig:** identical rows plus identical source/data inputs. Byte differences in regenerated artifacts are expected and recorded.
- **P2.8a:** bindposes_ok true, max 3.99e-6 (Skirt_BL_02; Skirt_L/R ≤ 1e-6). Minimum pitch margin 45.0° (Skirt_FL/FR/BL/BR_01), 0 bones under 30°.
  - **Fixed:** margin ≥ 30° blocking. Basis: the measured minimum is 45°, and the failure mode was < 1°.
- **P2.8b** (Blender, springs off): after ≤ before in every section.
  - cog: 11/7.2 → 0
  - leg_fk: 7/27.1 → 0
  - foot_roll_bank_l: 20/20.6 → **4 / 8.1 mm** (f477)
  - foot_roll_bank_r: 20/18.5 → **2 / 6.1 mm** (f513)
- **P2.8c:** 0 panel crossings, 0 opposite-leg increase. Hem stretch max 1.36 (foot bank), 1.27 (cog), 1.21 (locomotion); min 0.999. Report; visual judgement [U].
- **P2.8d:** skirt_follow 0 vs pre-P2e rig: 3.0e-7. ok.
- **P2.8f:** manifest and rig agree (8 drivers, 8 follow_copy). ok.
- **Unity, r15 + followAnimation:**
  - penetration springs on 5 v / 8.1 mm (clip max), off 4 / 8.1
  - leg_fk on 1 / 2.3
  - sway p95 over the clip 62 mm, max 75 mm (was 64 / 100)
- **Main visual check** (`inspect/P2e/P2e_before_after_sheet.png`, zoom on f477):
  - The large thigh-through-skirt is gone at f477, f513 and f554.
  - A **small visible thigh patch remains just under the belt** at f477 / f513 (the hinge region; weights did not remove it).
  - The hem is slightly uneven at the foot-bank frames.
- **User decisions (2026-09-27):**
  - P2.8e [U]: **approved** (the r15 + follow Unity result and loco_after.gif).
  - Residual poke: **thin the thigh top inside the skirt** (T257).

## 10. P2f: thigh-top thinning (T257)
- **Scope:**
  - Only leg-tube vertices that sit inside the skirt volume at rest, above a cut height (production picks it; the region is written to `data/thigh_thin_zone.json`: vertex ids, the cut z, the scale profile) are moved toward the thigh axis.
  - The taper is smooth, with no step at the zone boundary.
  - Weights, UVs and topology are unchanged. The change is in p20 (retopo), so it propagates through the chain.
- **Gates:**

| ID | Criterion | Status |
|---|---|---|
| P2.9a | The zone is invisible at rest: every modified vertex is inside the skirt wall at rest, and the change is 0 on vertices below the hem | blocking |
| P2.9b | Poke-out, springs off, whole rigtest: after ≤ P2e in every section. Target 0 at f477 / f513 | ≤ blocking; 0 is the target, report |
| P2.9c | P2.1 deviation-to-source excludes the zone as a design-change region (reported); P2.2 hip rows still pass | blocking as before |
| P2.9d | [U] | f477 / f513 zoom plus a front/3/4 sheet |

## 11. P2f result: reverted (2026-09-27)
- **T257 thinning had no effect.** Poke-out was identical before and after: foot_roll_bank_l 4 / 8.08, foot_roll_bank_r 2 / 6.10.
  - The poking vertices are the **knee**: rest z 0.258–0.270, below the hem at 0.290. At f477 / f513 the knee is raised to belt height and pokes through the top of the front panel, where the hinge rotation hardly moves the wall.
  - P2.9a also failed: 50 zone vertices lie above the belt bottom (0.408). Main's premise (thigh top) was wrong.
- **User decision:** keep the residual for now; judge it per clip in the animation phase.
  - Every player clip gate gets the springs-off poke-out row. If a clip shows the knee patch, the fix candidates are a front-panel push-out translation or a stronger FL/FR follow.
  - Diagnostic: d1 (cut below the hem) reached 1 v / 2.5 mm but changes the visible knee.
- **Revert:**
  - p20_retopo.py restored from `rig/backup_pre_P2f_2026-09-27/scripts/`.
  - The rejected version and `thigh_thin_zone.json` moved to `rig/scratch/t257/rejected/`.
  - P2.9 rows retired (T259).
  - The chain is re-run by run_player_gates runs 5 and 6.
- **P2.7 after the revert** (runs 5 and 6; chain + 7 checkers all exit 0; 101/101 rows ok):
  - Runs 5 vs 6: every row identical. Only the 11 regenerated artifacts differ in bytes (§9 basis).
  - Runs 3 (P2e) vs 6: the only row difference is P2.1i, which gained the zone info field (checker change). Every measured value is otherwise identical, so the revert restored the P2e state exactly.
  - **P2.7 passes.**
- **P2 machine gates are closed.** Remaining: **P2.5 [U] rig handling** (Sidebar > Player on pl_r04_ctrl.blend; now also has the Skirt box with skirt_follow).
