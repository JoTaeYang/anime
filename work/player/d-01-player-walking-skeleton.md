# Player P1: walking skeleton (crude rig → FBX → Unity round trip, 2026-09-26)

## 0. Purpose
Catch problems in axes, importer behaviour, bone connection, bounds, batch rendering and spring bones on the first day, before the real retopo and rig (playbook §3 step 1).

Everything here is **crude and throwaway** except:
- the skeleton naming / hierarchy contract (§2)
- the export preset
- the Unity folder layout
- the check tools

Source: `C:\Users\whxod\Downloads\Meshy_AI_Clay_Explorer_0926122001_generate.fbx` (read-only; P0b measured it). Height 1.1 m, A-pose, front −Y.

## 1. User decisions (2026-09-26)
- Scarf and scarf tail belong to **Armor_Default**.
- Unity rig type is **Generic**.
- Earlier decisions (d-00):
  - 1.1 m
  - ball fists (already in the model)
  - no naked body
  - skirt = skirt bones + runtime spring bones with thigh colliders
  - colours deferred

## 2. Shared skeleton contract (v0, walking skeleton)
Names follow the design doc (`modular_player_character_structure.md` §7), plus the doc's sockets and skirt chains. All export bones are unconnected (goblin G8 pitfall).

```
Root
└─ Pelvis
   ├─ Spine_01 ─ Spine_02 ─┬─ Neck ─ Head ─ HeadEquipmentSocket
   │                        ├─ Clavicle_L ─ Arm_L ─ Forearm_L ─ Hand_L ─ WeaponSocket_L
   │                        ├─ Clavicle_R ─ Arm_R ─ Forearm_R ─ Hand_R ─ WeaponSocket_R
   │                        ├─ BackSocket
   │                        └─ BackWeaponSocket
   ├─ Thigh_L ─ Calf_L ─ Foot_L
   ├─ Thigh_R ─ Calf_R ─ Foot_R
   └─ Skirt_{F,FL,L,BL,B,BR,R,FR}_01 ─ Skirt_*_02      (8 chains × 2)
```

- **Positions:** from the P0b landmarks.
  - The wrist sits inside the fist.
  - WeaponSocket_* is at the fist centre.
  - HeadEquipmentSocket is at the head top.
  - BackSocket and BackWeaponSocket are on the upper back surface.
  - The skirt chains start at the belt bottom (z ≈ .408) and end at the hem (z ≈ .288), evenly spaced by azimuth.
- **v0 = walking skeleton.** Names, parents and counts may still change in P2 by user or main decision. That change is recorded here.

## 3. Crude content
- **Rigid parts → one bone, 100 %:**

  | Part | Bone |
  |---|---|
  | head, eyes | Head |
  | fists | Hand |
  | shoes, cuffs | Foot |
  | belt, pouch | Pelvis |
  | sleeves | Arm |
  | scarf, scarf tail | Spine_02 |

- **Deforming parts:**
  - arm and leg tubes: automatic weights
  - tunic: Spine_01 / Spine_02 above the belt, Pelvis at the belt, skirt bones below the belt (azimuth / height blend)
- **The skirt stays closed** (bottom cap) in P1. Opening it is P2 work. P1 only proves the plumbing.
- **Test action `p1test`:** 24 fps, about 72 frames, root locked. It covers:
  - arm raise / lower
  - hip flex / abduction / extension to the P0b contact angles and beyond, so the skirt must react
  - a rough walk step
  - a fast turn, to excite the springs
- **Export:** goblin `export_preset.json` options (Forward −Z, Up Y, FBX_SCALE_ALL, leaf bones off). Files `Player.fbx` (rest) and `Player@p1test.fbx`.

## 4. Unity
- Folder `Assets/Player/`, Generic avatar, compression Off.
- Spring bones:
  - reuse `Assets/Play/SpringBoneChain.cs` / `SpringCollider.cs` (Phase 2a, `Step(dt)` seam)
  - thigh and calf colliders
  - one chain per Skirt_* chain
- Renders: BakeMesh with **one material per submesh** (goblin lesson), front + 3/4, at fixed frames, springs stepped with the same dt.

## 5. Gates (main judges; numeric thresholds marked *report* are fixed after this first measured run)
| ID | Condition | Verification |
|---|---|---|
| P1.1 | Blender rig matches §2: names, parents and counts; all bones unconnected; rest = A-pose; scale 1; front −Y | p1 checker |
| P1.2 | FBX reimport: same bone set and parents; baked vs reimported bone world transforms per frame (*report*; goblin reference 0.0006 mm / 0.1°) | p1 checker |
| P1.3 | Unity import: Generic, 0 import errors, every bone present, bind poses; per-frame bone world positions vs Blender through the axis mapping (*report*) | Unity report + p1 checker |
| P1.4 | Unity springs: every skirt chain deflects when the legs move (max tip deflection > 0, *report*); leg-vs-skirt penetration counted per frame (*report*; the skirt is still capped in P1) | Unity capture + report |
| P1.5 | Batch timing: each Unity batch run's wall time recorded (goblin reference 7–20 s) | wrapper log |
| P1.6 [U] | Front / 3/4 capture sheet of `p1test` in Unity with springs on | render |

## 6. P1 result (2026-09-26; production T210, checker T211; main re-ran check_p1: 18 rows identical, 12 input hashes match)
- **Chain:** p10 3.9 s → p11 2.7 s → Unity PlayerImport 6.2 s → PlayerP1Check 7.0 s, all exit 0. Unity errors [].
- **P1.1 skeleton:**
  - 41 bones as §2, 0 connected, scale 1, front −Y, L = +X, arms 45.9° down.
  - 2,223 verts, all weighted, max 3 influences.
  - Rigid parts 100 % on their bones.
- **P1.2 FBX:** bone set and parents equal; the importer auto-connected 20 bones (undone in the checker). Per frame max 0.0007 mm / 0.0008° (2,952 samples).
- **P1.3 Unity:**
  - Generic, 41 bones, bindposes ok.
  - Per-frame bone positions max 0.0007 mm; rotation relative to frame 1 max 0.0001° (skirt raw 0.0006°).
  - The absolute angle offset is a constant 90° (bone local-axis convention). Rest-relative comparison needs a Unity rest block.
- **P1.4 springs:**
  - 8 chains, colliders thigh .11 / calf .07.
  - Swing is far too large: Unity tip_deflection_max 116 mm vs a 120 mm skirt. The checker's cross-check on the Skirt_*_02 head gives 41 mm. The two metrics differ and need reconciling.
  - Leg verts inside the closed tunic: 26–37 every frame (the cap is still there by P1 design).
- **P1.5:** Unity batch runs 6–19 s.
- **Thresholds fixed from this measurement** (for P2+ runs):
  - FBX reimport and Unity bone position ≤ 0.01 mm, rotation ≤ 0.01° (measured 0.0007 mm / 0.0008°, the same order as the goblin's).
  - Spring and penetration metrics stay *report* until P2 opens the skirt.
- **Findings for P2:**
  1. Open the skirt (remove the cap, add an inner surface) and add loops between the belt and the hem. Today Skirt_*_01 carries no weights because the tunic has no vertices between z .31 and .408.
  2. Spring tuning (stiffness and damping; capsule colliders along the thigh and shin instead of spheres) and one agreed deflection metric.
  3. Sleeve and scarf intersect when the arm is raised (visible at f8).
  4. A player-owned export preset (the goblin preset is borrowed) and a Unity rest-rotation block.
  5. Unity import warning "inconsistent result" on Player.fbx: investigate.
- P1.6 [U]: capture sheet `work/player/inspect/P1/p1_capture_sheet.png` (rows: Unity front / Unity 3/4 / Blender front; frames 1, 8, 21, 31, 48, 61).
