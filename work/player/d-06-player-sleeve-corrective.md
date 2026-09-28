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

## 3. Step A result (2026-09-27, T330): no rig-only fix works
- **Grid excess before any fix** (pairs / depth mm / newly inside vs the ready pose):

  | Pose | Excess |
  |---|---|
  | fwd30 | 21 / 14 / 23 |
  | fwd60 | 59 / 38 / 45 |
  | fwd90 | 130 / 33 / 49 |
  | fwd120 | 110 / 42 / 43 |
  | side45 | 32 / 26 / 10 |
  | side90 | 155 / 30 / 11 |
  | across | 127 / 31 / 72 |
  | SA1 f11 | 128 / 38 / 13 |
  | SA1 f15 | 104 / 37 / 23 |
  | SA1 f22 | 68 / 34 / 41 |

- **Prototypes (scratch):**
  - A1, A1p, A1r: sleeve corrective keys
  - A2: sleeve weights on the clavicle. It also moves the approved idle sleeve up to 27.7 mm.
  - A3: scarf weight lift
  - A4: scarf / tunic push keys
  - combinations of the above
- **Outcome:**
  - Best is A1r + A4 (16 blend shapes). It only cuts depth about 20–40 % on some poses, and the visible pairs stay high.
  - The other variants distort the sleeve (edge ratio up to 3.85) or push the arm tube through the sleeve.
- **Main's view of `inspect/P2g/P2g_A_compare.png`:** the variants are hard to tell apart visually. The contact comes from the geometry: a big sleeve cap next to a thick scarf on a short-necked, big-headed body.
- **Options for the user:**
  1. Accept the sleeve-cap vs scarf / tunic contact as cloth bunching, and block only real penetration (arm tube or fist into torso / head, deep sleeve into head) in C11.
  2. Geometry change (P2a retopo): a smaller, tapered sleeve cap and a thinner scarf over the shoulders. This changes the look and re-runs the P2 chain and both clips.
  3. A1r + A4 corrective keys (partial effect).
- **User decision (2026-09-27): option 1.** Sleeve-cap contact with scarf / tunic is accepted as cloth bunching, and the model and rig stay unchanged.
  - C11 is split into two parts:
    - **cloth contact** (sleeve × scarf / scarf_tail / tunic / belt): report
    - **real penetration** (arm tube and fist × tunic / belt / pouch / head; plus sleeve × head): blocking
  - The blocking thresholds come from the approved clips (Sword_Idle, Sword_Attack_01) (T331).
  - The scratch prototypes stay in `rig/scratch/t330/`, not adopted.

## 4. C12 real-penetration thresholds (main, 2026-09-27; T331 measurement)
Calibrated on the two **user-approved** clips, measured over the C11 baseline:

| Measure | Sword_Idle max | Sword_Attack_01 max |
|---|---|---|
| fist × tunic / belt / pouch / head (pairs, vertices inside the volume) | 0 | 0 |
| arm tube × tunic / belt / pouch: excess pairs / depth | 2 / 16.7 mm | 3 / 16.9 mm |
| arm tube × head: pairs | 0 | 0 |
| arm tube inside the tunic volume | 1 / 0.35 mm | 7 / 35.0 mm (f18, the follow-through under the sleeve) |
| arm tube inside the head volume | 0 | 4 / 10.8 mm (f12) |
| sleeve × head | 0 | 22 pairs / 12.5 mm (f11–12) |

**Thresholds (blocking from the next clip).** A clip that exceeds a threshold is worse than an approved clip, so it blocks and goes to [U].

| Measure | Threshold |
|---|---|
| fist × body | 0 pairs and 0 vertices inside |
| arm tube × tunic / belt / pouch | ≤ 3 excess pairs and ≤ 20 mm |
| arm tube × head | 0 pairs |
| arm tube inside the tunic volume | ≤ 8 vertices and ≤ 40 mm |
| arm tube inside the head volume | ≤ 4 vertices and ≤ 12 mm |
| sleeve × head | ≤ 22 excess pairs and ≤ 13 mm |

- **Basis:** the approved maxima, rounded up. They are a regression guard, not a quality target.
- Sword_Attack_01's sleeve × head and arm-in-head values sit at the limit, from its approved wind-up.
- **Main re-ran both clips end to end** with C12 blocking (T332): exit 0.
  - Sword_Idle: 10 C rows + 5 U rows ok, C12 failed_limits [].
  - Sword_Attack_01: 12 C rows + 6 U rows ok, C12 failed_limits [].
  - C9 and U6 are switched to blocking in the checker (T333).
