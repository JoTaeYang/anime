# Playable character (Scarfed Clay Explorer): decisions and first steps (2026-09-26)

## Source
- `C:\Users\whxod\Downloads\Meshy_AI_Scarfed_Clay_Explorer_0926113142_generate.fbx`, 128 KB, read-only.
- main's probe (scratch `player/probe.json`):
  - one mesh, 16 connected components, 4,230 tris, 182 non-manifold edges
  - no UV, materials, textures or armature
  - height 0.96, normalised and centred at the origin
  - T-pose with straight arms and legs
  - hands are flat mittens (two finger lobes + thumb) fused to the arms
- System design: `C:\Users\whxod\Downloads\modular_player_character_structure.md`. Modular slots Head / Armor Set / Weapon / Back, one shared skeleton, shared animations.

## User decisions
| Item | Decision |
|---|---|
| Height | **1.1 m** (scale ≈ 1.146 from the source) |
| Hands | **Ball fists**: replace them with the goblin tool (s02c ball-fist approach); weapon sockets inside the fist |
| Base body | **No naked body.** This clothed model = `Player_Head` (head + eyes) + `Armor_Default` (tunic, arms, legs, shoes, belt, pouch, scarf — the scarf's slot is still open). A fitting mannequin for other armors is derived from the rig later |
| Colour / texture | Deferred. Flat material colours per part for now |
| Tunic skirt | **Skirt bones + runtime spring bones** with thigh capsule colliders in Unity. The skirt bones are part of the shared skeleton (armors without a skirt ignore them). Clips are authored without skirt keys. Skirt checks run in Unity with the springs on (Phase 2a spring-bone experience) |

## Next
- P0 model inspection against playbook §1 (rig-ready model spec). This is measurement only; criteria are set from its numbers:
  - joint positions and limb lengths at 1.1 m
  - skirt length, circumference and clearance to the legs
  - where the non-manifold edges are
  - part list and slot assignment
- Then a walking-skeleton round trip (crude rig → FBX → Unity) before the full rig.
- Still open: weapon handedness (hammer), locomotion clips and root-motion policy, Humanoid vs Generic, frame rate, gameplay timing data (see the earlier answer).

## P0 result (T200, main re-ran the measurement: all 7,779 numbers identical, 2026-09-26)
- **Transform:** scale ×1.147047, z offset +0.551. Facing −Y, up +Z, character left +X (already our convention).
- **Parts (16):**

  | Slot | Parts |
  |---|---|
  | Player_Head | head, eye_l / eye_r |
  | Armor | tunic (torso + skirt shell), belt, pouch, sleeve_l / sleeve_r, arm_l / arm_r (tube + mitten), leg_l / leg_r (one straight tube), shoe_l / shoe_r |
  | Open decision | scarf, scarf_tail |

  - The 182 non-manifold edges are all open rims hidden inside other parts (by design).
- **Landmarks (m):**

  | Joint | Position | Confidence |
  |---|---|---|
  | shoulder | ±.101, z .656 | low: no shoulder geometry, sleeves sunk .07 into the tunic |
  | elbow | ±.254, .644 | low: midpoint of a straight tube |
  | wrist | ±.407, .631 | medium |
  | hip | ±.087, .385 | low: hidden in the tunic |
  | knee | ±.099, .249 | low: midpoint |
  | ankle | ±.112, .113 | medium |
  | head centre | z .909 | high |

  - Segments: upper arm / forearm .155 / .152; thigh / shin .137 each.
- **Pose:** T-pose, arms 4.6° below horizontal, limbs straight (0° bend), no elbow or knee shaping.
- **Hands:** ball fist r .0678 by the goblin rule (0.346 × head width .392), or r .055–.056 from the equal volume of the mitten.
- **Skirt:**
  - The tunic is a closed bell with a bottom fan cap, and the legs pierce it at rest (30 / 29 triangle pairs).
  - Hem z .294, r ≈ .20. Skirt length below the belt .106.
  - A rigid leg touches the hem at abduction 28–30°, flex 47°, extension 50° (hip-landmark pivot); with the alternative pivot 39–40° / 58° / 58–59°.
- **Risks:**
  - The skirt needs opening and an inside surface before it can take skirt bones.
  - The short legs (hip at 0.35 H) limit crouch and foot roll, as on the goblin.
  - There is no shoulder geometry, so a raised arm exposes the sleeve/tunic intersection.
  - The T-pose is against playbook §1.

## New source in A-pose (user, 2026-09-26)
- `C:\Users\whxod\Downloads\Meshy_AI_Clay_Explorer_0926122001_generate.fbx`, 133 KB, read-only.
- main's probe: 20 components, 4,260 tris, 130 non-manifold edges, no UV or materials.
  - Already 1.1 m tall with z 0 at the feet.
  - Arms about 45° down.
  - **Ball fists already present** as separate parts.
  - Limbs still straight.
- T201 = P0b re-measures it. The questions on fist size and pose are withdrawn (the new model answers them); the scarf slot is still open.

## P0b result (T201, main re-ran it: all 8,460 numbers identical, 2026-09-26)
**Improvements over the T-pose model:**
- 1.1 m as delivered (no scaling).
- A-pose, arms 45.8° down.
- Separate closed ball fists, Ø .152 = 0.375 × head width.
- Longer limbs: upper arm / forearm .166, thigh / shin .160, hip height .386 H.
- Leg–skirt wall clearance .080 (was .059).

**Still ours to fix:**
- **Skirt:**
  - The tunic is still closed by a 72-tri bottom cap that the legs pierce (32 / 34 pairs).
  - Hem contact is reached at abduction 24.8–25.8°, extension 36°, flex 40.5–41.3°.
  - The leg tops cut .016 into the belt.
- **Joints:** limbs nearly straight (elbow 3.3°, knee 1.3°); forearm r tapers to .031.
- **Shoulder:** sleeve~tunic 57 / 55 pairs, sleeve~scarf 33 / 31.
- **Fists:** the forearm tube tip sits .064 inside the fist (seamless, so the wrist pivot and socket go inside the fist).

**Open decisions:** scarf slot; Unity rig type (Generic / Humanoid).

## User decisions (2026-09-26, "all recommended")
- Scarf and scarf tail go to **Armor_Default**.
- Unity rig type is **Generic**.
- Next: P1 walking skeleton, spec `d-01-player-walking-skeleton.md`. It reuses the Phase 2a spring bones (`Assets/Play/SpringBoneChain.cs` with the Step(dt) seam).

## P2 start (2026-09-26, user "진행해줘")
- Spec `d-02-player-rig.md` (P2.1–P2.7).
- Main defaults: ≤ 7,000 tris, UVs now, rigid shells, even-quad tubes with joint loops, open skirt with inner surface and rings, goblin control rig ported, player-owned export preset.
- The colours in the P0/P1 renders are debug colours per part; the source has no materials.
