# Player P3: clip tooling (2026-09-27)

## 0. Purpose
The user will supply one reference video per clip, as with the goblin. Before the first reference arrives, port the goblin clip tools to the player so that each new clip only needs a clip JSON and the same run sequence:

```
clip JSON → action → per-frame checks → [U] → FBX → Unity → clip checks
```

Goblin sources (read-only; copy the logic and cite it):
- `work/goblin_swing/rig/scripts/s12_clip_anim.py`
- `check_anim_clip.py`
- `s11_export_clip.py`
- `check_a3_clip.py`
- `unity/AvatarCheck/Assets/Editor/Goblin/GoblinClipCheck.cs`

## 1. Decisions (main defaults, 2026-09-27; the user may override)
- **References:** the user supplies one video per clip (user, 2026-09-27). The clip order follows the references.
- **fps = 30.** It is a clip-JSON field (`fps`, default 30), never hard-coded. rigtest stays 24.
- **Root motion is decided per clip:**
  - Locomotion plays in place; game code moves the character.
  - Distance-fixed actions (dodge, lunging attacks) use root motion on `Root`.
  - Clip JSON `root_motion`: `"locked"` | `"in_place_cycle"` | `"root"`. The Unity import settings follow it: loop, bake-into-pose flags, `motionNodeName` = Root when the value is `"root"`.
- **Skirt:**
  - Every clip keys PROPS `skirt_follow` (default 0.5; the clip JSON may animate it, BEZIER).
  - Unity springs r15 + `followAnimation`.
  - Every clip gate has the springs-off poke-out row (user decision, d-02 §11).
- **Rig ranges:** the P2c control sweeps (`ctrl_manifest.json`). A clip may widen them only through `sweep_overrides`, with a `_reason`, in memory, for that clip only (goblin rule).

## 2. Files (player)
| File | Role |
|---|---|
| `rig/data/clips/<clip>.json` | Keys per control (frame → value), interpolation, fps, frame range, loop, root_motion, contacts (planted-foot ranges, ground ranges), sweep_overrides |
| `rig/scripts/p30_clip_anim.py` | Clip JSON → action `<clip>` on PL_rig in `rig/anim/pl_a_<clip>.blend`, plus key-pose and in-between contact sheets in `inspect/clips/<clip>/` (front / side / 3/4, Blender) |
| `rig/scripts/p31_export_clip.py` | Stage bake (reuses p26 build) → `export/Player@<clip>.fbx` → copy to `Assets/Player/` |
| `Assets/Editor/Player/PlayerClipCheck.cs` | Per clip: import settings per the root_motion policy, per-frame bone world, springs r15 + follow, Unity poke on/off, BakeMesh renders at the key frames, bounds union over all `Player@*` clips (+ 0.1 m, prefab) → `unity/AvatarCheck/player_clip_report_<clip>.json` |
| `rig/scripts/check_player_clip.py` | Blender per-frame rows (C-rows) |
| `rig/scripts/check_player_clip_unity.py` | FBX + Unity rows (U-rows) |
| `rig/scripts/run_player_clip.ps1` | p30 → check_player_clip → p31 → Unity → check_player_clip_unity for one clip, exit codes propagated |

## 3. Gate rows per clip (generic; each clip's spec picks its prefix)
Report-only on the first real clip unless a threshold was already fixed.

| ID | Condition | Status |
|---|---|---|
| C1 | Control values inside the sweeps (plus the clip's sweep_overrides) on every frame | blocking |
| C2 | Euler / branch pops: per-frame rotation step of every DEF bone ≤ *k* × the neighbouring steps; no flips (goblin A2 rule) | blocking |
| C3 | Planted feet (declared ranges): slip ≤ 5 mm (G6.7), height within [−1, +3] mm | blocking |
| C4 | Ground: no body part below the ground by more than 1 mm (goblin C11), for ground ranges | blocking |
| C5 | Root motion matches the declared policy: locked → Root max 0; in_place_cycle → Root 0 and first = last pose (loop seam ≤ 0.01 mm / 0.01°); root → Root path reported | blocking |
| C6 | Skirt poke-out, springs off (the P2.8b rule), per frame | report; the user judges in the sheet |
| C7 | Visible self-intersection: fist vs head / torso, sleeve vs scarf, arm tube vs tunic (P2.2 rules) | report (calibration) |
| U1 | FBX reimport vs stage per frame | ≤ 0.01 mm / 0.01° (blocking, from P1/P2) |
| U2 | Unity bones vs stage (rest-relative), Generic, compression Off, 0 errors, import flags match root_motion | ≤ 0.01 mm / 0.01° (blocking) |
| U3 | Bounds: every frame of every Player@ clip inside the prefab bounds | blocking |
| U4 | Springs: sway p95 / max, poke on / off | report |
| [U] | Key-pose sheet, then in-between video with the reference side by side, then Unity capture | user |

## 4. Tool smoke test (before the first reference)
- A throwaway clip `smoke` covers the whole sequence: 30 fps, 30 frames, in_place_cycle, with small torso sway, an arm swing and a planted-feet range.
  - It is report-only and is deleted when the first real clip passes.
- Verification:
  - main runs `run_player_clip.ps1 -Clip smoke` twice and compares.
  - Then P2 `run_player_gates.ps1` once, to confirm the extra clip changes no P2 row other than the bounds union.

## 5. Tooling result (2026-09-27; production T300/T303, checker T301/T302)
- **Tools built:**
  - p30_clip_anim.py (786 lines), p31_export_clip.py, run_player_clip.ps1, PlayerClipCheck.cs
  - check_player_clip.py (C1–C7), check_player_clip_unity.py (U1–U4)
  - Clip JSON schema: see p30's docstring. Unknown keys are refused; out-of-sweep keys are refused.
- **root_motion → Unity import settings:**
  - locked / in_place_cycle: motionNodeName "" (Root baked into the pose), lockRoot* true.
  - root: motionNodeName `Player/Root`, root motion kept. **Untested:** the first root clip must confirm that CopyFromOther honours the clip's root node.
- **Loop convention:** the clip includes a last frame equal to the first (first_eq_last; Unity loop match).
- **main's runs of `run_player_clip.ps1 -Clip smoke`:**
  - Runs 1–2: every row identical between the two runs; U2 failed in both from a report/checker contract mismatch, fixed in T302.
  - Run 3: 11/11 ok, 26 s.
  - smoke numbers:
    - C3 slip 0.004 mm, C4 −0.006 mm, C5 seam 0
    - U1 0.0006 mm / 0.0007°, U2 body 0.0006 mm / 0.0001°
    - sway p95 13.4 / max 15.1 mm, poke 0
- **C1 tolerance fixed at 1e-5.** Basis: snap float noise of about 2e-6 measured on rigtest.
- **PlayerRigCheck reads each clip's frameRate** (T303; backup in `rig/backup_pre_P3/`). run_player_gates run 7: 101/101 ok. Vs run 6, the only row differences are P2.6c1 (clips_in_report + smoke) and P2.6d (smoke bones, 0 outside). Rigtest numbers are byte-identical.
- **Known report items:**
  - C7 sleeve vs scarf is visible already at rest (33 pairs); judge it by the clip-vs-rest delta.
  - PlayerRigCheck and PlayerClipCheck duplicate the spring, poke and bounds code (backlog).
- **Waiting on the user's first reference video.** Delete smoke (clip JSON, anim, stage, FBX + meta, report, inspect) when the first real clip passes.
