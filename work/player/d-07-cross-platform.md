# Cross-platform rig pipelines (Windows + macOS) (2026-09-28)

## 0. Request
- User: "뭐든 mac에서 쓸 수만 있게 해주렴" ("just make it usable on a Mac, whatever it takes").
- Scope: the **player** and **goblin** rig/clip pipelines.
- Out of scope, listed only: the older Phase 0–2a pipeline (`scripts/run.ps1`) and kimodo.cpp (a Windows build).

## 1. Windows-only today
- **PowerShell runners.**
  - Player:
    - `run_player_clip.ps1`
    - `run_player_p2d.ps1`
    - `run_player_gates.ps1` (its collector and compare step are embedded Python)
    - `unity_player.ps1`
  - Goblin:
    - `bl.ps1`
    - `unity_goblin.ps1`
    - `run_all_gates.ps1`
- **Hard-coded tool paths:**
  - `C:\Program Files\Blender Foundation\Blender 5.1\blender.exe`
  - `...\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe`
- **Hard-coded source paths:** `C:\Users\whxod\Downloads\Meshy_*.fbx` in `p10_crude_rig.py`, `p20_retopo.py`, `check_p2_static.py` and goblin `goblib.py`.
  - The sources are now in the repo: `assets/source/player/…`, `assets/source/goblin/…`, with the same sha256 values.
- **Font path:** `C:/Windows/Fonts/arial.ttf` for the ffmpeg drawtext labels (`p34`, possibly `p33` and the goblin sheet scripts).
- **Unity process handling:** wait for Unity.exe only, cache the process handle.

## 2. Design
- **One shared Python module** `tools/pipeline/platform_tools.py` (repo root, standard library only). It resolves:
  - **Blender:**
    - env `BLENDER`
    - else `tools/pipeline/tools.local.json` (git-ignored)
    - else OS defaults:
      - Windows: `Program Files\Blender Foundation\Blender 5.1\blender.exe`
      - macOS: `/Applications/Blender.app/Contents/MacOS/Blender`
      - Linux: `blender` on PATH
  - **Unity:** env `UNITY`, else the local JSON, else the OS default for version 6000.3.20f1:
    - Windows: `C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe`
    - macOS: `/Applications/Unity/Hub/Editor/6000.3.20f1/Unity.app/Contents/MacOS/Unity`
  - **ffmpeg:** env `FFMPEG`, else PATH.
  - **Label font:**
    - Windows: Arial
    - macOS: `/System/Library/Fonts/Supplemental/Arial.ttf`, falling back to `/System/Library/Fonts/Helvetica.ttc`
    - else ffmpeg's default font
  - A clear error names the missing tool and how to set it.
  - Also: a helper that runs a process and waits for it to exit only (no child-process waits), plus a check that no Unity editor is already open on the project.
- **Python runners replace the .ps1 logic, with the same steps, order, logs and exit codes:**
  - `work/player/rig/scripts/run_player.py` with subcommands `clip <name> [--prefix P]`, `p2d`, `gates --run N | --compare A B`, `unity <Method>`.
  - `work/goblin_swing/rig/scripts/run_goblin.py` with subcommands `bl <script> [blend] [-- args]`, `unity <Method>`, `gates`.
  - They run with any Python ≥ 3.10 (system Python or Blender's bundled one).
  - The `.ps1` files become thin wrappers that call the Python runner, so existing Windows habits keep working.
- **Source paths:** every script reads the sources from `assets/source/…` relative to the repo root. The sha256 checks stay.
- **README:** `docs/mac-setup.md` covers installing Blender 5.1, Unity 6000.3.20f1 and ffmpeg (brew), setting paths if non-default, and the one-line commands per task.

## 3. Gates
| ID | Criterion | Status |
|---|---|---|
| X1 | No hard-coded Windows paths (`C:\`, `C:/`, `Program Files`, `.exe`, `Windows/Fonts`) in the player/goblin rig scripts, runners and Unity editor scripts, except inside `platform_tools.py`'s Windows defaults (grep) | blocking |
| X2 | Windows equivalence (player): `run_player.py gates --run 9` gives the same row ok flags and measured-value hashes as `run_player_gates.ps1` run 8. The only differences allowed are source-path strings in the evidence inputs; the sha256 values must be identical. | blocking |
| X3 | Windows equivalence (clips): `run_player.py clip Sword_Idle --prefix SI` and `clip Sword_Attack_01 --prefix SA1` give identical rows (measured values) to the latest evidence | blocking |
| X4 | Windows equivalence (goblin): `run_goblin.py gates` gives the same rows as the last goblin run_all_gates evidence (G1–G10) | blocking |
| X5 | The tool resolution works without the Windows defaults: a dry-run with `--print-tools` under a simulated macOS (env overrides) prints the resolved paths and the commands it would run | blocking |
| X6 | [U] a real run on the user's Mac: `docs/mac-setup.md` steps, then `run_player.py clip Sword_Idle --prefix SI` exits 0 | user |

## 4. Result (2026-09-28, T340; main re-verified)
- **X1:** main grepped for `C:\`, `C:/`, `Program Files`, `.exe`, `Windows/Fonts` and `Downloads` in the player/goblin scripts, the Unity editor scripts and tools/pipeline. The only hits left are the Windows defaults inside `platform_tools.py` and `exec_module` / `.execute` substrings. **ok.**
- **X5:** the simulated-darwin dry-run resolves Blender from env and reports Unity as NOT FOUND with every place it tried. Main's Windows `--print-tools` resolves all the tools. **ok.**
- **X2:** player gates run 9 through `run_player.py` vs run 8: all 101 ok flags are equal. Three measured values differ, all explained:
  - P2.1h: the source path string only (sha256 identical)
  - P2.6c1 and P2.6d: Sword_Attack_01 was added to Unity after run 8 (clips list and bounds union 767 → 816), not caused by T340
  - **ok.**
- **X3:** both clips through `run_player.py`: all C rows identical. U3 differs in report-only fields only, from the regenerated player_report after X2. Main also ran Sword_Idle through the thin `.ps1` wrapper: 15/15 rows identical to the Python-runner run. **ok.**
- **X4:** goblin G1–G10: every row identical (190 rows). `source_prep.json` records the new source path (sha256 identical). **ok.**
- **Regenerated artifacts** from the verification runs (FBX, prefabs, control-rig blends, `mouth.json` with about 1e-4 px drift) were restored to HEAD. Only the source-path record in `source_prep.json` is kept.
- **X6 [U]:** a real Mac run is pending; the steps are in `docs/mac-setup.md` §4.
- **Known:** evidence input hashes follow the worktree's CRLF/LF (autocrlf=true). The rows are unaffected.
