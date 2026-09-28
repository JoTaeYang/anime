# macOS setup for the player and goblin rig pipelines

The player and goblin rig/clip pipelines run on macOS and Windows through two Python runners (spec `work/player/d-07-cross-platform.md`):

- `work/player/rig/scripts/run_player.py`
- `work/goblin_swing/rig/scripts/run_goblin.py`

Tools are found by `tools/pipeline/platform_tools.py`. On Windows the old `.ps1` files still work; they are thin wrappers around the same runners.

Out of scope: the older Phase 0–2a pipeline (`scripts/run.ps1`) and kimodo.cpp (a Windows build).

## 1. Install
| Tool | Version | How |
|---|---|---|
| Blender | 5.1 | Download from blender.org and drag it to `/Applications` (`/Applications/Blender.app`) |
| Unity | 6000.3.20f1 | Unity Hub, Installs, then add 6000.3.20f1 (default `/Applications/Unity/Hub/Editor/6000.3.20f1/Unity.app`). Sign in and activate a license once in the Hub. |
| ffmpeg | any recent | `brew install ffmpeg` (only needed for the sheet / preview scripts: p33, p34, goblin s10b / s12) |
| Python | ≥ 3.10 | `brew install python`, or use Blender's bundled one (below) |
| git | – | Xcode command line tools or `brew install git`. The source FBX files are in the repo under `assets/source/`. |

If you don't have a system Python, use Blender's bundled one:
`/Applications/Blender.app/Contents/Resources/5.1/python/bin/python3.13`. Check the file name with `ls`.

## 2. Tool paths (only if not at the defaults above)
The first match wins:
1. **Environment variables:** `BLENDER`, `UNITY`, `FFMPEG`. For example:
   ```sh
   export BLENDER=/Applications/Blender-5.1.app/Contents/MacOS/Blender
   ```
2. **`tools/pipeline/tools.local.json`.** It is git-ignored, one per machine. Copy it from `tools/pipeline/tools.local.example.json` and edit it:
   ```json
   {"blender": "/Applications/Blender.app/Contents/MacOS/Blender",
    "unity": "/Applications/Unity/Hub/Editor/6000.3.20f1/Unity.app/Contents/MacOS/Unity",
    "ffmpeg": "/opt/homebrew/bin/ffmpeg"}
   ```
3. **OS defaults:**
   - Blender: `/Applications/Blender.app/Contents/MacOS/Blender`
   - Unity: `/Applications/Unity/Hub/Editor/6000.3.20f1/Unity.app/Contents/MacOS/Unity`
   - ffmpeg: whatever is on `PATH`

The label font for ffmpeg drawtext is `/System/Library/Fonts/Supplemental/Arial.ttf`, falling back to `Helvetica.ttc` and then ffmpeg's default font.

Check what is resolved (nothing runs):
```sh
python3 tools/pipeline/platform_tools.py
python3 work/player/rig/scripts/run_player.py --print-tools clip Sword_Idle --prefix SI
```
A missing tool stops a run with an error that names the tool and says how to set it.

## 3. Commands (from the repo root)
**Close the Unity editor first.** An editor open on `unity/AvatarCheck` blocks batch runs, and `clip` stops with exit 3.

| Task | Command |
|---|---|
| Player clip, end to end | `python3 work/player/rig/scripts/run_player.py clip Sword_Idle --prefix SI` |
| | `python3 work/player/rig/scripts/run_player.py clip Sword_Attack_01 --prefix SA1` |
| Player P2d chain (rigtest, export, Unity import and rig check) | `python3 work/player/rig/scripts/run_player.py p2d` |
| Player gates (full chain and all P2 checkers) | `python3 work/player/rig/scripts/run_player.py gates --run N` |
| Compare two gate runs | `python3 work/player/rig/scripts/run_player.py gates --compare A B` |
| Player Unity method | `python3 work/player/rig/scripts/run_player.py unity PlayerRigCheck.Run` |
| One goblin Blender script | `python3 work/goblin_swing/rig/scripts/run_goblin.py bl s12_clip_anim.py gob_r04_ctrl.blend -- --clip idle` |
| Goblin Unity method | `python3 work/goblin_swing/rig/scripts/run_goblin.py unity GoblinRigCheck.Run` |
| Goblin all gates (G1–G10) | `python3 work/goblin_swing/rig/scripts/run_goblin.py gates` |

To see the commands without running them, add `--print-tools` before the subcommand.

Logs and evidence are the same as on Windows:
- Blender step logs: `work/player/rig/export/logs/`
- Unity logs: `unity/AvatarCheck/Logs/player_<Method>.log` and `goblin_<Method>.log`
- Evidence JSON: `work/player/inspect/…` and `work/goblin_swing/rig/inspect/…`

On Windows, the same commands work with `python` in place of `python3`. The `.ps1` wrappers also still work:
- `run_player_clip.ps1 -Clip Sword_Idle -Prefix SI`
- `run_player_gates.ps1 -Run N`
- `bl.ps1 -Script … -Blend …`
- …

## 4. First check on a Mac (gate X6)
1. Do steps 1 and 2 above.
2. Run `python3 work/player/rig/scripts/run_player.py clip Sword_Idle --prefix SI`. It should exit 0.
3. Open `work/player/inspect/clips/Sword_Idle/` and look at the sheets.
