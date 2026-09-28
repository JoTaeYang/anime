#!/usr/bin/env python3
"""run_goblin - cross-platform runner for the goblin rig pipeline (T340, spec work/player/d-07-cross-platform.md
section 2). Replaces the logic of bl.ps1, unity_goblin.ps1 and run_all_gates.ps1 with the same steps, order, logs and
exit codes; those .ps1 files are now thin wrappers. Any Python >= 3.10 (system Python or Blender's bundled one).
Tools come from tools/pipeline/platform_tools.py.

  python run_goblin.py bl <script> [blend] [-- <script args>]
      blender --background --factory-startup [rig/<blend>] --python-exit-code 1 --python scripts/<script> [-- args];
      output to the console; exits with Blender's exit code.
  python run_goblin.py unity <Method>
      Unity batch run (waits for the Unity process only); log unity/AvatarCheck/Logs/goblin_<Method>.log.
  python run_goblin.py gates
      run_all_gates: the whole chain s00 .. s09 with the G1..G8 checkers, copy of the FBX files to Unity,
      GoblinRigCheck.Run, compare_g9, then the G10.3 git-status scope list (inspect/G10/scope.json). Runs from the
      repo root; the first failing step stops the run (exit 1).

  --print-tools (before the subcommand): print the resolved tools and the commands the subcommand would run, run nothing.
The Unity editor must be closed.
"""
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

S = Path(__file__).resolve().parent
RIG = S.parent
REPO = S.parents[3]
sys.path.insert(0, str(REPO / "tools" / "pipeline"))
import platform_tools as pt  # noqa: E402

UNITY_PROJECT = REPO / "unity" / "AvatarCheck"
DRY = False

GATES_CHAIN = [
    ("s00_source_prep.py", ""),
    ("s01_pivots.py", "gob_r00_source.blend"),
    ("check_g1_source.py", "gob_r00_source.blend"),
    ("s02a_body.py", "gob_r00_source.blend"),
    ("s02b_limbs.py", "work/r01a_body.blend"),
    ("s02c_rigid.py", "work/r01b_limbs.blend"),
    ("s02d_finish.py", "work/r01c_rigid.blend"),
    ("s02e_mouth.py", "work/r01d_merged.blend"),
    ("check_g2_static.py", "gob_r01_retopo.blend"),
    ("check_g2_shape.py", "gob_r01_retopo.blend"),
    ("s03_temprig.py", "gob_r01_retopo.blend"),
    ("check_g3_deform.py", "gob_r01t_temprig.blend"),
    ("s04_skeleton.py", "gob_r01_retopo.blend"),
    ("check_g4_skeleton.py", "gob_r02_skeleton.blend"),
    ("s05_skin.py", "gob_r02_skeleton.blend"),
    ("check_g5_skin.py", "gob_r03_skinned.blend"),
    ("s06a_ctrl_torso_head.py", "gob_r03_skinned.blend"),
    ("s06b_ctrl_arms.py", "work/r04a.blend"),
    ("s06c_ctrl_legs.py", "work/r04b.blend"),
    ("s06d_ctrl_weapon.py", "work/r04c.blend"),
    ("s06e_rig_ui.py", "work/r04d.blend"),
    ("check_g6_ctrl.py", "gob_r04_ctrl.blend"),
    ("s07a_rigtest.py", "gob_r04_ctrl.blend"),
    ("check_g7_bake.py", "gob_r05_rigtest.blend"),
    ("s08_export_fbx.py", "gob_r05_rigtest.blend"),
    ("check_g8_fbx.py", ""),
]
UNITY_COPY = ["work/goblin_swing/rig/export/goblin.fbx", "work/goblin_swing/rig/export/goblin@rigtest.fbx",
              "work/goblin_swing/rig/export/goblin_club.fbx"]
UNITY_GOBLIN_DIR = "unity/AvatarCheck/Assets/Goblin"
G10_ALLOWED = (r'^(\?\?|.M|M.|A.)\s+"?(work/goblin_swing/rig/|unity/AvatarCheck/Assets/Editor/Goblin|'
               r'unity/AvatarCheck/Assets/Goblin)')


def say(msg):
    print(msg, flush=True)


def fmt_cmd(cmd):
    return " ".join(f'"{c}"' if (" " in str(c) or not str(c)) else str(c) for c in cmd)


def bl(script, blend="", rest=(), cwd=None):
    """bl.ps1. -> Blender's exit code."""
    a = ["--background", "--factory-startup"]
    if blend:
        a.append(str(RIG / blend))
    a += ["--python-exit-code", "1", "--python", str(S / script)]
    if rest:
        a += ["--"] + list(rest)
    if DRY:
        say(f"  would run: {fmt_cmd([pt.tool_or_placeholder('blender')] + a)}")
        return 0
    return pt.run_wait([pt.blender()] + a, cwd=cwd)


def unity_goblin(method, cwd=None):
    """unity_goblin.ps1: Unity batch run, waits for the Unity process only. -> exit code."""
    args = ["-batchmode", "-quit", "-projectPath", str(UNITY_PROJECT), "-executeMethod", method,
            "-logFile", str(UNITY_PROJECT / "Logs" / f"goblin_{method}.log")]
    if DRY:
        say(f"  would run: {fmt_cmd([pt.tool_or_placeholder('unity')] + args)}")
        return 0
    return pt.run_wait([pt.unity()] + args, cwd=cwd)


def _multiset_new(base, now):
    """Compare-Object base now | SideIndicator '=>': lines of `now` not matched in `base` (case-insensitive, each base
    line matches once)."""
    left = {}
    for x in base:
        left[x.lower()] = left.get(x.lower(), 0) + 1
    new = []
    for x in now:
        k = x.lower()
        if left.get(k, 0) > 0:
            left[k] -= 1
        else:
            new.append(x)
    return new


def cmd_gates():
    """run_all_gates.ps1."""
    def BL(script, blend=""):
        say(f">>> {script} {blend} ")
        code = bl(script, blend, cwd=REPO)
        if code != 0:
            raise SystemExit(f"FAILED {script} (exit {code})")

    for script, blend in GATES_CHAIN:
        BL(script, blend)
    dst = REPO / UNITY_GOBLIN_DIR
    if DRY:
        say(f"  would copy: {', '.join(UNITY_COPY)} -> {UNITY_GOBLIN_DIR}/")
    else:
        dst.mkdir(parents=True, exist_ok=True)
        for f in UNITY_COPY:
            shutil.copy2(REPO / f, dst / Path(f).name)
    BL("s09_blender_ref.py", "export/stage_rigtest.blend")
    code = unity_goblin("GoblinRigCheck.Run", cwd=REPO)
    if code != 0:
        raise SystemExit(f"FAILED GoblinRigCheck (exit {code})")
    BL("compare_g9.py")
    # G10.3: paths changed against the baseline
    base_file = RIG / "inspect" / "baseline_git_status.txt"
    out = RIG / "inspect" / "G10" / "scope.json"
    if DRY:
        say(f"  would run: git status --porcelain -uall (cwd {REPO}) vs {base_file} -> {out}")
        return 0
    base = base_file.read_text(encoding="utf-8-sig").splitlines()
    now = subprocess.run(["git", "status", "--porcelain", "-uall"], cwd=REPO, capture_output=True,
                         check=True).stdout.decode("utf-8").splitlines()
    new = _multiset_new(base, now)
    bad = [x for x in new if not re.match(G10_ALLOWED, x, re.I)]
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="") as f:
        f.write(json.dumps({"new_status_lines": new, "out_of_scope": bad}, indent=4, ensure_ascii=False))
    say(f"OUT_OF_SCOPE_COUNT={len(bad)}")
    return 0


USAGE = ("usage: run_goblin.py [--print-tools] bl <script> [blend] [-- <args>]\n"
         "       run_goblin.py [--print-tools] unity <Method>\n"
         "       run_goblin.py [--print-tools] gates")


def main(argv=None):
    global DRY
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "--print-tools":
        DRY = True
        argv = argv[1:]
    if not argv or argv[0] in ("-h", "--help"):
        say(USAGE)
        return 0 if argv else 64
    cmd, rest = argv[0], argv[1:]
    if DRY:
        say("\n".join(pt.describe()))
        say(f"dry-run: run_goblin.py {cmd}")
    try:
        if cmd == "bl":
            pos, script_args = (rest[:rest.index("--")], rest[rest.index("--") + 1:]) if "--" in rest else (rest, [])
            if not pos or len(pos) > 2:
                say(f"run_goblin.py bl: expected <script> [blend] [-- <args>], got {rest}")
                return 64
            return bl(pos[0], pos[1] if len(pos) > 1 else "", script_args)
        if cmd == "unity":
            if len(rest) != 1:
                say("run_goblin.py unity: expected <Method>")
                return 64
            return unity_goblin(rest[0])
        if cmd == "gates":
            return cmd_gates()
        say(USAGE)
        return 64
    except pt.ToolNotFound as e:
        say(f"[run_goblin] {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
