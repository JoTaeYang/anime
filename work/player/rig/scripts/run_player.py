#!/usr/bin/env python3
"""run_player - cross-platform runner for the player rig / clip pipeline (T340, spec work/player/d-07-cross-platform.md
section 2). Replaces the logic of run_player_clip.ps1 (T300), run_player_p2d.ps1 (T250), run_player_gates.ps1 (T254)
and unity_player.ps1 (T210) with the same steps, order, logs and exit codes; those .ps1 files are now thin wrappers.
Any Python >= 3.10 (system Python or Blender's bundled one). Tools come from tools/pipeline/platform_tools.py.

  python run_player.py clip <clip> [--prefix P]
      p30_clip_anim (pl_r04_ctrl.blend) -> check_player_clip.py (skipped with a notice when missing) -> p31_export_clip
      -> Unity PlayerClipCheck.Run (PLAYER_CLIP=<clip>) -> check_player_clip_unity.py (skipped when missing).
      Stops at the first non-zero exit and exits with that code. A running Unity editor stops the run before the Unity
      step (exit 3). Blender logs: work/player/rig/export/logs/clip_<clip>_<step>.log (+ .err); wall times:
      clip_<clip>_timing.log.
  python run_player.py p2d
      p25_rigtest -> p27_export_fbx -> Unity PlayerImport.Run -> Unity PlayerRigCheck.Run. Stops at the first failing
      step with its exit code. Blender logs: work/player/rig/export/logs/<step>.log; run_player_p2d_timing.log.
  python run_player.py gates [--run N] [--skip-chain] [--continue-on-chain-failure]
  python run_player.py gates --compare A B
      Chain: every p20*..p24* script in name order with the blend named on its docstring "CLI:" line, then p2d; the
      chain stops at the first non-zero exit (checkers then skipped unless --continue-on-chain-failure). Checkers (all,
      in order, exit recorded): check_p2_static, _deform, _skel, _skin, _ctrl, _export, _skirt. Collect / compare:
      gates_collect.py -> work/player/inspect/P2/gates_run_<n>.json / gates_compare_<a>_<b>.json. Logs:
      work/player/rig/export/logs/gates_<step>.log.
  python run_player.py unity <Method>
      Unity batch run; log unity/AvatarCheck/Logs/player_<Method>.log, wall time appended to player_timing.log.

  --print-tools (before the subcommand): print the resolved tools and the commands the subcommand would run, run nothing.
The Unity editor must be closed.
"""
import argparse
import json
import os
import re
import sys
import tempfile
import time
import traceback
from datetime import datetime
from pathlib import Path

S = Path(__file__).resolve().parent
REPO = S.parents[3]
sys.path.insert(0, str(REPO / "tools" / "pipeline"))
sys.path.insert(0, str(S))
import platform_tools as pt  # noqa: E402
import gates_collect  # noqa: E402

RIG = REPO / "work" / "player" / "rig"
LOG_DIR = RIG / "export" / "logs"
OUT = REPO / "work" / "player" / "inspect" / "P2"
UNITY_PROJECT = REPO / "unity" / "AvatarCheck"
UNITY_LOG_DIR = UNITY_PROJECT / "Logs"
CHECKERS = ["check_p2_static", "check_p2_deform", "check_p2_skel", "check_p2_skin", "check_p2_ctrl",
            "check_p2_export", "check_p2_skirt"]
DRY = False


def say(msg):
    print(msg, flush=True)


def fmt_cmd(cmd):
    return " ".join(f'"{c}"' if (" " in str(c) or not str(c)) else str(c) for c in cmd)


def write_lines(path, lines):
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("".join(f"{x}\n" for x in lines))


def ps_round(x):
    """[math]::Round(x, 1) as ConvertTo-Json writes it (4.0 -> 4)."""
    r = round(x, 1)
    return int(r) if r == int(r) else r


def blender_run(args, log):
    """Blender step, stdout -> log, stderr -> log.err. -> (exit, wall seconds)."""
    if DRY:
        say(f"  would run: {fmt_cmd([pt.tool_or_placeholder('blender')] + args)}  > {log}")
        return 0, 0.0
    t0 = time.perf_counter()
    code = pt.run_wait([pt.blender()] + args, stdout=log, stderr=f"{log}.err")
    return code, time.perf_counter() - t0


def unity_player(method, env=None):
    """unity_player.ps1: Unity batch run, waits for the Unity process only. -> exit code."""
    args = ["-batchmode", "-quit", "-projectPath", str(UNITY_PROJECT), "-executeMethod", method,
            "-logFile", str(UNITY_LOG_DIR / f"player_{method}.log")]
    if DRY:
        extra = f"  (env PLAYER_CLIP={env['PLAYER_CLIP']})" if env and "PLAYER_CLIP" in env else ""
        say(f"  would run: {fmt_cmd([pt.tool_or_placeholder('unity')] + args)}{extra}")
        return 0
    UNITY_LOG_DIR.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()
    code = pt.run_wait([pt.unity()] + args, env=env)
    wall = time.perf_counter() - t0
    line = f"{datetime.now().strftime('%Y-%m-%dT%H:%M:%S')} {method} exit {code} wall {wall:.1f} s"
    say(f"[unity_player] {line}")
    with open(UNITY_LOG_DIR / "player_timing.log", "a", encoding="utf-8", newline="") as f:
        f.write(line + "\n")
    return code


# ---------------------------------------------------------------- clip (run_player_clip.ps1)
def cmd_clip(clip, prefix):
    if not DRY:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
    timing = []
    pre = ["--prefix", prefix] if prefix else []
    bg = ["--background", "--factory-startup"]
    steps = [
        ("p30_clip_anim", "blender", "p30_clip_anim.py",
         bg + [str(RIG / "pl_r04_ctrl.blend"), "--python", str(S / "p30_clip_anim.py"), "--", "--clip", clip]),
        ("check_player_clip", "blender", "check_player_clip.py",
         bg + ["--python", str(S / "check_player_clip.py"), "--", "--clip", clip] + pre),
        ("p31_export_clip", "blender", "p31_export_clip.py",
         bg + ["--python", str(S / "p31_export_clip.py"), "--", "--clip", clip]),
        ("PlayerClipCheck.Run", "unity", None, None),
        ("check_player_clip_unity", "blender", "check_player_clip_unity.py",
         bg + ["--python", str(S / "check_player_clip_unity.py"), "--", "--clip", clip] + pre),
    ]
    t_all = time.perf_counter()
    for name, kind, script, args in steps:
        if script and not (S / script).exists():
            line = f"{name} skipped ({script} missing)"
            say(f"[run_player_clip] NOTICE {line}")
            timing.append(line)
            continue
        if kind == "blender":
            code, wall = blender_run(args, LOG_DIR / f"clip_{clip}_{name}.log")
        else:
            if DRY:
                say(f"  would check: no {pt.unity_process_name()} running and {UNITY_PROJECT} not open (else exit 3)")
                code, wall = unity_player(name, {"PLAYER_CLIP": clip}), 0.0
            else:
                pids = pt.unity_pids()
                if pids:
                    say(f"[run_player_clip] {pt.unity_process_name()} is running (pids {', '.join(map(str, pids))}); "
                        f"close it first")
                    code, wall = 3, 0.0
                elif pt.unity_project_locked(UNITY_PROJECT):
                    say(f"[run_player_clip] {UNITY_PROJECT} is open in a Unity editor; close it first")
                    code, wall = 3, 0.0
                else:
                    t0 = time.perf_counter()
                    code = unity_player(name, dict(os.environ, PLAYER_CLIP=clip))
                    wall = time.perf_counter() - t0
        if DRY:
            continue
        line = f"{name} exit {code} wall {wall:.1f} s"
        say(f"[run_player_clip] {line}")
        timing.append(line)
        if code != 0:
            say(f"[run_player_clip] FAILED at {name} (clip {clip})")
            timing.append(f"FAILED at {name}; total wall {time.perf_counter() - t_all:.1f} s")
            write_lines(LOG_DIR / f"clip_{clip}_timing.log", timing)
            return code
    if DRY:
        return 0
    total = time.perf_counter() - t_all
    timing.append(f"all steps exit 0; total wall {total:.1f} s")
    write_lines(LOG_DIR / f"clip_{clip}_timing.log", timing)
    say(f"[run_player_clip] all steps exit 0 (clip {clip}), total wall {total:.1f} s")
    return 0


# ---------------------------------------------------------------- p2d (run_player_p2d.ps1)
def cmd_p2d():
    if not DRY:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
    timing = []
    bg = ["--background", "--factory-startup"]
    steps = [
        ("p25_rigtest", "blender", bg + [str(RIG / "pl_r04_ctrl.blend"), "--python", str(S / "p25_rigtest.py")]),
        ("p27_export_fbx", "blender", bg + ["--python", str(S / "p27_export_fbx.py")]),
        ("PlayerImport.Run", "unity", None),
        ("PlayerRigCheck.Run", "unity", None),
    ]
    for name, kind, args in steps:
        if kind == "blender":
            code, wall = blender_run(args, LOG_DIR / f"{name}.log")
        else:
            t0 = time.perf_counter()
            code = unity_player(name)
            wall = time.perf_counter() - t0
        if DRY:
            continue
        line = f"{name} exit {code} wall {wall:.1f} s"
        say(f"[run_player_p2d] {line}")
        timing.append(line)
        if code != 0:
            say(f"[run_player_p2d] FAILED at {name}")
            write_lines(LOG_DIR / "run_player_p2d_timing.log", timing)
            return code
    if DRY:
        return 0
    write_lines(LOG_DIR / "run_player_p2d_timing.log", timing)
    say("[run_player_p2d] all steps exit 0")
    return 0


# ---------------------------------------------------------------- gates (run_player_gates.ps1)
def gates_step(name, args):
    log = LOG_DIR / f"gates_{name}.log"
    code, wall = blender_run(args, log)
    r = {"name": name, "exit": code, "wall_s": ps_round(wall), "log": str(log)}
    if not DRY:
        say(f"[run_player_gates] {name} exit {r['exit']} wall {r['wall_s']} s")
    return r


def cmd_gates(run, skip_chain, continue_on_chain_failure, compare):
    if not DRY:
        OUT.mkdir(parents=True, exist_ok=True)
        LOG_DIR.mkdir(parents=True, exist_ok=True)
    if compare:
        a, b = compare
        if a <= 0 or b <= 0:
            raise SystemExit("--compare needs A B > 0")
        ja, jb = OUT / f"gates_run_{a}.json", OUT / f"gates_run_{b}.json"
        out = OUT / f"gates_compare_{a}_{b}.json"
        if DRY:
            say(f"  would compare: {ja} vs {jb} -> {out}")
            return 0
        for j in (ja, jb):
            if not j.exists():
                raise SystemExit(f"missing {j}")
        code = gates_collect.compare(str(ja), str(jb), str(out))
        say(f"[run_player_gates] compare {a} vs {b} exit {code}")
        return code
    if run <= 0:
        nums = [int(m.group(1)) for p in OUT.glob("gates_run_*.json")
                if (m := re.match(r"^gates_run_(\d+)$", p.stem, re.I))]
        run = 1 + max(nums + [0])
    started = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    chain, chain_ok = [], True
    if not skip_chain:
        prod = sorted((p for p in S.glob("p2*.py") if re.match(r"^p2[0-4]", p.name, re.I)), key=lambda p: p.name.lower())
        for f in prod:
            cli = None
            for line in f.read_text(encoding="utf-8").splitlines():
                m = re.match(r"^CLI:\s+blender\s+(.*)$", line, re.I)
                if m:
                    cli = m
                    break
            if cli is None:
                raise SystemExit(f"no CLI: line in {f.name}")
            m = re.search(r"--factory-startup\s+(\S+\.blend)\s+--python", cli.group(1), re.I)
            blend = m.group(1) if m else None
            arg_list = ["--background", "--factory-startup"]
            if blend:
                arg_list.append(str(REPO / blend))
            arg_list += ["--python", str(f)]
            r = gates_step(f.stem, arg_list)
            r["blend"] = blend
            chain.append(r)
            if r["exit"] != 0:
                chain_ok = False
                break
        if chain_ok:
            t0 = time.perf_counter()
            try:
                code = cmd_p2d()
            except Exception:
                traceback.print_exc()
                code = 1
            chain.append({"name": "run_player_p2d", "exit": code, "wall_s": ps_round(time.perf_counter() - t0)})
            if not DRY:
                say(f"[run_player_gates] run_player_p2d exit {code}")
            if code != 0:
                chain_ok = False
    checks = []
    if chain_ok or continue_on_chain_failure:
        for c in CHECKERS:
            checks.append(gates_step(c, ["--background", "--factory-startup", "--python", str(S / f"{c}.py")]))
    else:
        say("[run_player_gates] chain failed: checkers skipped (use -ContinueOnChainFailure to run them)")
    out_json = OUT / f"gates_run_{run}.json"
    if DRY:
        say(f"  would collect: {out_json}")
        return 0
    steps = {"started": started, "finished": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"), "chain_ok": chain_ok,
             "skip_chain": bool(skip_chain), "chain": chain, "checkers": checks}
    steps_json = Path(tempfile.gettempdir()) / f"run_player_gates_steps_{os.getpid()}.json"
    with open(steps_json, "w", encoding="utf-8", newline="") as fh:
        fh.write(json.dumps(steps, indent=1))
    try:
        gates_collect.collect(str(out_json), run, str(steps_json), str(REPO), str(OUT), CHECKERS)
        code = 0
    except Exception:
        traceback.print_exc()
        code = 1
    finally:
        steps_json.unlink(missing_ok=True)
    say(f"[run_player_gates] wrote {out_json} (collector exit {code})")
    return 0 if chain_ok else 1


def main(argv=None):
    global DRY
    ap = argparse.ArgumentParser(prog="run_player.py", description="player rig / clip pipeline runner (T340)")
    ap.add_argument("--print-tools", action="store_true", help="print resolved tools and the commands; run nothing")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("clip")
    c.add_argument("clip")
    c.add_argument("--prefix", default="")
    sub.add_parser("p2d")
    g = sub.add_parser("gates")
    g.add_argument("--run", type=int, default=0)
    g.add_argument("--skip-chain", action="store_true")
    g.add_argument("--continue-on-chain-failure", action="store_true")
    g.add_argument("--compare", nargs=2, type=int, metavar=("A", "B"))
    u = sub.add_parser("unity")
    u.add_argument("method")
    a = ap.parse_args(argv)
    DRY = a.print_tools
    if DRY:
        say("\n".join(pt.describe()))
        say(f"dry-run: run_player.py {a.cmd}")
    try:
        if a.cmd == "clip":
            return cmd_clip(a.clip, a.prefix)
        if a.cmd == "p2d":
            return cmd_p2d()
        if a.cmd == "gates":
            return cmd_gates(a.run, a.skip_chain, a.continue_on_chain_failure, a.compare)
        return unity_player(a.method)
    except pt.ToolNotFound as e:
        say(f"[run_player] {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
