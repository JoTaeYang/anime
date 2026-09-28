"""platform_tools - tool discovery and process helpers for the player / goblin rig pipelines (T340, spec
work/player/d-07-cross-platform.md section 2). Standard library only; runs under any Python >= 3.10, including
Blender's bundled one.

Resolution order (first hit wins):
  Blender  env BLENDER -> tools/pipeline/tools.local.json "blender" -> OS default
  Unity    env UNITY   -> tools/pipeline/tools.local.json "unity"   -> OS default (Unity 6000.3.20f1)
  ffmpeg   env FFMPEG  -> tools/pipeline/tools.local.json "ffmpeg"  -> PATH
  font     OS default label font for ffmpeg drawtext (None -> ffmpeg's default font)

tools.local.json is git-ignored (per machine); see tools.local.example.json.
Env PIPELINE_PLATFORM (win32 | darwin | linux) overrides the detected OS; it exists for dry-runs (X5) only.

CLI:  python tools/pipeline/platform_tools.py      prints the resolved tools
"""
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
LOCAL_JSON = HERE / "tools.local.json"
UNITY_VERSION = "6000.3.20f1"
BLENDER_VERSION = "5.1"
UNITY_PROJECT = REPO / "unity" / "AvatarCheck"


class ToolNotFound(RuntimeError):
    pass


def platform():
    p = os.environ.get("PIPELINE_PLATFORM") or sys.platform
    if p.startswith("win"):
        return "win32"
    if p.startswith("darwin") or p in ("mac", "macos", "osx"):
        return "darwin"
    return "linux"


def _local():
    if not LOCAL_JSON.exists():
        return {}
    return json.loads(LOCAL_JSON.read_text(encoding="utf-8"))


def _win_dir(env, fallback):
    return Path(os.environ.get(env) or fallback)


def _defaults(tool):
    """OS default candidates for a tool (the only place Windows paths live)."""
    plat = platform()
    if tool == "blender":
        if plat == "win32":
            return [_win_dir("ProgramFiles", r"C:\Program Files") / "Blender Foundation" / f"Blender {BLENDER_VERSION}"
                    / "blender.exe"]
        if plat == "darwin":
            return [Path("/Applications/Blender.app/Contents/MacOS/Blender")]
        return ["blender"]
    if tool == "unity":
        if plat == "win32":
            return [_win_dir("ProgramFiles", r"C:\Program Files") / "Unity" / "Hub" / "Editor" / UNITY_VERSION
                    / "Editor" / "Unity.exe"]
        if plat == "darwin":
            return [Path(f"/Applications/Unity/Hub/Editor/{UNITY_VERSION}/Unity.app/Contents/MacOS/Unity")]
        return [Path.home() / "Unity" / "Hub" / "Editor" / UNITY_VERSION / "Editor" / "Unity"]
    if tool == "ffmpeg":
        return ["ffmpeg"]
    raise KeyError(tool)


ENV = {"blender": "BLENDER", "unity": "UNITY", "ffmpeg": "FFMPEG"}


def resolve(tool):
    """-> (path or None, how, tried). `how` names where the hit came from; `tried` lists every miss."""
    env = ENV[tool]
    tried = []
    v = os.environ.get(env)
    if v:
        if Path(v).exists() or shutil.which(v):
            return str(Path(v)) if Path(v).exists() else shutil.which(v), f"env {env}", tried
        tried.append(f"env {env}={v} (missing)")
    else:
        tried.append(f"env {env} (unset)")
    loc = _local().get(tool)
    if loc:
        if Path(loc).exists():
            return str(Path(loc)), f"{LOCAL_JSON.name} '{tool}'", tried
        tried.append(f"{LOCAL_JSON.name} '{tool}'={loc} (missing)")
    else:
        tried.append(f"{LOCAL_JSON.name} '{tool}' (absent)")
    for d in _defaults(tool):
        if isinstance(d, Path):
            if d.exists():
                return str(d), f"{platform()} default", tried
            tried.append(f"{platform()} default {d} (missing)")
        else:
            w = shutil.which(d)
            if w:
                return w, f"PATH ({d})", tried
            tried.append(f"PATH '{d}' (not found)")
    return None, None, tried


def _require(tool):
    path, _how, tried = resolve(tool)
    if path:
        return path
    raise ToolNotFound(
        f"{tool} not found. Tried: {'; '.join(tried)}. Fix: set env {ENV[tool]}=<path to {tool}> or add "
        f"{{\"{tool}\": \"<path>\"}} to {LOCAL_JSON} (see tools.local.example.json and docs/mac-setup.md).")


def blender():
    return _require("blender")


def unity():
    return _require("unity")


def ffmpeg(required=True):
    """Resolved ffmpeg path; required=False returns the bare name 'ffmpeg' instead of raising."""
    if required:
        return _require("ffmpeg")
    path, _how, _tried = resolve("ffmpeg")
    return path or "ffmpeg"


def label_font():
    """Label font file for ffmpeg drawtext, or None (ffmpeg's default font)."""
    plat = platform()
    if plat == "win32":
        cands = [_win_dir("WINDIR", r"C:\Windows") / "Fonts" / "arial.ttf"]
    elif plat == "darwin":
        cands = [Path("/System/Library/Fonts/Supplemental/Arial.ttf"), Path("/System/Library/Fonts/Helvetica.ttc")]
    else:
        cands = []
    for c in cands:
        if c.exists():
            return c
    return None


def drawtext_fontfile():
    """The `fontfile='...':` prefix for an ffmpeg drawtext filter ('' when no font file is found). The drive colon is
    escaped for the filter graph (C:/x -> C\\:/x)."""
    f = label_font()
    if f is None:
        return ""
    return "fontfile='{}':".format(f.as_posix().replace(":", "\\:"))


def unity_process_name():
    """Editor process name on this host (process checks always use the real host OS)."""
    return "Unity.exe" if sys.platform.startswith("win") else "Unity"


def unity_pids():
    """PIDs of running Unity editor processes (any project)."""
    name = unity_process_name()
    try:
        if sys.platform.startswith("win"):
            out = subprocess.run(["tasklist", "/FO", "CSV", "/NH", "/FI", f"IMAGENAME eq {name}"],
                                 capture_output=True).stdout.decode("utf-8", "replace")
            pids = []
            for line in out.splitlines():
                cells = [c.strip('"') for c in line.split('","')]
                if len(cells) > 1 and cells[0].lower() == name.lower():
                    pids.append(int(cells[1]))
            return pids
        out = subprocess.run(["pgrep", "-x", name], capture_output=True).stdout.decode("utf-8", "replace")
        return [int(x) for x in out.split()]
    except (OSError, ValueError):
        return []


def unity_project_locked(project=UNITY_PROJECT):
    """True when an editor holds the project's Temp/UnityLockfile (open on this project)."""
    lock = Path(project) / "Temp" / "UnityLockfile"
    if not lock.exists():
        return False
    if sys.platform.startswith("win"):
        try:
            with open(lock, "a"):
                return False
        except OSError:
            return True
    return bool(unity_pids())


def run_wait(cmd, stdout=None, stderr=None, env=None, cwd=None):
    """Start `cmd` and wait for that process only (not for children it leaves behind, e.g. Unity's dotnet
    helper). stdout / stderr: None (inherit) or a file path. -> exit code."""
    fo = open(stdout, "wb") if stdout else None
    fe = open(stderr, "wb") if stderr else None
    try:
        p = subprocess.Popen([str(c) for c in cmd], stdout=fo, stderr=fe, env=env, cwd=cwd)
        return p.wait()
    finally:
        for f in (fo, fe):
            if f:
                f.close()


def describe():
    """Lines describing every resolved tool (for --print-tools)."""
    lines = [f"platform: {platform()} (host {sys.platform}; PIPELINE_PLATFORM={os.environ.get('PIPELINE_PLATFORM', '')})",
             f"repo: {REPO}", f"tools.local.json: {LOCAL_JSON} ({'present' if LOCAL_JSON.exists() else 'absent'})"]
    for tool in ("blender", "unity", "ffmpeg"):
        path, how, tried = resolve(tool)
        if path:
            lines.append(f"{tool}: {path}  [{how}]")
        else:
            lines.append(f"{tool}: NOT FOUND  [tried: {'; '.join(tried)}]")
    f = label_font()
    lines.append(f"label font: {f if f else 'ffmpeg default font'}")
    return lines


def tool_or_placeholder(tool):
    """For dry-runs: the resolved path, or '<tool: not found>'."""
    path, _how, _tried = resolve(tool)
    return path or f"<{tool}: not found>"


if __name__ == "__main__":
    print("\n".join(describe()))
