# Phase 2a Mixamo 하이브리드 (2a-1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mixamo 모캡 클립을 Unity Humanoid 리타게팅으로 우리 캐릭터에 얹어, 액션 RPG 기본 액션 세트(idle/walk/run/구르기/베기 3콤보/피격/사망)가 실기동 씬에서 재생되게 한다.

**Architecture:** 스펙 §2의 2a-1만 이 계획의 범위다 — Blender 무개입, Unity 에디터 스크립트 2본(클립 임포트+단언 / 애니메이터+실기동 씬 빌드)과 run.ps1 스테이지 2개 추가. walk 파일럿 브랜치 마감(커밋·머지)이 선행 태스크. **2a-2(Blender 리타게팅)와 돌진 베기 실증(스펙 완료 기준 3)은 후속 계획으로 명시적 유보** — 사용자 실기동 게이트 통과 후, 받은 클립을 보고 스파이크부터 시작한다.

**Tech Stack:** Unity 6000.3.20f1 batchmode (`C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe`), UnityEditor.Animations API, PowerShell run.ps1, git.

## Global Constraints

- 단언 약화·화이트리스트·임계값 완화 절대 금지. 승인된 예외는 빈 포인터 조건부 규칙(AvatarCheck.cs 참조) 하나뿐이며 새 코드에도 같은 형태로만 적용한다.
- `docs/` 하위는 커밋하지 않는다 (사용자 방침). `.superpowers/sdd/`는 gitignore 상태 유지.
- `assets/mocap/*.fbx`는 커밋 금지 (Mixamo 원본 재배포 금지 — gitignore로 강제). `assets/mocap/clips.json`은 커밋한다.
- 기존 스테이지·검사(check_00~04, AvatarCheck.Run)와 dummy 프로필 회귀는 건드리지 않는다.
- Unity batchmode는 에디터가 닫혀 있어야 한다. 실행 전 `Get-Process Unity`로 확인, 열려 있으면 사용자에게 닫기 요청.
- Unity 종료 코드는 `$LASTEXITCODE`가 아니라 Start-Process -Wait -PassThru의 .ExitCode로 읽는다 (run.ps1 기존 패턴).
- 커밋 메시지 끝: `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`

---

### Task 1: walk 파일럿 브랜치 마감 — 검증·커밋·머지·새 브랜치

**Files:**
- Commit: `scripts/anim/walk_poses.py`, `scripts/anim/walk_build.py`, `scripts/anim/followthrough.py`, `scripts/preview/user_preview.py`, `PLAN.md` (모두 이미 수정돼 있음 — 새 편집 없음)

**Interfaces:**
- Produces: main에 머지된 walk 파일럿(판정 기록 포함), 새 브랜치 `feature/phase2a-mixamo` (이후 태스크 전부 이 브랜치에서)

- [ ] **Step 1: 워킹트리 상태 확인**

Run: `git status --short`
Expected: 위 5개 파일이 ` M`으로 표시. 그 외 수정 파일이 있으면 STOP하고 컨트롤러에 보고 (untracked인 `assets/`, `refs/`, `docs/`, `unity/.../Objects/` 등은 무시 — 커밋하지 않는다).

- [ ] **Step 2: 양 프로필 전체 회귀**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile character`
Expected: `ALL STAGES PASSED (character)`, report.json `allPass:true`
Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 all -Profile dummy`
Expected: `ALL STAGES PASSED (dummy)`, report.json `allPass:true`
(Unity 단계가 있으므로 시작 전 `Get-Process Unity -ErrorAction SilentlyContinue` 결과가 비어 있어야 한다.)

- [ ] **Step 3: 커밋**

```powershell
git add scripts\anim\walk_poses.py scripts\anim\walk_build.py scripts\anim\followthrough.py scripts\preview\user_preview.py PLAN.md
git commit -m @'
feat: walk quality iterations + pilot verdict (unsatisfactory -> Mixamo hybrid)

- torso_bob 0.028->0.012, per-chain follow-through gain (skirt damped, tail lateral sway from delayed pelvis yaw)
- dynamic elbow bend (22+14deg, 2f drag) + wrist drag keys, walk video framing fix (5.5m cam)
- PLAN.md: pilot verdict recorded per pre-agreed rule; direction = 2a Mixamo hybrid, 2b loop retained for signature actions

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```

- [ ] **Step 4: main 머지 + 푸시 + 새 브랜치**

```powershell
git checkout main
git merge --no-ff feature/phase2-walk-pilot -m @'
merge: phase2 walk pilot (verdict: unsatisfactory -> pivot to Mixamo hybrid)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
git push origin main
git checkout -b feature/phase2a-mixamo
```
Expected: 머지 커밋 생성, push 성공, 새 브랜치 체크아웃.

- [ ] **Step 5: 보고**

`.superpowers/sdd/p2a-task-1-report.md`에 회귀 로그 요지(양 프로필 결과 라인)와 머지/브랜치 해시 기록.

---

### Task 2: mocap 반입 규칙 + 클립 목록 + 사용자 다운로드 게이트

**Files:**
- Create: `refs/mixamo/clip-list.md`, `assets/mocap/clips.json`
- Modify: `.gitignore` (루트)

**Interfaces:**
- Produces: `assets/mocap/clips.json` — Task 3의 ClipImport가 읽는 스키마 `{"clips":[{"file","name","loop"}]}`. 파일명 규약(Idle.fbx 등)은 사용자 다운로드 시 저장 이름과 일치해야 한다.

- [ ] **Step 1: gitignore에 mocap FBX 제외 추가**

`.gitignore` 끝에 추가:

```gitignore
# Mixamo 원본 재배포 금지 (Adobe 약관) — 목록/설정만 커밋
assets/mocap/*.fbx
# Task 3/4 생성물 (스크립트로 재생성 가능)
unity/AvatarCheck/Assets/Clips/
unity/AvatarCheck/Assets/Clips.meta
unity/AvatarCheck/Assets/Play/Player.controller
unity/AvatarCheck/Assets/Play/Player.controller.meta
unity/AvatarCheck/Assets/Play/PlayScene.unity
unity/AvatarCheck/Assets/Play/PlayScene.unity.meta
unity/AvatarCheck/clips_report.json
unity/AvatarCheck/playscene_report.json
```

- [ ] **Step 2: clips.json 작성**

`assets/mocap/clips.json`:

```json
{"clips": [
  {"file": "Idle.fbx",    "name": "Idle",    "loop": true},
  {"file": "Walking.fbx", "name": "Walk",    "loop": true},
  {"file": "Running.fbx", "name": "Run",     "loop": true},
  {"file": "Roll.fbx",    "name": "Roll",    "loop": false},
  {"file": "Attack1.fbx", "name": "Attack1", "loop": false},
  {"file": "Attack2.fbx", "name": "Attack2", "loop": false},
  {"file": "Attack3.fbx", "name": "Attack3", "loop": false},
  {"file": "Hit.fbx",     "name": "Hit",     "loop": false},
  {"file": "Death.fbx",   "name": "Death",   "loop": false}
]}
```

- [ ] **Step 3: clip-list.md 작성 (사용자 안내문)**

`refs/mixamo/clip-list.md`:

```markdown
# Mixamo 다운로드 목록 (Phase 2a-1)

mixamo.com 로그인(Adobe 계정) → 캐릭터는 기본 Y Bot 그대로 두고 애니메이션만 검색.

## 공통 다운로드 설정
- Format: **FBX Binary (.fbx)**
- Skin: **Without Skin**
- Frames per Second: **30**
- Keyframe Reduction: **none**

## 클립 (저장 파일명을 정확히 맞출 것 → assets/mocap/에 배치)

| 저장 파일명 | Mixamo 검색어 | 다운로드 옵션 | 용도 |
|---|---|---|---|
| Idle.fbx | Idle (또는 Sword And Shield Idle) | 기본 | 대기 |
| Walking.fbx | Walking | **In Place 체크** | 걷기 |
| Running.fbx | Running | **In Place 체크** | 달리기 |
| Roll.fbx | Stand To Roll | 기본 | 구르기 |
| Attack1.fbx | Sword And Shield Slash | 기본 | 콤보 1타 |
| Attack2.fbx | Sword And Shield Attack (또는 다른 Slash 변형) | 기본 | 콤보 2타 |
| Attack3.fbx | Stable Sword Outward Slash (또는 다른 변형) | 기본 | 콤보 3타 |
| Hit.fbx | Sword And Shield Impact | 기본 | 피격 |
| Death.fbx | Sword And Shield Death | 기본 | 사망 |
| (선택) DashSlashBase.fbx | Running Slash 계열 | 기본 | 돌진 베기 베이스 (후속 계획) |

- 검색 결과에서 미리보기로 마음에 드는 변형을 고르면 됩니다 — 표의 검색어는 출발점.
- 콤보 3타는 서로 다른 동작 3개면 됩니다 (같은 클립 3번 아님).
- In Place 체크가 없는 클립(Walking/Running에만 있음)은 그대로 받으면 됩니다.
```

- [ ] **Step 4: 커밋**

```powershell
git add .gitignore assets\mocap\clips.json refs\mixamo\clip-list.md
git commit -m @'
feat: mixamo intake rules - clip list, download guide, gitignore guards

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```

- [ ] **Step 5: 사용자 게이트 (컨트롤러 수행)**

컨트롤러가 사용자에게 `refs/mixamo/clip-list.md` 기준 다운로드를 요청한다. **9개 파일이 `assets/mocap/`에 놓이기 전까지 Task 3 착수 금지.** 확인 명령: `Get-ChildItem assets\mocap\*.fbx` → 9개(선택 클립 제외) 존재.

---

### Task 3: Unity 클립 임포트 자동화 + 기계 단언 (`clips` 스테이지)

**Files:**
- Create: `unity/AvatarCheck/Assets/Editor/ClipImport.cs`
- Modify: `scripts/run.ps1` (Invoke-Unity 일반화 + `clips` 스테이지)

**Interfaces:**
- Consumes: `assets/mocap/clips.json` (Task 2 스키마), `assets/mocap/*.fbx` (사용자 다운로드)
- Produces: `unity/AvatarCheck/Assets/Clips/<file>` 임포트 자산 — 각 FBX 안의 AnimationClip 이름은 clips.json의 `name`으로 리네임됨 (Task 4가 이 이름으로 클립을 찾는다). `unity/AvatarCheck/clips_report.json` (`{"allPass":bool,"assertions":[...]}` — report.json과 동일 구조).

- [ ] **Step 1: run.ps1 Invoke-Unity 일반화 + clips 스테이지**

`scripts/run.ps1`의 `Invoke-UnityCheck` 함수를 다음으로 교체 (기존 호출부 `"unity" { Invoke-UnityCheck }`와 "all" 내 호출은 그대로 동작):

```powershell
function Invoke-Unity([string]$Method, [string]$LogName, [string]$ReportName) {
    Write-Host ">>> unity: $Method"
    # Unity relaunches as a separate process, so $LASTEXITCODE after `& $Unity` is
    # unreliable (Task 8). Use Start-Process -Wait -PassThru and read .ExitCode.
    $unityArgs = @(
        "-batchmode", "-nographics", "-quit",
        "-projectPath", (Join-Path $Root "unity\AvatarCheck"),
        "-executeMethod", $Method,
        "-logFile", (Join-Path $Root "unity\AvatarCheck\Logs\$LogName")
    )
    $proc = Start-Process -FilePath $Unity -ArgumentList $unityArgs -Wait -PassThru -NoNewWindow
    $code = $proc.ExitCode
    $report = Join-Path $Root "unity\AvatarCheck\$ReportName"
    if (Test-Path $report) { Get-Content $report }
    if ($code -ne 0) {
        Write-Host "FAILED: $Method (exit $code)" -ForegroundColor Red
        exit $code
    }
}
function Invoke-UnityCheck { Invoke-Unity "AvatarCheck.Run" "check.log" "report.json" }
```

switch에 추가 (`"unity"` 케이스 아래):

```powershell
    "clips" { Invoke-Unity "ClipImport.Run" "clips.log" "clips_report.json" }
```

usage 문자열의 스테이지 목록에 `clips`를 추가한다.

- [ ] **Step 2: ClipImport.cs 작성**

`unity/AvatarCheck/Assets/Editor/ClipImport.cs`:

```csharp
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

public static class ClipImport
{
    [Serializable] class ClipEntry { public string file; public string name; public bool loop; }
    [Serializable] class ClipConfig { public ClipEntry[] clips; }

    static readonly List<string> ImportErrors = new List<string>();
    static void CaptureLog(string condition, string stackTrace, LogType type)
    {
        if (type == LogType.Error || type == LogType.Exception || type == LogType.Warning)
            ImportErrors.Add($"[{type}] {condition}");
    }

    static string RepoRoot() =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));

    public static void Run()
    {
        var results = new List<(string name, bool pass, string detail)>();
        try
        {
            string cfgPath = Path.Combine(RepoRoot(), "assets", "mocap", "clips.json");
            var cfg = JsonUtility.FromJson<ClipConfig>(File.ReadAllText(cfgPath));

            if (!AssetDatabase.IsValidFolder("Assets/Clips"))
                AssetDatabase.CreateFolder("Assets", "Clips");

            foreach (var e in cfg.clips)
            {
                string src = Path.Combine(RepoRoot(), "assets", "mocap", e.file);
                if (!File.Exists(src))
                {
                    results.Add(($"fbx_present:{e.name}", false, $"missing {src} — refs/mixamo/clip-list.md 참조, 사용자 다운로드 필요"));
                    continue;
                }
                string assetPath = $"Assets/Clips/{e.file}";
                if (AssetDatabase.LoadMainAssetAtPath(assetPath) != null)
                    AssetDatabase.DeleteAsset(assetPath);   // Phase1 신규 반입 프로토콜: 항상 새 임포트
                File.Copy(src, Path.GetFullPath(assetPath), overwrite: true);
                AssetDatabase.ImportAsset(assetPath, ImportAssetOptions.ForceSynchronousImport);

                var importer = (ModelImporter)AssetImporter.GetAtPath(assetPath);
                importer.animationType = ModelImporterAnimationType.Human;   // Mixamo 표준 스켈레톤은 자동 매핑이 정상 동작
                importer.avatarSetup = ModelImporterAvatarSetup.CreateFromThisModel;
                var takes = importer.defaultClipAnimations;
                foreach (var t in takes) { t.name = e.name; t.loopTime = e.loop; }
                importer.clipAnimations = takes;

                ImportErrors.Clear();
                Application.logMessageReceived += CaptureLog;
                importer.SaveAndReimport();
                Application.logMessageReceived -= CaptureLog;

                // 사용자 승인(2026-07-19) 조건부 규칙 — AvatarCheck.cs와 동일 형태.
                // 빈 포인터 배너만 비차단, m_AnimationImportWarnings에 내용 있으면 그 내용으로 실패.
                const string PointerMarker = "has animation import warnings. See Import Messages";
                var pointerEntries = ImportErrors.Where(x => x.Contains(PointerMarker)).ToList();
                var otherErrors = ImportErrors.Where(x => !x.Contains(PointerMarker)).ToList();
                if (pointerEntries.Count > 0)
                {
                    var so = new SerializedObject(importer);
                    var warnProp = so.FindProperty("m_AnimationImportWarnings");
                    string store = warnProp != null ? warnProp.stringValue : null;
                    if (string.IsNullOrEmpty(store))
                        results.Add(($"anim_warning_pointer:{e.name}", true, "empty pointer warning (approved conditional rule)"));
                    else
                        otherErrors.AddRange(pointerEntries.Select(_ => $"[Warning] store: {store}"));
                }
                results.Add(($"import_clean:{e.name}", otherErrors.Count == 0,
                    otherErrors.Count == 0 ? "no errors/warnings" : string.Join(" | ", otherErrors.Take(50))));

                var avatar = AssetDatabase.LoadAllAssetsAtPath(assetPath).OfType<Avatar>().FirstOrDefault();
                results.Add(($"avatar_human:{e.name}", avatar != null && avatar.isValid && avatar.isHuman,
                    avatar == null ? "no avatar" : $"isValid={avatar.isValid} isHuman={avatar.isHuman}"));

                var clip = AssetDatabase.LoadAllAssetsAtPath(assetPath).OfType<AnimationClip>()
                    .FirstOrDefault(c => c.name == e.name);
                bool clipOk = clip != null && clip.length > 0.1f;
                results.Add(($"clip_present:{e.name}", clipOk,
                    clip == null ? $"no clip named {e.name}" : $"len={clip.length:F3}s"));
                if (clip != null)
                    results.Add(($"loop_flag:{e.name}", clip.isLooping == e.loop,
                        $"isLooping={clip.isLooping} expected {e.loop}"));
            }
        }
        catch (Exception ex)
        {
            results.Add(("run_exception", false, ex.ToString()));
        }
        WriteReport(results);
    }

    static void WriteReport(List<(string name, bool pass, string detail)> results)
    {
        bool allPass = results.Count > 0 && results.All(r => r.pass);
        string json = "{\"allPass\":" + (allPass ? "true" : "false") + ",\"assertions\":[" +
            string.Join(",", results.Select(r =>
                "{\"name\":\"" + r.name + "\",\"pass\":" + (r.pass ? "true" : "false") +
                ",\"detail\":\"" + r.detail.Replace("\\", "/").Replace("\"", "'") + "\"}")) + "]}";
        File.WriteAllText(Path.Combine(RepoRoot(), "unity", "AvatarCheck", "clips_report.json"), json);
        Debug.Log("CLIPS REPORT: " + json);
        EditorApplication.Exit(allPass ? 0 : 1);
    }
}
```

- [ ] **Step 3: 실패 경로 먼저 확인 (fbx 없는 상태 재현)**

임의 클립 하나를 임시로 옮겨 실패를 확인한다:

```powershell
Move-Item assets\mocap\Idle.fbx assets\mocap\Idle.fbx.bak
powershell -ExecutionPolicy Bypass -File scripts\run.ps1 clips
```
Expected: `FAILED: ClipImport.Run (exit 1)`, clips_report.json에 `fbx_present:Idle` false.

```powershell
Move-Item assets\mocap\Idle.fbx.bak assets\mocap\Idle.fbx
```

- [ ] **Step 4: 정상 경로 실행**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 clips`
Expected: exit 0, clips_report.json `allPass:true`, 클립 9개 각각 `import_clean`/`avatar_human`/`clip_present`/`loop_flag` 통과. 실패 시 원인 조사 — **단언을 약화하지 말 것.** Mixamo 클립이 실제 임포트 경고를 내면 그 내용을 그대로 컨트롤러에 보고하고 STOP.

- [ ] **Step 5: 커밋 + 보고**

```powershell
git add unity\AvatarCheck\Assets\Editor\ClipImport.cs unity\AvatarCheck\Assets\Editor\ClipImport.cs.meta scripts\run.ps1
git commit -m @'
feat: mixamo clip import automation with per-clip assertions (clips stage)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```
`.superpowers/sdd/p2a-task-3-report.md`에 clips_report.json 요지 기록.

---

### Task 4: Animator 블렌드트리 + 실기동 씬 (`playscene` 스테이지)

**Files:**
- Create: `unity/AvatarCheck/Assets/Play/PlayerDrive.cs` (런타임 — Editor 폴더 밖), `unity/AvatarCheck/Assets/Editor/PlaySceneBuild.cs`
- Modify: `scripts/run.ps1` (`playscene` 스테이지)

**Interfaces:**
- Consumes: Task 3의 `Assets/Clips/*` 클립(이름: Idle/Walk/Run/Roll/Attack1..3/Hit/Death), 기존 `Assets/Import/model.fbx` (character 프로필 임포트 상태 전제)
- Produces: `Assets/Play/Player.controller`, `Assets/Play/PlayScene.unity`, `playscene_report.json`. 사용자는 에디터에서 PlayScene을 열고 Play — WASD 이동(Shift 달리기), 마우스 좌클릭 콤보, Space 구르기, H 피격, K 사망.

- [ ] **Step 1: 전제 확인 — model.fbx가 character 임포트인지**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 unity -Profile character`
Expected: `allPass:true` (이로써 Assets/Import/model.fbx = character + 아바타 명시 매핑 최신 상태).

- [ ] **Step 2: PlayerDrive.cs 작성**

`unity/AvatarCheck/Assets/Play/PlayerDrive.cs`:

```csharp
using UnityEngine;

public class PlayerDrive : MonoBehaviour
{
    Animator anim;
    float speed;
    void Start() { anim = GetComponent<Animator>(); }
    void Update()
    {
        float h = Input.GetAxis("Horizontal"), v = Input.GetAxis("Vertical");
        var dir = new Vector3(h, 0, v);
        float target = dir.magnitude < 0.1f ? 0f : (Input.GetKey(KeyCode.LeftShift) ? 1f : 0.5f);
        speed = Mathf.MoveTowards(speed, target, Time.deltaTime * 3f);
        anim.SetFloat("Speed", speed);
        if (dir.magnitude > 0.1f)
        {
            transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(dir), Time.deltaTime * 10f);
            transform.position += dir.normalized * (speed < 0.75f ? 1.6f : 4.0f) * speed * Time.deltaTime;
        }
        if (Input.GetMouseButtonDown(0)) anim.SetTrigger("Attack");
        if (Input.GetKeyDown(KeyCode.Space)) anim.SetTrigger("Roll");
        if (Input.GetKeyDown(KeyCode.H)) anim.SetTrigger("Hit");
        if (Input.GetKeyDown(KeyCode.K)) anim.SetTrigger("Die");
    }
}

public class FollowCam : MonoBehaviour
{
    public Transform target; Vector3 offset;
    void Start() { offset = transform.position - target.position; }
    void LateUpdate() { transform.position = target.position + offset; transform.LookAt(target.position + Vector3.up * 1.0f); }
}
```

- [ ] **Step 3: PlaySceneBuild.cs 작성**

`unity/AvatarCheck/Assets/Editor/PlaySceneBuild.cs`:

```csharp
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Animations;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class PlaySceneBuild
{
    static string RepoRoot() =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));

    static AnimationClip Clip(string name)
    {
        foreach (var fbx in Directory.GetFiles(Path.GetFullPath("Assets/Clips"), "*.fbx"))
        {
            string ap = "Assets/Clips/" + Path.GetFileName(fbx);
            var c = AssetDatabase.LoadAllAssetsAtPath(ap).OfType<AnimationClip>().FirstOrDefault(x => x.name == name);
            if (c != null) return c;
        }
        return null;
    }

    public static void Run()
    {
        var results = new List<(string name, bool pass, string detail)>();
        try
        {
            if (!AssetDatabase.IsValidFolder("Assets/Play"))
                AssetDatabase.CreateFolder("Assets", "Play");

            string ctrlPath = "Assets/Play/Player.controller";
            AssetDatabase.DeleteAsset(ctrlPath);
            var ctrl = AnimatorController.CreateAnimatorControllerAtPath(ctrlPath);
            ctrl.AddParameter("Speed", AnimatorControllerParameterType.Float);
            foreach (var t in new[] { "Attack", "Roll", "Hit", "Die" })
                ctrl.AddParameter(t, AnimatorControllerParameterType.Trigger);
            var sm = ctrl.layers[0].stateMachine;

            BlendTree bt;
            var loco = ctrl.CreateBlendTreeInController("Locomotion", out bt);
            bt.blendParameter = "Speed";
            bt.useAutomaticThresholds = false;
            bt.AddChild(Clip("Idle"), 0f); bt.AddChild(Clip("Walk"), 0.5f); bt.AddChild(Clip("Run"), 1f);
            sm.defaultState = loco;

            AnimatorState MakeState(string clipName)
            {
                var s = sm.AddState(clipName);
                s.motion = Clip(clipName);
                return s;
            }
            AnimatorStateTransition Back(AnimatorState from)
            {
                var t = from.AddTransition(loco);
                t.hasExitTime = true; t.exitTime = 0.9f; t.duration = 0.1f;
                return t;
            }
            var a1 = MakeState("Attack1"); var a2 = MakeState("Attack2"); var a3 = MakeState("Attack3");
            var roll = MakeState("Roll"); var hit = MakeState("Hit"); var die = MakeState("Death");

            var tA1 = loco.AddTransition(a1); tA1.AddCondition(AnimatorConditionMode.If, 0, "Attack"); tA1.duration = 0.05f;
            var tA2 = a1.AddTransition(a2); tA2.AddCondition(AnimatorConditionMode.If, 0, "Attack"); tA2.duration = 0.05f;
            var tA3 = a2.AddTransition(a3); tA3.AddCondition(AnimatorConditionMode.If, 0, "Attack"); tA3.duration = 0.05f;
            Back(a1); Back(a2); Back(a3); Back(roll); Back(hit);

            foreach (var (trig, st) in new[] { ("Roll", roll), ("Hit", hit), ("Die", die) })
            {
                var t = sm.AddAnyStateTransition(st);
                t.AddCondition(AnimatorConditionMode.If, 0, trig);
                t.duration = 0.05f; t.canTransitionToSelf = false;
            }

            foreach (var s in new[] { a1, a2, a3, roll, hit, die })
                results.Add(($"state_motion:{s.name}", s.motion != null, s.motion == null ? "missing clip" : s.motion.name));
            results.Add(("blendtree_children", bt.children.Length == 3 && bt.children.All(c => c.motion != null),
                $"children={bt.children.Length}"));

            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            plane.transform.localScale = new Vector3(3, 1, 3);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Import/model.fbx");
            results.Add(("model_prefab", prefab != null, prefab == null ? "Assets/Import/model.fbx missing — run.ps1 unity -Profile character 선행" : "ok"));
            var player = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            var anim = player.GetComponent<Animator>() ?? player.AddComponent<Animator>();
            anim.runtimeAnimatorController = ctrl;
            anim.applyRootMotion = false;   // 이동은 PlayerDrive가 담당 (In Place 클립 전제)
            player.AddComponent<PlayerDrive>();
            var cam = Camera.main;
            cam.transform.position = new Vector3(0, 2.2f, -4.5f);
            var fc = cam.gameObject.AddComponent<FollowCam>();
            fc.target = player.transform;
            EditorSceneManager.SaveScene(scene, "Assets/Play/PlayScene.unity");
            results.Add(("scene_saved", File.Exists(Path.GetFullPath("Assets/Play/PlayScene.unity")), "Assets/Play/PlayScene.unity"));
        }
        catch (Exception ex) { results.Add(("run_exception", false, ex.ToString())); }

        bool allPass = results.Count > 0 && results.All(r => r.pass);
        string json = "{\"allPass\":" + (allPass ? "true" : "false") + ",\"assertions\":[" +
            string.Join(",", results.Select(r =>
                "{\"name\":\"" + r.name + "\",\"pass\":" + (r.pass ? "true" : "false") +
                ",\"detail\":\"" + r.detail.Replace("\\", "/").Replace("\"", "'") + "\"}")) + "]}";
        File.WriteAllText(Path.Combine(RepoRoot(), "unity", "AvatarCheck", "playscene_report.json"), json);
        Debug.Log("PLAYSCENE REPORT: " + json);
        EditorApplication.Exit(allPass ? 0 : 1);
    }
}
```

- [ ] **Step 4: run.ps1에 playscene 스테이지 추가**

switch의 `"clips"` 케이스 아래에:

```powershell
    "playscene" { Invoke-Unity "PlaySceneBuild.Run" "playscene.log" "playscene_report.json" }
```
usage 문자열에도 `playscene` 추가.

- [ ] **Step 5: 실행 + 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 playscene`
Expected: exit 0, playscene_report.json `allPass:true` (state_motion 6종 + blendtree_children + model_prefab + scene_saved).

- [ ] **Step 6: 커밋 + 보고**

```powershell
git add unity\AvatarCheck\Assets\Play\PlayerDrive.cs unity\AvatarCheck\Assets\Play\PlayerDrive.cs.meta unity\AvatarCheck\Assets\Play.meta unity\AvatarCheck\Assets\Editor\PlaySceneBuild.cs unity\AvatarCheck\Assets\Editor\PlaySceneBuild.cs.meta scripts\run.ps1
git commit -m @'
feat: animator blendtree + drivable play scene for mixamo clip verification

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```
`.superpowers/sdd/p2a-task-4-report.md`에 playscene_report.json 요지 기록.

---

### Task 5: 실기동 사용자 게이트 + PLAN 갱신 + 브랜치 마감 준비

**Files:**
- Modify: `PLAN.md`

**Interfaces:**
- Consumes: Task 4의 PlayScene (사용자가 에디터에서 직접 구동)
- Produces: PLAN.md Phase 2 갱신 커밋. 머지는 사용자 결정 후 컨트롤러가 finishing-a-development-branch로 수행.

- [ ] **Step 1: 사용자 실기동 게이트 (컨트롤러 수행)**

컨트롤러가 사용자에게 안내: Unity 에디터로 `unity/AvatarCheck` 열기 → `Assets/Play/PlayScene.unity` 열기 → Play. 조작: WASD 이동, Shift 달리기, 좌클릭 콤보(연타 시 1→2→3타), Space 구르기, H 피격, K 사망. **판정 질문: "이동·전투 모션이 게임 주인공으로 쓸 만한가?"** 판정 결과를 받기 전 Step 2 착수 금지. (부속물이 뻣뻣한 것은 스프링 본 후속 작업 소관 — 판정 범위 외로 안내.)

- [ ] **Step 2: PLAN.md Phase 2 갱신**

Phase 2 섹션의 2a 항목을 다음으로 교체 (2b 개편 문구 추가):

```markdown
**2a. Mixamo 리타게팅 (이동 + 기본 전투) — 하이브리드 확정 (2026-07-22 설계)**
- [x] Unity Humanoid 직접 리타게팅: 클립 임포트 자동화(`clips` 스테이지) + 실기동 씬(`playscene` 스테이지)
- [ ] 사용자 실기동 판정: PlayScene에서 이동/콤보/구르기/피격/사망 확인 (판정 후 체크)
- [ ] 2a-2 (필요 시): 편집 필요 클립만 Blender 최소 리타게팅 — 첫 수요(돌진 베기)에서 스파이크
- 설계: docs/superpowers/specs/2026-07-22-phase2-mixamo-hybrid-design.md

**2b. 시그니처 액션 (개편: 모캡 베이스 + 수제 편집)**
- walk 파일럿 인프라(디스패처/포즈 데이터/비교 루프/팔로우스루)를 편집 루프로 재사용
- [ ] 돌진 베기 1종: Mixamo 베이스(2a-2) 위에 편집 제작 — 후속 계획
```

사용자 판정이 긍정이면 실기동 체크박스를 `[x]`+날짜로, 부정이면 판정 내용을 그대로 기록한다 — **선주장 금지**.

- [ ] **Step 3: 커밋**

```powershell
git add PLAN.md
git commit -m @'
docs: record 2a-1 mixamo retargeting status + play-scene user gate in PLAN

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```

- [ ] **Step 4: 보고**

`.superpowers/sdd/p2a-task-5-report.md`에 사용자 판정 원문과 PLAN 변경 요지 기록. 이후 컨트롤러가 최종 브랜치 리뷰 → finishing-a-development-branch(머지 여부 사용자 결정) 순서로 마감.

---

## 계획 자기검토 결과 (작성 시 반영)

- 스펙 커버리지: §2 2a-1(T3·T4), §3 사용자 다운로드(T2), §4 Unity 구성(T3·T4), §5 검증(기계 T3·T4 / 사용자 T5), §6 YAGNI 준수(2a-2 미선제작 — T5 PLAN 문구로 유보 명시), §7 완료기준 1·2·4·5(T1~T5). **완료기준 3(돌진 베기)은 후속 계획으로 명시 유보** — 헤더와 T5에 기록.
- 타입 일관성: clips.json 스키마(file/name/loop) ↔ ClipImport.ClipEntry ↔ PlaySceneBuild.Clip(name) 일치. 클립 이름 9종(Idle/Walk/Run/Roll/Attack1-3/Hit/Death)이 T2 데이터 ↔ T4 상태명에서 동일.
- 플레이스홀더 없음: 전 코드 블록 완결. Mixamo 정확 클립 변형 선택은 사용자 미리보기 재량(스펙 §8 규칙)으로 의도적 위임.
- 리스크 반영: Unity 에디터 열림 차단(전역 제약), model.fbx 전제(T4 Step 1), FBX 부재 실패 경로(T3 Step 3), 루트모션 끔(In Place 클립 전제 주석).
