# Unity 스프링 본 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 부속물 본(꼬리·치마·목도리·후드 귀, 13체인)에 자체 스프링 물리를 붙여 Humanoid가 버려두는 2차 모션을 Unity 런타임에서 재현하고, 다리-치마 관통을 콜라이더로 완화한다.

**Architecture:** 런타임 2파일(SpringBoneChain=물리 1체인, SpringCollider=구체 마커) + 에디터 1파일(SpringBoneSetup=meta 기반 자동 배선). 시뮬 틱은 `Step(float dt)` 공개 메서드로 분리 — 런타임 LateUpdate와 에디트 모드 캡처(ClipCapture)가 같은 코드를 호출한다(스펙 §2 핵심 심). 검증은 기존 3층 패턴: playscene_report 단언 + 캡처 프레임 판독(컨트롤러) + 사용자 실기동.

**Tech Stack:** Unity 6000.3.20f1 batchmode, C# (UnityEngine/UnityEditor), run.ps1 기존 스테이지 (`playscene`, `capture`).

## Global Constraints

- 기존 단언·스테이지·검사 절대 불변, 약화 금지. 새 단언(`springs_attached`)은 순수 추가.
- 부속물 본 목록 단일 원천: `exports/character/character_meta.json`의 `appendageBones` — 실측 27본: Tail1..Tail5 (체인 1), Skirt{F,FR,R,BR,B,BL,L,FL}1a/1b (체인 8), Scarf{L,R}1a/1b (체인 2), HoodEarL/HoodEarR (단일 2) = **13체인**.
- Unity batchmode는 에디터 닫힘 필수. `Get-Process Unity`로 확인, 열려 있으면 STOP·보고.
- Unity 실행은 FOREGROUND, timeout 600000ms (백그라운드 완료 통지 불안정).
- docs/·.superpowers/ 미커밋. 생성물(씬·컨트롤러·리포트·캡처 PNG) 미커밋 (기존 gitignore).
- 커밋 메시지 끝: `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`
- 서브에이전트는 중첩 서브에이전트 생성 금지.

---

### Task 1: 선행 정리 커밋 + 스프링 물리 런타임 2파일

**Files:**
- Commit first (이미 수정돼 있음, 편집 금지): `unity/AvatarCheck/Assets/Editor/ClipCapture.cs`
- Create: `unity/AvatarCheck/Assets/Play/SpringBoneChain.cs`, `unity/AvatarCheck/Assets/Play/SpringCollider.cs`

**Interfaces:**
- Produces: `SpringBoneChain` — public fields `Transform[] bones; float stiffness, damping, gravity, maxAngleDeg; SpringCollider[] colliders;` public methods `void Init()`, `void Step(float dt)`. 런타임엔 LateUpdate가 Play 모드에서만 Step 호출. `SpringCollider` — public field `float radius`.
- Task 2의 SpringBoneSetup이 AddComponent 후 필드를 채우고 Init()을 호출한다.

- [ ] **Step 1: 워킹트리의 캡처 도구 진단 확장을 먼저 커밋**

컨트롤러가 진단 중 ClipCapture.cs에 누적한 미커밋 편집(레퍼런스 소스 테이블·ImportRef 일반화·관절 각도/바인딩 로깅)을 그대로 커밋한다. **파일을 편집하지 말 것** — 있는 그대로 커밋:

```powershell
git status --short   # unity/.../ClipCapture.cs 만 M 이어야 함 (아니면 STOP·보고)
git add unity\AvatarCheck\Assets\Editor\ClipCapture.cs
git commit -m @'
feat: capture tool - generic with-skin reference import + joint-angle logging

RefSources table (YBotWalk, RefAttack), ImportRef generalization, per-sample
knee/hips/spine metrics in capture JSON. Used to prove retarget fidelity
(joint angles match reference rig within 1-2deg on identical clips).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```

- [ ] **Step 2: SpringCollider.cs 작성**

```csharp
using UnityEngine;

// 구체 충돌 마커: 반경만 갖는다. SpringBoneChain.Step이 추적점을 구 밖으로 밀어낸다.
// 허벅지·종아리 본에 부착해 치마 관통을 "완화"한다 (완전 방지는 스펙상 비목표).
public class SpringCollider : MonoBehaviour
{
    public float radius = 0.08f;
}
```

- [ ] **Step 3: SpringBoneChain.cs 작성**

```csharp
using UnityEngine;

// 부속물 체인 1개의 스프링 물리 (스펙 §2). Humanoid가 구동하지 않는 비인간 본을
// 감쇠 진자로 흔든다: 각 본의 "끝점"을 관성 추적하고, 레스트 방향과의 차이를
// 본 회전으로 환산한다. 시뮬 틱은 Step(dt)로 분리 — 런타임 LateUpdate와
// 에디트 모드 캡처(ClipCapture)가 같은 코드를 호출한다 (검증 재현성의 핵심 심).
public class SpringBoneChain : MonoBehaviour
{
    public Transform[] bones;          // 체인 순서 (루트→끝)
    public float stiffness = 0.1f;     // 레스트 방향 복원 계수 (틱당 lerp)
    public float damping = 0.2f;       // 관성 감쇠 (0=관성 유지, 1=즉시 정지)
    public float gravity = 1.0f;       // 하방 가속 (m/s^2)
    public float maxAngleDeg = 40f;    // 레스트 방향 대비 최대 흔들림 각
    public SpringCollider[] colliders; // 밀어낼 구체들 (없으면 빈 배열)

    Quaternion[] restLocal;   // 레스트 로컬 회전 (Init 시점)
    Vector3[] restDirLocal;   // 본 로컬 공간에서 끝점 방향
    float[] len;              // 본 길이
    Vector3[] tip, prevTip;   // 끝점 verlet 상태 (world)
    bool ready;

    public void Init()
    {
        int n = bones.Length;
        restLocal = new Quaternion[n];
        restDirLocal = new Vector3[n];
        len = new float[n];
        tip = new Vector3[n];
        prevTip = new Vector3[n];
        for (int i = 0; i < n; i++)
        {
            restLocal[i] = bones[i].localRotation;
            // 끝점: 다음 본이 있으면 그 위치, 마지막 본은 자기 길이만큼 연장
            Vector3 childPos = i + 1 < n
                ? bones[i + 1].position
                : bones[i].position + (bones[i].position - (i > 0 ? bones[i - 1].position : bones[i].parent.position));
            len[i] = Mathf.Max(0.01f, Vector3.Distance(bones[i].position, childPos));
            restDirLocal[i] = bones[i].InverseTransformDirection((childPos - bones[i].position).normalized);
            tip[i] = childPos;
            prevTip[i] = childPos;
        }
        ready = true;
    }

    void LateUpdate()
    {
        if (Application.isPlaying && ready) Step(Time.deltaTime);
    }

    public void Step(float dt)
    {
        if (!ready || dt <= 0f) return;
        for (int i = 0; i < bones.Length; i++)
        {
            var b = bones[i];
            b.localRotation = restLocal[i];               // 부모 애니메이션 위에 레스트로 복귀
            Vector3 restDirW = b.TransformDirection(restDirLocal[i]);
            Vector3 target = b.position + restDirW * len[i];

            Vector3 vel = (tip[i] - prevTip[i]) * (1f - damping);
            Vector3 next = tip[i] + vel
                         + Vector3.down * (gravity * dt * dt)
                         + (target - tip[i]) * Mathf.Clamp01(stiffness);

            next = b.position + (next - b.position).normalized * len[i];   // 길이 유지

            if (colliders != null)
                foreach (var c in colliders)
                {
                    if (c == null) continue;
                    Vector3 d = next - c.transform.position;
                    if (d.magnitude < c.radius)
                        next = c.transform.position + d.normalized * c.radius;
                }

            Vector3 dir = (next - b.position).normalized;
            float ang = Vector3.Angle(restDirW, dir);
            if (ang > maxAngleDeg)
            {
                dir = Vector3.Slerp(restDirW, dir, maxAngleDeg / ang).normalized;
                next = b.position + dir * len[i];
            }

            b.rotation = Quaternion.FromToRotation(restDirW, dir) * b.rotation;
            prevTip[i] = tip[i];
            tip[i] = next;
        }
    }
}
```

- [ ] **Step 4: 컴파일 스모크 (Unity 배치로 컴파일만 확인)**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 playscene`
Expected: exit 0 (playscene은 새 파일과 무관하지만 배치 기동이 컴파일을 수행 — 컴파일 에러면 여기서 실패한다). 실패 시 로그 `unity/AvatarCheck/Logs/playscene.log`에서 CS 에러 확인·수정.

- [ ] **Step 5: 커밋**

```powershell
git add unity\AvatarCheck\Assets\Play\SpringBoneChain.cs unity\AvatarCheck\Assets\Play\SpringBoneChain.cs.meta unity\AvatarCheck\Assets\Play\SpringCollider.cs unity\AvatarCheck\Assets\Play\SpringCollider.cs.meta
git commit -m @'
feat: spring bone runtime - damped pendulum chains with Step(dt) seam

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```

---

### Task 2: SpringBoneSetup 배선 + PlaySceneBuild 통합 + springs_attached 단언 + Foot IK

**Files:**
- Create: `unity/AvatarCheck/Assets/Editor/SpringBoneSetup.cs`
- Modify: `unity/AvatarCheck/Assets/Editor/PlaySceneBuild.cs`

**Interfaces:**
- Consumes: Task 1의 `SpringBoneChain`(필드/Init) · `SpringCollider`(radius)
- Produces: `public static int SpringBoneSetup.Attach(GameObject player)` — 부속물 본을 찾아 체인 부착, **부착한 체인 수를 반환** (본이 없으면 0, 예외 없이). Task 3의 ClipCapture도 이 메서드를 호출한다.

- [ ] **Step 1: SpringBoneSetup.cs 작성**

```csharp
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// meta appendageBones(단일 원천)의 이름 규칙으로 체인을 묶어 SpringBoneChain을
// 부착하는 정적 유틸. PlaySceneBuild(씬 생성)와 ClipCapture(캡처) 양쪽에서 호출.
// 그룹 튜닝값은 아래 테이블이 단일 원천 — 수렴 루프(스펙 §3)의 수정 대상.
public static class SpringBoneSetup
{
    // (체인 본 이름 나열, stiffness, damping, gravity, maxAngleDeg, useLegColliders)
    static readonly (string[] chain, float st, float dp, float gr, float ang, bool col)[] Chains =
    {
        (new[]{"Tail1","Tail2","Tail3","Tail4","Tail5"}, 0.06f, 0.15f, 0.5f, 45f, false),
        (new[]{"SkirtF1a","SkirtF1b"},   0.20f, 0.25f, 1.5f, 30f, true),
        (new[]{"SkirtFR1a","SkirtFR1b"}, 0.20f, 0.25f, 1.5f, 30f, true),
        (new[]{"SkirtR1a","SkirtR1b"},   0.20f, 0.25f, 1.5f, 30f, true),
        (new[]{"SkirtBR1a","SkirtBR1b"}, 0.20f, 0.25f, 1.5f, 30f, true),
        (new[]{"SkirtB1a","SkirtB1b"},   0.20f, 0.25f, 1.5f, 30f, true),
        (new[]{"SkirtBL1a","SkirtBL1b"}, 0.20f, 0.25f, 1.5f, 30f, true),
        (new[]{"SkirtL1a","SkirtL1b"},   0.20f, 0.25f, 1.5f, 30f, true),
        (new[]{"SkirtFL1a","SkirtFL1b"}, 0.20f, 0.25f, 1.5f, 30f, true),
        (new[]{"ScarfL1a","ScarfL1b"},   0.08f, 0.20f, 2.0f, 50f, false),
        (new[]{"ScarfR1a","ScarfR1b"},   0.08f, 0.20f, 2.0f, 50f, false),
        (new[]{"HoodEarL"},              0.12f, 0.20f, 0.3f, 25f, false),
        (new[]{"HoodEarR"},              0.12f, 0.20f, 0.3f, 25f, false),
    };
    static readonly (string bone, float r)[] LegColliders =
    {
        ("LeftUpperLeg", 0.09f), ("RightUpperLeg", 0.09f),
        ("LeftLowerLeg", 0.07f), ("RightLowerLeg", 0.07f),
    };

    public static int Attach(GameObject player)
    {
        var byName = player.GetComponentsInChildren<Transform>(true)
            .GroupBy(t => t.name).ToDictionary(g => g.Key, g => g.First());

        var legCols = new List<SpringCollider>();
        foreach (var (bone, r) in LegColliders)
            if (byName.TryGetValue(bone, out var t))
            {
                var c = t.GetComponent<SpringCollider>() ?? t.gameObject.AddComponent<SpringCollider>();
                c.radius = r;
                legCols.Add(c);
            }

        int attached = 0;
        foreach (var (chain, st, dp, gr, ang, col) in Chains)
        {
            var bones = chain.Select(n => byName.TryGetValue(n, out var t) ? t : null).ToArray();
            if (bones.Any(b => b == null)) continue;   // 부속물 없는 모델(Y Bot 등)은 조용히 스킵
            var sb = bones[0].gameObject.GetComponent<SpringBoneChain>()
                     ?? bones[0].gameObject.AddComponent<SpringBoneChain>();
            sb.bones = bones;
            sb.stiffness = st; sb.damping = dp; sb.gravity = gr; sb.maxAngleDeg = ang;
            sb.colliders = col ? legCols.ToArray() : new SpringCollider[0];
            sb.Init();
            attached++;
        }
        return attached;
    }
}
```

- [ ] **Step 2: PlaySceneBuild 통합 + 단언 + Foot IK**

`PlaySceneBuild.cs`의 `player.AddComponent<PlayerDrive>();` 직후에 추가:

```csharp
            int springs = SpringBoneSetup.Attach(player);
            results.Add(("springs_attached", springs == 13, $"chains={springs} expected 13"));
```

`Back(...)` 정의부와 상태 생성부 사이(상태들이 전부 만들어진 뒤)에 Foot IK 추가 — `sm.defaultState = loco;` 아래에:

```csharp
            // Foot IK 시도 (스펙 §2): 상태 iKOnFeet + 레이어 IK pass. 캡처에서 발 관통
            // 개선이 확인되지 않으면 이 블록을 제거하고 보고서에 기록한다 (침묵 금지).
            var layers = ctrl.layers;
            layers[0].iKPass = true;
            ctrl.layers = layers;
```

그리고 상태 6종(a1,a2,a3,roll,hit,die)과 loco를 만들었던 곳 이후에:

```csharp
            foreach (var st in new[] { loco, a1, a2, a3, roll, hit })
                st.iKOnFeet = true;
```

(위치 주의: `a1`~`die` 변수 선언 이후여야 컴파일된다.)

- [ ] **Step 3: 실행 + 확인**

Run: `powershell -ExecutionPolicy Bypass -File scripts\run.ps1 playscene`
Expected: exit 0, playscene_report.json `allPass:true`에 `"springs_attached","pass":true,"detail":"chains=13 expected 13"` 포함. 기존 단언 전부 유지.

- [ ] **Step 4: 커밋**

```powershell
git add unity\AvatarCheck\Assets\Editor\SpringBoneSetup.cs unity\AvatarCheck\Assets\Editor\SpringBoneSetup.cs.meta unity\AvatarCheck\Assets\Editor\PlaySceneBuild.cs
git commit -m @'
feat: spring bone auto-wiring in play scene + springs_attached assertion + foot IK

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```

---

### Task 3: ClipCapture 물리 통합 + 검증 캡처 산출

**Files:**
- Modify: `unity/AvatarCheck/Assets/Editor/ClipCapture.cs`

**Interfaces:**
- Consumes: `SpringBoneSetup.Attach(GameObject) -> int`, `SpringBoneChain.Step(float)`
- Produces: 물리 포함 캡처 프레임(`Walk_*`, `RefAttack_*`) + 캡처 JSON에 `springs`(부착 수)와 샘플별 `tailZ`/`tailY`(Tail5 world 위치 — 물리 작동 증명; 정지 상태면 전 샘플 동일값이 나온다)

- [ ] **Step 1: 캡처에 스프링 부착 + Step 호출 추가**

`anim.Rebind(); anim.Update(0f);` (셋업 구간) 직후에:

```csharp
            var springChains = new List<SpringBoneChain>();
            int springCount = SpringBoneSetup.Attach(player);
            if (springCount > 0)
                springChains.AddRange(player.GetComponentsInChildren<SpringBoneChain>(true));
```

(파일 상단 `using System.Collections.Generic;` 이 없으면 추가.)

스텝 루프 `while (cur + DT * 0.5f < t) { anim.Update(DT); cur += DT; }` 를 다음으로 교체:

```csharp
                while (cur + DT * 0.5f < t)
                {
                    anim.Update(DT);
                    foreach (var sc in springChains) sc.Step(DT);   // 런타임 LateUpdate와 동일 틱
                    cur += DT;
                }
```

- [ ] **Step 2: JSON에 스프링 증거 추가**

poseLog Append 체인의 `"spineZlean"` 항목 뒤에 추가:

```csharp
                        .Append(",\"tailZ\":").Append(TailTip(player, "z"))
                        .Append(",\"tailY\":").Append(TailTip(player, "y"))
```

클래스에 헬퍼 추가:

```csharp
    static string TailTip(GameObject player, string axis)
    {
        var t = player.GetComponentsInChildren<Transform>(true).FirstOrDefault(x => x.name == "Tail5");
        if (t == null) return "-1";
        return (axis == "z" ? t.position.z : t.position.y).ToString("F3");
    }
```

최종 json 문자열의 `,\"frames\":16` 뒤에 `,\"springs\":" + springCount` 추가.

- [ ] **Step 3: 캡처 2종 실행**

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run.ps1 capture
powershell -ExecutionPolicy Bypass -Command "$env:ANIME_CAPTURE_CLIP='RefAttack'; & scripts\run.ps1 capture"
```
Expected: 각 exit 0. `previews/character/unity_capture/Walk_capture.json`의 `springs:13`, 샘플별 `tailZ`가 **서로 다른 값**(물리 작동), `RefAttack_capture.json` 동일. PNG 32장 생성.

- [ ] **Step 4: 커밋 + 보고**

```powershell
git add unity\AvatarCheck\Assets\Editor\ClipCapture.cs
git commit -m @'
feat: capture renders spring physics via shared Step(dt) tick

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
'@
```
`.superpowers/sdd/p2a-spring-task-3-report.md`에 캡처 JSON 요지(springs, tailZ 변화폭) 기록. **프레임 판독은 컨트롤러 몫** — 구현자는 판독하지 않는다.

---

### Task 4 (컨트롤러 수행): 튜닝 수렴 루프 + PLAN 갱신 + 사용자 게이트

**Files:**
- Modify (수렴 루프 중): `unity/AvatarCheck/Assets/Editor/SpringBoneSetup.cs`의 Chains 테이블 수치만
- Modify (마감): `PLAN.md`

- [ ] **Step 1: 컨트롤러 프레임 판독** — Walk/RefAttack 캡처에서 스펙 완료 기준 2 판정: 치마가 다리 스윙을 따라 벌어짐 / 관통 현저 감소 / 꼬리 위상 지연 출렁임. 발 IK 효과(발 관통 개선)도 여기서 판정 — 개선 없으면 PlaySceneBuild의 Foot IK 블록 제거 커밋 + 기록.
- [ ] **Step 2: 미달 항목은 Chains 수치만 수정 → `run.ps1 playscene` + 캡처 재실행 → 재판독.** 상한 3회, 초과 시 사용자 상의 (스펙 리스크 표).
- [ ] **Step 3: 수렴 시 수치 커밋** (`fix: spring tuning converged (rounds=N)` + Co-Authored-By 라인), 원장 기입.
- [ ] **Step 4: PLAN.md Phase 2 갱신** — 2a-1에 스프링 본 완료 항목 추가, 실기동 재판정 게이트 `[ ]` 유지. 커밋.
- [ ] **Step 5: 사용자 실기동 재판정 요청** — Unity에서 PlayScene Play. 판정 결과를 원장·PLAN에 기록. 긍정 시 최종 브랜치 리뷰 → finishing-a-development-branch.

---

## 계획 자기검토 결과 (작성 시 반영)

- 스펙 커버리지: §2 컴포넌트 3파일(T1·T2), Step 심(T1 코드 + T3 캡처 호출), Foot IK(T2 + T4 판정), §3 검증 3층(T2 단언 / T3 캡처+T4 컨트롤러 판독 / T4 사용자 게이트), §5 완료 기준 1~5(T2·T4), 튜닝 테이블 초깃값 확정(스펙의 "구현 계획에서 확정" 이행). 갭 없음.
- 타입 일관성: `SpringBoneSetup.Attach(GameObject)->int`를 T2 정의·T2 PlaySceneBuild·T3 ClipCapture가 동일 시그니처로 소비. `Step(float)` 동일. 본 이름 27종은 meta 실측값과 대조 완료.
- 플레이스홀더 없음: 전 코드 블록 완결. 튜닝 수치는 초깃값 명시 + 수렴 절차(T4) 정의.
- 리스크 반영: 에디터 열림(전역 제약), Y Bot/RefAttack 모델에서 Attach가 0 반환(조용한 스킵 — 캡처 JSON springs로 가시화), 폭주 방지(maxAngleDeg + RefAttack 캡처 포함).
