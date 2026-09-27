using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Animations;
using UnityEditor.SceneManagement;
using UnityEngine;

// Renders a retargeted humanoid clip playing on a character to PNG frames so the controller
// can VISUALLY diagnose retargeting problems (e.g. the reported abnormal knee/calf/foot motion
// during Walk) without opening the editor. This is a permanent verification tool — invoked
// headed (NOT -nographics) because it renders real frames.
//
// SAMPLING PATH (Phase2a fix): frames are driven through the RUNTIME Animator path — a temp
// single-state AnimatorController plays the clip on the model's own humanoid avatar and is
// stepped with animator.Update(), exactly the honest path RetargetProbe's TARGET view uses.
// The previous UnityEditor.AnimationMode.SampleAnimationClip path was proven a capture artifact:
// it rendered ~10-20% leg amplitude while the probe measured a full healthy stride (Walk knee
// range ~37deg, foot Z travel ~0.85m) on this same clip via the runtime path. Two camera
// framings per sample (a full side view and a lower-leg close-up) are written so knees/feet
// are readable.
//
// ANIME_CAPTURE_CLIP  selects the clip by name  (default "Walk").
// ANIME_CAPTURE_MODEL selects the model asset   (default "Assets/Import/model.fbx"); its own
//                     Animator + humanoid avatar is used as-is. Pointing it at
//                     "Assets/Clips/YBotWalking.fbx" auto-imports the Y Bot reference rig
//                     (with skin) if absent, for reference-rig comparison.
public static class ClipCapture
{
    const int W = 1280;
    const int H = 720;
    const int SAMPLES = 8;    // t = i/8 * length, i=0..7  -> 8 samples x 2 views = 16 PNGs
    const float DT = 1f / 60f; // runtime stepping increment

    const string DefaultModelPath = "Assets/Import/model.fbx";
    const string YBotAssetPath = "Assets/Clips/YBotWalking.fbx";
    const string YBotSourceRel = "YBotWalking.fbx"; // under assets/mocap/

    // With-Skin 레퍼런스 소스: 클립명 -> (Assets 경로, assets/mocap 파일명).
    // 같은 모션을 레퍼런스 리그(네이티브)와 우리 캐릭터(리타게팅)에 나란히 재생해
    // 리타게팅 차이를 눈으로 가르는 비교 재료. 필요 시 여기에 추가.
    static readonly System.Collections.Generic.Dictionary<string, (string asset, string src)> RefSources =
        new System.Collections.Generic.Dictionary<string, (string, string)>
        {
            { "YBotWalk", (YBotAssetPath, YBotSourceRel) },
            { "RefAttack", ("Assets/Clips/RefAttack.fbx", "RefAttack.fbx") },
        };

    static string RepoRoot() =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));

    // Same lookup pattern as PlaySceneBuild.Clip(): scan every imported clip FBX and match
    // the sub-asset AnimationClip by name.
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

    // Fresh-import the Y Bot reference (Mixamo "Walking" WITH SKIN) from assets/mocap into
    // Assets/Clips, same protocol as ClipImport: DeleteAsset first, Human type, avatar from
    // this model, take renamed to "YBotWalk", loopTime=true. Idempotent.
    public static void ImportYBot() => ImportRef("YBotWalk");

    // With-Skin 레퍼런스 FBX를 assets/mocap에서 Assets/Clips로 신규 반입 (항상 새 임포트,
    // Human 타입, 자체 아바타, 테이크명 = 클립명, 루프). Idempotent.
    static void ImportRef(string clipName)
    {
        var (assetPath, srcRel) = RefSources[clipName];
        string src = Path.Combine(RepoRoot(), "assets", "mocap", srcRel);
        if (!File.Exists(src))
            throw new Exception("reference source missing: " + src + " (assets/mocap/" + srcRel + " 선행)");
        if (!AssetDatabase.IsValidFolder("Assets/Clips"))
            AssetDatabase.CreateFolder("Assets", "Clips");
        if (AssetDatabase.LoadMainAssetAtPath(assetPath) != null)
            AssetDatabase.DeleteAsset(assetPath);   // 신규 반입 프로토콜: 항상 새 임포트
        File.Copy(src, Path.GetFullPath(assetPath), overwrite: true);
        AssetDatabase.ImportAsset(assetPath, ImportAssetOptions.ForceSynchronousImport);

        var importer = (ModelImporter)AssetImporter.GetAtPath(assetPath);
        importer.animationType = ModelImporterAnimationType.Human;
        importer.avatarSetup = ModelImporterAvatarSetup.CreateFromThisModel;
        var takes = importer.defaultClipAnimations;
        foreach (var t in takes) { t.name = clipName; t.loopTime = true; }
        importer.clipAnimations = takes;
        importer.SaveAndReimport();
        Debug.Log("ClipCapture.ImportRef: imported " + assetPath + " (take '" + clipName + "', loop)");
    }

    public static void Run()
    {
        string clipName = Environment.GetEnvironmentVariable("ANIME_CAPTURE_CLIP");
        if (string.IsNullOrEmpty(clipName)) clipName = "Walk";

        string modelPath = Environment.GetEnvironmentVariable("ANIME_CAPTURE_MODEL");
        if (string.IsNullOrEmpty(modelPath)) modelPath = DefaultModelPath;

        RenderTexture rt = null;
        GameObject camObj = null;
        string ctrlPath = null;
        try
        {
            // 레퍼런스 가드: 모델이 레퍼런스 자산을 가리키거나 클립이 레퍼런스 소스에
            // 정의되어 있는데 아직 미반입이면 assets/mocap에서 가져온다 (idempotent).
            foreach (var kv in RefSources)
                if ((modelPath == kv.Value.asset || clipName == kv.Key) &&
                    AssetDatabase.LoadMainAssetAtPath(kv.Value.asset) == null)
                    ImportRef(kv.Key);

            var clip = Clip(clipName);
            if (clip == null)
                throw new Exception($"clip not found: '{clipName}' across Assets/Clips/*.fbx (run.ps1 clips 선행)");

            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(modelPath);
            if (prefab == null)
                throw new Exception($"model prefab missing: '{modelPath}' — run.ps1 unity -Profile character 선행 (또는 ANIME_CAPTURE_MODEL 확인)");

            // Output dir (namespaced under character/unity_capture). Clear stale PNGs + JSON
            // for THIS clip only so a rerun cannot leave a mix of old/new frames.
            string outDir = Path.Combine(RepoRoot(), "previews", "character", "unity_capture");
            Directory.CreateDirectory(outDir);
            foreach (var f in Directory.GetFiles(outDir, clipName + "_side_*.png")) File.Delete(f);
            foreach (var f in Directory.GetFiles(outDir, clipName + "_leg_*.png")) File.Delete(f);
            string jsonPath = Path.Combine(outDir, clipName + "_capture.json");
            if (File.Exists(jsonPath)) File.Delete(jsonPath);

            // Temp single-state controller playing the clip (same pattern as RetargetProbe.Run).
            if (!AssetDatabase.IsValidFolder("Assets/Play"))
                AssetDatabase.CreateFolder("Assets", "Play");
            ctrlPath = "Assets/Play/__capture.controller";
            AssetDatabase.DeleteAsset(ctrlPath);
            var ctrl = AnimatorController.CreateAnimatorControllerAtPathWithClip(ctrlPath, clip);

            // Fresh scene with default camera + directional light, then a ground plane and
            // the character prefab at origin (Animator + avatar come from the humanoid import).
            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            plane.transform.localScale = new Vector3(3, 1, 3);
            var player = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            player.transform.position = Vector3.zero;

            var anim = player.GetComponent<Animator>() ?? player.AddComponent<Animator>();
            anim.runtimeAnimatorController = ctrl;
            anim.applyRootMotion = false;                        // walk-in-place: feet step fore/aft
            anim.cullingMode = AnimatorCullingMode.AlwaysAnimate; // never cull in edit stepping
            // 에디트 모드 batch에서 cam.Render()는 스킨 행렬을 갱신하지 않아 바인드 포즈에
            // 가까운 살을 그린다 (BakeMesh로 확인: 본·정점은 정상 보폭인데 렌더만 정지 —
            // 이 도구의 3번째이자 최종 캡처 아티팩트). 렌더마다 스킨 행렬 재계산을 강제한다.
            foreach (var s in player.GetComponentsInChildren<SkinnedMeshRenderer>(true))
            {
                s.updateWhenOffscreen = true;
                s.forceMatrixRecalculationPerRender = true;
            }
            anim.Rebind();
            anim.Update(0f);

            // 스프링 부착 — 런타임(PlaySceneBuild)과 동일한 SpringBoneSetup.Attach로 부속물
            // 체인을 붙인다. 우리 캐릭터는 13개, Y Bot/RefAttack 리그는 부속물이 없어 0 (조용히
            // 스킵). springCount는 JSON "springs"로 노출돼 이 비대칭을 가시화한다. Attach 내부
            // Init()이 여기서 Rebind/Update(0) 직후에 호출돼 rest 포즈를 기준으로 잡는다.
            var springChains = new List<SpringBoneChain>();
            int springCount = SpringBoneSetup.Attach(player);
            if (springCount > 0)
                springChains.AddRange(player.GetComponentsInChildren<SpringBoneChain>(true));

            // Guarantee a light so renders are not black even if the default light is missing.
            if (UnityEngine.Object.FindObjectsOfType<Light>().All(l => l.type != LightType.Directional))
            {
                var lgo = new GameObject("CaptureLight");
                var lg = lgo.AddComponent<Light>();
                lg.type = LightType.Directional;
                lg.intensity = 1.1f;
                lgo.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            }
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.45f, 0.45f, 0.5f);

            // Dedicated capture camera with an explicit RenderTexture target (never rely on
            // Camera.main having a valid target in batchmode). Solid non-black background so a
            // failed render (all-black) is obvious and fails the >30KB size gate.
            rt = new RenderTexture(W, H, 24, RenderTextureFormat.ARGB32);
            camObj = new GameObject("CaptureCam");
            var cam = camObj.AddComponent<Camera>();
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.30f, 0.34f, 0.40f);
            cam.fieldOfView = 45f;
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = 100f;
            cam.targetTexture = rt;

            var tex = new Texture2D(W, H, TextureFormat.RGB24, false);
            var pp = player.transform.position;

            // RUNTIME sampling — ONE continuous playback, stepping forward to each target time
            // (RetargetProbe's exact pattern). Per-sample Rebind() was a second capture artifact:
            // in edit-mode batch, Update(DT) after a Rebind does not advance the state machine,
            // so every sample rendered the same phase (YBot frames 0 and 4 were identical
            // instead of mirrored). Sample times are ascending, so continuous stepping visits
            // each phase exactly once.
            float cur = 0f;
            var poseLog = new System.Text.StringBuilder();
            string bindLog = null;
            for (int i = 0; i < SAMPLES; i++)
            {
                float t = (i / (float)SAMPLES) * clip.length;
                while (cur + DT * 0.5f < t)
                {
                    anim.Update(DT);
                    foreach (var sc in springChains) sc.Step(DT);   // 런타임 LateUpdate와 동일 틱
                    cur += DT;
                }

                // 샘플별 발/무릎 수치 — 프레임(눈)과 프로브(숫자)를 잇는 교차 검증.
                var lf = anim.GetBoneTransform(HumanBodyBones.LeftFoot);
                var rf = anim.GetBoneTransform(HumanBodyBones.RightFoot);
                if (lf != null && rf != null)
                {
                    float Knee(HumanBodyBones up, HumanBodyBones low, HumanBodyBones foot)
                    {
                        var u = anim.GetBoneTransform(up); var l = anim.GetBoneTransform(low); var f2 = anim.GetBoneTransform(foot);
                        if (u == null || l == null || f2 == null) return -1f;
                        return Vector3.Angle(u.position - l.position, f2.position - l.position);
                    }
                    var hips = anim.GetBoneTransform(HumanBodyBones.Hips);
                    var spine = anim.GetBoneTransform(HumanBodyBones.Spine);
                    var head = anim.GetBoneTransform(HumanBodyBones.Head);
                    if (poseLog.Length > 0) poseLog.Append(",");
                    poseLog.Append("{\"i\":").Append(i)
                        .Append(",\"lz\":").Append(lf.position.z.ToString("F3"))
                        .Append(",\"rz\":").Append(rf.position.z.ToString("F3"))
                        .Append(",\"sep\":").Append(Mathf.Abs(lf.position.z - rf.position.z).ToString("F3"))
                        .Append(",\"kneeL\":").Append((180f - Knee(HumanBodyBones.LeftUpperLeg, HumanBodyBones.LeftLowerLeg, HumanBodyBones.LeftFoot)).ToString("F1"))
                        .Append(",\"kneeR\":").Append((180f - Knee(HumanBodyBones.RightUpperLeg, HumanBodyBones.RightLowerLeg, HumanBodyBones.RightFoot)).ToString("F1"))
                        .Append(",\"hipsY\":").Append(hips != null ? hips.position.y.ToString("F3") : "-1")
                        .Append(",\"headZ\":").Append(head != null ? head.position.z.ToString("F3") : "-1")
                        .Append(",\"spineZlean\":").Append(spine != null && head != null ? (head.position.z - spine.position.z).ToString("F3") : "-1")
                        .Append(",\"tailZ\":").Append(TailTip(player, "z"))
                        .Append(",\"tailY\":").Append(TailTip(player, "y"))
                        // 자세 바이어스 계측 (배 내밈 진단): pelvisLean = 힙→척추 벡터의
                        // 전후 성분(+ = 상체 기저가 앞으로), hipsZoff = 힙이 두 발 중점보다
                        // 앞(+)/뒤(-)로 얼마나 나가 있는가.
                        .Append(",\"pelvisLean\":").Append(spine != null && hips != null
                            ? ((spine.position - hips.position).normalized.z).ToString("F3") : "-1")
                        .Append(",\"hipsZoff\":").Append(hips != null
                            ? (hips.position.z - (lf.position.z + rf.position.z) * 0.5f).ToString("F3") : "-1")
                        // 발 피치(도): 발→발끝 벡터의 지면 대비 기울기. +면 발끝 들림.
                        .Append(",\"footPitchL\":").Append(FootPitch(anim, HumanBodyBones.LeftFoot, HumanBodyBones.LeftToes))
                        .Append(",\"footPitchR\":").Append(FootPitch(anim, HumanBodyBones.RightFoot, HumanBodyBones.RightToes))
                        .Append("}");
                }

                // 바인딩 진단 (i==2, 최대 보폭 시점 1회): 스킨 렌더러가 참조하는 다리 본이
                // 애니메이터가 움직이는 그 Transform 인스턴스인지 + 이름 중복 여부 +
                // 스킨 본 배열 내 다리 관련 본들의 world z. 뼈는 걷는데 메시가 안 따라가는
                // 원인(중복 계층/트위스트 본 정지)을 가르는 데이터.
                if (i == 2)
                {
                    var smr = player.GetComponentsInChildren<SkinnedMeshRenderer>(true)
                        .OrderByDescending(s => s.bones.Length).FirstOrDefault();
                    if (smr != null)
                    {
                        var all = player.GetComponentsInChildren<Transform>(true);
                        var dup = all.GroupBy(x => x.name).Where(g => g.Count() > 1)
                                     .Select(g => g.Key + "x" + g.Count()).ToArray();
                        bool lfInSkin = smr.bones.Contains(lf);
                        var legBones = smr.bones.Where(b => b != null && b.name.Contains("Leg"))
                            .Select(b => "{\"n\":\"" + b.name + "\",\"z\":" + b.position.z.ToString("F3") +
                                         ",\"isAnim\":" + (b == anim.GetBoneTransform(HumanBodyBones.LeftUpperLeg) ||
                                                           b == anim.GetBoneTransform(HumanBodyBones.RightUpperLeg) ||
                                                           b == anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg) ||
                                                           b == anim.GetBoneTransform(HumanBodyBones.RightLowerLeg)
                                                           ? "true" : "false") + "}");
                        // 렌더러 전수 조사 + 스킨 결과를 직접 구워 정점 z-폭 측정: 눈에 보이는
                        // 메시가 이 SMR인지, 스키닝이 실제로 보폭만큼 정점을 벌리는지 확정.
                        var rends = player.GetComponentsInChildren<Renderer>(true)
                            .Select(r => "{\"n\":\"" + r.name + "\",\"type\":\"" + r.GetType().Name +
                                         "\",\"enabled\":" + (r.enabled ? "true" : "false") +
                                         ",\"activeGO\":" + (r.gameObject.activeInHierarchy ? "true" : "false") + "}");
                        var baked = new Mesh();
                        smr.BakeMesh(baked);
                        var verts = baked.vertices;
                        float minZ = float.MaxValue, maxZ = float.MinValue, minY = float.MaxValue;
                        foreach (var v in verts)
                        {
                            var w = smr.transform.TransformPoint(v);
                            if (w.y < 0.35f) { minZ = Mathf.Min(minZ, w.z); maxZ = Mathf.Max(maxZ, w.z); }
                            minY = Mathf.Min(minY, w.y);
                        }
                        UnityEngine.Object.DestroyImmediate(baked);
                        bindLog = "\"skin\":{\"renderer\":\"" + smr.name + "\",\"boneCount\":" + smr.bones.Length +
                                  ",\"leftFootInSkinBones\":" + (lfInSkin ? "true" : "false") +
                                  ",\"dupNames\":[" + string.Join(",", dup.Select(d => "\"" + d + "\"")) + "]" +
                                  ",\"legBonesAtMaxStride\":[" + string.Join(",", legBones) + "]" +
                                  ",\"renderers\":[" + string.Join(",", rends) + "]" +
                                  ",\"bakedLowMeshZSpan\":{\"minZ\":" + minZ.ToString("F3") + ",\"maxZ\":" + maxZ.ToString("F3") +
                                  ",\"minY\":" + minY.ToString("F3") + "}}";
                    }
                }

                // Side view: whole body from the character's left, eye-level.
                ShootView(cam, rt, tex,
                    pp + new Vector3(-3.2f, 1.0f, 0f), new Vector3(0f, 1.0f, 0f),
                    Path.Combine(outDir, $"{clipName}_side_{i}.png"));

                // Lower-leg close-up: knees/calves/feet readable.
                ShootView(cam, rt, tex,
                    new Vector3(-1.8f, 0.45f, 0f), new Vector3(0f, 0.35f, 0f),
                    Path.Combine(outDir, $"{clipName}_leg_{i}.png"));
            }

            UnityEngine.Object.DestroyImmediate(tex);

            // JSON summary (16 frames = 8 samples x 2 views).
            string json = "{\"clip\":\"" + clipName + "\",\"length\":" + clip.length.ToString("F4") +
                          ",\"model\":\"" + modelPath.Replace("\\", "/") + "\",\"frames\":16" +
                          ",\"springs\":" + springCount +
                          ",\"samples\":[" + poseLog + "]" +
                          (bindLog != null ? "," + bindLog : "") + "}";
            File.WriteAllText(jsonPath, json);

            Debug.Log($"ClipCapture: wrote 16 PNGs for '{clipName}' on '{modelPath}' (len={clip.length:F3}s, runtime path) to {outDir}");
        }
        catch (Exception ex)
        {
            Debug.LogError("ClipCapture FAILED: " + ex);
            Cleanup(ref rt, ref camObj, ctrlPath);
            EditorApplication.Exit(1);
            return;
        }

        Cleanup(ref rt, ref camObj, ctrlPath);
        EditorApplication.Exit(0);
    }

    // Position the camera, render into the target RT, read back, encode PNG, and verify the
    // file is non-empty. Throws on a zero-byte write so the caller can Exit(1).
    static void ShootView(Camera cam, RenderTexture rt, Texture2D tex, Vector3 pos, Vector3 lookAt, string path)
    {
        cam.transform.position = pos;
        cam.transform.LookAt(lookAt);

        var prev = RenderTexture.active;
        RenderTexture.active = rt;
        cam.Render();
        tex.ReadPixels(new Rect(0, 0, W, H), 0, 0);
        tex.Apply();
        RenderTexture.active = prev;

        byte[] png = tex.EncodeToPNG();
        File.WriteAllBytes(path, png);
        if (png == null || png.Length == 0 || new FileInfo(path).Length == 0)
            throw new Exception("zero-byte PNG: " + path);
    }

    // Tail5(스프링 체인 끝점)의 world 좌표 한 축 — 물리 작동 증명. 샘플마다 값이 달라지면
    // 꼬리가 흔들린다는 뜻; 전 샘플 동일값이면 Step이 안 돌거나 부속물이 없는 리그다.
    static string FootPitch(Animator anim, HumanBodyBones foot, HumanBodyBones toes)
    {
        var f = anim.GetBoneTransform(foot);
        var t = anim.GetBoneTransform(toes);
        if (f == null || t == null) return "-999";
        Vector3 v = t.position - f.position;
        float horiz = new Vector2(v.x, v.z).magnitude;
        return (Mathf.Atan2(v.y, Mathf.Max(0.0001f, horiz)) * Mathf.Rad2Deg).ToString("F1");
    }

    static string TailTip(GameObject player, string axis)
    {
        var t = player.GetComponentsInChildren<Transform>(true).FirstOrDefault(x => x.name == "Tail5");
        if (t == null) return "-1";
        return (axis == "z" ? t.position.z : t.position.y).ToString("F3");
    }

    static void Cleanup(ref RenderTexture rt, ref GameObject camObj, string ctrlPath)
    {
        if (camObj != null) UnityEngine.Object.DestroyImmediate(camObj);
        if (rt != null)
        {
            RenderTexture.active = null;
            rt.Release();
            UnityEngine.Object.DestroyImmediate(rt);
        }
        if (!string.IsNullOrEmpty(ctrlPath)) AssetDatabase.DeleteAsset(ctrlPath);
    }
}
