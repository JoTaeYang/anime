using System;
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
    public static void ImportYBot()
    {
        string src = Path.Combine(RepoRoot(), "assets", "mocap", YBotSourceRel);
        if (!File.Exists(src))
            throw new Exception("Y Bot source missing: " + src + " (assets/mocap/YBotWalking.fbx 선행)");
        if (!AssetDatabase.IsValidFolder("Assets/Clips"))
            AssetDatabase.CreateFolder("Assets", "Clips");
        if (AssetDatabase.LoadMainAssetAtPath(YBotAssetPath) != null)
            AssetDatabase.DeleteAsset(YBotAssetPath);   // 신규 반입 프로토콜: 항상 새 임포트
        File.Copy(src, Path.GetFullPath(YBotAssetPath), overwrite: true);
        AssetDatabase.ImportAsset(YBotAssetPath, ImportAssetOptions.ForceSynchronousImport);

        var importer = (ModelImporter)AssetImporter.GetAtPath(YBotAssetPath);
        importer.animationType = ModelImporterAnimationType.Human;
        importer.avatarSetup = ModelImporterAvatarSetup.CreateFromThisModel;
        var takes = importer.defaultClipAnimations;
        foreach (var t in takes) { t.name = "YBotWalk"; t.loopTime = true; }
        importer.clipAnimations = takes;
        importer.SaveAndReimport();
        Debug.Log("ClipCapture.ImportYBot: imported " + YBotAssetPath + " (take 'YBotWalk', loop)");
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
            // Y Bot reference guard: if the model points at the Y Bot asset and it isn't
            // imported yet, bring it in from assets/mocap first (idempotent).
            if (modelPath == YBotAssetPath && AssetDatabase.LoadMainAssetAtPath(YBotAssetPath) == null)
                ImportYBot();

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
            anim.Rebind();
            anim.Update(0f);

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

            for (int i = 0; i < SAMPLES; i++)
            {
                float t = (i / (float)SAMPLES) * clip.length;

                // RUNTIME sampling: rewind to clip start (Rebind) then step forward to t in
                // fixed DT increments — the honest Animator solve, not AnimationMode.
                anim.Rebind();
                anim.Update(0f);
                int stepCount = Mathf.RoundToInt(t / DT);
                for (int s = 0; s < stepCount; s++) anim.Update(DT);

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
                          ",\"model\":\"" + modelPath.Replace("\\", "/") + "\",\"frames\":16}";
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
