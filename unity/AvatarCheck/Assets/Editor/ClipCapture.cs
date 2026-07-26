using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Renders a retargeted humanoid clip playing on our character to PNG frames so the
// controller can VISUALLY diagnose retargeting problems (e.g. the reported abnormal
// knee/calf/foot motion during Walk) without opening the editor. This is a permanent
// verification tool — invoked headed (NOT -nographics) because it renders real frames.
//
// It intentionally does NOT wire an AnimatorController: the clip is sampled directly on
// the imported prefab via UnityEditor.AnimationMode, which is deterministic and needs no
// play mode. Two camera framings per sample (a full side view and a lower-leg close-up)
// are written so knees/feet are readable.
public static class ClipCapture
{
    const int W = 1280;
    const int H = 720;
    const int SAMPLES = 8;   // t = i/8 * length, i=0..7  -> 8 samples x 2 views = 16 PNGs

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

    public static void Run()
    {
        string clipName = Environment.GetEnvironmentVariable("ANIME_CAPTURE_CLIP");
        if (string.IsNullOrEmpty(clipName)) clipName = "Walk";

        RenderTexture rt = null;
        GameObject camObj = null;
        bool started = false;
        try
        {
            var clip = Clip(clipName);
            if (clip == null)
                throw new Exception($"clip not found: '{clipName}' across Assets/Clips/*.fbx (run.ps1 clips 선행)");

            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Import/model.fbx");
            if (prefab == null)
                throw new Exception("Assets/Import/model.fbx missing — run.ps1 unity -Profile character 선행");

            // Output dir (namespaced under character/unity_capture). Clear stale PNGs + JSON
            // for THIS clip only so a rerun cannot leave a mix of old/new frames.
            string outDir = Path.Combine(RepoRoot(), "previews", "character", "unity_capture");
            Directory.CreateDirectory(outDir);
            foreach (var f in Directory.GetFiles(outDir, clipName + "_side_*.png")) File.Delete(f);
            foreach (var f in Directory.GetFiles(outDir, clipName + "_leg_*.png")) File.Delete(f);
            string jsonPath = Path.Combine(outDir, clipName + "_capture.json");
            if (File.Exists(jsonPath)) File.Delete(jsonPath);

            // Fresh scene with default camera + directional light, then a ground plane and
            // the character prefab at origin (Animator comes from the humanoid import).
            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            plane.transform.localScale = new Vector3(3, 1, 3);
            var player = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            player.transform.position = Vector3.zero;
            if (player.GetComponent<Animator>() == null) player.AddComponent<Animator>();

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

            AnimationMode.StartAnimationMode();
            started = true;

            var tex = new Texture2D(W, H, TextureFormat.RGB24, false);
            var pp = player.transform.position;

            for (int i = 0; i < SAMPLES; i++)
            {
                float t = (i / (float)SAMPLES) * clip.length;

                AnimationMode.BeginSampling();
                AnimationMode.SampleAnimationClip(player, clip, t);
                AnimationMode.EndSampling();

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
                          ",\"frames\":16}";
            File.WriteAllText(jsonPath, json);

            Debug.Log($"ClipCapture: wrote 16 PNGs for '{clipName}' (len={clip.length:F3}s) to {outDir}");
        }
        catch (Exception ex)
        {
            Debug.LogError("ClipCapture FAILED: " + ex);
            Cleanup(ref rt, ref camObj, started);
            EditorApplication.Exit(1);
            return;
        }

        Cleanup(ref rt, ref camObj, started);
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

    static void Cleanup(ref RenderTexture rt, ref GameObject camObj, bool started)
    {
        if (started) AnimationMode.StopAnimationMode();
        if (camObj != null) UnityEngine.Object.DestroyImmediate(camObj);
        if (rt != null)
        {
            RenderTexture.active = null;
            rt.Release();
            UnityEngine.Object.DestroyImmediate(rt);
        }
    }
}
