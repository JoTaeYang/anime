using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.Animations;
using UnityEngine;

// DIAGNOSTIC (Phase2a Task 4): numerically localizes where the reported Walk leg-amplitude
// compression is introduced. It is a measurement tool, NOT a fix. It writes
// previews/character/unity_capture/retarget_probe.json with three cross-checkable views:
//
//   SOURCE  — muscle-space curve amplitudes read straight off the imported "Walk"
//             AnimationClip (AnimationUtility.GetCurveBindings). If the leg muscle ranges
//             here are already tiny, the clip import mis-generated the muscles.
//   TARGET  — runtime-fidelity replay: the clip is played through an AnimatorController on
//             our character's OWN humanoid avatar and stepped with animator.Update(1/30),
//             recording actual knee-flex angles / foot / hips motion. This is the honest
//             runtime path (NOT the AnimationMode path ClipCapture uses), so comparing the
//             two tells us whether the visual compression is a runtime fact or a capture
//             artifact.
//   AVATAR  — the model's humanDescription leg-bone HumanLimits + measured leg segment
//             lengths + hips height, so muscle->angle mapping anomalies (limits/T-pose/
//             proportion) are visible.
//
// Baselines: the same TARGET replay is also run for Idle (expect near-zero ranges — sanity
// check that the probe itself is not flattening motion) and Run (expect large ranges).
//
// Diagnostic tool => never aborts the pipeline: it always writes the JSON and Exit(0),
// embedding any per-section error text under "errors" instead of failing.
public static class RetargetProbe
{
    const string FbxAssetPath = "Assets/Import/model.fbx";
    const float DT = 1f / 30f;

    static string RepoRoot() =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));

    static string F(float v) => v.ToString("F5", CultureInfo.InvariantCulture);

    // Scan every imported clip FBX and match the sub-asset AnimationClip by name
    // (same lookup pattern as ClipCapture / PlaySceneBuild).
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

    // Running min/max/range accumulator.
    class Stat
    {
        public bool has; public float min, max; public int n;
        public void Add(float v)
        {
            if (!has) { min = max = v; has = true; }
            else { if (v < min) min = v; if (v > max) max = v; }
            n++;
        }
        public float Range => has ? max - min : 0f;
        public string Json() => "{\"min\":" + F(has ? min : 0f) + ",\"max\":" + F(has ? max : 0f) +
                                ",\"range\":" + F(Range) + ",\"n\":" + n + "}";
    }

    static readonly List<string> Errors = new List<string>();

    public static void Run()
    {
        var sb = new StringBuilder();
        sb.Append("{\n");

        // -------------------------------------------------------------- SOURCE (muscle curves)
        sb.Append("\"source\":");
        try { sb.Append(SourceJson("Walk")); }
        catch (Exception e) { Errors.Add("source: " + e.Message); sb.Append("null"); }
        sb.Append(",\n");

        // -------------------------------------------------------------- AVATAR (limits + proportions)
        sb.Append("\"avatar\":");
        try { sb.Append(AvatarJson()); }
        catch (Exception e) { Errors.Add("avatar: " + e.Message); sb.Append("null"); }
        sb.Append(",\n");

        // -------------------------------------------------------------- TARGET (runtime replay)
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(FbxAssetPath);
        sb.Append("\"target\":{\n");
        string[] clipsToRun = { "Walk", "Idle", "Run" };
        for (int i = 0; i < clipsToRun.Length; i++)
        {
            sb.Append("\"").Append(clipsToRun[i]).Append("\":");
            try { sb.Append(TargetJson(prefab, clipsToRun[i])); }
            catch (Exception e) { Errors.Add("target/" + clipsToRun[i] + ": " + e.Message); sb.Append("null"); }
            sb.Append(i < clipsToRun.Length - 1 ? ",\n" : "\n");
        }
        sb.Append("},\n");

        sb.Append("\"errors\":[")
          .Append(string.Join(",", Errors.Select(e => "\"" + Escape(e) + "\"")))
          .Append("]\n");
        sb.Append("}\n");

        string outDir = Path.Combine(RepoRoot(), "previews", "character", "unity_capture");
        Directory.CreateDirectory(outDir);
        string outPath = Path.Combine(outDir, "retarget_probe.json");
        File.WriteAllText(outPath, sb.ToString());
        Debug.Log("RETARGET PROBE:\n" + sb.ToString());
        Debug.Log("RetargetProbe: wrote " + outPath + (Errors.Count > 0 ? (" (errors=" + Errors.Count + ")") : ""));

        // Diagnostic tool: always exit 0 so run.ps1 prints the JSON regardless.
        EditorApplication.Exit(0);
    }

    // --- SOURCE ---------------------------------------------------------------------------
    static string SourceJson(string clipName)
    {
        var clip = Clip(clipName);
        if (clip == null) throw new Exception("clip not found: " + clipName + " (run.ps1 clips 선행)");

        var leg = new List<string>();
        var arm = new List<string>();
        foreach (var b in AnimationUtility.GetCurveBindings(clip))
        {
            string p = b.propertyName;
            bool isLeg = p.IndexOf("Leg", StringComparison.OrdinalIgnoreCase) >= 0 ||
                         p.IndexOf("Foot", StringComparison.OrdinalIgnoreCase) >= 0 ||
                         p.IndexOf("Toe", StringComparison.OrdinalIgnoreCase) >= 0;
            bool isArm = p.IndexOf("Arm", StringComparison.OrdinalIgnoreCase) >= 0;   // catches "Arm" + "Forearm"
            if (!isLeg && !isArm) continue;

            var curve = AnimationUtility.GetEditorCurve(clip, b);
            var s = new Stat();
            if (curve != null && curve.length > 0)
            {
                // Sample densely across the clip so interpolated extrema are captured,
                // and also fold in the raw keyframe values.
                foreach (var k in curve.keys) s.Add(k.value);
                int N = 240;
                for (int i = 0; i <= N; i++)
                    s.Add(curve.Evaluate(clip.length * (i / (float)N)));
            }
            string entry = "{\"name\":\"" + Escape(p) + "\",\"type\":\"" + Escape(b.type.Name) +
                           "\",\"min\":" + F(s.has ? s.min : 0f) + ",\"max\":" + F(s.has ? s.max : 0f) +
                           ",\"range\":" + F(s.Range) + ",\"keys\":" + (curve != null ? curve.length : 0) + "}";
            if (isLeg) leg.Add(entry); else arm.Add(entry);
        }

        var sb = new StringBuilder();
        sb.Append("{\"clip\":\"").Append(clipName).Append("\",\"length\":").Append(F(clip.length));
        sb.Append(",\"legBindings\":[").Append(string.Join(",", leg)).Append("]");
        sb.Append(",\"armBindings\":[").Append(string.Join(",", arm)).Append("]}");
        return sb.ToString();
    }

    // --- AVATAR ---------------------------------------------------------------------------
    static readonly (HumanBodyBones bone, string human)[] LegBones =
    {
        (HumanBodyBones.LeftUpperLeg, "LeftUpperLeg"),
        (HumanBodyBones.LeftLowerLeg, "LeftLowerLeg"),
        (HumanBodyBones.LeftFoot,     "LeftFoot"),
        (HumanBodyBones.RightUpperLeg,"RightUpperLeg"),
        (HumanBodyBones.RightLowerLeg,"RightLowerLeg"),
        (HumanBodyBones.RightFoot,    "RightFoot"),
    };

    static string AvatarJson()
    {
        var importer = AssetImporter.GetAtPath(FbxAssetPath) as ModelImporter;
        if (importer == null) throw new Exception(FbxAssetPath + " has no ModelImporter (run.ps1 unity 선행)");
        var hd = importer.humanDescription;

        // HumanLimit info for the leg bones.
        var limits = new List<string>();
        foreach (var lb in LegBones)
        {
            var hb = hd.human.FirstOrDefault(h => h.humanName == lb.human);
            if (hb.humanName == null)
            {
                limits.Add("{\"humanName\":\"" + lb.human + "\",\"mapped\":false}");
                continue;
            }
            var lim = hb.limit;
            limits.Add("{\"humanName\":\"" + Escape(hb.humanName) + "\",\"boneName\":\"" + Escape(hb.boneName) +
                       "\",\"mapped\":true,\"useDefaultValues\":" + (lim.useDefaultValues ? "true" : "false") +
                       ",\"min\":[" + F(lim.min.x) + "," + F(lim.min.y) + "," + F(lim.min.z) + "]" +
                       ",\"max\":[" + F(lim.max.x) + "," + F(lim.max.y) + "," + F(lim.max.z) + "]" +
                       ",\"center\":[" + F(lim.center.x) + "," + F(lim.center.y) + "," + F(lim.center.z) + "]" +
                       ",\"axisLength\":" + F(lim.axisLength) + "}");
        }

        // Measured segment lengths + hips height from the rest-pose skeleton.
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(FbxAssetPath);
        GameObject inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        string lengths;
        try
        {
            var anim = inst.GetComponent<Animator>() ?? inst.AddComponent<Animator>();
            Transform lu = anim.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
            Transform ll = anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
            Transform lf = anim.GetBoneTransform(HumanBodyBones.LeftFoot);
            Transform ru = anim.GetBoneTransform(HumanBodyBones.RightUpperLeg);
            Transform rl = anim.GetBoneTransform(HumanBodyBones.RightLowerLeg);
            Transform rf = anim.GetBoneTransform(HumanBodyBones.RightFoot);
            Transform hips = anim.GetBoneTransform(HumanBodyBones.Hips);

            float lThigh = (lu != null && ll != null) ? Vector3.Distance(lu.position, ll.position) : -1f;
            float lShin  = (ll != null && lf != null) ? Vector3.Distance(ll.position, lf.position) : -1f;
            float rThigh = (ru != null && rl != null) ? Vector3.Distance(ru.position, rl.position) : -1f;
            float rShin  = (rl != null && rf != null) ? Vector3.Distance(rl.position, rf.position) : -1f;
            float hipsY  = hips != null ? hips.position.y : -1f;

            lengths = "{\"leftThigh\":" + F(lThigh) + ",\"leftShin\":" + F(lShin) +
                      ",\"rightThigh\":" + F(rThigh) + ",\"rightShin\":" + F(rShin) +
                      ",\"hipsHeight\":" + F(hipsY) + "}";
        }
        finally { UnityEngine.Object.DestroyImmediate(inst); }

        return "{\"legLimits\":[" + string.Join(",", limits) + "],\"restPose\":" + lengths + "}";
    }

    // --- TARGET ---------------------------------------------------------------------------
    static string TargetJson(GameObject prefab, string clipName)
    {
        if (prefab == null) throw new Exception(FbxAssetPath + " missing (run.ps1 unity -Profile character 선행)");
        var clip = Clip(clipName);
        if (clip == null) throw new Exception("clip not found: " + clipName);

        // Single-state controller playing the clip on a temp asset.
        if (!AssetDatabase.IsValidFolder("Assets/Play"))
            AssetDatabase.CreateFolder("Assets", "Play");
        string ctrlPath = "Assets/Play/__probe.controller";
        AssetDatabase.DeleteAsset(ctrlPath);
        var ctrl = AnimatorController.CreateAnimatorControllerAtPathWithClip(ctrlPath, clip);

        GameObject inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        try
        {
            var anim = inst.GetComponent<Animator>() ?? inst.AddComponent<Animator>();
            anim.runtimeAnimatorController = ctrl;
            anim.applyRootMotion = false;
            anim.cullingMode = AnimatorCullingMode.AlwaysAnimate;   // never cull in headless/edit stepping
            anim.Rebind();
            anim.Update(0f);   // settle at t=0 on the state's clip

            Transform lu = anim.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
            Transform ll = anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
            Transform lf = anim.GetBoneTransform(HumanBodyBones.LeftFoot);
            Transform ru = anim.GetBoneTransform(HumanBodyBones.RightUpperLeg);
            Transform rl = anim.GetBoneTransform(HumanBodyBones.RightLowerLeg);
            Transform rf = anim.GetBoneTransform(HumanBodyBones.RightFoot);
            Transform hips = anim.GetBoneTransform(HumanBodyBones.Hips);
            if (lu == null || ll == null || lf == null || ru == null || rl == null || rf == null || hips == null)
                throw new Exception("leg bone(s) unresolved on avatar");

            var lKnee = new Stat(); var rKnee = new Stat();
            var lFx = new Stat(); var lFy = new Stat(); var lFz = new Stat();
            var rFx = new Stat(); var rFy = new Stat(); var rFz = new Stat();
            var hipsYStat = new Stat();

            int steps = Mathf.Max(4, Mathf.CeilToInt((2f * clip.length) / DT));
            for (int i = 0; i < steps; i++)
            {
                anim.Update(DT);

                lKnee.Add(KneeFlex(lu, ll, lf));
                rKnee.Add(KneeFlex(ru, rl, rf));
                Vector3 lp = lf.position, rp = rf.position;
                lFx.Add(lp.x); lFy.Add(lp.y); lFz.Add(lp.z);
                rFx.Add(rp.x); rFy.Add(rp.y); rFz.Add(rp.z);
                hipsYStat.Add(hips.position.y);
            }

            var sb = new StringBuilder();
            sb.Append("{\"clipLength\":").Append(F(clip.length)).Append(",\"steps\":").Append(steps);
            sb.Append(",\"leftKneeFlexDeg\":").Append(lKnee.Json());
            sb.Append(",\"rightKneeFlexDeg\":").Append(rKnee.Json());
            sb.Append(",\"leftFootX\":").Append(lFx.Json());
            sb.Append(",\"leftFootY\":").Append(lFy.Json());
            sb.Append(",\"leftFootZ\":").Append(lFz.Json());
            sb.Append(",\"rightFootX\":").Append(rFx.Json());
            sb.Append(",\"rightFootY\":").Append(rFy.Json());
            sb.Append(",\"rightFootZ\":").Append(rFz.Json());
            sb.Append(",\"hipsY\":").Append(hipsYStat.Json());
            sb.Append("}");
            return sb.ToString();
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(inst);
            AssetDatabase.DeleteAsset(ctrlPath);
        }
    }

    // Knee flex angle per the task spec: angle between (upper-lower) and (lower-foot).
    // ~0deg = straight leg (hip/knee/ankle collinear), grows as the knee bends.
    static float KneeFlex(Transform upper, Transform lower, Transform foot)
    {
        Vector3 thigh = upper.position - lower.position;   // knee -> hip
        Vector3 shin = lower.position - foot.position;      // ankle -> knee
        return Vector3.Angle(thigh, shin);
    }

    static string Escape(string s) =>
        (s ?? "").Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", " ").Replace("\r", " ");
}
