using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

public static class AvatarCheck
{
    const string FbxAssetPath = "Assets/Dummy/dummy.fbx";
    static readonly List<string> ImportErrors = new List<string>();

    // Blender 쪽 scripts/lib/bone_map.py의 REQUIRED_HUMAN_BONES와 일치해야 한다
    static readonly Dictionary<HumanBodyBones, string> Required = new Dictionary<HumanBodyBones, string>
    {
        { HumanBodyBones.Hips, "Hips" }, { HumanBodyBones.Spine, "Spine" }, { HumanBodyBones.Head, "Head" },
        { HumanBodyBones.LeftUpperLeg, "LeftUpperLeg" }, { HumanBodyBones.LeftLowerLeg, "LeftLowerLeg" }, { HumanBodyBones.LeftFoot, "LeftFoot" },
        { HumanBodyBones.RightUpperLeg, "RightUpperLeg" }, { HumanBodyBones.RightLowerLeg, "RightLowerLeg" }, { HumanBodyBones.RightFoot, "RightFoot" },
        { HumanBodyBones.LeftUpperArm, "LeftUpperArm" }, { HumanBodyBones.LeftLowerArm, "LeftLowerArm" }, { HumanBodyBones.LeftHand, "LeftHand" },
        { HumanBodyBones.RightUpperArm, "RightUpperArm" }, { HumanBodyBones.RightLowerArm, "RightLowerArm" }, { HumanBodyBones.RightHand, "RightHand" },
    };

    public static void Run()
    {
        var results = new List<(string name, bool pass, string detail)>();
        GameObject instance = null;
        try
        {
            CopyFbxIntoProject();

            Application.logMessageReceived += CaptureLog;
            AssetDatabase.ImportAsset(FbxAssetPath, ImportAssetOptions.ForceSynchronousImport);
            var importer = (ModelImporter)AssetImporter.GetAtPath(FbxAssetPath);
            importer.animationType = ModelImporterAnimationType.Human;
            importer.SaveAndReimport();
            Application.logMessageReceived -= CaptureLog;

            results.Add(("import_clean", ImportErrors.Count == 0,
                ImportErrors.Count == 0 ? "no errors/warnings" : string.Join(" | ", ImportErrors.Take(5))));

            var avatar = AssetDatabase.LoadAllAssetsAtPath(FbxAssetPath).OfType<Avatar>().FirstOrDefault();
            bool avatarOk = avatar != null && avatar.isValid && avatar.isHuman;
            results.Add(("avatar_valid_human", avatarOk,
                avatar == null ? "no avatar" : $"isValid={avatar.isValid} isHuman={avatar.isHuman}"));

            if (avatarOk)
            {
                var human = avatar.humanDescription.human.ToDictionary(h => h.humanName, h => h.boneName);
                var missing = new List<string>();
                foreach (var kv in Required)
                {
                    string humanName = HumanTrait.BoneName[(int)kv.Key];
                    if (!human.TryGetValue(humanName, out var mapped) || mapped != kv.Value)
                        missing.Add($"{humanName}->({(human.ContainsKey(humanName) ? human[humanName] : "UNMAPPED")}) expected {kv.Value}");
                }
                results.Add(("required_15_mapped", missing.Count == 0,
                    missing.Count == 0 ? "all mapped to intended bones" : string.Join(" | ", missing)));

                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(FbxAssetPath);
                instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                var anim = instance.GetComponent<Animator>();

                float hipsY = anim.GetBoneTransform(HumanBodyBones.Hips).position.y;
                results.Add(("hips_height", hipsY > 0.77f && hipsY < 0.97f, $"hipsY={hipsY:F4} expected 0.87±0.10"));

                var lh = anim.GetBoneTransform(HumanBodyBones.LeftHand).position;
                var rh = anim.GetBoneTransform(HumanBodyBones.RightHand).position;
                var lf = anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
                var lt = anim.GetBoneTransform(HumanBodyBones.LeftToes).position;
                bool facing = lh.x > rh.x && lt.z > lf.z;
                results.Add(("faces_plus_z", facing, $"leftHand.x={lh.x:F3} rightHand.x={rh.x:F3} toeZ-footZ={(lt.z - lf.z):F3}"));

                var badRots = new List<string>();
                foreach (Transform child in instance.transform)
                {
                    float angle = Quaternion.Angle(child.localRotation, Quaternion.identity);
                    if (angle > 1f) badRots.Add($"{child.name}:{angle:F1}deg");
                }
                results.Add(("root_children_identity_rotation", badRots.Count == 0,
                    badRots.Count == 0 ? "clean" : string.Join(" | ", badRots)));

                var clip = AssetDatabase.LoadAllAssetsAtPath(FbxAssetPath).OfType<AnimationClip>()
                    .FirstOrDefault(c => !c.name.StartsWith("__preview"));
                bool clipOk = clip != null && clip.length > 1.5f && clip.length < 2.5f;
                results.Add(("idle_clip_present", clipOk, clip == null ? "no clip" : $"{clip.name} len={clip.length:F3}s"));

                if (clipOk)
                {
                    var chest = anim.GetBoneTransform(HumanBodyBones.Spine);
                    clip.SampleAnimation(instance, 0f);
                    Vector3 p0 = chest.position;
                    clip.SampleAnimation(instance, clip.length * 0.5f);
                    Vector3 pMid = chest.position;
                    float delta = (p0 - pMid).magnitude;
                    results.Add(("idle_actually_moves", delta > 0.004f, $"spine delta={delta:F4}m at t=0 vs t=mid"));
                }
            }
        }
        catch (Exception e)
        {
            results.Add(("exception", false, e.ToString()));
        }
        finally
        {
            if (instance != null) UnityEngine.Object.DestroyImmediate(instance);
        }

        WriteReport(results);
        bool all = results.All(r => r.pass);
        Debug.Log($"AvatarCheck: {(all ? "PASS" : "FAIL")} ({results.Count(r => r.pass)}/{results.Count})");
        EditorApplication.Exit(all ? 0 : 1);
    }

    public static void CompileSmoke()
    {
        Debug.Log("AvatarCheck compile smoke OK");
        EditorApplication.Exit(0);
    }

    static void CopyFbxIntoProject()
    {
        string src = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "exports", "dummy.fbx"));
        string dst = Path.GetFullPath(Path.Combine(Application.dataPath, "Dummy", "dummy.fbx"));
        if (!File.Exists(src)) throw new FileNotFoundException($"export first: {src}");
        Directory.CreateDirectory(Path.GetDirectoryName(dst));
        // Delete any previously-imported asset (+ its .meta) so the humanoid avatar is
        // AUTO-mapped fresh from the current FBX. Unity bakes the auto-map result into the
        // .meta on first import and reuses it; without this, a re-run after a rig change
        // (exactly what the Task 9 correction loop does) applies a STALE bone mapping and
        // fails avatar creation. Fresh import = honest re-validation, not weakened checks.
        if (File.Exists(dst)) AssetDatabase.DeleteAsset(FbxAssetPath);
        File.Copy(src, dst, true);
        AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
    }

    static void CaptureLog(string condition, string stackTrace, LogType type)
    {
        if (type == LogType.Error || type == LogType.Exception || type == LogType.Warning)
            ImportErrors.Add($"[{type}] {condition}");
    }

    static void WriteReport(List<(string name, bool pass, string detail)> results)
    {
        var sb = new StringBuilder();
        sb.Append("{\"allPass\":").Append(results.All(r => r.pass) ? "true" : "false").Append(",\"assertions\":[");
        sb.Append(string.Join(",", results.Select(r =>
            $"{{\"name\":\"{Escape(r.name)}\",\"pass\":{(r.pass ? "true" : "false")},\"detail\":\"{Escape(r.detail)}\"}}")));
        sb.Append("]}");
        string path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "report.json"));
        File.WriteAllText(path, sb.ToString());
    }

    static string Escape(string s) => s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", " ");
}
