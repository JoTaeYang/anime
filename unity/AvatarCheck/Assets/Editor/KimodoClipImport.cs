using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

// kimodo.cpp(텍스트→모션)에서 생성해 Blender에서 캐릭터 리그로 리타게팅한 클립을 반입한다.
// ClipImport와 분리한 이유: Mixamo 클립은 mixamorig 스켈레톤이라 스킨 포함 Y Bot 아바타를
// 복사하지만(3e5450f), 이 클립은 이미 우리 캐릭터 스켈레톤 위에 구워져 있어 원천 아바타가
// 캐릭터 자신이어야 한다. 스켈레톤이 1:1로 같으므로 리타게팅이 항등이 되고, Without-Skin
// FBX의 T포즈 캘리브레이션 부실 문제 자체가 발생하지 않는다.
public static class KimodoClipImport
{
    const string SourceRel = "WalkKimodo.fbx";           // assets/mocap/ 아래
    const string AssetPath = "Assets/Clips/WalkKimodo.fbx";
    const string ClipName = "WalkKimodo";
    const string ModelPath = "Assets/Import/model.fbx";

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
            string src = Path.Combine(RepoRoot(), "assets", "mocap", SourceRel);
            if (!File.Exists(src))
                throw new Exception($"clip source missing: {src} — scripts/tools/kimodo_retarget.ps1 선행");

            var charAvatar = AssetDatabase.LoadAllAssetsAtPath(ModelPath)
                .OfType<Avatar>().FirstOrDefault();
            if (charAvatar == null || !charAvatar.isValid || !charAvatar.isHuman)
                throw new Exception($"character avatar invalid/missing at {ModelPath} — run.ps1 unity -Profile character 선행");

            if (!AssetDatabase.IsValidFolder("Assets/Clips"))
                AssetDatabase.CreateFolder("Assets", "Clips");
            if (AssetDatabase.LoadMainAssetAtPath(AssetPath) != null)
                AssetDatabase.DeleteAsset(AssetPath);     // 신규 반입 프로토콜: 항상 새 임포트
            File.Copy(src, Path.GetFullPath(AssetPath), overwrite: true);
            AssetDatabase.ImportAsset(AssetPath, ImportAssetOptions.ForceSynchronousImport);

            var importer = (ModelImporter)AssetImporter.GetAtPath(AssetPath);
            importer.animationType = ModelImporterAnimationType.Human;
            importer.avatarSetup = ModelImporterAvatarSetup.CopyFromOther;
            importer.sourceAvatar = charAvatar;
            var takes = importer.defaultClipAnimations;
            foreach (var t in takes) { t.name = ClipName; t.loopTime = true; }
            importer.clipAnimations = takes;

            ImportErrors.Clear();
            Application.logMessageReceived += CaptureLog;
            importer.SaveAndReimport();
            Application.logMessageReceived -= CaptureLog;

            // 사용자 승인(2026-07-19) 조건부 규칙 — ClipImport.cs와 동일 형태.
            const string PointerMarker = "has animation import warnings. See Import Messages";
            var pointerEntries = ImportErrors.Where(x => x.Contains(PointerMarker)).ToList();
            var otherErrors = ImportErrors.Where(x => !x.Contains(PointerMarker)).ToList();
            if (pointerEntries.Count > 0)
            {
                var so = new SerializedObject(importer);
                var warnProp = so.FindProperty("m_AnimationImportWarnings");
                string store = warnProp != null ? warnProp.stringValue : null;
                if (string.IsNullOrEmpty(store))
                    results.Add(("anim_warning_pointer", true, "empty pointer warning (approved conditional rule)"));
                else
                    otherErrors.AddRange(pointerEntries.Select(_ => $"[Warning] store: {store}"));
            }
            results.Add(("import_clean", otherErrors.Count == 0,
                otherErrors.Count == 0 ? "no errors/warnings" : string.Join(" | ", otherErrors.Take(50))));

            var assigned = ((ModelImporter)AssetImporter.GetAtPath(AssetPath)).sourceAvatar;
            results.Add(("avatar_human", assigned != null && assigned.isValid && assigned.isHuman,
                assigned == null ? "no source avatar"
                    : $"copied character avatar isValid={assigned.isValid} isHuman={assigned.isHuman}"));

            var clip = AssetDatabase.LoadAllAssetsAtPath(AssetPath).OfType<AnimationClip>()
                .FirstOrDefault(c => c.name == ClipName);
            results.Add(("clip_present", clip != null && clip.length > 0.1f,
                clip == null ? $"no clip named {ClipName}" : $"len={clip.length:F3}s"));
            if (clip != null)
            {
                results.Add(("loop_flag", clip.isLooping, $"isLooping={clip.isLooping} expected True"));
                // 루프 이음매: kimodo 원본은 3초 직진 보행이라 통째로는 안 붙고,
                // 잔차가 가장 작은 구간(원본 18..87프레임)만 잘라 내보낸다. 그 구간 길이.
                results.Add(("clip_length_window", Mathf.Abs(clip.length - 70f / 30f) < 0.15f,
                    $"len={clip.length:F3}s expected ~{70f / 30f:F3}s (70 frames @30fps)"));
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
        File.WriteAllText(Path.Combine(RepoRoot(), "unity", "AvatarCheck", "kimodo_report.json"), json);
        Debug.Log("KIMODO REPORT: " + json);
        EditorApplication.Exit(allPass ? 0 : 1);
    }
}
