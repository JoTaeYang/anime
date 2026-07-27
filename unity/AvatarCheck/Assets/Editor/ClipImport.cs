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

            // 아바타 원천: 스킨 포함 Y Bot (assets/mocap/YBotWalking.fbx). 미반입이면 반입.
            if (AssetDatabase.LoadMainAssetAtPath("Assets/Clips/YBotWalking.fbx") == null)
                ClipCapture.ImportYBot();
            var ybotAvatar = AssetDatabase.LoadAllAssetsAtPath("Assets/Clips/YBotWalking.fbx")
                .OfType<Avatar>().FirstOrDefault();
            if (ybotAvatar == null || !ybotAvatar.isValid || !ybotAvatar.isHuman)
                throw new Exception("Y Bot source avatar invalid/missing — assets/mocap/YBotWalking.fbx (With Skin) 필요");

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
                importer.animationType = ModelImporterAnimationType.Human;
                // 아바타는 스킨 포함 Y Bot 임포트에서 복사한다 (2026-07-27 근본 원인 수정).
                // Without-Skin FBX는 뼈대만 있어 CreateFromThisModel의 T포즈 캘리브레이션이
                // 부실해지고, 그 아바타로 구운 근육 커브는 "다른" 아바타로 리타게팅할 때만
                // 왜곡된다 (무릎 20~30도 얕아짐, 왼발목 +31도 — 네이티브 재생은 왜곡이
                // 상쇄되어 정상으로 보임. 교차 계측으로 실증: WalkNative deep vs Walk-on-
                // character shallow, YBotWalk[with-skin] 리타게팅은 1~4도 일치).
                // 모든 Mixamo 클립은 동일 mixamorig 스켈레톤이므로 아바타 공유가 표준 관행.
                importer.avatarSetup = ModelImporterAvatarSetup.CopyFromOther;
                importer.sourceAvatar = ybotAvatar;
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

                // CopyFromOther는 자산 내 Avatar 서브에셋을 만들지 않으므로, 단언의 대상은
                // "실제로 배정된 아바타"(= sourceAvatar)의 유효성이다 — 검사 의도(휴머노이드
                // 유효 임포트) 동일, 대상만 정직하게 재지정 (약화 아님: 위에서 원천 아바타가
                // invalid면 전체 run이 throw로 실패한다).
                var assigned = ((ModelImporter)AssetImporter.GetAtPath(assetPath)).sourceAvatar;
                results.Add(($"avatar_human:{e.name}", assigned != null && assigned.isValid && assigned.isHuman,
                    assigned == null ? "no source avatar" : $"copied YBot avatar isValid={assigned.isValid} isHuman={assigned.isHuman}"));

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
