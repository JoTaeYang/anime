using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

// T210 (P1.3): player FBX import settings, applied by file name. Only direct children of Assets/Player/ are touched.
//   Player.fbx          Generic, Avatar CreateFromThisModel, importBlendShapes true
//   Player@<clip>.fbx   Generic, Avatar CopyFromOther (Player.fbx Avatar), animation compression Off,
//                       importBlendShapes true
// Every other importer setting stays at the Unity default (recorded by Describe). The same Apply() runs in
// OnPreprocessModel and in EnsureImported(), which re-saves an importer whose settings still differ (e.g. a clip
// imported before Player.fbx had an Avatar).
// JSON output reuses JObj (Assets/Editor/Goblin/GoblinImport.cs, read only).
public static class PlayerImport
{
    public const string Folder = "Assets/Player";
    public const string ModelPath = Folder + "/Player.fbx";

    static readonly Regex ClipName = new Regex(@"^Player@[A-Za-z0-9_]+\.fbx$", RegexOptions.CultureInvariant);

    public enum Kind { None, Model, Clip }

    public static Kind Classify(string assetPath)
    {
        if (string.IsNullOrEmpty(assetPath)) return Kind.None;
        string p = assetPath.Replace('\\', '/');
        if (!p.StartsWith(Folder + "/", StringComparison.Ordinal)) return Kind.None;
        string file = p.Substring(Folder.Length + 1);
        if (file.Contains("/")) return Kind.None;
        if (file == "Player.fbx") return Kind.Model;
        if (ClipName.IsMatch(file)) return Kind.Clip;
        return Kind.None;
    }

    public static Avatar LoadModelAvatar() =>
        AssetDatabase.LoadAllAssetsAtPath(ModelPath).OfType<Avatar>().FirstOrDefault();

    public static List<string> Apply(ModelImporter mi, Kind kind)
    {
        var changed = new List<string>();
        void Set<T>(string field, T cur, T want, Action<T> setter)
        {
            if (EqualityComparer<T>.Default.Equals(cur, want)) return;
            setter(want);
            changed.Add($"{field}: {cur} -> {want}");
        }
        switch (kind)
        {
            case Kind.Model:
                Set("animationType", mi.animationType, ModelImporterAnimationType.Generic, v => mi.animationType = v);
                Set("avatarSetup", mi.avatarSetup, ModelImporterAvatarSetup.CreateFromThisModel, v => mi.avatarSetup = v);
                Set("importBlendShapes", mi.importBlendShapes, true, v => mi.importBlendShapes = v);
                break;
            case Kind.Clip:
                Set("animationType", mi.animationType, ModelImporterAnimationType.Generic, v => mi.animationType = v);
                var avatar = LoadModelAvatar();
                if (avatar != null)
                {
                    if (mi.sourceAvatar != avatar)
                    {
                        changed.Add($"sourceAvatar: {(mi.sourceAvatar ? mi.sourceAvatar.name : "null")} -> {avatar.name}");
                        mi.sourceAvatar = avatar;
                    }
                    Set("avatarSetup", mi.avatarSetup, ModelImporterAvatarSetup.CopyFromOther, v => mi.avatarSetup = v);
                }
                else
                {
                    Debug.LogWarning($"PlayerImport: {ModelPath} Avatar not available yet; {mi.assetPath} needs a reimport after Player.fbx");
                }
                Set("animationCompression", mi.animationCompression, ModelImporterAnimationCompression.Off, v => mi.animationCompression = v);
                Set("importBlendShapes", mi.importBlendShapes, true, v => mi.importBlendShapes = v);
                break;
        }
        return changed;
    }

    public static string RepoRoot() =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));

    public static string FullPath(string assetPath) =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", assetPath));

    public static List<string> ClipPaths()
    {
        string dir = FullPath(Folder);
        if (!Directory.Exists(dir)) return new List<string>();
        return Directory.GetFiles(dir, "*.fbx").Select(Path.GetFileName)
            .Where(f => ClipName.IsMatch(f)).OrderBy(f => f, StringComparer.Ordinal)
            .Select(f => Folder + "/" + f).ToList();
    }

    public static List<string> AllPaths()
    {
        var l = new List<string> { ModelPath };
        l.AddRange(ClipPaths());
        return l.Where(p => File.Exists(FullPath(p))).ToList();
    }

    // Refresh, (re)import Player.fbx first, then the clips; re-save any importer whose settings still differ.
    public static JObj EnsureImported(bool force)
    {
        AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
        var opts = ImportAssetOptions.ForceSynchronousImport | (force ? ImportAssetOptions.ForceUpdate : ImportAssetOptions.Default);
        var resaved = new JObj();
        foreach (var path in AllPaths())
        {
            AssetDatabase.ImportAsset(path, opts);
            var mi = AssetImporter.GetAtPath(path) as ModelImporter;
            if (mi == null) throw new Exception($"no ModelImporter for {path}");
            var changed = Apply(mi, Classify(path));
            if (changed.Count > 0)
            {
                mi.SaveAndReimport();
                resaved.Add(path, changed);
            }
        }
        return resaved;
    }

    public static string PathOf(Transform t, Transform root)
    {
        var parts = new List<string>();
        for (var c = t; c != null && c != root; c = c.parent) parts.Add(c.name);
        parts.Reverse();
        return string.Join("/", parts);
    }

    public static JObj Describe(string path)
    {
        var o = new JObj { { "path", path }, { "exists", File.Exists(FullPath(path)) } };
        var mi = AssetImporter.GetAtPath(path) as ModelImporter;
        if (mi == null) { o.Add("importer", null); return o; }
        o.Add("kind", Classify(path).ToString());
        o.Add("animationType", mi.animationType.ToString());
        o.Add("avatarSetup", mi.avatarSetup.ToString());
        o.Add("sourceAvatar", mi.sourceAvatar == null ? null : new JObj {
            { "name", mi.sourceAvatar.name }, { "assetPath", AssetDatabase.GetAssetPath(mi.sourceAvatar) } });
        o.Add("motionNodeName", mi.motionNodeName);
        o.Add("importAnimation", mi.importAnimation);
        o.Add("animationCompression", mi.animationCompression.ToString());
        o.Add("resampleCurves", mi.resampleCurves);
        o.Add("globalScale", mi.globalScale);
        o.Add("useFileScale", mi.useFileScale);
        o.Add("fileScale", mi.fileScale);
        o.Add("bakeAxisConversion", mi.bakeAxisConversion);
        o.Add("optimizeGameObjects", mi.optimizeGameObjects);
        o.Add("materialImportMode", mi.materialImportMode.ToString());
        o.Add("importBlendShapes", mi.importBlendShapes);
        var settings = mi.clipAnimations.Length > 0 ? mi.clipAnimations : mi.defaultClipAnimations;
        o.Add("clipSettings", settings.Select(c => (object)new JObj {
            { "name", c.name }, { "takeName", c.takeName }, { "firstFrame", c.firstFrame },
            { "lastFrame", c.lastFrame }, { "loopTime", c.loopTime } }).ToList());
        var subs = AssetDatabase.LoadAllAssetsAtPath(path);
        var avatar = subs.OfType<Avatar>().FirstOrDefault();
        o.Add("avatar", avatar == null ? null : new JObj {
            { "name", avatar.name }, { "isValid", avatar.isValid }, { "isHuman", avatar.isHuman } });
        o.Add("clips", subs.OfType<AnimationClip>().Where(c => !c.name.StartsWith("__preview__"))
            .Select(c => (object)new JObj { { "name", c.name }, { "length_s", c.length }, { "frameRate", c.frameRate } }).ToList());
        o.Add("subAssetTypes", subs.Where(s => s != null).GroupBy(s => s.GetType().Name)
            .OrderBy(g => g.Key).Select(g => (object)$"{g.Key} x{g.Count()}").ToList());
        return o;
    }

    public static void WriteJson(string fullPath, object value)
    {
        Directory.CreateDirectory(Path.GetDirectoryName(fullPath));
        var sb = new StringBuilder();
        JObj.Write(sb, value, 0);
        sb.Append('\n');
        File.WriteAllText(fullPath, sb.ToString(), new UTF8Encoding(false));
    }

    // First Unity batch run of the P1 chain: import (forced) + settings; log only. Exit 0 when no error was logged.
    public static void Run()
    {
        var errors = new List<string>();
        var warnings = new List<string>();
        void Capture(string c, string st, LogType t)
        {
            if (t == LogType.Error || t == LogType.Exception || t == LogType.Assert) errors.Add($"[{t}] {c}");
            else if (t == LogType.Warning) warnings.Add(c);
        }
        Application.logMessageReceived += Capture;
        try
        {
            if (AllPaths().Count == 0) throw new Exception($"no player assets in {Folder}");
            var resaved = EnsureImported(true);
            var sb = new StringBuilder();
            JObj.Write(sb, new JObj {
                { "resaved", resaved },
                { "files", AllPaths().Select(p => (object)Describe(p)).ToList() } }, 0);
            Debug.Log("PLAYER IMPORT:\n" + sb);
        }
        catch (Exception ex)
        {
            errors.Add("exception: " + ex);
        }
        Application.logMessageReceived -= Capture;
        Debug.Log($"PLAYER IMPORT done: errors {errors.Count}, warnings {warnings.Count}");
        foreach (var e in errors) Debug.Log("PLAYER IMPORT error: " + e);
        foreach (var w in warnings) Debug.Log("PLAYER IMPORT warning: " + w);
        EditorApplication.Exit(errors.Count == 0 ? 0 : 1);
    }
}

class PlayerImportPostprocessor : AssetPostprocessor
{
    void OnPreprocessModel()
    {
        var kind = PlayerImport.Classify(assetPath);
        if (kind == PlayerImport.Kind.None) return;
        if (kind == PlayerImport.Kind.Clip) context.DependsOnSourceAsset(PlayerImport.ModelPath);
        var changed = PlayerImport.Apply((ModelImporter)assetImporter, kind);
        if (changed.Count > 0) Debug.Log($"PlayerImport preprocess {assetPath}: {string.Join("; ", changed)}");
    }
}
