using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;

// G9.1 (T37): goblin FBX import settings, applied explicitly by file name (not by Unity's model@clip rule).
// Only direct children of Assets/Goblin/ are touched; every other asset is ignored.
//   goblin.fbx          Generic, Avatar CreateFromThisModel, root node MotionNodePath, importAnimation off
//   goblin@<clip>.fbx   Generic, Avatar CopyFromOther (goblin.fbx Avatar), animation compression Off
//                       (resampleCurves left at the importer default and recorded)
//   goblin_club.fbx     animationType None
//   HR2 (rig/data/hr2_contract.md): goblin.fbx and goblin@<clip>.fbx keep importBlendShapes = true (the importer
//   default, stated explicitly) so goblin_mesh has the blend shape mouth_open and clips keep blendShape.mouth_open
//   curves; materials are imported as before (no material setting is touched).
// The same Apply() runs in OnPreprocessModel and in EnsureImported(); EnsureImported re-saves an importer
// whose serialized settings still differ (e.g. the clip was imported before goblin.fbx had an Avatar).
public static class GoblinImport
{
    public const string Folder = "Assets/Goblin";
    public const string ModelPath = Folder + "/goblin.fbx";
    public const string ClubPath = Folder + "/goblin_club.fbx";
    public const string RigtestPath = Folder + "/goblin@rigtest.fbx";
    // goblin.fbx has two FBX root nodes (armature `goblin`, mesh `goblin_mesh`), so Unity keeps a wrapper
    // root and the root bone path is armature/root. OnPostprocessModel logs an error if this does not resolve.
    public const string MotionNodePath = "goblin/root";
    const string RootBoneName = "root";

    static readonly Regex ClipName = new Regex(@"^goblin@[A-Za-z0-9_]+\.fbx$", RegexOptions.CultureInvariant);

    public enum Kind { None, Model, Clip, Club }

    public static Kind Classify(string assetPath)
    {
        if (string.IsNullOrEmpty(assetPath)) return Kind.None;
        string p = assetPath.Replace('\\', '/');
        if (!p.StartsWith(Folder + "/", StringComparison.Ordinal)) return Kind.None;
        string file = p.Substring(Folder.Length + 1);
        if (file.Contains("/")) return Kind.None;
        if (file == "goblin.fbx") return Kind.Model;
        if (file == "goblin_club.fbx") return Kind.Club;
        if (ClipName.IsMatch(file)) return Kind.Clip;
        return Kind.None;
    }

    public static Avatar LoadModelAvatar() =>
        AssetDatabase.LoadAllAssetsAtPath(ModelPath).OfType<Avatar>().FirstOrDefault();

    // Applies the settings for `kind`; returns "field: old -> new" for every value it changed.
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
                Set("motionNodeName", mi.motionNodeName, MotionNodePath, v => mi.motionNodeName = v);
                Set("importAnimation", mi.importAnimation, false, v => mi.importAnimation = v);
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
                    Debug.LogWarning($"GoblinImport: {ModelPath} Avatar not available yet; {mi.assetPath} needs a reimport after goblin.fbx");
                }
                Set("animationCompression", mi.animationCompression, ModelImporterAnimationCompression.Off, v => mi.animationCompression = v);
                Set("importBlendShapes", mi.importBlendShapes, true, v => mi.importBlendShapes = v);
                break;
            case Kind.Club:
                Set("animationType", mi.animationType, ModelImporterAnimationType.None, v => mi.animationType = v);
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
        l.Add(ClubPath);
        return l.Where(p => File.Exists(FullPath(p))).ToList();
    }

    // Refresh, (re)import goblin.fbx first, then clips, then the club; re-save any importer whose
    // settings still differ. Returns {assetPath: [changes]} for re-saved importers.
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

    public static string FindPath(Transform root, string name)
    {
        var hit = root.GetComponentsInChildren<Transform>(true).FirstOrDefault(x => x != root && x.name == name);
        return hit == null ? null : PathOf(hit, root);
    }

    // Importer settings + import result for one FBX (probe and goblin_report "importers").
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
        o.Add("preserveHierarchy", mi.preserveHierarchy);
        o.Add("isReadable", mi.isReadable);
        o.Add("optimizeGameObjects", mi.optimizeGameObjects);
        o.Add("materialImportMode", mi.materialImportMode.ToString());
        o.Add("importBlendShapes", mi.importBlendShapes);
        var settings = mi.clipAnimations.Length > 0 ? mi.clipAnimations : mi.defaultClipAnimations;
        o.Add("clipSettingsSource", mi.clipAnimations.Length > 0 ? "clipAnimations" : "defaultClipAnimations");
        o.Add("clipSettings", settings.Select(c => (object)new JObj {
            { "name", c.name }, { "takeName", c.takeName }, { "firstFrame", c.firstFrame },
            { "lastFrame", c.lastFrame }, { "loopTime", c.loopTime } }).ToList());

        var subs = AssetDatabase.LoadAllAssetsAtPath(path);
        var avatar = subs.OfType<Avatar>().FirstOrDefault();
        o.Add("avatar", avatar == null ? null : new JObj {
            { "name", avatar.name }, { "isValid", avatar.isValid }, { "isHuman", avatar.isHuman } });
        o.Add("clips", subs.OfType<AnimationClip>().Where(c => !c.name.StartsWith("__preview__"))
            .Select(c => (object)new JObj { { "name", c.name }, { "length_s", c.length }, { "frameRate", c.frameRate } }).ToList());
        var main = AssetDatabase.LoadAssetAtPath<GameObject>(path);
        o.Add("prefabRoot", main == null ? null : main.name);
        o.Add("rootBonePathResolved", main == null ? null : FindPath(main.transform, RootBoneName));
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
}

class GoblinImportPostprocessor : AssetPostprocessor
{
    void OnPreprocessModel()
    {
        var kind = GoblinImport.Classify(assetPath);
        if (kind == GoblinImport.Kind.None) return;
        if (kind == GoblinImport.Kind.Clip) context.DependsOnSourceAsset(GoblinImport.ModelPath);
        var changed = GoblinImport.Apply((ModelImporter)assetImporter, kind);
        if (changed.Count > 0) Debug.Log($"GoblinImport preprocess {assetPath}: {string.Join("; ", changed)}");
    }

    void OnPostprocessModel(GameObject g)
    {
        if (GoblinImport.Classify(assetPath) != GoblinImport.Kind.Model) return;
        string found = GoblinImport.FindPath(g.transform, "root");
        if (found != GoblinImport.MotionNodePath)
            Debug.LogError($"GoblinImport: root bone path is '{found ?? "missing"}', motionNodeName is '{GoblinImport.MotionNodePath}'");
    }
}

public static class GoblinImportProbe
{
    public static void Run()
    {
        string outPath = Path.Combine(GoblinImport.RepoRoot(), "unity", "AvatarCheck", "goblin_import_probe.json");
        int code = 0;
        var log = new List<string>();
        void Capture(string c, string st, LogType t)
        {
            if (t != LogType.Log) log.Add($"[{t}] {c}");
        }
        var o = new JObj { { "unity_version", Application.unityVersion } };
        try
        {
            if (GoblinImport.AllPaths().Count == 0)
            {
                o.Add("status", "no goblin assets");
                GoblinImport.WriteJson(outPath, o);
                Debug.Log("GOBLIN IMPORT PROBE: no goblin assets");
                EditorApplication.Exit(0);
                return;
            }
            Application.logMessageReceived += Capture;
            var resaved = GoblinImport.EnsureImported(false);
            Application.logMessageReceived -= Capture;
            o.Add("status", "ok");
            o.Add("resaved_by_EnsureImported", resaved);
            var files = new JObj();
            var expected = new List<string> { GoblinImport.ModelPath, GoblinImport.RigtestPath, GoblinImport.ClubPath };
            foreach (var p in expected.Concat(GoblinImport.ClipPaths()).Distinct())
                files.Add(Path.GetFileName(p), GoblinImport.Describe(p));
            o.Add("files", files);
            o.Add("import_log", log);
        }
        catch (Exception ex)
        {
            Application.logMessageReceived -= Capture;
            o.Add("exception", ex.ToString());
            o.Add("import_log", log);
            code = 1;
        }
        GoblinImport.WriteJson(outPath, o);
        Debug.Log("GOBLIN IMPORT PROBE written: " + outPath);
        EditorApplication.Exit(code);
    }
}

// Ordered JSON object + minimal pretty writer (numbers invariant, NaN/Inf -> null).
public class JObj : List<KeyValuePair<string, object>>
{
    public void Add(string key, object value) => Add(new KeyValuePair<string, object>(key, value));

    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    static string Num(double d) => double.IsNaN(d) || double.IsInfinity(d) ? "null" : d.ToString("R", Inv);
    static string Num(float f) => float.IsNaN(f) || float.IsInfinity(f) ? "null" : f.ToString("R", Inv);

    static void Str(StringBuilder sb, string s)
    {
        sb.Append('"');
        foreach (char c in s)
        {
            switch (c)
            {
                case '"': sb.Append("\\\""); break;
                case '\\': sb.Append("\\\\"); break;
                case '\n': sb.Append("\\n"); break;
                case '\r': sb.Append("\\r"); break;
                case '\t': sb.Append("\\t"); break;
                default:
                    if (c < 0x20) sb.Append("\\u").Append(((int)c).ToString("x4")); else sb.Append(c);
                    break;
            }
        }
        sb.Append('"');
    }

    static bool IsInline(object v) =>
        v == null || v is string || v is bool || v is int || v is long || v is float || v is double
        || v is Vector3 || v is Quaternion || v is float[] || v is double[] || v is Enum;

    public static void Write(StringBuilder sb, object v, int ind)
    {
        switch (v)
        {
            case null: sb.Append("null"); return;
            case string s: Str(sb, s); return;
            case bool b: sb.Append(b ? "true" : "false"); return;
            case int i: sb.Append(i.ToString(Inv)); return;
            case long l: sb.Append(l.ToString(Inv)); return;
            case float f: sb.Append(Num(f)); return;
            case double d: sb.Append(Num(d)); return;
            case Enum e: Str(sb, e.ToString()); return;
            case Vector3 p: sb.Append('[').Append(Num(p.x)).Append(',').Append(Num(p.y)).Append(',').Append(Num(p.z)).Append(']'); return;
            case Quaternion q: sb.Append('[').Append(Num(q.x)).Append(',').Append(Num(q.y)).Append(',').Append(Num(q.z)).Append(',').Append(Num(q.w)).Append(']'); return;
            case float[] fa: sb.Append('[').Append(string.Join(",", fa.Select(x => Num(x)))).Append(']'); return;
            case double[] da: sb.Append('[').Append(string.Join(",", da.Select(x => Num(x)))).Append(']'); return;
            case Matrix4x4 m:
                sb.Append('[');
                for (int r = 0; r < 4; r++)
                {
                    if (r > 0) sb.Append(',');
                    sb.Append('[').Append(Num(m[r, 0])).Append(',').Append(Num(m[r, 1])).Append(',')
                      .Append(Num(m[r, 2])).Append(',').Append(Num(m[r, 3])).Append(']');
                }
                sb.Append(']');
                return;
            case JObj o:
                if (o.Count == 0) { sb.Append("{}"); return; }
                sb.Append("{\n");
                for (int k = 0; k < o.Count; k++)
                {
                    sb.Append(' ', ind + 1);
                    Str(sb, o[k].Key);
                    sb.Append(": ");
                    Write(sb, o[k].Value, ind + 1);
                    sb.Append(k < o.Count - 1 ? ",\n" : "\n");
                }
                sb.Append(' ', ind).Append('}');
                return;
            case IEnumerable seq:
                var items = seq.Cast<object>().ToList();
                if (items.Count == 0) { sb.Append("[]"); return; }
                if (items.All(IsInline))
                {
                    sb.Append('[');
                    for (int k = 0; k < items.Count; k++) { if (k > 0) sb.Append(", "); Write(sb, items[k], ind); }
                    sb.Append(']');
                    return;
                }
                sb.Append("[\n");
                for (int k = 0; k < items.Count; k++)
                {
                    sb.Append(' ', ind + 1);
                    Write(sb, items[k], ind + 1);
                    sb.Append(k < items.Count - 1 ? ",\n" : "\n");
                }
                sb.Append(' ', ind).Append(']');
                return;
            default:
                Str(sb, v.ToString());
                return;
        }
    }
}
