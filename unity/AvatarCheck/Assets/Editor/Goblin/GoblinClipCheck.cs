using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

// A3 (T70) / I3 (T72): data collection for the Unity clip check. For every Assets/Goblin/goblin@*.fbx clip except
// rigtest, writes unity/AvatarCheck/goblin_clip_report_<clip>.json in the schema of
// work/goblin_swing/rig/data/a3_contract.md, plus a `rest` section (24 bones, world, fresh goblin.prefab instance
// before any sampling) and an "extra" object with diagnostics. Records only; no judgement. No renders.
// The old single-clip goblin_clip_report.json is deleted. Command line `-clip <name>` restricts the run to one clip.
// HR2 (2026-09-26, work/goblin_swing/rig/data/hr2_contract.md): "blend_shapes" = the goblin_mesh blend shapes, the
// blendShape.mouth_open curve binding and the mouth vertex indices (blend-shape delta > MouthDeltaMin); every sample
// gets "blend_shape" {weight after SampleAnimation, curve value at t (null = no curve), bake_world = BakeMesh
// positions of the mouth vertices in world when weight > 0}.
// Exit 0 when every report's `errors` is empty and at least one clip was checked, otherwise 1.
public static class GoblinClipCheck
{
    // canonical_skeleton.json TREE order
    static readonly string[] Bones = {
        "root", "pelvis", "spine_01", "spine_02", "head",
        "shoulder_l", "upperarm_l", "upperarm_twist_l", "lowerarm_l", "lowerarm_twist_l", "hand_l",
        "shoulder_r", "upperarm_r", "upperarm_twist_r", "lowerarm_r", "lowerarm_twist_r", "hand_r", "weapon_socket_r",
        "thigh_l", "calf_l", "foot_l", "thigh_r", "calf_r", "foot_r",
    };
    const string ExcludedClip = "rigtest";
    const double Fps = 24.0;
    const string PrefabPath = GoblinImport.Folder + "/goblin.prefab";
    const string MouthShape = "mouth_open";
    const double MouthDeltaMin = 1e-6;   // m: a vertex belongs to the mouth when its mouth_open delta is larger

    static string ClipArg()
    {
        var a = Environment.GetCommandLineArgs();
        for (int i = 0; i + 1 < a.Length; i++)
            if (a[i] == "-clip" && !string.IsNullOrEmpty(a[i + 1])) return a[i + 1];
        return null;
    }

    static string ClipNameOf(string assetPath)
    {
        string f = Path.GetFileNameWithoutExtension(assetPath);
        return f.Substring(f.IndexOf('@') + 1);
    }

    public static void Run()
    {
        string repo = GoblinImport.RepoRoot();
        string outDir = Path.Combine(repo, "unity", "AvatarCheck");
        List<string> current = new List<string>();
        var warnings = new List<string>();
        void Capture(string c, string st, LogType t)
        {
            if (t == LogType.Error || t == LogType.Exception || t == LogType.Assert) current.Add($"log [{t}] {c}");
            else if (t == LogType.Warning && warnings.Count < 100) warnings.Add(c);
        }

        int failed = 0, written = 0;
        var importErrors = new List<string>();
        JObj resaved = null;
        List<string> clipPaths = new List<string>();
        string only = ClipArg();
        string oldReport = Path.Combine(outDir, "goblin_clip_report.json");
        bool oldDeleted = false;

        Application.logMessageReceived += Capture;
        try
        {
            if (File.Exists(oldReport)) { File.Delete(oldReport); oldDeleted = true; }
            current = importErrors;
            foreach (var p in new[] { GoblinImport.ModelPath, GoblinImport.ClubPath, PrefabPath })
                if (!File.Exists(GoblinImport.FullPath(p))) throw new Exception($"missing asset: {p}");
            resaved = GoblinImport.EnsureImported(true);
            clipPaths = GoblinImport.ClipPaths().Where(p => ClipNameOf(p) != ExcludedClip)
                .Where(p => only == null || ClipNameOf(p) == only).ToList();
            if (clipPaths.Count == 0)
                importErrors.Add(only == null ? "no goblin@*.fbx clips besides rigtest" : $"clip {only} not found");
        }
        catch (Exception ex)
        {
            importErrors.Add("exception: " + ex);
        }

        foreach (var clipPath in clipPaths)
        {
            var errors = new List<string>(importErrors);
            warnings.Clear();
            current = errors;
            var extra = new JObj {
                { "clip_path", clipPath }, { "clips_checked_this_run", clipPaths.Select(p => (object)p).ToList() },
                { "clip_filter_arg", only }, { "old_report_deleted", oldDeleted }, { "import_resaved", resaved } };
            var report = CheckClip(clipPath, ClipNameOf(clipPath), errors, extra);
            extra.Add("log_warnings", warnings.ToList());
            report.Add("errors", errors);
            report.Add("extra", extra);
            string outPath = Path.Combine(outDir, $"goblin_clip_report_{ClipNameOf(clipPath)}.json");
            GoblinImport.WriteJson(outPath, report);
            written++;
            if (errors.Count > 0) failed++;
            Debug.Log($"GOBLIN CLIP CHECK written: {outPath} errors={errors.Count}");
            foreach (var e in errors) Debug.Log($"GOBLIN CLIP CHECK error [{ClipNameOf(clipPath)}]: " + e);
        }
        Application.logMessageReceived -= Capture;
        if (clipPaths.Count == 0)
            foreach (var e in importErrors) Debug.Log("GOBLIN CLIP CHECK error: " + e);
        Debug.Log($"GOBLIN CLIP CHECK done: reports {written}, with errors {failed}, old goblin_clip_report.json deleted {oldDeleted}");
        EditorApplication.Exit(written > 0 && failed == 0 && importErrors.Count == 0 ? 0 : 1);
    }

    static JObj CheckClip(string clipPath, string clipName, List<string> errors, JObj extra)
    {
        var report = new JObj();
        GameObject inst = null, clubGo = null;
        Mesh baked = null, mouthBaked = null;
        try
        {
            report.Add("unity_version", Application.unityVersion);
            if (GoblinImport.Classify(clipPath) != GoblinImport.Kind.Clip)
                errors.Add($"{clipPath} is not classified as a clip by GoblinImport ({GoblinImport.Classify(clipPath)})");

            // --- 1. clip + importer settings -------------------------------------------------------
            var clip = AssetDatabase.LoadAllAssetsAtPath(clipPath).OfType<AnimationClip>()
                .FirstOrDefault(c => !c.name.StartsWith("__preview__"));
            if (clip == null) throw new Exception($"no AnimationClip in {clipPath}");
            var mi = (ModelImporter)AssetImporter.GetAtPath(clipPath);
            var settings = mi.clipAnimations.Length > 0 ? mi.clipAnimations : mi.defaultClipAnimations;
            var cs = settings.FirstOrDefault(c => c.name == clip.name) ?? settings.FirstOrDefault();
            if (cs == null) throw new Exception($"no clip settings in {clipPath}");
            double firstFrame = cs.firstFrame, lastFrame = cs.lastFrame;
            report.Add("clip", new JObj {
                { "name", clip.name }, { "length_s", clip.length }, { "frameRate", clip.frameRate },
                { "firstFrame", firstFrame }, { "lastFrame", lastFrame },
                { "hasRootCurves", clip.hasRootCurves }, { "hasMotionCurves", clip.hasMotionCurves },
                { "hasGenericRootTransform", clip.hasGenericRootTransform } });
            if (clip.name != clipName) errors.Add($"clip name {clip.name} != {clipName}");
            if (Math.Abs(clip.frameRate - Fps) > 1e-3) errors.Add($"clip frameRate {clip.frameRate} != 24");
            var bindings = AnimationUtility.GetCurveBindings(clip);
            extra.Add("clip_detail", new JObj {
                { "takeName", cs.takeName }, { "clipSettingsSource", mi.clipAnimations.Length > 0 ? "clipAnimations" : "defaultClipAnimations" },
                { "loopTime", cs.loopTime }, { "legacy", clip.legacy },
                { "hasMotionFloatCurves", clip.hasMotionFloatCurves },
                { "time_mapping", "t = (f - firstFrame) / 24" },
                { "curve_bindings", bindings.Length },
                { "curve_paths", bindings.Select(b => b.path).Distinct().Count() },
                { "root_node_bindings", bindings.Where(b => b.path == GoblinImport.MotionNodePath)
                    .Select(b => (object)b.propertyName).ToList() },
                { "non_transform_bindings", bindings.Where(b => b.type != typeof(Transform))
                    .Select(b => (object)$"{b.path}:{b.type.Name}.{b.propertyName}").Take(50).ToList() } });

            report.Add("importer", new JObj {
                { "animationType", mi.animationType.ToString() },
                { "avatarSetup", mi.avatarSetup.ToString() },
                { "sourceAvatar", mi.sourceAvatar == null ? null : new JObj {
                    { "name", mi.sourceAvatar.name }, { "assetPath", AssetDatabase.GetAssetPath(mi.sourceAvatar) } } },
                { "animationCompression", mi.animationCompression.ToString() },
                { "resampleCurves", mi.resampleCurves } });
            extra.Add("importer_describe", GoblinImport.Describe(clipPath));

            // --- 2. goblin.prefab instance (rest recorded before any sampling) + club ------------------
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var prefabAsset = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            if (prefabAsset == null) throw new Exception($"{PrefabPath} not loadable");
            inst = (GameObject)PrefabUtility.InstantiatePrefab(prefabAsset);
            var all = inst.GetComponentsInChildren<Transform>(true);
            var bone = new Dictionary<string, Transform>();
            foreach (var name in Bones)
            {
                var hits = all.Where(t => t != inst.transform && t.name == name).ToList();
                if (hits.Count != 1) errors.Add($"bone {name}: {hits.Count} transforms in goblin.prefab instance");
                if (hits.Count >= 1) bone[name] = hits[0];
            }
            var rest = new JObj();
            foreach (var name in Bones.Where(bone.ContainsKey))
                rest.Add(name, new JObj { { "pos", bone[name].position }, { "rot", bone[name].rotation } });
            report.Add("rest", rest);
            if (!bone.TryGetValue("root", out var rootBone)) throw new Exception("root bone missing");
            if (!bone.TryGetValue("weapon_socket_r", out var socket)) throw new Exception("weapon_socket_r missing");
            var smrs = inst.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            if (smrs.Length != 1) errors.Add($"goblin.prefab instance has {smrs.Length} SkinnedMeshRenderers (expected 1)");
            var smr = smrs.FirstOrDefault();

            var clubPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(GoblinImport.ClubPath);
            clubGo = Object.Instantiate(clubPrefab);
            clubGo.name = clubPrefab.name;
            clubGo.transform.SetParent(socket, false);
            clubGo.transform.localPosition = Vector3.zero;
            clubGo.transform.localRotation = Quaternion.identity;
            clubGo.transform.localScale = Vector3.one;
            extra.Add("instance", new JObj {
                { "source", PrefabPath }, { "instance_root_name", inst.name },
                { "rootBone_path", GoblinImport.PathOf(rootBone, inst.transform) },
                { "rest", "world pos/rot of a fresh goblin.prefab instance before any SampleAnimation" },
                { "club", "goblin_club.fbx instance under weapon_socket_r: localPosition 0, localRotation identity, localScale 1" } });

            // --- HR2: blend shape mouth_open (mesh, curve binding, mouth vertices) ----------------------
            int mouthIdx = -1;
            int[] mouthVerts = new int[0];
            var bsNames = new List<object>();
            if (smr != null && smr.sharedMesh != null)
            {
                var sm = smr.sharedMesh;
                for (int i = 0; i < sm.blendShapeCount; i++) bsNames.Add(sm.GetBlendShapeName(i));
                mouthIdx = sm.GetBlendShapeIndex(MouthShape);
                if (mouthIdx >= 0)
                {
                    int nfr = sm.GetBlendShapeFrameCount(mouthIdx);
                    var dv = new Vector3[sm.vertexCount];
                    var dn = new Vector3[sm.vertexCount];
                    var dt = new Vector3[sm.vertexCount];
                    sm.GetBlendShapeFrameVertices(mouthIdx, nfr - 1, dv, dn, dt);
                    mouthVerts = Enumerable.Range(0, dv.Length).Where(i => dv[i].magnitude > MouthDeltaMin).ToArray();
                }
            }
            var mouthBindings = bindings.Where(b => b.type == typeof(SkinnedMeshRenderer)
                                                    && b.propertyName == "blendShape." + MouthShape).ToList();
            AnimationCurve mouthCurve = mouthBindings.Count > 0 ? AnimationUtility.GetEditorCurve(clip, mouthBindings[0]) : null;
            report.Add("blend_shapes", new JObj {
                { "smr_path", smr == null ? null : GoblinImport.PathOf(smr.transform, inst.transform) },
                { "names", bsNames }, { "count", bsNames.Count }, { "mouth_open_index", mouthIdx },
                { "curve_bindings", mouthBindings.Select(b => (object)$"{b.path}:{b.type.Name}.{b.propertyName}").ToList() },
                { "curve_keys", mouthCurve == null ? 0 : mouthCurve.length },
                { "mouth_delta_min_m", MouthDeltaMin }, { "n_mouth_vertices", mouthVerts.Length },
                { "mouth_vertex_indices", mouthVerts.Select(i => (object)i).ToList() },
                { "rendererLossyScale", smr == null ? Vector3.zero : smr.transform.lossyScale },
                { "note", "weight = GetBlendShapeWeight after clip.SampleAnimation; curve = editor curve at t (null: no curve, counts as 0); bake_world = BakeMesh(useScale true) vertex at mouth_vertex_indices, world = TRS(renderer position, rotation, 1) * v, only when weight > 0" } });

            // --- 3. samples: every integer frame firstFrame..lastFrame ------------------------------
            int fs = (int)Math.Ceiling(firstFrame - 1e-6), fe = (int)Math.Floor(lastFrame + 1e-6);
            var samples = new List<object>();
            for (int f = fs; f <= fe; f++)
            {
                double t = (f - firstFrame) / Fps;
                clip.SampleAnimation(inst, (float)t);
                var bones = new JObj();
                foreach (var name in Bones.Where(bone.ContainsKey))
                    bones.Add(name, new JObj { { "pos", bone[name].position }, { "rot", bone[name].rotation } });
                var sample = new JObj {
                    { "frame", f }, { "time_s", t }, { "bones", bones },
                    { "club", new JObj { { "pos", clubGo.transform.position }, { "rot", clubGo.transform.rotation } } },
                    { "root_local", new JObj { { "pos", rootBone.localPosition }, { "rot", rootBone.localRotation } } },
                    { "instance_root", new JObj { { "pos", inst.transform.position }, { "rot", inst.transform.rotation } } } };
                if (smr != null && mouthIdx >= 0)
                {
                    double w = smr.GetBlendShapeWeight(mouthIdx);
                    var bs = new JObj { { "weight", w }, { "curve", mouthCurve == null ? (object)null : (double)mouthCurve.Evaluate((float)t) } };
                    if (w > 0 && mouthVerts.Length > 0)
                    {
                        if (mouthBaked == null) mouthBaked = new Mesh();
                        smr.BakeMesh(mouthBaked, true);
                        var bv = mouthBaked.vertices;
                        var toWorld = Matrix4x4.TRS(smr.transform.position, smr.transform.rotation, Vector3.one);
                        bs.Add("bake_world", mouthVerts.Select(i => (object)toWorld.MultiplyPoint3x4(bv[i])).ToList());
                    }
                    sample.Add("blend_shape", bs);
                }
                samples.Add(sample);
            }
            report.Add("samples", samples);

            // --- 4. bounds: BakeMesh AABB (rootBone-local) vs goblin.prefab SMR localBounds ----------
            if (smr != null)
            {
                var lb = smr.localBounds;
                var space = smr.rootBone != null ? smr.rootBone : smr.transform;
                Vector3 bmin = lb.min, bmax = lb.max;
                baked = new Mesh();
                int checkedN = 0, worstFrame = -1;
                double maxOut = 0.0, worstSigned = double.NegativeInfinity;
                var perFrame = new List<object>();
                Vector3 unionMin = Vector3.positiveInfinity, unionMax = Vector3.negativeInfinity;
                for (int f = fs; f <= fe; f++)
                {
                    clip.SampleAnimation(inst, (float)((f - firstFrame) / Fps));
                    smr.BakeMesh(baked, true);
                    var verts = baked.vertices;
                    if (verts.Length == 0) { errors.Add($"BakeMesh returned 0 vertices at frame {f}"); break; }
                    var toSpace = space.worldToLocalMatrix * smr.transform.localToWorldMatrix;
                    Vector3 mn = Vector3.positiveInfinity, mx = Vector3.negativeInfinity;
                    foreach (var v in verts)
                    {
                        var p = toSpace.MultiplyPoint3x4(v);
                        mn = Vector3.Min(mn, p);
                        mx = Vector3.Max(mx, p);
                    }
                    unionMin = Vector3.Min(unionMin, mn);
                    unionMax = Vector3.Max(unionMax, mx);
                    double signed = double.NegativeInfinity;
                    for (int a = 0; a < 3; a++)
                        signed = Math.Max(signed, Math.Max(mx[a] - bmax[a], bmin[a] - mn[a]));
                    if (signed > worstSigned) { worstSigned = signed; worstFrame = f; }
                    if (signed > 0) { perFrame.Add(new double[] { f, signed }); maxOut = Math.Max(maxOut, signed); }
                    checkedN++;
                }
                report.Add("bounds", new JObj {
                    { "prefab", PrefabPath },
                    { "localBounds", new JObj { { "center", lb.center }, { "extents", lb.extents } } },
                    { "frames_checked", checkedN },
                    { "max_outside_m", maxOut },
                    { "per_frame_outside", perFrame },
                    { "rootBone", smr.rootBone == null ? null : smr.rootBone.name },
                    { "worst_frame", worstFrame },
                    { "worst_signed_m", worstSigned },
                    { "union_baked_min", unionMin }, { "union_baked_max", unionMax } });
                extra.Add("bounds_detail", new JObj {
                    { "space", smr.rootBone != null ? "rootBone local (localBounds space)" : "renderer local" },
                    { "note", "signed = max over axes of (bakedMax - boundsMax, boundsMin - bakedMin); < 0 means margin; bounds not widened" },
                    { "updateWhenOffscreen", smr.updateWhenOffscreen },
                    { "vertexCount", smr.sharedMesh.vertexCount } });
            }
            else errors.Add("no SkinnedMeshRenderer; bounds skipped");
        }
        catch (Exception ex)
        {
            errors.Add("exception: " + ex);
        }
        finally
        {
            if (inst != null) Object.DestroyImmediate(inst);
            if (clubGo != null) Object.DestroyImmediate(clubGo);
            if (baked != null) Object.DestroyImmediate(baked);
            if (mouthBaked != null) Object.DestroyImmediate(mouthBaked);
            try { EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single); } catch (Exception) { }
        }
        return report;
    }
}
