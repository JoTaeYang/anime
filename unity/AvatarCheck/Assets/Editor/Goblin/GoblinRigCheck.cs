using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;

// G9 (T38): data collection for G9.1-G9.10. Writes unity/AvatarCheck/goblin_report.json in the schema of
// work/goblin_swing/rig/data/g9_contract.md (+ an "extra" object with diagnostics). Records only; no judgement.
// G9.11 (2026-09-26 HR2, rig/data/hr2_contract.md): "mesh_contract" = blend shapes (names, count, frames) and
// submesh / material count and names of the goblin.fbx SkinnedMeshRenderer.
// Exit 0 when `errors` is empty, otherwise 1 (an exception is recorded in `errors`).
public static class GoblinRigCheck
{
    // canonical_skeleton.json TREE order
    static readonly string[] Bones = {
        "root", "pelvis", "spine_01", "spine_02", "head",
        "shoulder_l", "upperarm_l", "upperarm_twist_l", "lowerarm_l", "lowerarm_twist_l", "hand_l",
        "shoulder_r", "upperarm_r", "upperarm_twist_r", "lowerarm_r", "lowerarm_twist_r", "hand_r", "weapon_socket_r",
        "thigh_l", "calf_l", "foot_l", "thigh_r", "calf_r", "foot_r",
    };
    // non-bone nodes expected under the goblin.fbx prefab root (FBX root nodes)
    static readonly string[] ExpectedNodes = { "goblin", "goblin_mesh" };
    const double Fps = 24.0;
    const int RenderHeightDefault = 1024;
    const float RenderGray = 0.35f;
    const float BoundsMargin = 0.10f;
    const string PrefabPath = GoblinImport.Folder + "/goblin.prefab";
    const float LitGray = 0.8f;
    const float LitSmoothness = 0.2f;
    const float LitAmbient = 0.3f;
    // light rotation in camera-local Euler degrees: pitch 30 down, yaw 30 toward camera +X (light travels
    // right / down / away from the camera, i.e. it comes from the camera's front-upper-left)
    static readonly Vector3 LitLightEuler = new Vector3(30f, 30f, 0f);
    const string LitPattern = "rig/inspect/G9/unity_lit_<cam>_<frame>.png";

    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static void Run()
    {
        var errors = new List<string>();
        var logWarnings = new List<string>();
        void Capture(string c, string st, LogType t)
        {
            if (t == LogType.Error || t == LogType.Exception || t == LogType.Assert) errors.Add($"log [{t}] {c}");
            else if (t == LogType.Warning && logWarnings.Count < 100) logWarnings.Add(c);
        }

        string repo = GoblinImport.RepoRoot();
        string rig = Path.Combine(repo, "work", "goblin_swing", "rig");
        string outPath = Path.Combine(repo, "unity", "AvatarCheck", "goblin_report.json");
        var report = new JObj();
        var extra = new JObj();
        GameObject inst = null, clubGo = null, camGo = null, lightGo = null;
        Mesh baked = null;
        Material mat = null;
        var renders = new List<object>();
        bool rendersAdded = false;

        Application.logMessageReceived += Capture;
        try
        {
            report.Add("unity_version", Application.unityVersion);
            report.Add("fps", 24);
            foreach (var p in new[] { GoblinImport.ModelPath, GoblinImport.RigtestPath, GoblinImport.ClubPath })
                if (!File.Exists(GoblinImport.FullPath(p))) throw new Exception($"no goblin assets: {p} missing");

            // --- 1. re-import + importer settings (G9.1) ---------------------------------------------
            extra.Add("import_resaved", GoblinImport.EnsureImported(true));

            var clip = AssetDatabase.LoadAllAssetsAtPath(GoblinImport.RigtestPath).OfType<AnimationClip>()
                .FirstOrDefault(c => !c.name.StartsWith("__preview__"));
            if (clip == null) throw new Exception($"no AnimationClip in {GoblinImport.RigtestPath}");
            var clipImporter = (ModelImporter)AssetImporter.GetAtPath(GoblinImport.RigtestPath);
            var clipSettings = clipImporter.clipAnimations.Length > 0 ? clipImporter.clipAnimations : clipImporter.defaultClipAnimations;
            var cs = clipSettings.FirstOrDefault(c => c.name == clip.name) ?? clipSettings.FirstOrDefault();
            double firstFrame = 1.0;
            if (cs == null) errors.Add("no clip settings for rigtest; firstFrame assumed 1");
            else firstFrame = cs.firstFrame;
            report.Add("clip", new JObj {
                { "name", clip.name }, { "length_s", clip.length }, { "frameRate", clip.frameRate }, { "firstFrame", firstFrame } });
            if (Math.Abs(clip.frameRate - Fps) > 1e-3) errors.Add($"clip frameRate {clip.frameRate} != 24");
            var bindings = AnimationUtility.GetCurveBindings(clip);
            extra.Add("clip_detail", new JObj {
                { "lastFrame", cs == null ? (object)null : cs.lastFrame },
                { "takeName", cs == null ? null : cs.takeName },
                { "time_mapping", "t = (f - firstFrame) / 24 (contract t = (f - 1) / 24 when firstFrame = 1)" },
                { "legacy", clip.legacy },
                { "hasGenericRootTransform", clip.hasGenericRootTransform },
                { "hasMotionCurves", clip.hasMotionCurves },
                { "hasRootCurves", clip.hasRootCurves },
                { "hasMotionFloatCurves", clip.hasMotionFloatCurves },
                { "curve_bindings", bindings.Length },
                { "curve_paths", bindings.Select(b => b.path).Distinct().Count() },
                { "root_node_bindings", bindings.Where(b => b.path == GoblinImport.MotionNodePath)
                    .Select(b => (object)b.propertyName).ToList() },
                { "non_transform_bindings", bindings.Where(b => b.type != typeof(Transform))
                    .Select(b => (object)$"{b.path}:{b.type.Name}.{b.propertyName}").Take(50).ToList() },
            });

            var importers = new JObj();
            foreach (var p in new[] { GoblinImport.ModelPath, GoblinImport.RigtestPath, GoblinImport.ClubPath })
                importers.Add(Path.GetFileName(p), GoblinImport.Describe(p));
            report.Add("importers", importers);

            // --- 2. instance, hierarchy (G9.2) -------------------------------------------------------
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(GoblinImport.ModelPath);
            inst = Object.Instantiate(prefab);
            inst.name = prefab.name;
            var all = inst.GetComponentsInChildren<Transform>(true);
            var allowed = new HashSet<string>(Bones.Concat(ExpectedNodes));
            var hierarchy = new List<object>();
            var unexpected = new List<object>();
            foreach (var t in all)
            {
                bool isRoot = t == inst.transform;
                string path = isRoot ? "" : GoblinImport.PathOf(t, inst.transform);
                hierarchy.Add(new JObj {
                    { "path", path }, { "name", t.name }, { "parent", t.parent == null ? null : t.parent.name },
                    { "localPosition", t.localPosition }, { "localRotation", t.localRotation },
                    { "localScale", t.localScale }, { "lossyScale", t.lossyScale },
                    { "components", t.GetComponents<Component>().Where(c => c != null && !(c is Transform))
                        .Select(c => (object)c.GetType().Name).ToList() } });
                if (!isRoot && !allowed.Contains(t.name)) unexpected.Add(path);
            }
            report.Add("hierarchy", hierarchy);
            report.Add("unexpected_nodes", unexpected);

            var bone = new Dictionary<string, Transform>();
            foreach (var name in Bones)
            {
                var hits = all.Where(t => t != inst.transform && t.name == name).ToList();
                if (hits.Count != 1) errors.Add($"bone {name}: {hits.Count} transforms in goblin.fbx instance");
                if (hits.Count >= 1) bone[name] = hits[0];
            }

            // --- 3. rest (G9.3) ----------------------------------------------------------------------
            var rest = new JObj();
            foreach (var name in Bones.Where(bone.ContainsKey))
            {
                var t = bone[name];
                rest.Add(name, new JObj {
                    { "pos", t.position }, { "rot", t.rotation }, { "localPos", t.localPosition }, { "localRot", t.localRotation } });
            }
            report.Add("rest", rest);
            if (bone.ContainsKey("root") && bone.ContainsKey("head") && bone.ContainsKey("hand_r") && bone.ContainsKey("hand_l"))
                extra.Add("rest_vectors", new JObj {
                    { "root_to_head", bone["head"].position - bone["root"].position },
                    { "hand_r_to_hand_l", bone["hand_l"].position - bone["hand_r"].position } });

            // --- 4. bindposes (G9.9) -----------------------------------------------------------------
            var smrs = inst.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            if (smrs.Length != 1) errors.Add($"goblin.fbx instance has {smrs.Length} SkinnedMeshRenderers (expected 1)");
            var smr = smrs.FirstOrDefault();
            if (smr != null) report.Add("bindposes", Bindposes(smr, errors, "goblin.fbx"));
            if (smr != null) report.Add("mesh_contract", MeshContract(smr));
            var rigtestAsset = AssetDatabase.LoadAssetAtPath<GameObject>(GoblinImport.RigtestPath);
            var rigtestSmr = rigtestAsset == null ? null : rigtestAsset.GetComponentsInChildren<SkinnedMeshRenderer>(true).FirstOrDefault();
            if (rigtestSmr == null) errors.Add("goblin@rigtest.fbx has no SkinnedMeshRenderer");
            else extra.Add("bindposes_rigtest", Bindposes(rigtestSmr, errors, "goblin@rigtest.fbx"));

            // --- club at weapon_socket_r local 0 (G9.5) ----------------------------------------------
            var clubPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(GoblinImport.ClubPath);
            extra.Add("club_asset", new JObj {
                { "rootName", clubPrefab.name },
                { "rootLocalPosition", clubPrefab.transform.localPosition },
                { "rootLocalRotation", clubPrefab.transform.localRotation },
                { "rootLocalScale", clubPrefab.transform.localScale },
                { "meshNodes", clubPrefab.GetComponentsInChildren<MeshFilter>(true)
                    .Select(m => (object)(m.transform == clubPrefab.transform ? "" : GoblinImport.PathOf(m.transform, clubPrefab.transform))).ToList() },
                { "attach", "instance root parented to weapon_socket_r with localPosition 0, localRotation identity, localScale 1" } });
            clubGo = Object.Instantiate(clubPrefab);
            clubGo.name = clubPrefab.name;
            if (!bone.TryGetValue("weapon_socket_r", out var socket)) throw new Exception("weapon_socket_r missing");
            clubGo.transform.SetParent(socket, false);
            clubGo.transform.localPosition = Vector3.zero;
            clubGo.transform.localRotation = Quaternion.identity;
            clubGo.transform.localScale = Vector3.one;

            // --- 5/6. samples (G9.4/G9.5) and subsamples (G9.8) --------------------------------------
            var manifest = AsDict(GoblinJsonReader.Parse(File.ReadAllText(Path.Combine(rig, "data", "rigtest_manifest.json"))));
            int frameStart = (int)Num(Get(manifest, "frame_start") ?? 1.0);
            int frameEnd = (int)Num(Get(manifest, "frame_end") ?? 583.0);
            var sampleFrames = new SortedSet<int>();
            foreach (var segObj in AsList(Get(manifest, "segments")))
            {
                var seg = AsDict(segObj);
                int s = (int)Num(Get(seg, "start")), e = (int)Num(Get(seg, "end"));
                sampleFrames.Add(s);
                sampleFrames.Add((int)Math.Floor((s + e) / 2.0));
                sampleFrames.Add(e);
            }
            var subFrames = new List<double>();
            foreach (int f in sampleFrames)
                if (f + 1 <= frameEnd) { subFrames.Add(f + 0.25); subFrames.Add(f + 0.5); subFrames.Add(f + 0.75); }
            extra.Add("sample_frames", sampleFrames.Select(f => (object)f).ToList());

            JObj Sample(double f)
            {
                double t = (f - firstFrame) / Fps;
                clip.SampleAnimation(inst, (float)t);
                var bones = new JObj();
                foreach (var name in Bones.Where(bone.ContainsKey))
                    bones.Add(name, new JObj { { "pos", bone[name].position }, { "rot", bone[name].rotation } });
                return new JObj {
                    { "frame", f }, { "time_s", t }, { "bones", bones },
                    { "club", new JObj { { "pos", clubGo.transform.position }, { "rot", clubGo.transform.rotation } } } };
            }
            report.Add("samples", sampleFrames.Select(f => (object)Sample(f)).ToList());
            report.Add("subsamples", subFrames.Select(f => (object)Sample(f)).ToList());

            // --- 7. bounds (G9.10) ---------------------------------------------------------------------
            // (a) import bounds check: baked skin AABB (rootBone-local) vs the imported localBounds, frames 1..583
            // (b) applied bounds (T70b, A3.3c) = union of the baked AABBs over every integer frame
            //     (clip settings firstFrame..lastFrame) of every Assets/Goblin/goblin@*.fbx clip + BoundsMargin,
            //     saved as SMR localBounds in goblin.prefab (goblin.fbx instance; importer untouched;
            //     updateWhenOffscreen untouched)
            // (c) the reported check re-samples a goblin.prefab instance with rigtest and compares against its saved
            //     localBounds
            if (smr != null)
            {
                var lb = smr.localBounds;
                baked = new Mesh();
                var importCheck = BoundsPass(clip, inst, smr, lb, frameStart, frameEnd, firstFrame, baked, errors,
                                             out var rigtestUnionMin, out var rigtestUnionMax);
                var unionMin = Vector3.positiveInfinity;
                var unionMax = Vector3.negativeInfinity;
                var unionSources = new List<object>();
                int unionFrames = 0;
                foreach (var cp in GoblinImport.ClipPaths())
                {
                    var uc = AssetDatabase.LoadAllAssetsAtPath(cp).OfType<AnimationClip>()
                        .FirstOrDefault(c => !c.name.StartsWith("__preview__"));
                    if (uc == null) { errors.Add($"bounds union: no AnimationClip in {cp}"); continue; }
                    var um = (ModelImporter)AssetImporter.GetAtPath(cp);
                    var us = um.clipAnimations.Length > 0 ? um.clipAnimations : um.defaultClipAnimations;
                    var ucs = us.FirstOrDefault(c => c.name == uc.name) ?? us.FirstOrDefault();
                    if (ucs == null) { errors.Add($"bounds union: no clip settings in {cp}"); continue; }
                    int ufs = (int)Math.Ceiling(ucs.firstFrame - 1e-6), ufe = (int)Math.Floor(ucs.lastFrame + 1e-6);
                    var ur = BoundsPass(uc, inst, smr, lb, ufs, ufe, ucs.firstFrame, baked, errors, out var cMin, out var cMax);
                    unionMin = Vector3.Min(unionMin, cMin);
                    unionMax = Vector3.Max(unionMax, cMax);
                    unionFrames += ur.framesChecked;
                    unionSources.Add(new JObj {
                        { "path", cp }, { "clip", uc.name }, { "firstFrame", ucs.firstFrame }, { "lastFrame", ucs.lastFrame },
                        { "frames", ur.framesChecked }, { "baked_min", cMin }, { "baked_max", cMax } });
                }
                if (unionSources.Count == 0) throw new Exception("bounds union: no goblin@*.fbx clips");
                var applied = new Bounds();
                applied.SetMinMax(unionMin - Vector3.one * BoundsMargin, unionMax + Vector3.one * BoundsMargin);

                GameObject tmp = null, pinst = null;
                try
                {
                    tmp = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                    var tsmr = tmp.GetComponentInChildren<SkinnedMeshRenderer>(true);
                    tsmr.localBounds = applied;
                    PrefabUtility.SaveAsPrefabAsset(tmp, PrefabPath, out bool saved);
                    if (!saved) errors.Add($"SaveAsPrefabAsset failed: {PrefabPath}");
                    Object.DestroyImmediate(tmp);
                    tmp = null;

                    var prefabAsset = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
                    if (prefabAsset == null) throw new Exception($"{PrefabPath} not loadable after save");
                    pinst = (GameObject)PrefabUtility.InstantiatePrefab(prefabAsset);
                    var psmr = pinst.GetComponentInChildren<SkinnedMeshRenderer>(true);
                    var plb = psmr.localBounds;
                    var check = BoundsPass(clip, pinst, psmr, plb, frameStart, frameEnd, firstFrame, baked, errors, out _, out _);
                    report.Add("bounds", new JObj {
                        { "localBounds", new JObj { { "center", plb.center }, { "extents", plb.extents } } },
                        { "rootBone", psmr.rootBone == null ? null : psmr.rootBone.name },
                        { "frames_checked", check.framesChecked },
                        { "max_outside_m", check.maxOut },
                        { "worst_frame", check.worstFrame },
                        { "per_frame_outside", check.perFrame },
                        { "worst_signed_m", check.worstSigned },
                        { "applied_localBounds", new JObj {
                            { "center", applied.center }, { "extents", applied.extents },
                            { "min", applied.min }, { "max", applied.max },
                            { "margin_m", BoundsMargin },
                            { "union_baked_min", unionMin }, { "union_baked_max", unionMax },
                            { "union_rule", "every integer frame firstFrame..lastFrame of every Assets/Goblin/goblin@*.fbx clip, rootBone local" },
                            { "union_clips", unionSources.Count }, { "union_frames", unionFrames },
                            { "union_sources", unionSources },
                            { "rigtest_manifest_union_min", rigtestUnionMin }, { "rigtest_manifest_union_max", rigtestUnionMax } } },
                        { "prefab", new JObj {
                            { "path", PrefabPath }, { "saved", saved },
                            { "guid", AssetDatabase.AssetPathToGUID(PrefabPath) },
                            { "assetType", PrefabUtility.GetPrefabAssetType(prefabAsset).ToString() },
                            { "source", GoblinImport.ModelPath },
                            { "localBounds_read_back", new JObj { { "center", plb.center }, { "extents", plb.extents } } },
                            { "updateWhenOffscreen", psmr.updateWhenOffscreen } } },
                        { "import_bounds_check", new JObj {
                            { "localBounds", new JObj { { "center", lb.center }, { "extents", lb.extents } } },
                            { "rootBone", smr.rootBone == null ? null : smr.rootBone.name },
                            { "frames_checked", importCheck.framesChecked },
                            { "max_outside_m", importCheck.maxOut },
                            { "worst_frame", importCheck.worstFrame },
                            { "worst_signed_m", importCheck.worstSigned },
                            { "frames_outside", importCheck.perFrame.Count },
                            { "per_frame_outside", importCheck.perFrame } } } });
                }
                finally
                {
                    if (tmp != null) Object.DestroyImmediate(tmp);
                    if (pinst != null) Object.DestroyImmediate(pinst);
                }
                extra.Add("bounds_detail", new JObj {
                    { "space", smr.rootBone != null ? "rootBone local (localBounds space)" : "renderer local" },
                    { "note", "signed = max over axes of (bakedMax - boundsMax, boundsMin - bakedMin); < 0 means margin" },
                    { "updateWhenOffscreen", smr.updateWhenOffscreen },
                    { "rendererLossyScale", smr.transform.lossyScale },
                    { "vertexCount", smr.sharedMesh.vertexCount } });
            }

            // --- 8. renders (G9.6/G9.7) --------------------------------------------------------------
            // Per frame: SampleAnimation -> SMR.BakeMesh -> temporary MeshFilter/MeshRenderer at the SMR world transform
            // (the SMR itself is disabled). Body and club: flat unlit gray 0.35. Background white with alpha 0 (RGBA PNG).
            string camsPath = Path.Combine(rig, "data", "compare_cams.json");
            report.Add("renders", renders);
            rendersAdded = true;
            if (!File.Exists(camsPath)) errors.Add("compare_cams.json missing");
            else if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null) errors.Add("no graphics device; renders skipped");
            else if (smr == null) errors.Add("no SkinnedMeshRenderer; renders skipped");
            else
            {
                var shader = Shader.Find("Unlit/Color");
                if (shader == null) throw new Exception("shader Unlit/Color not found");
                mat = new Material(shader) { color = new Color(RenderGray, RenderGray, RenderGray, 1f) };
                GameObject bakeGo = null;
                Mesh renderMesh = null;
                Material litMat = null;
                try
                {
                    renderMesh = new Mesh { name = "goblin_tmp_baked" };
                    bakeGo = new GameObject("goblin_tmp_baked");
                    var bakeFilter = bakeGo.AddComponent<MeshFilter>();
                    bakeFilter.sharedMesh = renderMesh;
                    var bakeRenderer = bakeGo.AddComponent<MeshRenderer>();
                    // HR2: one material per submesh (goblin_mesh has 4 slots: skin, mouth_inner, teeth, tongue);
                    // a single sharedMaterial draws only submesh 0 and leaves the mouth see-through (G9.6, 2026-09-26).
                    bakeRenderer.sharedMaterials = Enumerable.Repeat(mat, Math.Max(1, smr.sharedMesh.subMeshCount)).ToArray();
                    smr.enabled = false;
                    foreach (var r in clubGo.GetComponentsInChildren<Renderer>(true))
                        r.sharedMaterials = Enumerable.Repeat(mat, Math.Max(1, r.sharedMaterials.Length)).ToArray();
                    lightGo = new GameObject("goblin_tmp_light");
                    var light = lightGo.AddComponent<Light>();
                    light.type = LightType.Directional;
                    light.intensity = 1f;
                    light.color = Color.white;
                    light.shadows = LightShadows.None;
                    camGo = new GameObject("goblin_tmp_cam");
                    var cam = camGo.AddComponent<Camera>();
                    cam.enabled = false;
                    cam.orthographic = true;
                    cam.clearFlags = CameraClearFlags.SolidColor;
                    cam.backgroundColor = new Color(1f, 1f, 1f, 0f);
                    extra.Add("render_setup", new JObj {
                        { "body", "SkinnedMeshRenderer.BakeMesh(useScale true) per frame -> temporary MeshRenderer at the SMR world transform; SMR disabled" },
                        { "club", "goblin_club instance under weapon_socket_r (unchanged)" },
                        { "shader", shader.name }, { "color", new[] { RenderGray, RenderGray, RenderGray } },
                        { "background", "white, alpha 0" }, { "png", "RGBA32 (alpha: object 1, background 0)" },
                        { "renderTexture", "ARGB32 sRGB, depth 24" }, { "antiAliasing", 1 },
                        { "renderPipeline", GraphicsSettings.currentRenderPipeline == null ? "built-in" : GraphicsSettings.currentRenderPipeline.name } });
                    // lit pass (G9.7): Standard, gray LitGray, smoothness LitSmoothness, one directional light
                    // (camera rotation * LitLightEuler) + flat ambient LitAmbient, no environment reflections.
                    var litShader = Shader.Find("Standard");
                    if (litShader == null) throw new Exception("shader Standard not found");
                    litMat = new Material(litShader) { color = new Color(LitGray, LitGray, LitGray, 1f) };
                    litMat.SetFloat("_Glossiness", LitSmoothness);
                    litMat.SetFloat("_Metallic", 0f);
                    RenderSettings.ambientMode = AmbientMode.Flat;
                    RenderSettings.ambientLight = new Color(LitAmbient, LitAmbient, LitAmbient, 1f);
                    RenderSettings.skybox = null;
                    RenderSettings.reflectionIntensity = 0f;
                    var swapRenderers = new List<Renderer> { bakeRenderer };
                    swapRenderers.AddRange(clubGo.GetComponentsInChildren<Renderer>(true));
                    var rendersLit = new List<object>();
                    var litCams = new JObj();
                    extra.Add("render_cams", RenderCams(File.ReadAllText(camsPath), rig, clip, inst, firstFrame, cam, light,
                                                        smr, bakeFilter, renders, errors,
                                                        swapRenderers, mat, litMat, rendersLit, litCams));
                    var lastBaked = bakeFilter.sharedMesh;
                    extra.Add("render_lit", new JObj {
                        { "files", "rig/inspect/G9/unity_lit_<cam>_<frame>.png (same cameras / frames as the silhouette renders)" },
                        { "mesh", "same per-frame BakeMesh result as the silhouette pass (imported normals kept); club same material" },
                        { "baked_vertexCount", lastBaked.vertexCount }, { "baked_normals", lastBaked.normals.Length },
                        { "shader", litShader.name }, { "color", new[] { LitGray, LitGray, LitGray } },
                        { "smoothness", LitSmoothness }, { "metallic", 0f },
                        { "ambient", new JObj { { "mode", "Flat" }, { "color", new[] { LitAmbient, LitAmbient, LitAmbient } } } },
                        { "reflectionIntensity", 0f }, { "skybox", null },
                        { "light", new JObj {
                            { "type", "Directional" }, { "intensity", light.intensity }, { "color", new[] { 1f, 1f, 1f } },
                            { "shadows", light.shadows.ToString() },
                            { "rotation_rule", "camera rotation * Euler(LitLightEuler), Euler in camera local axes (x right, y up, z forward)" },
                            { "euler_camera_local_deg", LitLightEuler },
                            { "note", "forward = direction the light travels (Unity world); to_light = -forward" } } },
                        { "per_camera", litCams },
                        { "background", "white, alpha 1" }, { "png", "RGBA32" },
                        { "colorSpace", QualitySettings.activeColorSpace.ToString() },
                        { "renderPipeline", GraphicsSettings.currentRenderPipeline == null ? "built-in" : GraphicsSettings.currentRenderPipeline.name } });
                    extra.Add("renders_lit", rendersLit);
                }
                finally
                {
                    if (smr != null) smr.enabled = true;
                    if (bakeGo != null) Object.DestroyImmediate(bakeGo);
                    if (renderMesh != null) Object.DestroyImmediate(renderMesh);
                    if (litMat != null) Object.DestroyImmediate(litMat);
                }
            }
        }
        catch (Exception ex)
        {
            errors.Add("exception: " + ex);
        }
        finally
        {
            Application.logMessageReceived -= Capture;
            RenderTexture.active = null;
            if (inst != null) Object.DestroyImmediate(inst);
            if (clubGo != null) Object.DestroyImmediate(clubGo);
            if (camGo != null) Object.DestroyImmediate(camGo);
            if (lightGo != null) Object.DestroyImmediate(lightGo);
            if (baked != null) Object.DestroyImmediate(baked);
            if (mat != null) Object.DestroyImmediate(mat);
            try { EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single); } catch (Exception) { }
        }

        if (!rendersAdded) report.Add("renders", renders);
        extra.Add("log_warnings", logWarnings);
        report.Add("extra", extra);
        report.Add("errors", errors);
        GoblinImport.WriteJson(outPath, report);
        Debug.Log($"GOBLIN RIG CHECK written: {outPath} errors={errors.Count}");
        foreach (var e in errors) Debug.Log("GOBLIN RIG CHECK error: " + e);
        EditorApplication.Exit(errors.Count == 0 ? 0 : 1);
    }

    static JObj Bindposes(SkinnedMeshRenderer smr, List<string> errors, string label)
    {
        var o = new JObj();
        var bp = smr.sharedMesh.bindposes;
        var bs = smr.bones;
        if (bp.Length != bs.Length) errors.Add($"{label}: bindposes {bp.Length} != bones {bs.Length}");
        for (int i = 0; i < bs.Length; i++)
            o.Add(bs[i] == null ? $"<null bone {i}>" : bs[i].name, i < bp.Length ? (object)bp[i] : null);
        return o;
    }

    // G9.11 (HR2): blend shapes and submeshes / materials of a SkinnedMeshRenderer (records only).
    static JObj MeshContract(SkinnedMeshRenderer smr)
    {
        var m = smr.sharedMesh;
        if (m == null) return new JObj { { "source", "goblin.fbx instance SkinnedMeshRenderer" }, { "mesh", null } };
        var names = new List<object>();
        var frames = new List<object>();
        double mouthMax = -1.0;
        for (int i = 0; i < m.blendShapeCount; i++)
        {
            string n = m.GetBlendShapeName(i);
            int nf = m.GetBlendShapeFrameCount(i);
            names.Add(n);
            frames.Add(nf);
            if (n == "mouth_open" && nf > 0)
            {
                var dv = new Vector3[m.vertexCount];
                var dn = new Vector3[m.vertexCount];
                var dt = new Vector3[m.vertexCount];
                m.GetBlendShapeFrameVertices(i, nf - 1, dv, dn, dt);
                mouthMax = dv.Length == 0 ? 0.0 : dv.Max(d => (double)d.magnitude);
            }
        }
        var mats = smr.sharedMaterials;
        return new JObj {
            { "source", "goblin.fbx instance SkinnedMeshRenderer (sharedMesh, sharedMaterials)" },
            { "mesh", m.name }, { "vertexCount", m.vertexCount },
            { "blendShapeCount", m.blendShapeCount }, { "blendShapes", names }, { "blendShapeFrameCounts", frames },
            { "mouth_open_max_delta_m", mouthMax < 0 ? (object)null : mouthMax },
            { "subMeshCount", m.subMeshCount }, { "materialCount", mats.Length },
            { "materials", mats.Select(x => (object)(x == null ? null : x.name)).ToList() } };
    }

    class BoundsResult
    {
        public int framesChecked, worstFrame = -1;
        public double maxOut, worstSigned = double.NegativeInfinity;
        public List<object> perFrame = new List<object>();
    }

    // Samples frames fs..fe on `go`, bakes `r`, and compares the baked AABB (in localBounds space = rootBone local,
    // or renderer local without rootBone) with `lb`. Also returns the union of the per-frame AABBs in that space.
    static BoundsResult BoundsPass(AnimationClip clip, GameObject go, SkinnedMeshRenderer r, Bounds lb, int fs, int fe,
                                   double firstFrame, Mesh baked, List<string> errors, out Vector3 unionMin, out Vector3 unionMax)
    {
        var res = new BoundsResult();
        unionMin = Vector3.positiveInfinity;
        unionMax = Vector3.negativeInfinity;
        var space = r.rootBone != null ? r.rootBone : r.transform;
        Vector3 bmin = lb.min, bmax = lb.max;
        for (int f = fs; f <= fe; f++)
        {
            clip.SampleAnimation(go, (float)((f - firstFrame) / Fps));
            r.BakeMesh(baked, true);
            var verts = baked.vertices;
            if (verts.Length == 0) { errors.Add($"BakeMesh returned 0 vertices at frame {f}"); break; }
            // BakeMesh(useScale true) vertices are in renderer-local space -> world -> bounds space
            var toSpace = space.worldToLocalMatrix * r.transform.localToWorldMatrix;
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
            if (signed > res.worstSigned) { res.worstSigned = signed; res.worstFrame = f; }
            if (signed > 0) { res.perFrame.Add(new double[] { f, signed }); res.maxOut = Math.Max(res.maxOut, signed); }
            res.framesChecked++;
        }
        return res;
    }

    // compare_cams.json (s09_blender_ref.py), exact field names:
    //   frames[], png.unity ("rig/inspect/G9/unity_<cam>_<frame>.png", relative to work/goblin_swing, integer frame, no padding),
    //   cameras.<cam>.unity.{position[3], rotation[x,y,z,w], forward[3], up[3], orthographicSize, nearClipPlane, farClipPlane}.
    // A missing / malformed field is written to errors and that camera (or the whole render step) is skipped.
    // Output is 1024 x 1024 (contract: 1024 px height, square aspect). The camera uses `rotation`; forward/up are
    // recorded as their angle to rotation * (+Z / +Y) in the returned per-camera evidence.
    static JObj RenderCams(string json, string rig, AnimationClip clip, GameObject inst, double firstFrame,
                           Camera cam, Light light, SkinnedMeshRenderer smr, MeshFilter bakeFilter,
                           List<object> renders, List<string> errors,
                           List<Renderer> swapRenderers, Material silMat, Material litMat, List<object> rendersLit, JObj litCams)
    {
        var info = new JObj();
        var root = AsDict(GoblinJsonReader.Parse(json));
        if (root == null) { errors.Add("compare_cams.json: root is not an object"); return info; }

        object Req(Dictionary<string, object> d, string key, string where)
        {
            if (d != null && d.TryGetValue(key, out var v) && v != null) return v;
            errors.Add($"compare_cams.json: {where}{key} missing");
            return null;
        }
        double[] Vec(Dictionary<string, object> d, string key, string where, int n)
        {
            var v = Req(d, key, where);
            if (v == null) return null;
            var l = v as List<object>;
            if (l == null || l.Count != n || !l.All(x => x is double))
            { errors.Add($"compare_cams.json: {where}{key} is not a list of {n} numbers"); return null; }
            return l.Select(x => (double)x).ToArray();
        }
        double? Scalar(Dictionary<string, object> d, string key, string where)
        {
            var v = Req(d, key, where);
            if (v == null) return null;
            if (v is double x) return x;
            errors.Add($"compare_cams.json: {where}{key} is not a number");
            return null;
        }

        var framesObj = Req(root, "frames", "");
        var frames = framesObj as List<object>;
        if (framesObj != null && (frames == null || !frames.All(x => x is double d && Math.Abs(d - Math.Round(d)) < 1e-9)))
        { errors.Add("compare_cams.json: frames is not a list of integer frames"); frames = null; }
        var png = Req(root, "png", "");
        var pngD = AsDict(png);
        if (png != null && pngD == null) errors.Add("compare_cams.json: png is not an object");
        var patternObj = pngD == null ? null : Req(pngD, "unity", "png.");
        string pattern = patternObj as string;
        if (patternObj != null && (pattern == null || !pattern.Contains("<cam>") || !pattern.Contains("<frame>")))
        { errors.Add("compare_cams.json: png.unity is not a string with <cam> and <frame>"); pattern = null; }
        var camsObj = Req(root, "cameras", "");
        var cams = AsDict(camsObj);
        if (camsObj != null && cams == null) errors.Add("compare_cams.json: cameras is not an object");
        if (frames == null || pattern == null || cams == null) return info;

        string workDir = Path.GetFullPath(Path.Combine(rig, ".."));   // png.unity is relative to work/goblin_swing
        const int w = RenderHeightDefault, h = RenderHeightDefault;
        foreach (var kv in cams)
        {
            string name = kv.Key;
            string where = $"cameras.{name}.unity.";
            var camD = AsDict(kv.Value);
            if (camD == null) { errors.Add($"compare_cams.json: cameras.{name} is not an object"); continue; }
            var uObj = Req(camD, "unity", $"cameras.{name}.");
            var u = AsDict(uObj);
            if (uObj != null && u == null) errors.Add($"compare_cams.json: cameras.{name}.unity is not an object");
            if (u == null) continue;
            var pos = Vec(u, "position", where, 3);
            var rq = Vec(u, "rotation", where, 4);
            var fw = Vec(u, "forward", where, 3);
            var up = Vec(u, "up", where, 3);
            var size = Scalar(u, "orthographicSize", where);
            var near = Scalar(u, "nearClipPlane", where);
            var far = Scalar(u, "farClipPlane", where);
            if (pos == null || rq == null || fw == null || up == null || size == null || near == null || far == null) continue;

            var rot = new Quaternion((float)rq[0], (float)rq[1], (float)rq[2], (float)rq[3]);
            var fwV = new Vector3((float)fw[0], (float)fw[1], (float)fw[2]);
            var upV = new Vector3((float)up[0], (float)up[1], (float)up[2]);
            cam.transform.SetPositionAndRotation(new Vector3((float)pos[0], (float)pos[1], (float)pos[2]), rot);
            cam.orthographicSize = (float)size.Value;
            cam.nearClipPlane = (float)near.Value;
            cam.farClipPlane = (float)far.Value;
            cam.aspect = (float)w / h;
            light.transform.rotation = rot * Quaternion.Euler(LitLightEuler);
            var lightFwd = light.transform.forward;
            litCams.Add(name, new JObj {
                { "light_rotation", light.transform.rotation },
                { "light_forward", lightFwd }, { "to_light", -lightFwd },
                { "to_light_camera_local", Quaternion.Inverse(rot) * -lightFwd } });
            info.Add(name, new JObj {
                { "position", cam.transform.position }, { "rotation", cam.transform.rotation },
                { "rotation_norm", Math.Sqrt(rq.Sum(x => x * x)) },
                { "forward_vs_rotation_deg", Vector3.Angle(rot * Vector3.forward, fwV) },
                { "up_vs_rotation_deg", Vector3.Angle(rot * Vector3.up, upV) },
                { "orthographicSize", cam.orthographicSize }, { "nearClipPlane", cam.nearClipPlane },
                { "farClipPlane", cam.farClipPlane }, { "width", w }, { "height", h } });

            var rt = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 1 };
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
            try
            {
                foreach (var fo in frames)
                {
                    long f = (long)Math.Round((double)fo);
                    clip.SampleAnimation(inst, (float)((f - firstFrame) / Fps));
                    var bakedMesh = bakeFilter.sharedMesh;
                    smr.BakeMesh(bakedMesh, true);
                    bakedMesh.RecalculateBounds();
                    bakeFilter.transform.SetPositionAndRotation(smr.transform.position, smr.transform.rotation);
                    bakeFilter.transform.localScale = smr.transform.lossyScale;
                    cam.targetTexture = rt;
                    cam.Render();
                    RenderTexture.active = rt;
                    tex.ReadPixels(new Rect(0, 0, w, h), 0, 0);
                    tex.Apply();
                    RenderTexture.active = null;
                    cam.targetTexture = null;
                    string rel = pattern.Replace("<cam>", name).Replace("<frame>", f.ToString(Inv));
                    string full = Path.GetFullPath(Path.Combine(workDir, rel));
                    Directory.CreateDirectory(Path.GetDirectoryName(full));
                    File.WriteAllBytes(full, tex.EncodeToPNG());
                    renders.Add(rel);

                    // lit pass (G9.7): same bake / camera, lit material, opaque white background
                    var silBg = cam.backgroundColor;
                    try
                    {
                        foreach (var r in swapRenderers)
                            r.sharedMaterials = Enumerable.Repeat(litMat, Math.Max(1, r.sharedMaterials.Length)).ToArray();
                        cam.backgroundColor = new Color(1f, 1f, 1f, 1f);
                        cam.targetTexture = rt;
                        cam.Render();
                        RenderTexture.active = rt;
                        tex.ReadPixels(new Rect(0, 0, w, h), 0, 0);
                        tex.Apply();
                        RenderTexture.active = null;
                        cam.targetTexture = null;
                        string relLit = LitPattern.Replace("<cam>", name).Replace("<frame>", f.ToString(Inv));
                        string fullLit = Path.GetFullPath(Path.Combine(workDir, relLit));
                        Directory.CreateDirectory(Path.GetDirectoryName(fullLit));
                        File.WriteAllBytes(fullLit, tex.EncodeToPNG());
                        rendersLit.Add(relLit);
                    }
                    finally
                    {
                        foreach (var r in swapRenderers)
                            r.sharedMaterials = Enumerable.Repeat(silMat, Math.Max(1, r.sharedMaterials.Length)).ToArray();
                        cam.backgroundColor = silBg;
                    }
                }
            }
            finally
            {
                cam.targetTexture = null;
                RenderTexture.active = null;
                rt.Release();
                Object.DestroyImmediate(rt);
                Object.DestroyImmediate(tex);
            }
        }
        return info;
    }

    // ---- JSON access (rigtest_manifest) --------------------------------------------------------------
    static Dictionary<string, object> AsDict(object o) => o as Dictionary<string, object>;
    static List<object> AsList(object o) => o as List<object> ?? new List<object>();

    static object Get(Dictionary<string, object> d, params string[] keys)
    {
        if (d == null) return null;
        foreach (var k in keys)
        {
            if (d.TryGetValue(k, out var v)) return v;
            var hit = d.Keys.FirstOrDefault(x => string.Equals(x, k, StringComparison.OrdinalIgnoreCase));
            if (hit != null) return d[hit];
        }
        return null;
    }

    static double Num(object o)
    {
        switch (o)
        {
            case double d: return d;
            case string s: return double.Parse(s, Inv);
            case bool b: return b ? 1 : 0;
            default: throw new FormatException($"not a number: {o ?? "null"}");
        }
    }
}

// Minimal JSON reader: objects -> Dictionary<string, object>, arrays -> List<object>, numbers -> double.
static class GoblinJsonReader
{
    public static object Parse(string s)
    {
        int i = 0;
        var v = Val(s, ref i);
        Ws(s, ref i);
        if (i != s.Length) throw new FormatException($"trailing data at {i}");
        return v;
    }

    static void Ws(string s, ref int i) { while (i < s.Length && char.IsWhiteSpace(s[i])) i++; }

    static void Expect(string s, ref int i, char c)
    {
        Ws(s, ref i);
        if (i >= s.Length || s[i] != c) throw new FormatException($"expected '{c}' at {i}");
        i++;
    }

    static bool Lit(string s, ref int i, string lit)
    {
        if (string.CompareOrdinal(s, i, lit, 0, lit.Length) != 0) return false;
        i += lit.Length;
        return true;
    }

    static object Val(string s, ref int i)
    {
        Ws(s, ref i);
        if (i >= s.Length) throw new FormatException("unexpected end");
        char c = s[i];
        if (c == '{')
        {
            i++;
            var d = new Dictionary<string, object>();
            Ws(s, ref i);
            if (i < s.Length && s[i] == '}') { i++; return d; }
            while (true)
            {
                Ws(s, ref i);
                string k = Str(s, ref i);
                Expect(s, ref i, ':');
                d[k] = Val(s, ref i);
                Ws(s, ref i);
                if (i < s.Length && s[i] == ',') { i++; continue; }
                Expect(s, ref i, '}');
                return d;
            }
        }
        if (c == '[')
        {
            i++;
            var l = new List<object>();
            Ws(s, ref i);
            if (i < s.Length && s[i] == ']') { i++; return l; }
            while (true)
            {
                l.Add(Val(s, ref i));
                Ws(s, ref i);
                if (i < s.Length && s[i] == ',') { i++; continue; }
                Expect(s, ref i, ']');
                return l;
            }
        }
        if (c == '"') return Str(s, ref i);
        if (Lit(s, ref i, "true")) return true;
        if (Lit(s, ref i, "false")) return false;
        if (Lit(s, ref i, "null")) return null;
        if (Lit(s, ref i, "NaN")) return double.NaN;
        if (Lit(s, ref i, "Infinity")) return double.PositiveInfinity;
        if (Lit(s, ref i, "-Infinity")) return double.NegativeInfinity;
        int st = i;
        while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
        if (st == i) throw new FormatException($"unexpected '{c}' at {i}");
        return double.Parse(s.Substring(st, i - st), NumberStyles.Float, CultureInfo.InvariantCulture);
    }

    static string Str(string s, ref int i)
    {
        if (i >= s.Length || s[i] != '"') throw new FormatException($"expected string at {i}");
        i++;
        var sb = new StringBuilder();
        while (i < s.Length && s[i] != '"')
        {
            char c = s[i++];
            if (c != '\\') { sb.Append(c); continue; }
            char e = s[i++];
            switch (e)
            {
                case 'n': sb.Append('\n'); break;
                case 't': sb.Append('\t'); break;
                case 'r': sb.Append('\r'); break;
                case 'b': sb.Append('\b'); break;
                case 'f': sb.Append('\f'); break;
                case 'u': sb.Append((char)Convert.ToInt32(s.Substring(i, 4), 16)); i += 4; break;
                default: sb.Append(e); break;
            }
        }
        i++;
        return sb.ToString();
    }
}
