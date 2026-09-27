using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

// T210 (P1.3/P1.4/P1.6): data collection for the player walking skeleton. Writes unity/AvatarCheck/player_p1_report.json
// (data contract with work/player/rig/scripts/check_p1.py) and work/player/inspect/P1/unity_<cam>_<frame>.png.
// Records only; no judgement. Exit 0 when `errors` is empty, otherwise 1.
//
// Per integer frame f = frame_start..frame_end of p1test (t = (f - firstFrame) / 24):
//   clip.SampleAnimation(instance, t) -> record raw bones -> SpringBoneChain.Step(1/24) for every skirt chain ->
//   record the skirt bones again (springs applied) -> BakeMesh -> penetration counts -> renders at the render frames.
// The springs (Assets/Play/SpringBoneChain.cs, SpringCollider.cs, unchanged) are Init()ed once after sampling frame
// frame_start; their state carries over frame to frame (same dt every frame).
// Inputs: Assets/Player/Player.fbx + Player@p1test.fbx, work/player/rig/data/p1_manifest.json (bones, skirt chains,
// materials / colours, render frames, cameras). JSON helpers reuse JObj / GoblinJsonReader (Assets/Editor/Goblin, read only).
public static class PlayerP1Check
{
    const float Fps = 24f;
    const float Dt = 1f / Fps;
    const string ClipPath = PlayerImport.Folder + "/Player@p1test.fbx";
    const int RenderSize = 1024;
    // SpringBoneSetup.cs skirt row (Phase 2a): stiffness, damping, gravity, maxAngleDeg; leg colliders thigh / calf
    const float Stiffness = 0.15f, Damping = 0.25f, Gravity = 1.5f, MaxAngleDeg = 40f;
    static readonly (string bone, float r)[] Colliders = {
        ("Thigh_L", 0.11f), ("Thigh_R", 0.11f), ("Calf_L", 0.07f), ("Calf_R", 0.07f) };
    const float Ambient = 0.35f;
    static readonly Vector3 LightEuler = new Vector3(30f, 30f, 0f);   // camera-local, as GoblinRigCheck
    static readonly Vector3 RayDir = new Vector3(0.3141f, 0.8317f, 0.4579f).normalized;   // point-in-mesh parity ray

    public static void Run()
    {
        var total = Stopwatch.StartNew();
        var timing = new JObj();
        var errors = new List<string>();
        var warnings = new List<string>();
        void Capture(string c, string st, LogType t)
        {
            if (t == LogType.Error || t == LogType.Exception || t == LogType.Assert) errors.Add($"log [{t}] {c}");
            else if (t == LogType.Warning && warnings.Count < 100) warnings.Add(c);
        }
        string repo = PlayerImport.RepoRoot();
        string outPath = Path.Combine(repo, "unity", "AvatarCheck", "player_p1_report.json");
        string manifestPath = Path.Combine(repo, "work", "player", "rig", "data", "p1_manifest.json");
        var report = new JObj();
        var extra = new JObj();
        var renders = new List<object>();
        report.Add("unity_version", Application.unityVersion);
        report.Add("errors", errors);
        GameObject inst = null, bakeGo = null, camGo = null, lightGo = null;
        Mesh pen = null, renderMesh = null;
        var mats = new List<Material>();
        Application.logMessageReceived += Capture;
        try
        {
            // --- 1. import --------------------------------------------------------------------------------
            var sw = Stopwatch.StartNew();
            var resaved = PlayerImport.EnsureImported(true);
            timing.Add("import", sw.Elapsed.TotalSeconds);
            var model = PlayerImport.Describe(PlayerImport.ModelPath);
            var clipDesc = PlayerImport.Describe(ClipPath);
            var mi = (ModelImporter)AssetImporter.GetAtPath(PlayerImport.ModelPath);
            var ci = (ModelImporter)AssetImporter.GetAtPath(ClipPath);
            if (mi == null || ci == null) throw new Exception("Player.fbx / Player@p1test.fbx importer missing");
            var avatar = PlayerImport.LoadModelAvatar();
            report.Add("import", new JObj {
                { "animationType", mi.animationType.ToString() },
                { "avatar", avatar == null ? null : new JObj {
                    { "name", avatar.name }, { "isValid", avatar.isValid }, { "isHuman", avatar.isHuman },
                    { "clip_avatarSetup", ci.avatarSetup.ToString() },
                    { "clip_sourceAvatar", ci.sourceAvatar == null ? null : AssetDatabase.GetAssetPath(ci.sourceAvatar) } } },
                { "compression", ci.animationCompression.ToString() },
                { "importBlendShapes", mi.importBlendShapes && ci.importBlendShapes },
                { "clip_animationType", ci.animationType.ToString() },
                { "resaved_by_EnsureImported", resaved },
                { "files", new JObj { { "Player.fbx", model }, { "Player@p1test.fbx", clipDesc } } } });

            var man = GoblinJsonReader.Parse(File.ReadAllText(manifestPath)) as Dictionary<string, object>;
            if (man == null) throw new Exception("p1_manifest.json is not an object");
            int f0 = (int)(double)man["frame_start"], f1 = (int)(double)man["frame_end"];
            var expBones = ((List<object>)man["bones"]).Cast<Dictionary<string, object>>()
                .Select(b => (name: (string)b["name"], parent: b["parent"] as string)).ToList();
            var chains = ((List<object>)man["skirt_chains"]).Select(c => ((List<object>)c).Cast<string>().ToArray()).ToList();
            var renderFrames = new HashSet<int>(((List<object>)man["render_frames"]).Select(x => (int)(double)x));
            var matColors = ((Dictionary<string, object>)man["materials"]).ToDictionary(
                kv => kv.Key, kv => ((List<object>)((Dictionary<string, object>)kv.Value)["color"]).Select(x => (float)(double)x).ToArray());
            var cams = (Dictionary<string, object>)man["cameras"];

            var clip = AssetDatabase.LoadAllAssetsAtPath(ClipPath).OfType<AnimationClip>()
                .FirstOrDefault(c => !c.name.StartsWith("__preview__"));
            if (clip == null) throw new Exception($"no AnimationClip in {ClipPath}");
            var cs = (ci.clipAnimations.Length > 0 ? ci.clipAnimations : ci.defaultClipAnimations).FirstOrDefault();
            double firstFrame = cs == null ? 1.0 : cs.firstFrame;
            if (cs == null) errors.Add("no clip settings for p1test; firstFrame assumed 1");
            extra.Add("clip", new JObj {
                { "name", clip.name }, { "length_s", clip.length }, { "frameRate", clip.frameRate },
                { "firstFrame", firstFrame }, { "lastFrame", cs == null ? (object)null : cs.lastFrame },
                { "time_mapping", "t = (f - firstFrame) / 24" }, { "curve_bindings", AnimationUtility.GetCurveBindings(clip).Length },
                { "hasGenericRootTransform", clip.hasGenericRootTransform }, { "hasMotionCurves", clip.hasMotionCurves } });

            // --- 2. instance, bones, bind poses ---------------------------------------------------------------
            sw.Restart();
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PlayerImport.ModelPath);
            inst = Object.Instantiate(prefab);
            inst.name = prefab.name;
            var armNode = inst.transform.Find("Player");
            if (armNode == null) throw new Exception("armature node 'Player' not under the Player.fbx instance");
            var boneTs = armNode.GetComponentsInChildren<Transform>(true).Where(t => t != armNode).ToList();
            report.Add("bones", boneTs.Select(t => (object)new JObj { { "name", t.name }, { "parent", t.parent.name } }).ToList());
            var bone = new Dictionary<string, Transform>();
            foreach (var t in boneTs) if (!bone.ContainsKey(t.name)) bone[t.name] = t;
            extra.Add("hierarchy_root_children", inst.transform.Cast<Transform>().Select(t => (object)t.name).ToList());
            extra.Add("bones_vs_manifest", new JObj {
                { "manifest", expBones.Count }, { "unity", boneTs.Count },
                { "missing", expBones.Where(b => !bone.ContainsKey(b.name)).Select(b => (object)b.name).ToList() },
                { "unexpected", boneTs.Where(t => !expBones.Any(b => b.name == t.name)).Select(t => (object)t.name).ToList() },
                { "duplicates", boneTs.GroupBy(t => t.name).Where(g => g.Count() > 1).Select(g => (object)g.Key).ToList() },
                { "parent_mismatch", expBones.Where(b => bone.ContainsKey(b.name)
                        && bone[b.name].parent.name != (b.parent ?? "Player"))
                    .Select(b => (object)$"{b.name}: unity {bone[b.name].parent.name}, manifest {b.parent ?? "(armature node)"}").ToList() } });
            var boneNames = boneTs.Select(t => t.name).ToList();

            var smrs = inst.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            if (smrs.Length != 1) errors.Add($"Player.fbx instance has {smrs.Length} SkinnedMeshRenderers (expected 1)");
            var smr = smrs[0];
            var bp = smr.sharedMesh.bindposes;
            var sb = smr.bones;
            double bpMax = 0;
            bool bpOk = bp.Length == sb.Length && sb.All(b => b != null);
            for (int i = 0; i < Math.Min(bp.Length, sb.Length); i++)
            {
                if (sb[i] == null) continue;
                var want = sb[i].worldToLocalMatrix * smr.transform.localToWorldMatrix;
                for (int e = 0; e < 16; e++) bpMax = Math.Max(bpMax, Math.Abs(want[e] - bp[i][e]));
            }
            bpOk &= bpMax < 1e-4;
            report.Add("bindposes_ok", bpOk);
            var smrMats = smr.sharedMaterials.Select(m => m == null ? "" : m.name).ToList();
            extra.Add("skinned_mesh", new JObj {
                { "node", PlayerImport.PathOf(smr.transform, inst.transform) }, { "mesh", smr.sharedMesh.name },
                { "vertexCount", smr.sharedMesh.vertexCount }, { "subMeshCount", smr.sharedMesh.subMeshCount },
                { "materials", smrMats.Select(x => (object)x).ToList() }, { "smr_bones", sb.Length },
                { "bindposes", bp.Length }, { "bindpose_max_abs_diff_vs_rest", bpMax },
                { "bindpose_rule", "bindposes[i] vs bones[i].worldToLocalMatrix * smr.localToWorldMatrix on the fresh instance, max abs element; ok = counts equal, no null bone, max < 1e-4" },
                { "rootBone", smr.rootBone == null ? null : smr.rootBone.name },
                { "localBounds", new JObj { { "center", smr.localBounds.center }, { "extents", smr.localBounds.extents } } },
                { "lossyScale", smr.transform.lossyScale }, { "blendShapeCount", smr.sharedMesh.blendShapeCount } });

            // --- 3. springs ------------------------------------------------------------------------------------
            clip.SampleAnimation(inst, (float)((f0 - firstFrame) / Fps));
            var cols = new List<SpringCollider>();
            var colInfo = new List<object>();
            foreach (var (bn, r) in Colliders)
            {
                if (!bone.TryGetValue(bn, out var t)) { errors.Add($"collider bone {bn} missing"); continue; }
                var c = t.gameObject.AddComponent<SpringCollider>();
                c.radius = r;
                cols.Add(c);
                colInfo.Add(new JObj { { "bone", bn }, { "radius_m", r } });
            }
            var springChains = new List<(string name, SpringBoneChain sbc, Transform[] ts, Vector3 tipLocal)>();
            foreach (var ch in chains)
            {
                var ts = ch.Select(n => bone.TryGetValue(n, out var t) ? t : null).ToArray();
                if (ts.Any(t => t == null)) { errors.Add($"skirt chain {string.Join(",", ch)}: bone missing"); continue; }
                var sbc = ts[0].gameObject.AddComponent<SpringBoneChain>();
                sbc.bones = ts;
                sbc.stiffness = Stiffness; sbc.damping = Damping; sbc.gravity = Gravity; sbc.maxAngleDeg = MaxAngleDeg;
                sbc.colliders = cols.ToArray();
                sbc.Init();
                // chain tip = the point SpringBoneChain tracks for the last bone: last + (last - previous), in last-bone local
                var last = ts[ts.Length - 1];
                var prev = ts.Length > 1 ? ts[ts.Length - 2].position : last.parent.position;
                var tipLocal = last.InverseTransformPoint(last.position + (last.position - prev));
                springChains.Add((ch[0].Substring(0, ch[0].LastIndexOf('_')), sbc, ts, tipLocal));
            }
            var skirtNames = new HashSet<string>(chains.SelectMany(c => c));

            // --- 4. penetration setup (rest bake) ------------------------------------------------------------
            int Sub(string matName) => smrMats.FindIndex(n => n == matName);
            int subTunic = Sub("M_tunic"), subLegL = Sub("M_leg_l"), subLegR = Sub("M_leg_r");
            if (subTunic < 0 || subLegL < 0 || subLegR < 0) throw new Exception($"submesh for M_tunic / M_leg_l / M_leg_r not found in {string.Join(",", smrMats)}");
            var tunicTris = smr.sharedMesh.GetTriangles(subTunic);
            // leg vertices welded by rest position (the importer splits vertices per normal)
            var restMesh = smr.sharedMesh.vertices;
            var weld = new Dictionary<Vector3Int, int>();
            int Weld(int i)
            {
                var v = restMesh[i];
                var key = new Vector3Int(Mathf.RoundToInt(v.x * 1e5f), Mathf.RoundToInt(v.y * 1e5f), Mathf.RoundToInt(v.z * 1e5f));
                if (!weld.TryGetValue(key, out int w)) { w = i; weld[key] = i; }
                return w;
            }
            var legRaw = smr.sharedMesh.GetTriangles(subLegL).Concat(smr.sharedMesh.GetTriangles(subLegR)).ToArray();
            var legIdx = legRaw.Select(Weld).ToArray();
            var legVerts = legIdx.Distinct().OrderBy(i => i).ToArray();
            var legEdges = new HashSet<(int, int)>();
            for (int i = 0; i < legIdx.Length; i += 3)
                for (int k = 0; k < 3; k++)
                {
                    int a = legIdx[i + k], b = legIdx[i + (k + 1) % 3];
                    legEdges.Add(a < b ? (a, b) : (b, a));
                }
            var legEdgeList = legEdges.ToList();
            pen = new Mesh { name = "player_tmp_pen" };
            Vector3[] World()
            {
                smr.BakeMesh(pen, true);
                var m = Matrix4x4.TRS(smr.transform.position, smr.transform.rotation, Vector3.one);
                return pen.vertices.Select(v => m.MultiplyPoint3x4(v)).ToArray();
            }
            var rest = World();
            float tunicMinY = tunicTris.Select(i => rest[i].y).Min();
            var isCap = new bool[tunicTris.Length / 3];
            for (int k = 0; k < isCap.Length; k++)
            {
                Vector3 a = rest[tunicTris[3 * k]], b = rest[tunicTris[3 * k + 1]], c = rest[tunicTris[3 * k + 2]];
                var n = Vector3.Cross(b - a, c - a).normalized;
                isCap[k] = Mathf.Abs(n.y) > 0.9f && (a.y + b.y + c.y) / 3f < tunicMinY + 0.025f;
            }
            (int inside, int capX, int wallX) Pen(Vector3[] w)
            {
                int inside = 0, capX = 0, wallX = 0;
                foreach (int v in legVerts)
                {
                    int hits = 0;
                    for (int k = 0; k < isCap.Length; k++)
                        if (RayTri(w[v], RayDir, w[tunicTris[3 * k]], w[tunicTris[3 * k + 1]], w[tunicTris[3 * k + 2]], float.PositiveInfinity)) hits++;
                    if ((hits & 1) == 1) inside++;
                }
                foreach (var (a, b) in legEdgeList)
                {
                    var d = w[b] - w[a];
                    float len = d.magnitude;
                    if (len < 1e-9f) continue;
                    d /= len;
                    for (int k = 0; k < isCap.Length; k++)
                        if (RayTri(w[a], d, w[tunicTris[3 * k]], w[tunicTris[3 * k + 1]], w[tunicTris[3 * k + 2]], len))
                        { if (isCap[k]) capX++; else wallX++; }
                }
                return (inside, capX, wallX);
            }
            timing.Add("setup", sw.Elapsed.TotalSeconds);

            // --- 5. render setup -----------------------------------------------------------------------------
            Camera cam = null;
            Light light = null;
            MeshFilter bakeFilter = null;
            RenderTexture rt = null;
            Texture2D tex = null;
            string inspect = Path.Combine(repo, "work", "player", "inspect", "P1");
            bool canRender = SystemInfo.graphicsDeviceType != GraphicsDeviceType.Null;
            if (!canRender) errors.Add("no graphics device; renders skipped");
            else
            {
                var shader = Shader.Find("Standard");
                if (shader == null) throw new Exception("shader Standard not found");
                renderMesh = new Mesh { name = "player_tmp_baked" };
                bakeGo = new GameObject("player_tmp_baked");
                bakeFilter = bakeGo.AddComponent<MeshFilter>();
                bakeFilter.sharedMesh = renderMesh;
                var mr = bakeGo.AddComponent<MeshRenderer>();
                // one material per submesh (goblin lesson): colour by the imported material name (manifest colours)
                var perSub = new List<Material>();
                for (int i = 0; i < smr.sharedMesh.subMeshCount; i++)
                {
                    string n = i < smrMats.Count ? smrMats[i] : "";
                    var col = matColors.TryGetValue(n, out var cc) ? new Color(cc[0], cc[1], cc[2], 1f) : new Color(0.8f, 0.8f, 0.8f, 1f);
                    if (!matColors.ContainsKey(n)) errors.Add($"render: no manifest colour for material '{n}' (submesh {i})");
                    var m = new Material(shader) { color = col };
                    m.SetFloat("_Glossiness", 0.2f);
                    m.SetFloat("_Metallic", 0f);
                    mats.Add(m);
                    perSub.Add(m);
                }
                mr.sharedMaterials = perSub.ToArray();
                smr.enabled = false;
                RenderSettings.ambientMode = AmbientMode.Flat;
                RenderSettings.ambientLight = new Color(Ambient, Ambient, Ambient, 1f);
                RenderSettings.skybox = null;
                RenderSettings.reflectionIntensity = 0f;
                lightGo = new GameObject("player_tmp_light");
                light = lightGo.AddComponent<Light>();
                light.type = LightType.Directional;
                light.intensity = 1f;
                light.shadows = LightShadows.None;
                camGo = new GameObject("player_tmp_cam");
                cam = camGo.AddComponent<Camera>();
                cam.enabled = false;
                cam.orthographic = true;
                cam.clearFlags = CameraClearFlags.SolidColor;
                cam.backgroundColor = Color.white;
                cam.aspect = 1f;
                rt = new RenderTexture(RenderSize, RenderSize, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 1 };
                tex = new Texture2D(RenderSize, RenderSize, TextureFormat.RGBA32, false);
                extra.Add("render_setup", new JObj {
                    { "mesh", "SkinnedMeshRenderer.BakeMesh(useScale true) after the spring step -> temporary MeshRenderer at the SMR position / rotation (scale 1); SMR disabled" },
                    { "materials", "one Standard material per submesh, colour = manifest materials[<imported material name>].color, smoothness 0.2" },
                    { "light", "directional, intensity 1, rotation = camera rotation * Euler(30, 30, 0)" },
                    { "ambient", Ambient }, { "background", "white" }, { "size", RenderSize },
                    { "cameras", "manifest cameras.<cam>.unity (position, forward, up, orthographicSize, near, far)" },
                    { "renderPipeline", GraphicsSettings.currentRenderPipeline == null ? "built-in" : GraphicsSettings.currentRenderPipeline.name } });
            }

            // --- 6. frames ---------------------------------------------------------------------------------------
            var frames = new List<object>();
            var penFrames = new List<object>();
            var defl = springChains.ToDictionary(c => c.name, c => new List<double>());
            double tSample = 0, tPen = 0, tRender = 0;
            var swStep = new Stopwatch();
            float[] Pose(Transform t) =>
                new[] { t.position.x, t.position.y, t.position.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w };
            for (int f = f0; f <= f1; f++)
            {
                swStep.Restart();
                clip.SampleAnimation(inst, (float)((f - firstFrame) / Fps));
                var skirtRaw = new JObj();
                foreach (var n in boneNames.Where(skirtNames.Contains)) skirtRaw.Add(n, Pose(bone[n]));
                var tipsRaw = springChains.Select(c => c.ts[c.ts.Length - 1].TransformPoint(c.tipLocal)).ToList();
                tSample += swStep.Elapsed.TotalSeconds;
                swStep.Restart();
                var pr = Pen(World());
                tPen += swStep.Elapsed.TotalSeconds;
                swStep.Restart();
                foreach (var c in springChains) c.sbc.Step(Dt);
                var bw = new JObj();
                foreach (var n in boneNames) bw.Add(n, Pose(bone[n]));
                var skirtSpring = new JObj();
                foreach (var n in boneNames.Where(skirtNames.Contains)) skirtSpring.Add(n, Pose(bone[n]));
                for (int i = 0; i < springChains.Count; i++)
                {
                    var c = springChains[i];
                    defl[c.name].Add((c.ts[c.ts.Length - 1].TransformPoint(c.tipLocal) - tipsRaw[i]).magnitude);
                }
                frames.Add(new JObj { { "f", f }, { "bones_world", bw }, { "skirt_raw", skirtRaw }, { "skirt_spring", skirtSpring } });
                tSample += swStep.Elapsed.TotalSeconds;
                swStep.Restart();
                var ps = Pen(World());
                penFrames.Add(new JObj {
                    { "f", f }, { "leg_vertices_inside_skirt", ps.inside },
                    { "leg_edges_crossing_skirt", new JObj { { "cap", ps.capX }, { "wall", ps.wallX } } },
                    { "raw_no_spring", new JObj { { "leg_vertices_inside_skirt", pr.inside },
                        { "leg_edges_crossing_skirt", new JObj { { "cap", pr.capX }, { "wall", pr.wallX } } } } } });
                tPen += swStep.Elapsed.TotalSeconds;

                if (canRender && renderFrames.Contains(f))
                {
                    swStep.Restart();
                    smr.BakeMesh(renderMesh, true);
                    renderMesh.RecalculateBounds();
                    bakeGo.transform.SetPositionAndRotation(smr.transform.position, smr.transform.rotation);
                    bakeGo.transform.localScale = Vector3.one;
                    foreach (var kv in cams)
                    {
                        var u = (Dictionary<string, object>)((Dictionary<string, object>)kv.Value)["unity"];
                        Vector3 V(string k) { var l = (List<object>)u[k]; return new Vector3((float)(double)l[0], (float)(double)l[1], (float)(double)l[2]); }
                        var rot = Quaternion.LookRotation(V("forward"), V("up"));
                        cam.transform.SetPositionAndRotation(V("position"), rot);
                        cam.orthographicSize = (float)(double)u["orthographicSize"];
                        cam.nearClipPlane = (float)(double)u["nearClipPlane"];
                        cam.farClipPlane = (float)(double)u["farClipPlane"];
                        light.transform.rotation = rot * Quaternion.Euler(LightEuler);
                        cam.targetTexture = rt;
                        cam.Render();
                        RenderTexture.active = rt;
                        tex.ReadPixels(new Rect(0, 0, RenderSize, RenderSize), 0, 0);
                        tex.Apply();
                        RenderTexture.active = null;
                        cam.targetTexture = null;
                        string file = $"unity_{kv.Key}_{f}.png";
                        Directory.CreateDirectory(inspect);
                        File.WriteAllBytes(Path.Combine(inspect, file), tex.EncodeToPNG());
                        renders.Add("work/player/inspect/P1/" + file);
                    }
                    tRender += swStep.Elapsed.TotalSeconds;
                }
            }
            if (rt != null) { rt.Release(); Object.DestroyImmediate(rt); }
            if (tex != null) Object.DestroyImmediate(tex);

            report.Add("frames", frames);
            report.Add("spring", new JObj {
                { "chains", springChains.Select(c => (object)new JObj {
                    { "name", c.name }, { "bones", c.ts.Select(t => (object)t.name).ToList() },
                    { "tip_deflection_m_per_frame", defl[c.name].ToArray() },
                    { "tip_deflection_max_m", defl[c.name].Count == 0 ? 0.0 : defl[c.name].Max() },
                    { "tip_deflection_max_frame", defl[c.name].Count == 0 ? -1 : f0 + defl[c.name].IndexOf(defl[c.name].Max()) } }).ToList() },
                { "params", new JObj {
                    { "stiffness", Stiffness }, { "damping", Damping }, { "gravity", Gravity }, { "maxAngleDeg", MaxAngleDeg },
                    { "dt", Dt }, { "source", "SpringBoneSetup.cs skirt row (Phase 2a) and its leg collider radii" },
                    { "init", $"Init() once after SampleAnimation(frame {f0}); state carried frame to frame" },
                    { "order", "per frame: SampleAnimation -> Step(dt) for each chain in manifest order" },
                    { "tip", "chain tip = last bone position + (last - previous bone position) at rest, carried in the last bone's local space (the point SpringBoneChain tracks); deflection = |tip_spring - tip_raw|" } } },
                { "colliders", colInfo } });
            report.Add("penetration", new JObj {
                { "method", "per frame, on the world-space BakeMesh after the spring step (raw_no_spring: before it): "
                    + "leg_vertices_inside_skirt = leg_l + leg_r tube vertices inside the closed tunic surface (ray parity along a fixed oblique ray, all tunic triangles incl. the bottom cap); "
                    + "leg_edges_crossing_skirt = leg tube edges that intersect a tunic triangle, split into bottom-cap triangles (|normal.y| > 0.9 within 25 mm of the tunic bottom at rest) and the rest (wall). "
                    + "Note: at rest the leg tops sit inside the closed tunic by design, so the inside count is not zero at rest." },
                { "tunic_triangles", isCap.Length }, { "cap_triangles", isCap.Count(x => x) },
                { "leg_vertices", legVerts.Length }, { "leg_vertices_unwelded", legRaw.Distinct().Count() },
                { "leg_edges", legEdgeList.Count },
                { "leg_weld", "leg vertices / edges welded by rest mesh position (1e-5 m grid); the importer splits vertices per normal" },
                { "per_frame", penFrames } });
            timing.Add("sampling_and_springs", tSample);
            timing.Add("penetration", tPen);
            timing.Add("renders", tRender);
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
            if (bakeGo != null) Object.DestroyImmediate(bakeGo);
            if (camGo != null) Object.DestroyImmediate(camGo);
            if (lightGo != null) Object.DestroyImmediate(lightGo);
            if (pen != null) Object.DestroyImmediate(pen);
            if (renderMesh != null) Object.DestroyImmediate(renderMesh);
            foreach (var m in mats) Object.DestroyImmediate(m);
            try { EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single); } catch (Exception) { }
        }
        report.Add("renders", renders);
        timing.Add("total", total.Elapsed.TotalSeconds);
        report.Add("timing_s", timing);
        extra.Add("log_warnings", warnings);
        report.Add("extra", extra);
        PlayerImport.WriteJson(outPath, report);
        Debug.Log($"PLAYER P1 CHECK written: {outPath} errors={errors.Count}");
        foreach (var e in errors) Debug.Log("PLAYER P1 CHECK error: " + e);
        EditorApplication.Exit(errors.Count == 0 ? 0 : 1);
    }

    // Moller-Trumbore, two-sided; hit when 0 < t < maxT.
    static bool RayTri(Vector3 o, Vector3 d, Vector3 a, Vector3 b, Vector3 c, float maxT)
    {
        var e1 = b - a;
        var e2 = c - a;
        var p = Vector3.Cross(d, e2);
        float det = Vector3.Dot(e1, p);
        if (Mathf.Abs(det) < 1e-12f) return false;
        float inv = 1f / det;
        var s = o - a;
        float u = Vector3.Dot(s, p) * inv;
        if (u < 0f || u > 1f) return false;
        var q = Vector3.Cross(s, e1);
        float v = Vector3.Dot(d, q) * inv;
        if (v < 0f || u + v > 1f) return false;
        float t = Vector3.Dot(e2, q) * inv;
        return t > 1e-7f && t < maxT;
    }
}
