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

// T250 (P2.6): data collection for the player rig in Unity. Writes unity/AvatarCheck/player_report.json (data contract
// with work/player/rig/scripts/check_p2_export.py, spec work/player/d-02-player-rig.md section 5) and BakeMesh renders
// work/player/inspect/P2d/unity_<cam>_<frame>.png. Records only; no judgement. Exit 0 when `errors` is empty, else 1.
// Adapted from Assets/Editor/Goblin/GoblinRigCheck.cs (bounds union + prefab localBounds, one material per submesh,
// bindposes) and Assets/Editor/Player/PlayerP1Check.cs (import block, springs, BakeMesh renders); Goblin files and
// Assets/Play/SpringBoneChain.cs / SpringCollider.cs are used unchanged.
//
// Per clip Assets/Player/Player@<clip>.fbx, integer frames f = firstFrame..lastFrame (t = (f - firstFrame) / frameRate), on a
// fresh Player.fbx instance at the origin (root identity):
//   SampleAnimation -> skirt_raw (Skirt_* world, springs OFF) + raw tips -> BakeMesh -> penetration spring_off ->
//   SpringBoneChain.Step(1/24) for the 8 skirt chains -> bones_world (all bones, springs ON) -> tips -> sway ->
//   BakeMesh -> penetration spring_on (+ renders at the rigtest render frames).
// Springs are Init()ed once after sampling the first frame. Tip = Skirt_*_02 tail (canonical_skeleton.json tail mapped
// unity = (-x, z, -y) at rest, carried in the Skirt_*_02 local space); sway_mm(f) = |tip_spring - tip_raw| * 1000.
// Penetration = the check_p2_deform poke-out rule applied in the pelvis rest frame (see PenRule).
public static class PlayerRigCheck
{
    // T303: per-clip frameRate (was a fixed 24 fps; rigtest is 24, P3 clips carry their own fps, e.g. smoke 30)
    const float BoundsMargin = 0.10f;
    const string PrefabPath = PlayerImport.Folder + "/Player.prefab";
    const int RenderSize = 1024;
    // T253: user choice r15 of the T252 sweep (inspect/P2d/spring_sweep.md; was the P1 skirt row .15 / .25 / 1.5 / 40);
    // set to these defaults at the start of Run()
    const float DefStiffness = 0.3f, DefDamping = 0.5f, DefGravity = 1.5f, DefMaxAngleDeg = 25f;
    static float Stiffness, Damping, Gravity, MaxAngleDeg;
    // capsule approximation: 3 spheres along hip -> knee (Thigh) and knee -> ankle (Calf)
    static readonly float[] ColliderT = { 1f / 6f, 0.5f, 5f / 6f };
    const float DefThighRadius = 0.045f, DefCalfRadius = 0.055f;
    static float ThighRadius, CalfRadius;
    // T252: optional override for report-only experiment runs. Env var PLAYER_RIGCHECK_OVERRIDE = path of a JSON file
    // (absolute or relative to the repo root). Unset = the T250 run unchanged. Keys (all optional): stiffness, damping,
    // gravity, maxAngleDeg, thigh_radius, calf_radius (m); out (report path, repo-relative; default player_report.json);
    // frames (bool, false = omit the per-frame bones_world / skirt_raw block); apply_bounds (bool, false = Player.prefab
    // not saved); renders (bool); render_frames (int list); render_cams (camera name list); render_dir (repo-relative);
    // render_prefix (file = <prefix><cam>_<f>_on.png); render_springs_off (bool, also <prefix><cam>_<f>_off.png rendered
    // right after SampleAnimation, before the spring step). The applied override is recorded in the report ("override").
    const string OverrideEnv = "PLAYER_RIGCHECK_OVERRIDE";
    static readonly string[] SkirtTags = { "F", "FL", "L", "BL", "B", "BR", "R", "FR" };
    const float Ambient = 0.35f;
    static readonly Vector3 LightEuler = new Vector3(30f, 30f, 0f);
    const float PokeRayMax = 0.60f;
    const float CeilTol = 0.002f;
    const string PenRule =
        "check_p2_deform poke-out rule (T223 d) per frame on the world BakeMesh, evaluated in the pelvis rest frame "
        + "(every vertex p -> PelvisRest.localToWorld * PelvisPose.worldToLocal * p, so the rule's vertical skirt axis "
        + "follows the pelvis): leg vertices = M_leg_l / M_leg_r submesh vertices (welded by rest position) with rest y "
        + "below the skirt ceiling (retopo_loops tunic.ceiling_z); skirt wall = M_tunic triangles with rest centroid y "
        + "below the belt_bot row z, minus the ceiling triangles (all vertices within 2 mm of ceiling_z). A horizontal "
        + "ray from the skirt axis point (retopo_loops tunic.axis_xy -> unity (-x, -y)) at the vertex height through the "
        + "vertex collects every wall hit up to 0.6 m: footprint = at least one hit; poke-out = footprint and every hit "
        + "closer than the vertex (depth = vertex radius - farthest hit). spring_off = raw sampled frame, spring_on = "
        + "after the spring step. Counts are poke-out vertices.";

    static float[] Pose(Transform t) =>
        new[] { t.position.x, t.position.y, t.position.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w };

    static Vector3 FromBlender(IList<object> p) =>
        new Vector3(-(float)(double)p[0], (float)(double)p[2], -(float)(double)p[1]);

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
        string rigDir = Path.Combine(repo, "work", "player", "rig");
        string outPath = Path.Combine(repo, "unity", "AvatarCheck", "player_report.json");
        string inspect = Path.Combine(repo, "work", "player", "inspect", "P2d");
        var report = new JObj();
        var extra = new JObj();
        var renders = new List<object>();
        report.Add("unity_version", Application.unityVersion);
        report.Add("errors", errors);
        Stiffness = DefStiffness; Damping = DefDamping; Gravity = DefGravity; MaxAngleDeg = DefMaxAngleDeg;
        ThighRadius = DefThighRadius; CalfRadius = DefCalfRadius;
        Dictionary<string, object> ov = null;
        string ovPath = Environment.GetEnvironmentVariable(OverrideEnv);
        if (!string.IsNullOrEmpty(ovPath))
        {
            if (!Path.IsPathRooted(ovPath)) ovPath = Path.Combine(repo, ovPath);
            ov = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(ovPath));
            float F(string k, float d) => ov.TryGetValue(k, out var v) ? (float)(double)v : d;
            Stiffness = F("stiffness", Stiffness); Damping = F("damping", Damping); Gravity = F("gravity", Gravity);
            MaxAngleDeg = F("maxAngleDeg", MaxAngleDeg); ThighRadius = F("thigh_radius", ThighRadius); CalfRadius = F("calf_radius", CalfRadius);
            if (ov.TryGetValue("out", out var o)) outPath = Path.Combine(repo, (string)o);
            var ovValues = new JObj();
            foreach (var kv in ov) ovValues.Add(kv.Key, kv.Value);
            report.Add("override", new JObj { { "env", OverrideEnv }, { "file", ovPath }, { "values", ovValues } });
        }
        bool OvBool(string k, bool d) => ov != null && ov.TryGetValue(k, out var v) ? (bool)v : d;
        bool keepFrames = OvBool("frames", true), applyBounds = OvBool("apply_bounds", true), rendersOn = OvBool("renders", true);
        bool renderOff = OvBool("render_springs_off", false);
        string renderPrefix = ov != null && ov.TryGetValue("render_prefix", out var rp) ? (string)rp : null;
        var renderCams = ov != null && ov.TryGetValue("render_cams", out var rc) ? new HashSet<string>(((List<object>)rc).Cast<string>()) : null;
        if (ov != null && ov.TryGetValue("render_dir", out var rd)) inspect = Path.Combine(repo, (string)rd);
        var tempObjects = new List<Object>();
        Application.logMessageReceived += Capture;
        try
        {
            // --- 1. import ------------------------------------------------------------------------------------
            var sw = Stopwatch.StartNew();
            var resaved = PlayerImport.EnsureImported(true);
            timing.Add("import", sw.Elapsed.TotalSeconds);
            var clipPaths = PlayerImport.ClipPaths();
            if (clipPaths.Count == 0) throw new Exception("no Player@<clip>.fbx in " + PlayerImport.Folder);
            var mi = (ModelImporter)AssetImporter.GetAtPath(PlayerImport.ModelPath);
            if (mi == null) throw new Exception("Player.fbx importer missing");
            var avatar = PlayerImport.LoadModelAvatar();
            var files = new JObj { { "Player.fbx", PlayerImport.Describe(PlayerImport.ModelPath) } };
            var clipImp = new JObj();
            foreach (var cp in clipPaths)
            {
                files.Add(Path.GetFileName(cp), PlayerImport.Describe(cp));
                var ci = (ModelImporter)AssetImporter.GetAtPath(cp);
                clipImp.Add(Path.GetFileName(cp), new JObj {
                    { "animationType", ci.animationType.ToString() }, { "avatarSetup", ci.avatarSetup.ToString() },
                    { "sourceAvatar", ci.sourceAvatar == null ? null : AssetDatabase.GetAssetPath(ci.sourceAvatar) },
                    { "compression", ci.animationCompression.ToString() } });
                if (ci.animationType != ModelImporterAnimationType.Generic) errors.Add($"{cp}: animationType {ci.animationType}");
            }
            if (mi.animationType != ModelImporterAnimationType.Generic) errors.Add($"Player.fbx animationType {mi.animationType}");
            report.Add("import", new JObj {
                { "animationType", mi.animationType.ToString() },
                { "avatar", avatar == null ? null : new JObj {
                    { "name", avatar.name }, { "isValid", avatar.isValid }, { "isHuman", avatar.isHuman } } },
                { "clips", clipImp }, { "resaved_by_EnsureImported", resaved }, { "files", files } });

            // --- 2. data files --------------------------------------------------------------------------------
            var canon = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(Path.Combine(rigDir, "data", "canonical_skeleton.json")));
            var canonBones = ((List<object>)canon["bones"]).Cast<Dictionary<string, object>>().ToDictionary(b => (string)b["name"]);
            var parts = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(Path.Combine(rigDir, "data", "parts.json")));
            var matColors = ((Dictionary<string, object>)parts["materials"]).ToDictionary(
                kv => kv.Key, kv => ((List<object>)((Dictionary<string, object>)kv.Value)["color"]).Select(x => (float)(double)x).ToArray());
            var loops = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(Path.Combine(rigDir, "data", "retopo_loops.json")));
            var tunic = (Dictionary<string, object>)loops["tunic"];
            var axisXY = (List<object>)tunic["axis_xy"];
            float axX = -(float)(double)axisXY[0], axZ = -(float)(double)axisXY[1];
            float ceilY = (float)(double)tunic["ceiling_z"];
            float beltBotY = (float)((List<object>)tunic["outer_rows"]).Cast<Dictionary<string, object>>()
                .Where(r => (string)r["name"] == "belt_bot").Select(r => (double)r["z"]).First();
            var rigtestMan = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(Path.Combine(rigDir, "data", "rigtest_manifest.json")));
            var renderFrames = new HashSet<int>(((List<object>)(ov != null && ov.ContainsKey("render_frames") ? ov["render_frames"] : rigtestMan["render_frames"])).Select(x => (int)(double)x));
            var p1 = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(Path.Combine(rigDir, "data", "p1_manifest.json")));
            var cams = (Dictionary<string, object>)p1["cameras"];

            // --- 3. rest instance: bones, bind poses, rest block ------------------------------------------------
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PlayerImport.ModelPath);
            GameObject NewInstance()
            {
                var go = Object.Instantiate(prefab);
                go.name = prefab.name;
                go.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
                go.transform.localScale = Vector3.one;
                tempObjects.Add(go);
                return go;
            }
            var restInst = NewInstance();
            var armNode = restInst.transform.Find("Player");
            if (armNode == null) throw new Exception("armature node 'Player' not under the Player.fbx instance");
            var restBones = armNode.GetComponentsInChildren<Transform>(true).Where(t => t != armNode).ToList();
            report.Add("bones", restBones.Select(t => (object)new JObj { { "name", t.name }, { "parent", t.parent.name } }).ToList());
            var boneNames = restBones.Select(t => t.name).ToList();
            var missing = canonBones.Keys.Where(n => !boneNames.Contains(n)).ToList();
            if (missing.Count > 0) errors.Add("bones missing in Unity: " + string.Join(",", missing));
            var rest = new JObj();
            foreach (var t in restBones) rest.Add(t.name, Pose(t));
            report.Add("rest", rest);
            var smr0 = restInst.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            if (smr0.Length != 1) errors.Add($"Player.fbx instance has {smr0.Length} SkinnedMeshRenderers (expected 1)");
            var rsmr = smr0[0];
            var bp = rsmr.sharedMesh.bindposes;
            var sb = rsmr.bones;
            double bpMax = 0;
            string bpWorst = null;
            var bpPerBone = new JObj();
            bool bpOk = bp.Length == sb.Length && sb.All(b => b != null);
            for (int i = 0; i < Math.Min(bp.Length, sb.Length); i++)
            {
                if (sb[i] == null) continue;
                var want = sb[i].worldToLocalMatrix * rsmr.transform.localToWorldMatrix;
                double d = 0;
                for (int e = 0; e < 16; e++) d = Math.Max(d, Math.Abs(want[e] - bp[i][e]));
                bpPerBone.Add(sb[i].name, d);
                if (d > bpMax) { bpMax = d; bpWorst = sb[i].name; }
            }
            bpOk &= bpMax < 1e-4;
            report.Add("bindposes_ok", bpOk);
            var smrMats = rsmr.sharedMaterials.Select(m => m == null ? "" : m.name).ToList();
            extra.Add("skinned_mesh", new JObj {
                { "mesh", rsmr.sharedMesh.name }, { "vertexCount", rsmr.sharedMesh.vertexCount },
                { "subMeshCount", rsmr.sharedMesh.subMeshCount }, { "materials", smrMats.Select(x => (object)x).ToList() },
                { "smr_bones", sb.Length }, { "bindposes", bp.Length }, { "bindpose_max_abs_diff_vs_rest", bpMax },
                { "bindpose_worst_bone", bpWorst }, { "bindpose_diff_per_bone", bpPerBone },
                { "bindpose_rule", "bindposes[i] vs bones[i].worldToLocalMatrix * smr.localToWorldMatrix on a fresh Player.fbx instance at the origin, max abs element; ok = counts equal, no null bone, max < 1e-4" },
                { "rootBone", rsmr.rootBone == null ? null : rsmr.rootBone.name } });

            // penetration geometry from the rest bake (world, character at origin)
            int Sub(string n) => smrMats.FindIndex(x => x == n);
            int subTunic = Sub("M_tunic"), subLegL = Sub("M_leg_l"), subLegR = Sub("M_leg_r");
            if (subTunic < 0 || subLegL < 0 || subLegR < 0) throw new Exception("submesh M_tunic / M_leg_l / M_leg_r not found");
            var tmpMesh = new Mesh { name = "player_tmp_bake" };
            tempObjects.Add(tmpMesh);
            Vector3[] BakeWorld(SkinnedMeshRenderer r)
            {
                r.BakeMesh(tmpMesh, true);
                var m = r.transform.localToWorldMatrix;
                var v = tmpMesh.vertices;
                var o = new Vector3[v.Length];
                for (int i = 0; i < v.Length; i++) o[i] = m.MultiplyPoint3x4(v[i]);
                return o;
            }
            var restW = BakeWorld(rsmr);
            // rest bake vs the imported mesh (bind): shows the visible effect of a bind pose / node rest mismatch
            {
                var mv = rsmr.sharedMesh.vertices;
                var m2w = rsmr.transform.localToWorldMatrix;
                var perSub = new JObj();
                double worstAll = 0;
                for (int si = 0; si < rsmr.sharedMesh.subMeshCount; si++)
                {
                    double w = 0;
                    foreach (int vi in rsmr.sharedMesh.GetTriangles(si).Distinct())
                        w = Math.Max(w, (restW[vi] - m2w.MultiplyPoint3x4(mv[vi])).magnitude);
                    perSub.Add(si < smrMats.Count ? smrMats[si] : si.ToString(), w * 1000.0);
                    worstAll = Math.Max(worstAll, w);
                }
                extra.Add("rest_bake_vs_mesh_mm", new JObj { { "max", worstAll * 1000.0 }, { "per_submesh", perSub },
                    { "rule", "fresh instance BakeMesh (bones at the FBX node rest) vs sharedMesh vertices, world, max per submesh" } });
            }
            var tunicTris = rsmr.sharedMesh.GetTriangles(subTunic);
            var wallTris = new List<int>();
            for (int k = 0; k < tunicTris.Length; k += 3)
            {
                Vector3 a = restW[tunicTris[k]], b = restW[tunicTris[k + 1]], c = restW[tunicTris[k + 2]];
                bool ceiling = Mathf.Abs(a.y - ceilY) < CeilTol && Mathf.Abs(b.y - ceilY) < CeilTol && Mathf.Abs(c.y - ceilY) < CeilTol;
                if ((a.y + b.y + c.y) / 3f < beltBotY && !ceiling) { wallTris.Add(tunicTris[k]); wallTris.Add(tunicTris[k + 1]); wallTris.Add(tunicTris[k + 2]); }
            }
            var weld = new Dictionary<Vector3Int, int>();
            var legVerts = new List<int>();
            foreach (int i in rsmr.sharedMesh.GetTriangles(subLegL).Concat(rsmr.sharedMesh.GetTriangles(subLegR)).Distinct())
            {
                var v = restW[i];
                var key = new Vector3Int(Mathf.RoundToInt(v.x * 1e5f), Mathf.RoundToInt(v.y * 1e5f), Mathf.RoundToInt(v.z * 1e5f));
                if (weld.ContainsKey(key)) continue;
                weld[key] = i;
                if (v.y < ceilY) legVerts.Add(i);
            }
            var pelvisRest = restBones.First(t => t.name == "Pelvis").localToWorldMatrix;
            Object.DestroyImmediate(restInst);   // keep only the per-clip instance in the scene (renders)
            tempObjects.Remove(restInst);
            extra.Add("penetration_setup", new JObj {
                { "wall_triangles", wallTris.Count / 3 }, { "tunic_triangles", tunicTris.Length / 3 },
                { "leg_vertices_below_ceiling_welded", legVerts.Count }, { "axis_unity_xz", new[] { axX, axZ } },
                { "ceiling_y", ceilY }, { "belt_bot_y", beltBotY } });

            (int count, float maxDepth) Poke(Vector3[] w, Matrix4x4 toRest)
            {
                int nWall = wallTris.Count / 3;
                var tri = new Vector3[wallTris.Count];
                var ymin = new float[nWall];
                var ymax = new float[nWall];
                for (int k = 0; k < wallTris.Count; k++) tri[k] = toRest.MultiplyPoint3x4(w[wallTris[k]]);
                for (int k = 0; k < nWall; k++)
                {
                    ymin[k] = Mathf.Min(tri[3 * k].y, Mathf.Min(tri[3 * k + 1].y, tri[3 * k + 2].y));
                    ymax[k] = Mathf.Max(tri[3 * k].y, Mathf.Max(tri[3 * k + 1].y, tri[3 * k + 2].y));
                }
                int count = 0;
                float worst = 0f;
                var hits = new List<float>();
                foreach (int vi in legVerts)
                {
                    var q = toRest.MultiplyPoint3x4(w[vi]);
                    float dx = q.x - axX, dz = q.z - axZ;
                    float r = Mathf.Sqrt(dx * dx + dz * dz);
                    if (r < 1e-6f) continue;
                    var d = new Vector3(dx / r, 0f, dz / r);
                    var o = new Vector3(axX, q.y, axZ);
                    hits.Clear();
                    for (int k = 0; k < nWall; k++)
                    {
                        if (q.y < ymin[k] || q.y > ymax[k]) continue;
                        float t = RayTri(o, d, tri[3 * k], tri[3 * k + 1], tri[3 * k + 2]);
                        if (t > 0f && t <= PokeRayMax) hits.Add(t);
                    }
                    if (hits.Count == 0) continue;
                    float far = hits.Max();
                    if (far < r)
                    {
                        count++;
                        worst = Mathf.Max(worst, r - far);
                    }
                }
                return (count, worst);
            }

            // --- 4. per clip -------------------------------------------------------------------------------------
            var clipsOut = new JObj();
            var unionMin = Vector3.positiveInfinity;
            var unionMax = Vector3.negativeInfinity;
            int unionFrames = 0;
            var shader = Shader.Find("Standard");
            bool canRender = SystemInfo.graphicsDeviceType != GraphicsDeviceType.Null && shader != null;
            if (!canRender) errors.Add("no graphics device or Standard shader; renders skipped");
            double tSample = 0, tPen = 0, tRender = 0;
            foreach (var cp in clipPaths)
            {
                var clip = AssetDatabase.LoadAllAssetsAtPath(cp).OfType<AnimationClip>().FirstOrDefault(c => !c.name.StartsWith("__preview__"));
                if (clip == null) { errors.Add($"no AnimationClip in {cp}"); continue; }
                var ci = (ModelImporter)AssetImporter.GetAtPath(cp);
                var cs = (ci.clipAnimations.Length > 0 ? ci.clipAnimations : ci.defaultClipAnimations).FirstOrDefault();
                if (cs == null) { errors.Add($"no clip settings in {cp}"); continue; }
                int f0 = (int)Math.Ceiling(cs.firstFrame - 1e-6), f1 = (int)Math.Floor(cs.lastFrame + 1e-6);
                string clipName = Path.GetFileNameWithoutExtension(cp).Substring("Player@".Length);
                float fps = clip.frameRate, dt = 1f / fps;   // T303
                string clipJson = Path.Combine(rigDir, "data", "clips", clipName + ".json");
                double wantFps = clipName == "rigtest" ? (double)rigtestMan["fps"]
                    : File.Exists(clipJson) ? (((Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(clipJson))).TryGetValue("fps", out var jf) ? (double)jf : 30.0)
                    : double.NaN;
                if (double.IsNaN(wantFps)) errors.Add($"{cp}: no expected fps (rigtest_manifest.json / data/clips/{clipName}.json)");
                else if (Math.Abs(fps - wantFps) > 1e-3) errors.Add($"{cp}: frameRate {clip.frameRate} != expected {wantFps}");

                var inst = NewInstance();
                var arm = inst.transform.Find("Player");
                var bone = arm.GetComponentsInChildren<Transform>(true).Where(t => t != arm).GroupBy(t => t.name).ToDictionary(g => g.Key, g => g.First());
                var smr = inst.GetComponentInChildren<SkinnedMeshRenderer>(true);
                // skirt tips (rest instance = bind pose), canonical tail of Skirt_*_02
                var chains = new List<(string name, SpringBoneChain sbc, Transform[] ts, Vector3 tipLocal)>();
                var tipLocals = new Dictionary<string, Vector3>();
                foreach (var tag in SkirtTags)
                {
                    string b2 = $"Skirt_{tag}_02";
                    var tail = FromBlender((List<object>)canonBones[b2]["tail"]);
                    tipLocals[tag] = bone[b2].InverseTransformPoint(tail);
                }
                clip.SampleAnimation(inst, (float)((f0 - cs.firstFrame) / fps));
                var cols = new List<SpringCollider>();
                var colInfo = new List<object>();
                foreach (var side in new[] { "L", "R" })
                    foreach (var (bn, nextBn, rad) in new[] { ($"Thigh_{side}", $"Calf_{side}", ThighRadius), ($"Calf_{side}", $"Foot_{side}", CalfRadius) })
                    {
                        var bt = bone[bn];
                        for (int k = 0; k < ColliderT.Length; k++)
                        {
                            var go = new GameObject($"SpringCol_{bn}_{k}");
                            go.transform.SetParent(bt, false);
                            go.transform.position = Vector3.Lerp(bt.position, bone[nextBn].position, ColliderT[k]);
                            var c = go.AddComponent<SpringCollider>();
                            c.radius = rad;
                            cols.Add(c);
                            colInfo.Add(new JObj { { "bone", bn }, { "center_local", go.transform.localPosition }, { "radius_m", rad },
                                                   { "t_along", ColliderT[k] }, { "segment", $"{bn} head -> {nextBn} head" } });
                        }
                    }
                foreach (var tag in SkirtTags)
                {
                    var ts = new[] { bone[$"Skirt_{tag}_01"], bone[$"Skirt_{tag}_02"] };
                    var sbc = ts[0].gameObject.AddComponent<SpringBoneChain>();
                    sbc.bones = ts;
                    sbc.stiffness = Stiffness; sbc.damping = Damping; sbc.gravity = Gravity; sbc.maxAngleDeg = MaxAngleDeg; sbc.followAnimation = true;  // P2e: keep the baked skirt follow (main, 2026-09-27)
                    sbc.colliders = cols.ToArray();
                    sbc.Init();
                    chains.Add(($"Skirt_{tag}", sbc, ts, tipLocals[tag]));
                }
                var skirtNames = chains.SelectMany(c => c.ts.Select(t => t.name)).ToList();
                var sway = chains.ToDictionary(c => c.name, c => new List<double>());
                var frames = new List<object>();
                var penFrames = new List<object>();
                var pelvis = bone["Pelvis"];

                GameObject bakeGo = null, camGo = null, lightGo = null;
                Mesh renderMesh = null;
                Camera cam = null;
                Light light = null;
                RenderTexture rt = null;
                Texture2D tex = null;
                bool renderClip = canRender && clipName == "rigtest" && rendersOn;
                if (renderClip)
                {
                    renderMesh = new Mesh { name = "player_tmp_render" };
                    tempObjects.Add(renderMesh);
                    bakeGo = new GameObject("player_tmp_render");
                    tempObjects.Add(bakeGo);
                    bakeGo.AddComponent<MeshFilter>().sharedMesh = renderMesh;
                    var mr = bakeGo.AddComponent<MeshRenderer>();
                    var perSub = new List<Material>();
                    for (int i = 0; i < smr.sharedMesh.subMeshCount; i++)
                    {
                        string n = i < smrMats.Count ? smrMats[i] : "";
                        var col = matColors.TryGetValue(n, out var cc) ? new Color(cc[0], cc[1], cc[2], 1f) : new Color(0.8f, 0.8f, 0.8f, 1f);
                        if (!matColors.ContainsKey(n)) errors.Add($"render: no parts.json colour for material '{n}'");
                        var m = new Material(shader) { color = col };
                        m.SetFloat("_Glossiness", 0.2f);
                        m.SetFloat("_Metallic", 0f);
                        tempObjects.Add(m);
                        perSub.Add(m);
                    }
                    mr.sharedMaterials = perSub.ToArray();   // one material per submesh (goblin HR2 lesson)
                    RenderSettings.ambientMode = AmbientMode.Flat;
                    RenderSettings.ambientLight = new Color(Ambient, Ambient, Ambient, 1f);
                    RenderSettings.skybox = null;
                    RenderSettings.reflectionIntensity = 0f;
                    lightGo = new GameObject("player_tmp_light");
                    tempObjects.Add(lightGo);
                    light = lightGo.AddComponent<Light>();
                    light.type = LightType.Directional;
                    light.intensity = 1f;
                    light.shadows = LightShadows.None;
                    camGo = new GameObject("player_tmp_cam");
                    tempObjects.Add(camGo);
                    cam = camGo.AddComponent<Camera>();
                    cam.enabled = false;
                    cam.orthographic = true;
                    cam.clearFlags = CameraClearFlags.SolidColor;
                    cam.backgroundColor = Color.white;
                    cam.aspect = 1f;
                    rt = new RenderTexture(RenderSize, RenderSize, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 1 };
                    tex = new Texture2D(RenderSize, RenderSize, TextureFormat.RGBA32, false);
                }
                smr.enabled = !renderClip;
                void RenderAt(int f, string tag)
                {
                    smr.BakeMesh(renderMesh, true);
                    renderMesh.RecalculateBounds();
                    bakeGo.transform.SetPositionAndRotation(smr.transform.position, smr.transform.rotation);
                    bakeGo.transform.localScale = Vector3.one;
                    foreach (var kv in cams)
                    {
                        if (renderCams != null && !renderCams.Contains(kv.Key)) continue;
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
                        string file = renderPrefix == null ? $"unity_{kv.Key}_{f}.png" : $"{renderPrefix}{kv.Key}_{f}_{tag}.png";
                        Directory.CreateDirectory(inspect);
                        File.WriteAllBytes(Path.Combine(inspect, file), tex.EncodeToPNG());
                        renders.Add(Path.GetRelativePath(repo, Path.Combine(inspect, file)).Replace('\\', '/'));
                    }
                }
                var swStep = new Stopwatch();
                for (int f = f0; f <= f1; f++)
                {
                    swStep.Restart();
                    clip.SampleAnimation(inst, (float)((f - cs.firstFrame) / fps));
                    var skirtRaw = new JObj();
                    foreach (var n in skirtNames) skirtRaw.Add(n, Pose(bone[n]));
                    var tipsRaw = chains.Select(c => c.ts[1].TransformPoint(c.tipLocal)).ToList();
                    var poseRaw = pelvis.worldToLocalMatrix;
                    tSample += swStep.Elapsed.TotalSeconds;
                    swStep.Restart();
                    var wOff = BakeWorld(smr);
                    var pOff = Poke(wOff, pelvisRest * poseRaw);
                    foreach (var v in wOff) { unionMin = Vector3.Min(unionMin, v); unionMax = Vector3.Max(unionMax, v); }
                    tPen += swStep.Elapsed.TotalSeconds;
                    if (cam != null && renderOff && renderFrames.Contains(f))
                    {
                        swStep.Restart();
                        RenderAt(f, "off");
                        tRender += swStep.Elapsed.TotalSeconds;
                    }
                    swStep.Restart();
                    foreach (var c in chains) c.sbc.Step(dt);
                    var bw = new JObj();
                    foreach (var n in boneNames) bw.Add(n, Pose(bone[n]));
                    for (int i = 0; i < chains.Count; i++)
                        sway[chains[i].name].Add((chains[i].ts[1].TransformPoint(chains[i].tipLocal) - tipsRaw[i]).magnitude * 1000.0);
                    if (keepFrames) frames.Add(new JObj { { "f", f }, { "bones_world", bw }, { "skirt_raw", skirtRaw } });
                    tSample += swStep.Elapsed.TotalSeconds;
                    swStep.Restart();
                    var wOn = BakeWorld(smr);
                    var pOn = Poke(wOn, pelvisRest * pelvis.worldToLocalMatrix);
                    foreach (var v in wOn) { unionMin = Vector3.Min(unionMin, v); unionMax = Vector3.Max(unionMax, v); }
                    unionFrames++;
                    penFrames.Add(new JObj { { "f", f }, { "spring_on", pOn.count }, { "spring_off", pOff.count },
                                             { "max_depth_mm_on", pOn.maxDepth * 1000.0 }, { "max_depth_mm_off", pOff.maxDepth * 1000.0 } });
                    tPen += swStep.Elapsed.TotalSeconds;
                    if (cam != null && renderFrames.Contains(f))
                    {
                        swStep.Restart();
                        RenderAt(f, "on");
                        tRender += swStep.Elapsed.TotalSeconds;
                    }
                }
                if (rt != null) { rt.Release(); Object.DestroyImmediate(rt); }
                if (tex != null) Object.DestroyImmediate(tex);
                clipsOut.Add(clipName, new JObj {
                    { "path", cp }, { "firstFrame", cs.firstFrame }, { "lastFrame", cs.lastFrame }, { "frameRate", clip.frameRate },
                    { "time_mapping", $"t = (f - firstFrame) / {fps}" },
                    { "frames_rule", "bones_world = every bone after the spring step (springs ON); skirt_raw = Skirt_* bones right after SampleAnimation (springs OFF); [px, py, pz, qx, qy, qz, qw], Unity world, character root at the origin" },
                    { "frames", keepFrames ? frames : null },
                    { "spring", new JObj {
                        { "params", new JObj {
                            { "stiffness", Stiffness }, { "damping", Damping }, { "gravity", Gravity }, { "maxAngleDeg", MaxAngleDeg },
                            { "dt", dt }, { "source", ov == null ? "T253 default = T252 sweep r15 (user choice 2026-09-27)" : "PLAYER_RIGCHECK_OVERRIDE" },
                            { "init", $"Init() once after SampleAnimation(frame {f0}); state carried frame to frame" },
                            { "order", "per frame: SampleAnimation -> Step(dt) for each chain in F, FL, L, BL, B, BR, R, FR order" },
                            { "tip", "Skirt_*_02 tail = canonical_skeleton.json tail (Blender) -> unity (-x, z, -y) at rest, carried in the Skirt_*_02 local space; sway_mm = |tip_spring - tip_raw| * 1000" } } },
                        { "colliders", colInfo },
                        { "chains", chains.Select(c => (object)new JObj {
                            { "name", c.name }, { "bones", c.ts.Select(t => (object)t.name).ToList() },
                            { "tip_sway_mm_per_frame", sway[c.name].ToArray() },
                            { "max_sway_mm", sway[c.name].Count == 0 ? 0.0 : sway[c.name].Max() },
                            { "max_frame", sway[c.name].Count == 0 ? -1 : f0 + sway[c.name].IndexOf(sway[c.name].Max()) } }).ToList() } } },
                    { "penetration", new JObj { { "rule", PenRule }, { "per_frame", penFrames } } } });
                Object.DestroyImmediate(inst);
                tempObjects.Remove(inst);
            }
            report.Add("clips", clipsOut);
            timing.Add("sampling_and_springs", tSample);
            timing.Add("penetration_and_bounds", tPen);
            timing.Add("renders", tRender);

            // --- 5. bounds: union of all Player@ clips (springs off and on) + margin, applied to Assets/Player/Player.prefab
            var world = new Bounds();
            world.SetMinMax(unionMin - Vector3.one * BoundsMargin, unionMax + Vector3.one * BoundsMargin);
            bool applied = false;
            JObj prefabInfo = null;
            GameObject tmp = null;
            try
            {
                tmp = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                tmp.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
                var tsmr = tmp.GetComponentInChildren<SkinnedMeshRenderer>(true);
                var space = tsmr.rootBone != null ? tsmr.rootBone : tsmr.transform;
                var toLocal = space.worldToLocalMatrix;
                Vector3 lmin = Vector3.positiveInfinity, lmax = Vector3.negativeInfinity;
                foreach (var x in new[] { world.min.x, world.max.x })
                    foreach (var y in new[] { world.min.y, world.max.y })
                        foreach (var z in new[] { world.min.z, world.max.z })
                        {
                            var p = toLocal.MultiplyPoint3x4(new Vector3(x, y, z));
                            lmin = Vector3.Min(lmin, p);
                            lmax = Vector3.Max(lmax, p);
                        }
                var lb = new Bounds();
                lb.SetMinMax(lmin, lmax);
                tsmr.localBounds = lb;
                if (applyBounds)
                {
                    PrefabUtility.SaveAsPrefabAsset(tmp, PrefabPath, out applied);
                    if (!applied) errors.Add($"SaveAsPrefabAsset failed: {PrefabPath}");
                }
                var pa = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
                var psmr = pa == null ? null : pa.GetComponentInChildren<SkinnedMeshRenderer>(true);
                prefabInfo = new JObj {
                    { "path", PrefabPath }, { "saved", applied }, { "rootBone", psmr == null || psmr.rootBone == null ? null : psmr.rootBone.name },
                    { "localBounds_rootBone_space", psmr == null ? null : new JObj { { "center", psmr.localBounds.center }, { "extents", psmr.localBounds.extents } } },
                    { "conversion", "the 8 corners of the world AABB (character root at the origin) -> rootBone local, their AABB (conservative)" },
                    { "updateWhenOffscreen", psmr != null && psmr.updateWhenOffscreen } };
            }
            finally
            {
                if (tmp != null) Object.DestroyImmediate(tmp);
            }
            report.Add("bounds", new JObj {
                { "space", "unity_world_char_at_origin" }, { "center", world.center }, { "extents", world.extents },
                { "min", world.min }, { "max", world.max }, { "margin_m", BoundsMargin },
                { "union_min", unionMin }, { "union_max", unionMax }, { "union_frames", unionFrames },
                { "union_rule", "every integer frame firstFrame..lastFrame of every Assets/Player/Player@*.fbx clip, BakeMesh world vertices with springs off and on, character root at the origin" },
                { "applied", applied }, { "prefab", prefabInfo } });
        }
        catch (Exception ex)
        {
            errors.Add("exception: " + ex);
        }
        finally
        {
            Application.logMessageReceived -= Capture;
            RenderTexture.active = null;
            foreach (var o in tempObjects) if (o != null) Object.DestroyImmediate(o);
            try { EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single); } catch (Exception) { }
        }
        report.Add("renders", renders);
        timing.Add("total", total.Elapsed.TotalSeconds);
        report.Add("timing_s", timing);
        extra.Add("log_warnings", warnings);
        report.Add("extra", extra);
        PlayerImport.WriteJson(outPath, report);
        Debug.Log($"PLAYER RIG CHECK written: {outPath} errors={errors.Count}");
        foreach (var e in errors) Debug.Log("PLAYER RIG CHECK error: " + e);
        EditorApplication.Exit(errors.Count == 0 ? 0 : 1);
    }

    // Moller-Trumbore, two-sided; returns t > 0 or -1.
    static float RayTri(Vector3 o, Vector3 d, Vector3 a, Vector3 b, Vector3 c)
    {
        var e1 = b - a;
        var e2 = c - a;
        var p = Vector3.Cross(d, e2);
        float det = Vector3.Dot(e1, p);
        if (Mathf.Abs(det) < 1e-12f) return -1f;
        float inv = 1f / det;
        var s = o - a;
        float u = Vector3.Dot(s, p) * inv;
        if (u < 0f || u > 1f) return -1f;
        var q = Vector3.Cross(s, e1);
        float v = Vector3.Dot(d, q) * inv;
        if (v < 0f || u + v > 1f) return -1f;
        float t = Vector3.Dot(e2, q) * inv;
        return t > 1e-7f ? t : -1f;
    }
}
