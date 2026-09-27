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

// T300 (P3 tooling, spec work/player/d-03-player-anim-tools.md section 2): data collection for ONE player clip in Unity.
// The clip name comes from the environment variable PLAYER_CLIP. Writes unity/AvatarCheck/player_clip_report_<clip>.json
// (player_report.json schema + root_motion) and BakeMesh renders work/player/inspect/clips/<clip>/unity_<cam>_<f>.png at
// the clip's key frames. Records only; no judgement. Exit 0 when `errors` is empty, else 1.
// Adapted from Assets/Editor/Player/PlayerRigCheck.cs (T250/T253; read only: spring r15 + followAnimation, collider
// capsules, poke rule, BakeMesh renders with one material per submesh, bounds union -> Player.prefab) and
// Assets/Editor/Goblin/GoblinClipCheck.cs (clip / importer block). Goblin files, PlayerRigCheck.cs, SpringBoneChain.cs and
// SpringCollider.cs are used unchanged.
//
// Clip import (Assets/Player/Player@<clip>.fbx), from work/player/rig/data/clips/<clip>.json:
//   PlayerImport (Generic, CopyFromOther Player.fbx avatar, compression Off) + clipAnimations = one clip named <clip>
//   (the default take, frames unchanged), loopTime = JSON loop, loopPose false, and the root-motion policy:
//     "locked" / "in_place_cycle": motionNodeName "" (no root node: the Root curves stay plain transform curves, i.e.
//        baked into the pose), lockRootRotation / lockRootHeightY / lockRootPositionXZ true, keepOriginal* true.
//     "root": motionNodeName "Player/Root" (root motion extracted from Root), lockRoot* false, keepOriginal* true.
//   Every setting is recorded in import.clip_settings.
// Per frame of the clip (t = (f - firstFrame) / frameRate, a fresh Player.fbx instance at the origin):
//   SampleAnimation -> skirt_raw (springs OFF) + raw tips -> BakeMesh -> poke spring_off -> SpringBoneChain.Step(1/fps) ->
//   bones_world (springs ON) -> sway -> BakeMesh -> poke spring_on (+ renders at the key frames) -> root_path.
// Bounds: every frame of EVERY Assets/Player/Player@*.fbx clip (each at its own frameRate, springs off and on), + 0.1 m,
// applied to Assets/Player/Player.prefab (space unity_world_char_at_origin).
// T313 weapon preview: when the clip JSON has "weapon" {side: name}, Assets/Player/Preview/<object>.fbx (object from
// work/player/rig/data/weapon_socket_contract.json weapons.<name>.object; p34_weapon_preview.py, which bakes the measured
// Unity-vs-Blender bone-frame compensation into the mesh) is imported as a plain model (animationType None, no animation)
// and instantiated under WeaponSocket_<side> with an identity local pose (the approved contract, unity.attach); it is
// drawn in the BakeMesh renders (one material per submesh) and its blade tip / guard ends / pommel are reported per frame
// under "weapon". It is never part of the bounds (character SkinnedMeshRenderer only).
// T328 events: when the clip JSON has "events" [{name, frame}], they are written into the importer's clipAnimations[0].events
// (AnimationEvent functionName = name, time = normalized (frame - firstFrame) / (lastFrame - firstFrame)) and reported in
// import.clip_settings.after[0].events and clips.<clip>.events (imported clip, seconds). No receiver is added:
// AnimationClip.SampleAnimation does not fire events.
// Optional env PLAYER_CLIP_RENDER_FRAMES = "all" or a comma list: extra render frames of the checked clip (default: the
// key frames only); used for review media (p34 --unity-media).
public static class PlayerClipCheck
{
    const string ClipEnv = "PLAYER_CLIP";
    const float BoundsMargin = 0.10f;
    const string PrefabPath = PlayerImport.Folder + "/Player.prefab";
    const string MotionNodePath = "Player/Root";
    const string PreviewFolder = PlayerImport.Folder + "/Preview";      // T313
    const string RenderFramesEnv = "PLAYER_CLIP_RENDER_FRAMES";          // T313
    const int RenderSize = 1024;
    // PlayerRigCheck.cs:37 r15 defaults (user choice 2026-09-27)
    const float Stiffness = 0.3f, Damping = 0.5f, Gravity = 1.5f, MaxAngleDeg = 25f;
    static readonly float[] ColliderT = { 1f / 6f, 0.5f, 5f / 6f };
    const float ThighRadius = 0.045f, CalfRadius = 0.055f;
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
        + "after the spring step. Counts are poke-out vertices. (Copied from PlayerRigCheck.cs.)";

    static float[] Pose(Transform t) =>
        new[] { t.position.x, t.position.y, t.position.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w };

    static Vector3 FromBlender(IList<object> p) =>
        new Vector3(-(float)(double)p[0], (float)(double)p[2], -(float)(double)p[1]);

    static JObj ClipSettingsObj(ModelImporter mi, ModelImporterClipAnimation c, double fps = 0) => new JObj {
        { "events", (c.events ?? new AnimationEvent[0]).Select(e => (object)new JObj {   // T328
            { "functionName", e.functionName }, { "time_normalized", (double)e.time },
            { "time_s", fps > 0 ? e.time * (c.lastFrame - c.firstFrame) / fps : (double?)null },
            { "time", fps > 0 ? e.time * (c.lastFrame - c.firstFrame) / fps : (double?)null },
            { "frame", c.firstFrame + e.time * (c.lastFrame - c.firstFrame) } }).ToList() },
        { "motionNodeName", mi.motionNodeName }, { "name", c.name }, { "takeName", c.takeName },
        { "firstFrame", c.firstFrame }, { "lastFrame", c.lastFrame }, { "loopTime", c.loopTime }, { "loopPose", c.loopPose },
        { "cycleOffset", c.cycleOffset }, { "lockRootRotation", c.lockRootRotation }, { "lockRootHeightY", c.lockRootHeightY },
        { "lockRootPositionXZ", c.lockRootPositionXZ }, { "keepOriginalOrientation", c.keepOriginalOrientation },
        { "keepOriginalPositionY", c.keepOriginalPositionY }, { "keepOriginalPositionXZ", c.keepOriginalPositionXZ },
        { "heightFromFeet", c.heightFromFeet }, { "mirror", c.mirror }, { "rotationOffset", c.rotationOffset },
        { "heightOffset", c.heightOffset }, { "wrapMode", c.wrapMode.ToString() } };

    // Clip importer settings per the JSON (loop, root_motion). Returns the recorded block; re-imports when anything differs.
    static JObj EnsureClipSettings(string clipPath, string clipName, bool loop, string policy, List<string> errors,
                                   List<(string name, int frame)> events = null, double fps = 0)
    {
        var mi = (ModelImporter)AssetImporter.GetAtPath(clipPath);
        if (mi == null) throw new Exception($"no ModelImporter for {clipPath}");
        var defs = mi.defaultClipAnimations;
        if (defs.Length != 1) errors.Add($"{clipPath}: {defs.Length} default takes (expected 1)");
        var before = (mi.clipAnimations.Length > 0 ? mi.clipAnimations : defs).Select(c => (object)ClipSettingsObj(mi, c, fps)).ToList();
        bool root = policy == "root";
        var want = defs[0];
        want.name = clipName;
        want.loopTime = loop;
        want.loopPose = false;
        want.lockRootRotation = !root;
        want.lockRootHeightY = !root;
        want.lockRootPositionXZ = !root;
        want.keepOriginalOrientation = true;
        want.keepOriginalPositionY = true;
        want.keepOriginalPositionXZ = true;
        if (events != null && events.Count > 0)   // T328: AnimationEvents, normalized time over the take
        {
            double span = want.lastFrame - want.firstFrame;
            want.events = events.Select(e => new AnimationEvent {
                functionName = e.name, time = (float)((e.frame - want.firstFrame) / span) }).ToArray();
        }
        else want.events = new AnimationEvent[0];
        string wantNode = root ? MotionNodePath : "";
        var changed = new List<object>();
        var cur = mi.clipAnimations.Length == 1 ? mi.clipAnimations[0] : null;
        bool sameEvents = cur != null && (cur.events ?? new AnimationEvent[0]).Length == want.events.Length
            && (cur.events ?? new AnimationEvent[0]).Zip(want.events, (x, y) => x.functionName == y.functionName
                && Math.Abs(x.time - y.time) < 1e-6f).All(b => b);   // T328
        bool same = sameEvents && cur != null && cur.name == want.name && cur.takeName == want.takeName
            && Math.Abs(cur.firstFrame - want.firstFrame) < 1e-6 && Math.Abs(cur.lastFrame - want.lastFrame) < 1e-6
            && cur.loopTime == want.loopTime && cur.loopPose == want.loopPose
            && cur.lockRootRotation == want.lockRootRotation && cur.lockRootHeightY == want.lockRootHeightY
            && cur.lockRootPositionXZ == want.lockRootPositionXZ && cur.keepOriginalOrientation == want.keepOriginalOrientation
            && cur.keepOriginalPositionY == want.keepOriginalPositionY && cur.keepOriginalPositionXZ == want.keepOriginalPositionXZ;
        if (!same) { mi.clipAnimations = new[] { want }; changed.Add("clipAnimations"); }
        if ((mi.motionNodeName ?? "") != wantNode) { changed.Add($"motionNodeName: '{mi.motionNodeName}' -> '{wantNode}'"); mi.motionNodeName = wantNode; }
        if (changed.Count > 0) mi.SaveAndReimport();
        mi = (ModelImporter)AssetImporter.GetAtPath(clipPath);
        var after = mi.clipAnimations.Select(c => (object)ClipSettingsObj(mi, c, fps)).ToList();
        if (mi.clipAnimations.Length != 1) errors.Add($"{clipPath}: clipAnimations count {mi.clipAnimations.Length} after ensure");
        else
        {
            var a = mi.clipAnimations[0];
            if (a.loopTime != loop) errors.Add($"{clipPath}: loopTime {a.loopTime} != JSON loop {loop}");
            if (a.name != clipName) errors.Add($"{clipPath}: clip name {a.name} != {clipName}");
            if (a.lockRootPositionXZ != !root) errors.Add($"{clipPath}: lockRootPositionXZ {a.lockRootPositionXZ} for policy {policy}");
        }
        if ((mi.motionNodeName ?? "") != wantNode) errors.Add($"{clipPath}: motionNodeName '{mi.motionNodeName}' != '{wantNode}' for policy {policy}");
        if (mi.animationType != ModelImporterAnimationType.Generic) errors.Add($"{clipPath}: animationType {mi.animationType}");
        if (mi.avatarSetup != ModelImporterAvatarSetup.CopyFromOther) errors.Add($"{clipPath}: avatarSetup {mi.avatarSetup}");
        if (mi.sourceAvatar == null || AssetDatabase.GetAssetPath(mi.sourceAvatar) != PlayerImport.ModelPath)
            errors.Add($"{clipPath}: sourceAvatar {(mi.sourceAvatar == null ? "null" : AssetDatabase.GetAssetPath(mi.sourceAvatar))}");
        if (mi.animationCompression != ModelImporterAnimationCompression.Off) errors.Add($"{clipPath}: compression {mi.animationCompression}");
        return new JObj {
            { "path", clipPath }, { "policy", policy }, { "json_loop", loop },
            { "policy_mapping", root
                ? "root: motionNodeName = Player/Root (root motion extracted from the Root bone), lockRootRotation / lockRootHeightY / lockRootPositionXZ false (kept as root motion), keepOriginal* true"
                : policy + ": motionNodeName '' (no root node: Root curves stay plain transform curves = baked into the pose), lockRoot* true, keepOriginal* true" },
            { "animationType", mi.animationType.ToString() }, { "avatarSetup", mi.avatarSetup.ToString() },
            { "sourceAvatar", mi.sourceAvatar == null ? null : AssetDatabase.GetAssetPath(mi.sourceAvatar) },
            { "compression", mi.animationCompression.ToString() }, { "resampleCurves", mi.resampleCurves },
            { "before", before }, { "changed", changed }, { "resaved", changed.Count > 0 }, { "after", after } };
    }

    // T313: the weapon preview FBX as a plain model (no rig, no animation). Re-imports when a setting differs.
    static JObj EnsureWeaponImport(string path, List<string> errors)
    {
        if (!File.Exists(PlayerImport.FullPath(path))) throw new Exception($"weapon preview {path} missing (p34_weapon_preview.py)");
        AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceSynchronousImport);
        var mi = (ModelImporter)AssetImporter.GetAtPath(path);
        if (mi == null) throw new Exception($"no ModelImporter for {path}");
        var changed = new List<object>();
        if (mi.animationType != ModelImporterAnimationType.None) { changed.Add($"animationType: {mi.animationType} -> None"); mi.animationType = ModelImporterAnimationType.None; }
        if (mi.importAnimation) { changed.Add("importAnimation: True -> False"); mi.importAnimation = false; }
        if (mi.importBlendShapes) { changed.Add("importBlendShapes: True -> False"); mi.importBlendShapes = false; }
        if (mi.importCameras) { changed.Add("importCameras: True -> False"); mi.importCameras = false; }
        if (mi.importLights) { changed.Add("importLights: True -> False"); mi.importLights = false; }
        if (changed.Count > 0) mi.SaveAndReimport();
        mi = (ModelImporter)AssetImporter.GetAtPath(path);
        if (mi.animationType != ModelImporterAnimationType.None) errors.Add($"{path}: animationType {mi.animationType}");
        return new JObj {
            { "path", path }, { "changed", changed }, { "resaved", changed.Count > 0 },
            { "animationType", mi.animationType.ToString() }, { "importAnimation", mi.importAnimation },
            { "importBlendShapes", mi.importBlendShapes }, { "importCameras", mi.importCameras }, { "importLights", mi.importLights },
            { "globalScale", mi.globalScale }, { "useFileScale", mi.useFileScale }, { "fileScale", mi.fileScale },
            { "bakeAxisConversion", mi.bakeAxisConversion }, { "materialImportMode", mi.materialImportMode.ToString() },
            { "importNormals", mi.importNormals.ToString() },
            { "subAssetTypes", AssetDatabase.LoadAllAssetsAtPath(path).Where(s => s != null).GroupBy(s => s.GetType().Name)
                .OrderBy(g => g.Key).Select(g => (object)$"{g.Key} x{g.Count()}").ToList() } };
    }

    // T313: renders for the checked clip at the key frames, or every / listed frame (PLAYER_CLIP_RENDER_FRAMES)
    static bool RenderClipFor(bool full, bool canRender, HashSet<int> keyFrames, bool renderAll, HashSet<int> extraFrames) =>
        full && canRender && (keyFrames.Count > 0 || renderAll || extraFrames.Count > 0);

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
        string clipName = Environment.GetEnvironmentVariable(ClipEnv);
        if (string.IsNullOrEmpty(clipName))
        {
            Debug.Log($"PLAYER CLIP CHECK error: environment variable {ClipEnv} not set");
            EditorApplication.Exit(1);
            return;
        }
        string outPath = Path.Combine(repo, "unity", "AvatarCheck", $"player_clip_report_{clipName}.json");
        string inspect = Path.Combine(repo, "work", "player", "inspect", "clips", clipName);
        string clipAsset = PlayerImport.Folder + $"/Player@{clipName}.fbx";
        var report = new JObj();
        var extra = new JObj();
        var renders = new List<object>();
        report.Add("unity_version", Application.unityVersion);
        report.Add("errors", errors);
        report.Add("clip", clipName);
        var tempObjects = new List<Object>();
        Application.logMessageReceived += Capture;
        try
        {
            // --- 0. clip JSON + resolved JSON --------------------------------------------------------------------
            string clipJsonPath = Path.Combine(rigDir, "data", "clips", clipName + ".json");
            var cj = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(clipJsonPath));
            bool loop = (bool)cj["loop"];
            string policy = (string)cj["root_motion"];
            double jsonFps = cj.ContainsKey("fps") ? (double)cj["fps"] : 30.0;
            var fr = ((List<object>)cj["frame_range"]).Select(x => (int)(double)x).ToArray();
            var clipEvents = cj.TryGetValue("events", out var evSpec) && evSpec is List<object> evList   // T328
                ? evList.Cast<Dictionary<string, object>>().Select(e => ((string)e["name"], (int)(double)e["frame"])).ToList()
                : new List<(string name, int frame)>();
            string resolvedPath = Path.Combine(rigDir, "anim", clipName + "_resolved.json");
            List<object> kf = null;
            string keySource = null;
            if (File.Exists(resolvedPath))
            {
                var rj = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(resolvedPath));
                if (rj.TryGetValue("key_frames", out var k)) { kf = (List<object>)k; keySource = "rig/anim/" + clipName + "_resolved.json key_frames"; }
            }
            if (kf == null && cj.TryGetValue("key_frames", out var k2)) { kf = (List<object>)k2; keySource = "clip JSON key_frames"; }
            var keyFrames = new HashSet<int>((kf ?? new List<object>()).Select(x => (int)(double)x));
            if (kf == null) errors.Add("no key_frames (resolved JSON or clip JSON); renders skipped");
            extra.Add("clip_json", new JObj { { "path", clipJsonPath.Replace('\\', '/') }, { "loop", loop }, { "root_motion", policy },
                { "fps", jsonFps }, { "frame_range", fr.Select(x => (object)x).ToList() },
                { "key_frames", keyFrames.OrderBy(x => x).Select(x => (object)x).ToList() }, { "key_frames_source", keySource } });
            // T313: weapon preview + optional extra render frames
            string wSide = null, wName = null, wAsset = null, wObject = null;
            if (cj.TryGetValue("weapon", out var wSpec) && wSpec is Dictionary<string, object> wDict && wDict.Count > 0)
            {
                if (wDict.Count != 1) errors.Add($"weapon: {wDict.Count} sides (one supported)");
                var w0 = wDict.First();
                wSide = w0.Key;
                wName = (string)w0.Value;
                var con = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(Path.Combine(rigDir, "data", "weapon_socket_contract.json")));
                wObject = (string)((Dictionary<string, object>)((Dictionary<string, object>)con["weapons"])[wName])["object"];
                wAsset = PreviewFolder + "/" + wObject + ".fbx";
            }
            string renderEnv = Environment.GetEnvironmentVariable(RenderFramesEnv);
            bool renderAll = renderEnv == "all";
            var extraFrames = new HashSet<int>(renderAll || string.IsNullOrEmpty(renderEnv) ? new int[0]
                : renderEnv.Split(',').Select(s => int.Parse(s.Trim())));
            extra.Add("render_frames", new JObj { { "env", RenderFramesEnv }, { "value", renderEnv }, { "all", renderAll },
                { "extra", extraFrames.OrderBy(x => x).Select(x => (object)x).ToList() } });
            JObj weaponOut = null, weaponImport = null;

            // --- 1. import -----------------------------------------------------------------------------------------
            var sw = Stopwatch.StartNew();
            var resaved = PlayerImport.EnsureImported(true);
            var clipPaths = PlayerImport.ClipPaths();
            if (!clipPaths.Contains(clipAsset)) throw new Exception($"{clipAsset} not found in {PlayerImport.Folder}");
            var clipSettings = EnsureClipSettings(clipAsset, clipName, loop, policy, errors, clipEvents, jsonFps);
            if (wAsset != null) weaponImport = EnsureWeaponImport(wAsset, errors);   // T313
            timing.Add("import", sw.Elapsed.TotalSeconds);
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
                    { "compression", ci.animationCompression.ToString() }, { "motionNodeName", ci.motionNodeName } });
                if (ci.animationType != ModelImporterAnimationType.Generic) errors.Add($"{cp}: animationType {ci.animationType}");
            }
            if (mi.animationType != ModelImporterAnimationType.Generic) errors.Add($"Player.fbx animationType {mi.animationType}");
            report.Add("import", new JObj {
                { "animationType", mi.animationType.ToString() },
                { "avatar", avatar == null ? null : new JObj {
                    { "name", avatar.name }, { "isValid", avatar.isValid }, { "isHuman", avatar.isHuman } } },
                { "Player.fbx_motionNodeName", mi.motionNodeName },
                { "clips", clipImp }, { "clip_settings", clipSettings }, { "resaved_by_EnsureImported", resaved }, { "files", files } });

            // --- 2. data files --------------------------------------------------------------------------------------
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
            var p1 = (Dictionary<string, object>)GoblinJsonReader.Parse(File.ReadAllText(Path.Combine(rigDir, "data", "p1_manifest.json")));
            var cams = (Dictionary<string, object>)p1["cameras"];

            // --- 3. rest instance: bones, bind poses, rest block ---------------------------------------------------
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
            bool bpOk = bp.Length == sb.Length && sb.All(b => b != null);
            for (int i = 0; i < Math.Min(bp.Length, sb.Length); i++)
            {
                if (sb[i] == null) continue;
                var want = sb[i].worldToLocalMatrix * rsmr.transform.localToWorldMatrix;
                double d = 0;
                for (int e = 0; e < 16; e++) d = Math.Max(d, Math.Abs(want[e] - bp[i][e]));
                if (d > bpMax) { bpMax = d; bpWorst = sb[i].name; }
            }
            bpOk &= bpMax < 1e-4;
            report.Add("bindposes_ok", bpOk);
            var smrMats = rsmr.sharedMaterials.Select(m => m == null ? "" : m.name).ToList();
            extra.Add("skinned_mesh", new JObj {
                { "mesh", rsmr.sharedMesh.name }, { "vertexCount", rsmr.sharedMesh.vertexCount },
                { "subMeshCount", rsmr.sharedMesh.subMeshCount }, { "materials", smrMats.Select(x => (object)x).ToList() },
                { "smr_bones", sb.Length }, { "bindposes", bp.Length }, { "bindpose_max_abs_diff_vs_rest", bpMax },
                { "bindpose_worst_bone", bpWorst },
                { "bindpose_rule", "bindposes[i] vs bones[i].worldToLocalMatrix * smr.localToWorldMatrix on a fresh Player.fbx instance at the origin, max abs element; ok = counts equal, no null bone, max < 1e-4" },
                { "rootBone", rsmr.rootBone == null ? null : rsmr.rootBone.name } });

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
            Object.DestroyImmediate(restInst);
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
                    if (far < r) { count++; worst = Mathf.Max(worst, r - far); }
                }
                return (count, worst);
            }

            // --- 4. every Player@ clip: bounds (springs off / on); the checked clip also frames, springs, poke, renders
            var clipsOut = new JObj();
            var boundsPerClip = new JObj();
            var unionMin = Vector3.positiveInfinity;
            var unionMax = Vector3.negativeInfinity;
            int unionFrames = 0;
            var shader = Shader.Find("Standard");
            bool canRender = SystemInfo.graphicsDeviceType != GraphicsDeviceType.Null && shader != null;
            if (!canRender) errors.Add("no graphics device or Standard shader; renders skipped");
            double tSample = 0, tPen = 0, tRender = 0, tOther = 0;
            JObj rootMotion = null;
            foreach (var cp in clipPaths)
            {
                bool full = cp == clipAsset;
                var swClip = Stopwatch.StartNew();
                var clip = AssetDatabase.LoadAllAssetsAtPath(cp).OfType<AnimationClip>().FirstOrDefault(c => !c.name.StartsWith("__preview__"));
                if (clip == null) { errors.Add($"no AnimationClip in {cp}"); continue; }
                var ci = (ModelImporter)AssetImporter.GetAtPath(cp);
                var cs = (ci.clipAnimations.Length > 0 ? ci.clipAnimations : ci.defaultClipAnimations).FirstOrDefault();
                if (cs == null) { errors.Add($"no clip settings in {cp}"); continue; }
                int f0 = (int)Math.Ceiling(cs.firstFrame - 1e-6), f1 = (int)Math.Floor(cs.lastFrame + 1e-6);
                string cn = Path.GetFileNameWithoutExtension(cp).Substring("Player@".Length);
                float fps = clip.frameRate;
                float dt = 1f / fps;
                if (full)
                {
                    if (Math.Abs(fps - jsonFps) > 1e-3) errors.Add($"{cp}: frameRate {fps} != JSON fps {jsonFps}");
                    if (f0 != fr[0] || f1 != fr[1]) errors.Add($"{cp}: frames {f0}..{f1} != JSON frame_range {fr[0]}..{fr[1]}");
                }

                var inst = NewInstance();
                var arm = inst.transform.Find("Player");
                var bone = arm.GetComponentsInChildren<Transform>(true).Where(t => t != arm).GroupBy(t => t.name).ToDictionary(g => g.Key, g => g.First());
                var smr = inst.GetComponentInChildren<SkinnedMeshRenderer>(true);
                var tipLocals = new Dictionary<string, Vector3>();
                foreach (var tag in SkirtTags)
                {
                    string b2 = $"Skirt_{tag}_02";
                    tipLocals[tag] = bone[b2].InverseTransformPoint(FromBlender((List<object>)canonBones[b2]["tail"]));
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
                var chains = new List<(string name, SpringBoneChain sbc, Transform[] ts, Vector3 tipLocal)>();
                foreach (var tag in SkirtTags)
                {
                    var ts = new[] { bone[$"Skirt_{tag}_01"], bone[$"Skirt_{tag}_02"] };
                    var sbc = ts[0].gameObject.AddComponent<SpringBoneChain>();
                    sbc.bones = ts;
                    sbc.stiffness = Stiffness; sbc.damping = Damping; sbc.gravity = Gravity; sbc.maxAngleDeg = MaxAngleDeg;
                    sbc.followAnimation = true;   // P2e: keep the baked skirt follow
                    sbc.colliders = cols.ToArray();
                    sbc.Init();
                    chains.Add(($"Skirt_{tag}", sbc, ts, tipLocals[tag]));
                }
                var skirtNames = chains.SelectMany(c => c.ts.Select(t => t.name)).ToList();
                var sway = chains.ToDictionary(c => c.name, c => new List<double>());
                var frames = new List<object>();
                var penFrames = new List<object>();
                var rootPath = new List<object>();
                var pelvis = bone["Pelvis"];
                var rootBone = bone["Root"];
                var bindings = AnimationUtility.GetCurveBindings(clip);
                var motionBindings = bindings.Where(b => b.type == typeof(Animator)
                    && (b.propertyName.StartsWith("MotionT") || b.propertyName.StartsWith("MotionQ")
                        || b.propertyName.StartsWith("RootT") || b.propertyName.StartsWith("RootQ"))).ToList();
                var motionCurves = motionBindings.Select(b => (name: b.propertyName, curve: AnimationUtility.GetEditorCurve(clip, b))).ToList();
                Vector3 cMin = Vector3.positiveInfinity, cMax = Vector3.negativeInfinity;
                bool renderClip = RenderClipFor(full, canRender, keyFrames, renderAll, extraFrames);

                // T313: weapon preview under WeaponSocket_<side>, identity local pose (approved contract)
                GameObject wGo = null;
                Transform wMeshT = null;
                var wPoints = new Dictionary<string, Vector3>();
                var wFrames = new List<object>();
                JObj wInfo = null;
                if (full && wAsset != null)
                {
                    var wPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(wAsset);
                    if (wPrefab == null) throw new Exception($"weapon preview {wAsset} not loadable");
                    var sock = bone[$"WeaponSocket_{wSide}"];
                    wGo = Object.Instantiate(wPrefab);
                    wGo.name = wPrefab.name;
                    wGo.transform.SetParent(sock, false);
                    wGo.transform.localPosition = Vector3.zero;
                    wGo.transform.localRotation = Quaternion.identity;
                    wGo.transform.localScale = Vector3.one;
                    var mfs = wGo.GetComponentsInChildren<MeshFilter>(true);
                    if (mfs.Length != 1) errors.Add($"weapon preview {wAsset}: {mfs.Length} MeshFilters (expected 1)");
                    wMeshT = mfs[0].transform;
                    var wm = mfs[0].sharedMesh;
                    var wvs = wm.vertices;
                    Vector3 Cen(Func<Vector3, bool> sel) { var s = wvs.Where(sel).ToList(); return s.Aggregate(Vector3.zero, (x, y) => x + y) / s.Count; }
                    float zMax = wvs.Max(v => v.z), zMin = wvs.Min(v => v.z), yMax = wvs.Max(v => v.y), yMin = wvs.Min(v => v.y);
                    wPoints["tip"] = Cen(v => v.z >= zMax - 1e-5f);
                    wPoints["guard_a"] = Cen(v => v.y >= yMax - 1e-4f);
                    wPoints["guard_b"] = Cen(v => v.y <= yMin + 1e-4f);
                    wPoints["pommel"] = Cen(v => v.z <= zMin + 1e-5f);
                    var wmr = mfs[0].GetComponent<MeshRenderer>();
                    var impMats = wmr == null ? new Material[0] : wmr.sharedMaterials;
                    wInfo = new JObj {
                        { "hierarchy", wGo.GetComponentsInChildren<Transform>(true).Select(x => (object)new JObj {
                            { "path", x == wGo.transform ? wGo.name : wGo.name + "/" + PlayerImport.PathOf(x, wGo.transform) },
                            { "localPosition", x.localPosition }, { "localRotation", x.localRotation }, { "localScale", x.localScale } }).ToList() },
                        { "mesh", new JObj { { "name", wm.name }, { "vertexCount", wm.vertexCount }, { "subMeshCount", wm.subMeshCount },
                            { "bounds_min", wm.bounds.min }, { "bounds_max", wm.bounds.max } } },
                        { "materials", impMats.Select(m => (object)new JObj { { "name", m == null ? null : m.name },
                            { "color", m != null && m.HasProperty("_Color") ? (object)m.color : null } }).ToList() } };
                    if (renderClip && wmr != null)
                    {
                        var wSub = new List<Material>();
                        for (int i = 0; i < wm.subMeshCount; i++)
                        {
                            var src = i < impMats.Length ? impMats[i] : null;
                            var col = src != null && src.HasProperty("_Color") ? src.color : new Color(0.7f, 0.7f, 0.7f, 1f);
                            var m = new Material(shader) { color = new Color(col.r, col.g, col.b, 1f) };
                            m.SetFloat("_Glossiness", 0.2f);
                            m.SetFloat("_Metallic", 0f);
                            tempObjects.Add(m);
                            wSub.Add(m);
                        }
                        wmr.sharedMaterials = wSub.ToArray();   // one material per submesh
                    }
                }

                GameObject bakeGo = null, camGo = null, lightGo = null;
                Mesh renderMesh = null;
                Camera cam = null;
                Light light = null;
                RenderTexture rt = null;
                Texture2D tex = null;
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
                void RenderAt(int f)
                {
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
                        renders.Add(Path.GetRelativePath(repo, Path.Combine(inspect, file)).Replace('\\', '/'));
                    }
                }
                var swStep = new Stopwatch();
                for (int f = f0; f <= f1; f++)
                {
                    swStep.Restart();
                    float t = (float)((f - cs.firstFrame) / fps);
                    clip.SampleAnimation(inst, t);
                    var skirtRaw = new JObj();
                    if (full) foreach (var n in skirtNames) skirtRaw.Add(n, Pose(bone[n]));
                    var tipsRaw = chains.Select(c => c.ts[1].TransformPoint(c.tipLocal)).ToList();
                    var poseRaw = pelvis.worldToLocalMatrix;
                    if (full)
                    {
                        var mc = new JObj();
                        foreach (var m in motionCurves) mc.Add(m.name, m.curve == null ? (object)null : (double)m.curve.Evaluate(t));
                        rootPath.Add(new JObj { { "f", f }, { "t", t }, { "root_world", Pose(rootBone) },
                            { "root_local", new[] { rootBone.localPosition.x, rootBone.localPosition.y, rootBone.localPosition.z,
                                rootBone.localRotation.x, rootBone.localRotation.y, rootBone.localRotation.z, rootBone.localRotation.w } },
                            { "motion_curves", mc } });
                    }
                    tSample += swStep.Elapsed.TotalSeconds;
                    swStep.Restart();
                    var wOff = BakeWorld(smr);
                    var pOff = full ? Poke(wOff, pelvisRest * poseRaw) : (0, 0f);
                    foreach (var v in wOff) { cMin = Vector3.Min(cMin, v); cMax = Vector3.Max(cMax, v); }
                    tPen += swStep.Elapsed.TotalSeconds;
                    swStep.Restart();
                    foreach (var c in chains) c.sbc.Step(dt);
                    if (full)
                    {
                        var bw = new JObj();
                        foreach (var n in boneNames) bw.Add(n, Pose(bone[n]));
                        for (int i = 0; i < chains.Count; i++)
                            sway[chains[i].name].Add((chains[i].ts[1].TransformPoint(chains[i].tipLocal) - tipsRaw[i]).magnitude * 1000.0);
                        frames.Add(new JObj { { "f", f }, { "bones_world", bw }, { "skirt_raw", skirtRaw } });
                        if (wGo != null)   // T313: after the spring step (the hand is not a spring bone)
                            wFrames.Add(new JObj { { "f", f }, { "tip", wMeshT.TransformPoint(wPoints["tip"]) },
                                { "guard_a", wMeshT.TransformPoint(wPoints["guard_a"]) }, { "guard_b", wMeshT.TransformPoint(wPoints["guard_b"]) },
                                { "pommel", wMeshT.TransformPoint(wPoints["pommel"]) } });
                    }
                    tSample += swStep.Elapsed.TotalSeconds;
                    swStep.Restart();
                    var wOn = BakeWorld(smr);
                    foreach (var v in wOn) { cMin = Vector3.Min(cMin, v); cMax = Vector3.Max(cMax, v); }
                    unionFrames++;
                    if (full)
                    {
                        var pOn = Poke(wOn, pelvisRest * pelvis.worldToLocalMatrix);
                        penFrames.Add(new JObj { { "f", f }, { "spring_on", pOn.count }, { "spring_off", pOff.Item1 },
                                                 { "max_depth_mm_on", pOn.maxDepth * 1000.0 }, { "max_depth_mm_off", pOff.Item2 * 1000.0 } });
                    }
                    tPen += swStep.Elapsed.TotalSeconds;
                    if (cam != null && (keyFrames.Contains(f) || renderAll || extraFrames.Contains(f)))
                    {
                        swStep.Restart();
                        RenderAt(f);
                        tRender += swStep.Elapsed.TotalSeconds;
                    }
                }
                if (rt != null) { rt.Release(); Object.DestroyImmediate(rt); }
                if (tex != null) Object.DestroyImmediate(tex);
                unionMin = Vector3.Min(unionMin, cMin);
                unionMax = Vector3.Max(unionMax, cMax);
                boundsPerClip.Add(cn, new JObj {
                    { "path", cp }, { "frameRate", fps }, { "firstFrame", cs.firstFrame }, { "lastFrame", cs.lastFrame },
                    { "frames", f1 - f0 + 1 }, { "min", cMin }, { "max", cMax },
                    { "motionNodeName", ci.motionNodeName }, { "hasMotionCurves", clip.hasMotionCurves },
                    { "root_eval", clip.hasMotionCurves
                        ? "root motion curves extracted by the importer: the character is evaluated at the origin (Root path not in these bounds; see root_motion.root_path for the checked clip)"
                        : "no motion node: Root curves sampled as ordinary transform curves (included in these bounds)" } });
                if (full)
                {
                    clipsOut.Add(cn, new JObj {
                        { "path", cp }, { "firstFrame", cs.firstFrame }, { "lastFrame", cs.lastFrame }, { "frameRate", fps },
                        { "length_s", clip.length }, { "isLooping", clip.isLooping },
                        { "events", AnimationUtility.GetAnimationEvents(clip).Select(e => (object)new JObj {   // T328: imported clip
                            { "functionName", e.functionName }, { "time", (double)e.time }, { "time_s", (double)e.time },
                            { "frame", cs.firstFrame + e.time * fps } }).ToList() },
                        { "events_note", "AnimationUtility.GetAnimationEvents on the imported clip (time in seconds from the clip start; frame = firstFrame + time * frameRate). SampleAnimation does not fire events, so no receiver is needed for this check" },
                        { "hasRootCurves", clip.hasRootCurves }, { "hasMotionCurves", clip.hasMotionCurves },
                        { "hasGenericRootTransform", clip.hasGenericRootTransform },
                        { "time_mapping", "t = (f - firstFrame) / frameRate" },
                        { "frames_rule", "bones_world = every bone after the spring step (springs ON); skirt_raw = Skirt_* bones right after SampleAnimation (springs OFF); [px, py, pz, qx, qy, qz, qw], Unity world, character root at the origin" },
                        { "frames", frames },
                        { "spring", new JObj {
                            { "params", new JObj {
                                { "stiffness", Stiffness }, { "damping", Damping }, { "gravity", Gravity }, { "maxAngleDeg", MaxAngleDeg },
                                { "followAnimation", true }, { "dt", dt }, { "source", "r15 (PlayerRigCheck.cs defaults, user choice 2026-09-27), dt = 1 / clip frameRate" },
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
                    rootMotion = new JObj {
                        { "policy", policy }, { "motionNodeName", ci.motionNodeName }, { "hasMotionCurves", clip.hasMotionCurves },
                        { "motion_bindings", motionBindings.Select(b => (object)$"{b.path}:{b.type.Name}.{b.propertyName}").ToList() },
                        { "evaluation", "the character is evaluated with its root at the origin (bounds, frames); the Root path is reported here per frame" },
                        { "root_path_rule", "root_world / root_local = Root bone after SampleAnimation ([px, py, pz, qx, qy, qz, qw], Unity); motion_curves = the clip's Animator MotionT / MotionQ / RootT / RootQ editor curves at t (empty when no motion node)" },
                        { "root_path", rootPath } };
                    if (wGo != null)
                    {
                        var sockLocal = new JObj();
                        foreach (var kv in wPoints) sockLocal.Add(kv.Key, wGo.transform.InverseTransformPoint(wMeshT.TransformPoint(kv.Value)));
                        weaponOut = new JObj {
                            { "side", wSide }, { "name", wName }, { "asset", wAsset }, { "object", wObject },
                            { "socket", $"WeaponSocket_{wSide}" }, { "import", weaponImport },
                            { "instance", "Instantiate(preview model) -> SetParent(WeaponSocket_" + wSide + ", false), localPosition 0, localRotation identity, localScale 1 (weapon_socket_contract.json unity.attach)" },
                            { "points_local", sockLocal },
                            { "points_local_info", new JObj {
                                { "space", "points_local = the weapon's Unity local space (weapon root = identity child of the socket, compensation baked by p34); mesh = the same points in the MeshFilter's mesh space" },
                                { "rule", "tip = centroid of the mesh vertices with z >= max z - 1e-5 (blade tip vertex); pommel = z <= min z + 1e-5 (pommel pole); guard_a / guard_b = centroids of the vertices with y >= max y - 1e-4 / y <= min y + 1e-4 (guard end faces)" },
                                { "mesh", new JObj { { "tip", wPoints["tip"] }, { "guard_a", wPoints["guard_a"] }, { "guard_b", wPoints["guard_b"] }, { "pommel", wPoints["pommel"] } } } } },
                            { "detail", wInfo },
                            { "frames_rule", "Unity world positions after SampleAnimation and the spring step, character root at the origin" },
                            { "frames", wFrames },
                            { "bounds", "excluded: bounds are the character SkinnedMeshRenderer BakeMesh union only" } };
                    }
                }
                else tOther += swClip.Elapsed.TotalSeconds;
                Object.DestroyImmediate(inst);
                tempObjects.Remove(inst);
            }
            report.Add("clips", clipsOut);
            report.Add("root_motion", rootMotion);
            report.Add("weapon", weaponOut);   // T313 (null when the clip JSON has no weapon)
            timing.Add("sampling_and_springs", tSample);
            timing.Add("penetration_and_bounds", tPen);
            timing.Add("renders", tRender);
            timing.Add("other_clips_bounds", tOther);

            // --- 5. bounds: union of all Player@ clips (springs off and on) + margin -> Assets/Player/Player.prefab ---
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
                PrefabUtility.SaveAsPrefabAsset(tmp, PrefabPath, out applied);
                if (!applied) errors.Add($"SaveAsPrefabAsset failed: {PrefabPath}");
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
                { "union_rule", "every integer frame firstFrame..lastFrame of every Assets/Player/Player@*.fbx clip at its own frameRate, BakeMesh world vertices with springs off and on (r15 + followAnimation), character root at the origin; a clip with importer root motion (motion node) is evaluated at the origin and its Root path is reported in root_motion" },
                { "per_clip", boundsPerClip }, { "applied", applied }, { "prefab", prefabInfo },
                { "weapon", "excluded (T313): the weapon preview is not part of the bounds; character SkinnedMeshRenderer only" } });
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
        Debug.Log($"PLAYER CLIP CHECK written: {outPath} errors={errors.Count}");
        foreach (var e in errors) Debug.Log("PLAYER CLIP CHECK error: " + e);
        EditorApplication.Exit(errors.Count == 0 ? 0 : 1);
    }

    // Moller-Trumbore, two-sided; returns t > 0 or -1 (PlayerRigCheck.cs:610-626).
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
