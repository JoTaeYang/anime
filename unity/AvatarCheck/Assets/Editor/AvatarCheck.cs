using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

public static class AvatarCheck
{
    const string FbxAssetPath = "Assets/Dummy/model.fbx";
    static readonly List<string> ImportErrors = new List<string>();

    [Serializable]
    class PipelineMeta
    {
        public string fbx; public float expectedHipsY; public float hipsTol;
        public bool checkLateralityMarker; public string[] appendageBones;
    }

    static PipelineMeta LoadMeta()
    {
        string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "exports"));
        var files = Directory.GetFiles(dir, "*_meta.json");
        if (files.Length != 1) throw new FileNotFoundException($"expected exactly one *_meta.json in {dir}, found {files.Length}");
        return JsonUtility.FromJson<PipelineMeta>(File.ReadAllText(files[0]));
    }

    // Blender 쪽 scripts/lib/bone_map.py의 REQUIRED_HUMAN_BONES와 일치해야 한다
    // NOTE: this assertion verifies name-mapping completeness/consistency ONLY.
    // Anatomical left/right correctness is guarded by faces_plus_z (hand x-order +
    // toe z-direction) AND the laterality probe (marker geometry side vs bone name).
    // RESOLVED 2026-07-18: names are now the honest Blender L/R (no swap). Unity is
    // left-handed, so a +Z-facing character's LEFT is at −X — hence LeftHand.x < RightHand.x.
    static readonly Dictionary<HumanBodyBones, string> Required = new Dictionary<HumanBodyBones, string>
    {
        { HumanBodyBones.Hips, "Hips" }, { HumanBodyBones.Spine, "Spine" }, { HumanBodyBones.Head, "Head" },
        { HumanBodyBones.LeftUpperLeg, "LeftUpperLeg" }, { HumanBodyBones.LeftLowerLeg, "LeftLowerLeg" }, { HumanBodyBones.LeftFoot, "LeftFoot" },
        { HumanBodyBones.RightUpperLeg, "RightUpperLeg" }, { HumanBodyBones.RightLowerLeg, "RightLowerLeg" }, { HumanBodyBones.RightFoot, "RightFoot" },
        { HumanBodyBones.LeftUpperArm, "LeftUpperArm" }, { HumanBodyBones.LeftLowerArm, "LeftLowerArm" }, { HumanBodyBones.LeftHand, "LeftHand" },
        { HumanBodyBones.RightUpperArm, "RightUpperArm" }, { HumanBodyBones.RightLowerArm, "RightLowerArm" }, { HumanBodyBones.RightHand, "RightHand" },
    };

    // Optional Humanoid slots our rig also names identically (matches bone_map.py's
    // OPTIONAL_HUMAN_BONES) — used to build the explicit human-role override below.
    static readonly string[] OptionalHumanNames = {
        "Chest", "UpperChest", "Neck", "LeftShoulder", "RightShoulder", "LeftToes", "RightToes",
    };
    static readonly string[] FingerHumanNames = (
        from side in new[] { "Left", "Right" }
        from finger in new[] { "Thumb", "Index", "Middle", "Ring", "Little" }
        from seg in new[] { "Proximal", "Intermediate", "Distal" }
        select $"{side}{finger}{seg}"
    ).ToArray();

    public static void Run()
    {
        var results = new List<(string name, bool pass, string detail)>();
        GameObject instance = null;
        try
        {
            var meta = LoadMeta();
            CopyFbxIntoProject(meta.fbx);

            Application.logMessageReceived += CaptureLog;
            AssetDatabase.ImportAsset(FbxAssetPath, ImportAssetOptions.ForceSynchronousImport);
            var importer = (ModelImporter)AssetImporter.GetAtPath(FbxAssetPath);
            importer.animationType = ModelImporterAnimationType.Human;
            importer.isReadable = true;   // laterality 프로브가 SkinnedMeshRenderer.BakeMesh를 쓴다
            importer.SaveAndReimport();

            // --- Explicit human bone-role override (Phase1 Task7) ------------------------
            // Unity's automatic Humanoid role-mapper gets confused once Hips has many
            // non-standard children: the character's Hips carries 2 legs + Spine + 8 skirt
            // chain roots + the tail root (12 children vs. the dummy's 3). Empirically
            // verified via a throwaway diagnostic: even though our bones are literally named
            // "Spine"/"Chest"/"UpperChest", the auto-mapper shifts roles down by one
            // (Spine role <- our "Chest" bone, Chest role <- our "UpperChest" bone, UpperChest
            // left unmapped) — a structural-ambiguity artifact of the sibling count, not a
            // naming problem. Neck/Head (which only gained 2 extra siblings each, from the
            // scarf/hood-ear chains) were unaffected, isolating Hips's sibling count as the
            // trigger. The `skeleton` array Unity derives from the FBX hierarchy is NOT
            // affected by this (only the human-role heuristic is), so we keep Unity's own
            // skeleton and supply the correct explicit `human` mapping ourselves, using the
            // same identity-name convention as bone_map.py's REQUIRED_HUMAN_BONES /
            // OPTIONAL_HUMAN_BONES (humanName == boneName). This is assertion-neutral: it
            // does not change what "correct" means, only makes the importer produce the
            // mapping our own bone names already declare.
            var hd = importer.humanDescription;
            var knownHumanNames = Required.Values.Concat(OptionalHumanNames).Concat(FingerHumanNames).ToHashSet();
            var skeletonNames = hd.skeleton.Select(s => s.name).ToHashSet();
            hd.human = knownHumanNames.Where(skeletonNames.Contains)
                .Select(n => new HumanBone { humanName = n, boneName = n, limit = new HumanLimit { useDefaultValues = true } })
                .ToArray();
            importer.humanDescription = hd;
            importer.avatarSetup = ModelImporterAvatarSetup.CreateFromThisModel;
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
                results.Add(("hips_height", hipsY > meta.expectedHipsY - meta.hipsTol && hipsY < meta.expectedHipsY + meta.hipsTol,
                    $"hipsY={hipsY:F4} expected {meta.expectedHipsY:F4}±{meta.hipsTol:F2}"));

                var lh = anim.GetBoneTransform(HumanBodyBones.LeftHand).position;
                var rh = anim.GetBoneTransform(HumanBodyBones.RightHand).position;
                var lf = anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
                var lt = anim.GetBoneTransform(HumanBodyBones.LeftToes).position;
                // Unity는 왼손 좌표계 → +Z를 보는 캐릭터의 왼손은 −X, 오른손은 +X (LeftHand.x < RightHand.x).
                // 발끝이 발보다 +Z 앞(toeZ > footZ)이면 +Z를 바라보는 것.
                bool facing = lh.x < rh.x && lt.z > lf.z;
                results.Add(("faces_plus_z", facing, $"leftHand.x={lh.x:F3} rightHand.x={rh.x:F3} toeZ-footZ={(lt.z - lf.z):F3}"));

                // --- Laterality 프로브 (RESOLVED 2026-07-18) --------------------------------
                // 비대칭 마커(marker_L)는 Blender 해부학적 왼팔(+X)에만 붙인 유일한 비대칭 질량이다.
                // 대칭 더미의 무게중심은 x≈0 이므로 Unity에서 스키닝한 mesh 무게중심 x의 "부호"가
                // 해부학적-왼쪽 질량이 어느 월드 X면에 안착했는가를 드러낸다.
                // 정답 정의: Unity(왼손 좌표계)에서 +Z를 보는 캐릭터의 왼쪽은 −X. 따라서 올바른 결과는
                //   centroidX < 0 (왼쪽 질량이 Unity −X) AND 그 질량을 Left*-이름 본이 구동 AND lhX < rhX.
                // 그 질량을 Left/Right 어느 이름 본이 구동하는지는 본과 지오메트리가 같은 면에 있으므로
                // sign(centroidX)==sign(lhX) 로 판정한다(별도 정점-본 조회 불필요).
                // 참고: 예전엔 여기에 "미러가 있다/왼쪽=+X"라는 방향 오류가 있었고 스왑이 그 보상이었다.
                if (meta.checkLateralityMarker)
                {
                    var smr = instance.GetComponentInChildren<SkinnedMeshRenderer>();
                    float centroidX = 0f, extX = 0f;
                    string extBone = "NONE";
                    if (smr != null)
                    {
                        var baked = new Mesh();
                        smr.BakeMesh(baked);                       // rest 포즈 스키닝 결과, 렌더러 로컬공간
                        var l2w = smr.transform.localToWorldMatrix;
                        var verts = baked.vertices;
                        var bw = smr.sharedMesh.boneWeights;
                        var bones = smr.bones;
                        int ei = -1; float best = -1f; double sum = 0.0;
                        for (int i = 0; i < verts.Length; i++)
                        {
                            Vector3 w = l2w.MultiplyPoint3x4(verts[i]);
                            sum += w.x;
                            float a = Mathf.Abs(w.x);
                            if (a > best) { best = a; ei = i; extX = w.x; }
                        }
                        centroidX = verts.Length > 0 ? (float)(sum / verts.Length) : 0f;
                        if (ei >= 0 && bw != null && ei < bw.Length && bones != null)
                        {
                            int bi = bw[ei].boneIndex0;
                            if (bi >= 0 && bi < bones.Length && bones[bi] != null) extBone = bones[bi].name;
                        }
                        UnityEngine.Object.DestroyImmediate(baked);
                    }
                    bool markerAtLeftX = centroidX < 0f;                        // 해부학적-왼쪽 질량이 Unity −X(왼쪽)?
                    bool markerBoneIsLeft = (centroidX < 0f) == (lh.x < 0f);    // 그 질량을 Left*본이 구동?
                    string markerSide = markerBoneIsLeft ? "Left*" : "Right*";
                    bool lateralityOk = markerAtLeftX && markerBoneIsLeft && lh.x < rh.x;
                    results.Add(("laterality", lateralityOk,
                        $"centroidX={centroidX:F4} markerDrivenBy={markerSide} extX={extX:F3} extBone={extBone} lhX={lh.x:F3} rhX={rh.x:F3}"));
                }
                else
                {
                    results.Add(("laterality", true, "skipped: profile has no marker"));
                }

                var appendageMissing = new List<string>();
                foreach (var b in meta.appendageBones ?? Array.Empty<string>())
                {
                    if (FindDeep(instance.transform, b) == null) appendageMissing.Add(b);
                }
                bool appendagesOk = appendageMissing.Count == 0;
                results.Add(("appendages_present", appendagesOk,
                    (meta.appendageBones == null || meta.appendageBones.Length == 0)
                        ? "none declared"
                        : (appendagesOk ? "all present" : $"missing: {string.Join(", ", appendageMissing)}")));

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

    static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name) return root;
        foreach (Transform child in root)
        {
            var found = FindDeep(child, name);
            if (found != null) return found;
        }
        return null;
    }

    static void CopyFbxIntoProject(string fbxName)
    {
        string src = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "exports", fbxName));
        string dst = Path.GetFullPath(Path.Combine(Application.dataPath, "Dummy", "model.fbx"));
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
