using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

public static class AvatarCheck
{
    const string FbxAssetPath = "Assets/Import/model.fbx";
    static readonly List<string> ImportErrors = new List<string>();

    [Serializable]
    class PipelineMeta
    {
        public string fbx; public float expectedHipsY; public float hipsTol;
        public bool checkLateralityMarker; public string[] appendageBones;
        public string clipName; public float clipLen;
    }

    // task-10 Fix 2: outputs are namespaced per profile (build|exports|previews/<profile>/) so a
    // dummy run can no longer clobber a character run's exports (or vice versa). run.ps1 sets
    // ANIME_PROFILE before launching Unity via Start-Process, which inherits the parent's
    // environment block by default — so the env var read below matches the Blender-side profile.
    static string ProfileName() => Environment.GetEnvironmentVariable("ANIME_PROFILE") ?? "character";

    static PipelineMeta LoadMeta()
    {
        string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "exports", ProfileName()));
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

            AssetDatabase.ImportAsset(FbxAssetPath, ImportAssetOptions.ForceSynchronousImport);
            var importer = (ModelImporter)AssetImporter.GetAtPath(FbxAssetPath);
            importer.animationType = ModelImporterAnimationType.Human;
            importer.isReadable = true;   // laterality 프로브가 SkinnedMeshRenderer.BakeMesh를 쓴다
            // 검증 대상은 최종 휴머노이드 임포트다. 신규 반입 프로토콜의 제네릭 스테이징
            // 임포트(위 ImportAsset)는 과도기 콘솔 경고를 낼 수 있으나 최종 임포트 상태와
            // 무관 (에디터 확인 결과 최종 상태 경고 0 — 2026-07-19 사용자 확인). 캡처 창을
            // 최종 임포트(아래 SaveAndReimport)로 정렬 — 특정 메시지 필터링이 아니다.
            Application.logMessageReceived += CaptureLog;
            importer.SaveAndReimport();

            // --- Explicit human bone-role override (Phase1 Task7) ------------------------
            // Unity's automatic Humanoid role-mapper gets confused once Hips has many
            // non-standard children: the character's Hips originally carried 2 legs + Spine +
            // 8 skirt chain roots + the tail root (12 children vs. the dummy's 3). Empirically
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
            //
            // RE-TESTED 2026-07-18 (arch fix b): reparented the 6 lateral skirt chains from
            // spine to pelvis.L/pelvis.R at the metarig source (scripts/lib/profiles/
            // character.py), cutting Hips's direct children to ~8 (2 legs + Spine + tail +
            // LeftPelvis + RightPelvis + skirt_f + skirt_b). Temporarily disabled this block
            // and re-ran: auto-map FAILED IDENTICALLY (`required_15_mapped`:
            // "Spine->(Chest) expected Spine"). So Hips's immediate child count alone isn't
            // the (only) trigger — Unity's heuristic is evidently sensitive to something else
            // in the hierarchy (e.g. total descendant count/depth under Hips, or the two new
            // pelvis-with-3-children siblings themselves reading as "spine-like" branches).
            // The override below is therefore retained as a real requirement, not a
            // workaround for something now fixed upstream.
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

            // 사용자 승인(2026-07-19) 조건부 규칙 — 빈 포인터 경고만 비차단, 저장소에 내용
            // 있으면 그 내용으로 실패. 침묵 화이트리스트 아님: "has animation import
            // warnings" 배너는 콘솔 로그 포인터일 뿐 실제 내용이 아니므로, 임포터 내부
            // 메시지 저장소(m_AnimationImportWarnings)를 진짜 근거로 대조한다. 저장소가
            // 비어 있으면(에디터의 Import Messages 섹션과 동일 상태 — 2026-07-19 사용자
            // 확인) 그 배너 항목만 비차단 처리하고 정보성 결과로 남긴다. 저장소에 실제
            // 내용이 있으면 import_clean은 그 내용을 그대로 실패 사유에 담아 실패한다.
            const string PointerMarker = "has animation import warnings. See Import Messages";
            var pointerEntries = ImportErrors.Where(e => e.Contains(PointerMarker)).ToList();
            var otherErrors = ImportErrors.Where(e => !e.Contains(PointerMarker)).ToList();
            if (pointerEntries.Count > 0)
            {
                var so = new SerializedObject(importer);
                var warnProp = so.FindProperty("m_AnimationImportWarnings");
                string store = warnProp != null ? warnProp.stringValue : null;
                if (string.IsNullOrEmpty(store))
                {
                    results.Add(("anim_warning_pointer", true,
                        "empty pointer warning — importer message store empty; editor shows no Import Messages (user-verified 2026-07-19, approved conditional rule)"));
                }
                else
                {
                    otherErrors.AddRange(pointerEntries.Select(_ => $"[Warning] animation import warnings store content: {store}"));
                }
            }
            ImportErrors.Clear();
            ImportErrors.AddRange(otherErrors);

            results.Add(("import_clean", ImportErrors.Count == 0,
                ImportErrors.Count == 0 ? "no errors/warnings" : string.Join(" | ", ImportErrors.Take(50))));

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
                bool clipOk = clip != null && clip.length > meta.clipLen - 0.25f && clip.length < meta.clipLen + 0.25f;
                results.Add(("clip_present", clipOk, clip == null ? "no clip" : $"{clip.name} len={clip.length:F3}s"));

                if (clipOk)
                {
                    // t=0 vs t=len/2만 비교하면 walk에서 실패한다: 수직 bob 주기가 사이클의
                    // 절반이라 t=0과 t=len/2가 같은 위상을 샘플링해 delta가 임계값 밑으로
                    // 떨어진다 (check_02.py의 torso 이동 단언이 이미 겪은 동일한 이중-bob
                    // 위상 충돌). t=0, 0.25, 0.5, 0.75 4개 지점을 샘플링해 t=0 대비 최대
                    // 변위로 판정 — 위상에 무관하며, 정지된 클립에 대해서는 기존과 최소
                    // 동일하거나 더 엄격하다 (임계값 0.004f 불변).
                    var chest = anim.GetBoneTransform(HumanBodyBones.Spine);
                    clip.SampleAnimation(instance, 0f);
                    Vector3 p0 = chest.position;
                    float maxDelta = 0f; float maxT = 0f;
                    foreach (float frac in new[] { 0.25f, 0.5f, 0.75f })
                    {
                        float t = clip.length * frac;
                        clip.SampleAnimation(instance, t);
                        float d = (p0 - chest.position).magnitude;
                        if (d > maxDelta) { maxDelta = d; maxT = t; }
                    }
                    results.Add(("clip_moves", maxDelta > 0.004f, $"spine max delta={maxDelta:F4}m at t={maxT:F3}s (vs t=0)"));
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

        // 임베드 텍스처를 추출해 뷰포트에서 재질이 보이게 한다 (사용자 눈검증 UX).
        // 반환값은 추출된 텍스처 수 관련 정보가 아니므로 무시; 실패해도 단언에는 영향 없음.
        // 모든 검증 후에 수행하므로 추출 오류가 검증 결과에 영향을 주지 않음.
        // CopyFbxIntoProject에서 매 실행마다 이전 추출을 삭제하므로 매번 신선한 상태로 추출.
        // Tracks whether extraction actually produced texture files before any risky
        // rename/remap/reimport step. If an exception fires after this point, the failure
        // can affect texture wiring, so the materials_textured assertion must record a
        // failure rather than be silently omitted (which would leave allPass true despite
        // a broken texture pipeline). When no files were extracted (dummy-style profiles
        // with no embedded textures), extraction hiccups stay non-critical (log only).
        bool extractedFilesDetected = false;
        try
        {
            string texDir = "Assets/Import/Textures";
            if (!Directory.Exists(texDir)) Directory.CreateDirectory(texDir);
            var importerForExtraction = (ModelImporter)AssetImporter.GetAtPath(FbxAssetPath);
            importerForExtraction.ExtractTextures(texDir);
            AssetDatabase.Refresh();

            string absTexDir = Path.GetFullPath(Path.Combine(Application.dataPath, "Import", "Textures"));
            extractedFilesDetected = Directory.Exists(absTexDir)
                && Directory.GetFiles(absTexDir).Any(f => !f.EndsWith(".meta"));

            // ExtractTextures writes each FBX-embedded texture under its Blender-embedded
            // name, which carries NO file extension (e.g. "Image_0"). Left alone this breaks
            // in THREE compounding ways (all confirmed empirically against this asset):
            //   1. An extensionless file imports as a DefaultAsset — not a Texture2D — so it
            //      can never satisfy a material's texture slot.
            //   2. After renaming to a real extension, Unity's TextureImporter AUTO-DETECTS a
            //      Cubemap for these lat-long-aspect images (textureShape=TextureCube), which
            //      still is not a Texture2D — so we force textureShape=Texture2D.
            //   3. ExtractTextures creates NO external-object remap here (GetExternalObjectMap
            //      stays empty) and re-running it does not help — so the material description,
            //      which references the embedded textures by their base name ("Image_0"),
            //      resolves every texture slot to null and the character renders WHITE.
            // Fix: sniff magic bytes and rename (GUID-preserving MoveAsset), force each to a
            // 2D texture, then explicitly AddRemap each SourceAssetIdentifier(Texture/Texture2D,
            // "<base name>") to the renamed Texture2D and reimport the model with the standard
            // material pipeline. The material description then binds its slots by name (verified:
            // Material_0._MainTex -> Image_0). Idempotent: files that already have an extension
            // (a prior run, or a re-run) are left untouched; shape/remap steps are safe to repeat.
            bool renamedAny = false;
            if (Directory.Exists(absTexDir))
            {
                foreach (var absFile in Directory.GetFiles(absTexDir))
                {
                    if (absFile.EndsWith(".meta")) continue;
                    if (Path.HasExtension(absFile)) continue;      // already has extension (idempotent)
                    string ext = SniffImageExtension(absFile);
                    if (ext == null) continue;                     // unknown magic — leave as-is
                    string fileName = Path.GetFileName(absFile);
                    string assetPath = $"{texDir}/{fileName}";
                    string newAssetPath = $"{texDir}/{fileName}{ext}";
                    string moveErr = AssetDatabase.MoveAsset(assetPath, newAssetPath);
                    if (string.IsNullOrEmpty(moveErr)) renamedAny = true;
                    else Debug.Log($"Texture rename failed for {assetPath}: {moveErr}");
                }
            }
            if (renamedAny)
            {
                AssetDatabase.Refresh();
                // (2) Force every extracted texture to import as a plain 2D texture — Unity's
                // Cubemap auto-detection on lat-long-aspect images otherwise leaves a Cubemap
                // that no material slot can bind.
                foreach (var absFile in Directory.GetFiles(absTexDir))
                {
                    if (absFile.EndsWith(".meta")) continue;
                    string p = $"{texDir}/{Path.GetFileName(absFile)}";
                    if (AssetImporter.GetAtPath(p) is TextureImporter ti && ti.textureShape != TextureImporterShape.Texture2D)
                    {
                        ti.textureShape = TextureImporterShape.Texture2D;
                        ti.SaveAndReimport();
                    }
                }
                AssetDatabase.Refresh();

                // (3) Explicitly remap each embedded texture identifier (keyed by the base name
                // the material description references) to the renamed Texture2D, then reimport
                // the model so its materials resolve. Register both Texture and Texture2D
                // identifier types so the binding survives regardless of the slot's declared type.
                var modelImporter = (ModelImporter)AssetImporter.GetAtPath(FbxAssetPath);
                modelImporter.materialImportMode = ModelImporterMaterialImportMode.ImportStandard;
                foreach (var absFile in Directory.GetFiles(absTexDir))
                {
                    if (absFile.EndsWith(".meta")) continue;
                    string p = $"{texDir}/{Path.GetFileName(absFile)}";
                    var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(p);
                    if (tex == null) continue;
                    string baseName = Path.GetFileNameWithoutExtension(absFile);
                    modelImporter.AddRemap(new AssetImporter.SourceAssetIdentifier(typeof(Texture), baseName), tex);
                    modelImporter.AddRemap(new AssetImporter.SourceAssetIdentifier(typeof(Texture2D), baseName), tex);
                }
                modelImporter.SaveAndReimport();
                AssetDatabase.ImportAsset(FbxAssetPath, ImportAssetOptions.ForceSynchronousImport);
            }

            // Post-fix machine check (assertion ADDITION, strengthening — never touches an
            // existing assertion). If the FBX embedded textures (character profile), at least
            // one material on the model must now resolve a non-null mainTexture; a null here
            // means the white-character bug is still present. If nothing was extracted (the
            // dummy profile embeds no textures), record a conditional skip-pass — mirrors the
            // laterality assertion's "skipped: profile has no marker" pattern.
            var extractedFiles = Directory.Exists(absTexDir)
                ? Directory.GetFiles(absTexDir).Where(f => !f.EndsWith(".meta")).ToArray()
                : Array.Empty<string>();
            if (extractedFiles.Length == 0)
            {
                results.Add(("materials_textured", true, "skipped: no extracted textures"));
            }
            else
            {
                var mats = AssetDatabase.LoadAllAssetsAtPath(FbxAssetPath).OfType<Material>().ToList();
                Texture resolved = null;
                foreach (var m in mats)
                {
                    if (m != null && m.mainTexture != null) { resolved = m.mainTexture; break; }
                }
                results.Add(("materials_textured", resolved != null,
                    resolved != null
                        ? $"mainTexture={resolved.name} across {mats.Count} material(s), {extractedFiles.Length} texture(s) extracted"
                        : $"no material resolved mainTexture across {mats.Count} material(s) despite {extractedFiles.Length} extracted texture(s)"));
            }
        }
        catch (System.Exception ex)
        {
            if (extractedFilesDetected)
            {
                // Extraction already produced texture files, so a failure here can leave the
                // texture pipeline broken. Record it as a real assertion failure instead of
                // silently omitting materials_textured (which would keep allPass true).
                results.Add(("materials_textured", false, "extraction/remap exception: " + ex.Message));
            }
            else
            {
                Debug.Log($"Texture extraction failed (non-critical): {ex.Message}");
            }
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

    // Magic-byte sniff for the two texture formats Blender embeds. Returns the file
    // extension (with leading dot) or null when the header is unrecognized.
    static string SniffImageExtension(string absPath)
    {
        try
        {
            byte[] head = new byte[8];
            int n;
            using (var fs = File.OpenRead(absPath)) n = fs.Read(head, 0, head.Length);
            if (n >= 2 && head[0] == 0xFF && head[1] == 0xD8) return ".jpg";                       // JPEG (FF D8)
            if (n >= 4 && head[0] == 0x89 && head[1] == 0x50 && head[2] == 0x4E && head[3] == 0x47) // PNG (89 50 4E 47)
                return ".png";
            return null;
        }
        catch { return null; }
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
        string src = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "exports", ProfileName(), fbxName));
        string dst = Path.GetFullPath(Path.Combine(Application.dataPath, "Import", "model.fbx"));
        if (!File.Exists(src)) throw new FileNotFoundException($"export first: {src}");
        Directory.CreateDirectory(Path.GetDirectoryName(dst));
        // Delete any previously-imported asset (+ its .meta) so the humanoid avatar is
        // AUTO-mapped fresh from the current FBX. Unity bakes the auto-map result into the
        // .meta on first import and reuses it; without this, a re-run after a rig change
        // (exactly what the Task 9 correction loop does) applies a STALE bone mapping and
        // fails avatar creation. Fresh import = honest re-validation, not weakened checks.
        if (File.Exists(dst)) AssetDatabase.DeleteAsset(FbxAssetPath);
        // Also delete any previously-extracted textures to guarantee fresh import state
        // (prevents post-import-callback recursion when texture assets already exist).
        string texturesPath = "Assets/Import/Textures";
        if (Directory.Exists(Path.GetFullPath(Path.Combine(Application.dataPath, "Import/Textures"))))
            AssetDatabase.DeleteAsset(texturesPath);
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
