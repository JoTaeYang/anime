using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Animations;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class PlaySceneBuild
{
    static string RepoRoot() =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));

    static AnimationClip Clip(string name)
    {
        foreach (var fbx in Directory.GetFiles(Path.GetFullPath("Assets/Clips"), "*.fbx"))
        {
            string ap = "Assets/Clips/" + Path.GetFileName(fbx);
            var c = AssetDatabase.LoadAllAssetsAtPath(ap).OfType<AnimationClip>().FirstOrDefault(x => x.name == name);
            if (c != null) return c;
        }
        return null;
    }

    public static void Run()
    {
        var results = new List<(string name, bool pass, string detail)>();
        try
        {
            if (!AssetDatabase.IsValidFolder("Assets/Play"))
                AssetDatabase.CreateFolder("Assets", "Play");

            string ctrlPath = "Assets/Play/Player.controller";
            AssetDatabase.DeleteAsset(ctrlPath);
            var ctrl = AnimatorController.CreateAnimatorControllerAtPath(ctrlPath);
            ctrl.AddParameter("Speed", AnimatorControllerParameterType.Float);
            foreach (var t in new[] { "Attack", "Roll", "Hit", "Die" })
                ctrl.AddParameter(t, AnimatorControllerParameterType.Trigger);
            var sm = ctrl.layers[0].stateMachine;

            BlendTree bt;
            var loco = ctrl.CreateBlendTreeInController("Locomotion", out bt);
            bt.blendParameter = "Speed";
            bt.useAutomaticThresholds = false;
            bt.AddChild(Clip("Idle"), 0f); bt.AddChild(Clip("Walk"), 0.5f); bt.AddChild(Clip("Run"), 1f);
            sm.defaultState = loco;

            // Foot IK 시도 (스펙 §2): 상태 iKOnFeet + 레이어 IK pass. 캡처에서 발 관통
            // 개선이 확인되지 않으면 이 블록을 제거하고 보고서에 기록한다 (침묵 금지).
            var layers = ctrl.layers;
            layers[0].iKPass = true;
            ctrl.layers = layers;

            AnimatorState MakeState(string clipName)
            {
                var s = sm.AddState(clipName);
                s.motion = Clip(clipName);
                return s;
            }
            AnimatorStateTransition Back(AnimatorState from)
            {
                var t = from.AddTransition(loco);
                t.hasExitTime = true; t.exitTime = 0.9f; t.duration = 0.1f;
                return t;
            }
            var a1 = MakeState("Attack1"); var a2 = MakeState("Attack2"); var a3 = MakeState("Attack3");
            var roll = MakeState("Roll"); var hit = MakeState("Hit"); var die = MakeState("Death");

            foreach (var st in new[] { loco, a1, a2, a3, roll, hit })
                st.iKOnFeet = true;

            var tA1 = loco.AddTransition(a1); tA1.AddCondition(AnimatorConditionMode.If, 0, "Attack"); tA1.duration = 0.05f;
            var tA2 = a1.AddTransition(a2); tA2.AddCondition(AnimatorConditionMode.If, 0, "Attack"); tA2.duration = 0.05f;
            var tA3 = a2.AddTransition(a3); tA3.AddCondition(AnimatorConditionMode.If, 0, "Attack"); tA3.duration = 0.05f;
            Back(a1); Back(a2); Back(a3); Back(roll); Back(hit);

            foreach (var (trig, st) in new[] { ("Roll", roll), ("Hit", hit), ("Die", die) })
            {
                var t = sm.AddAnyStateTransition(st);
                t.AddCondition(AnimatorConditionMode.If, 0, trig);
                t.duration = 0.05f; t.canTransitionToSelf = false;
            }

            foreach (var s in new[] { a1, a2, a3, roll, hit, die })
                results.Add(($"state_motion:{s.name}", s.motion != null, s.motion == null ? "missing clip" : s.motion.name));
            results.Add(("blendtree_children", bt.children.Length == 3 && bt.children.All(c => c.motion != null),
                $"children={bt.children.Length}"));

            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            plane.transform.localScale = new Vector3(3, 1, 3);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Import/model.fbx");
            results.Add(("model_prefab", prefab != null, prefab == null ? "Assets/Import/model.fbx missing — run.ps1 unity -Profile character 선행" : "ok"));
            var player = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            var anim = player.GetComponent<Animator>() ?? player.AddComponent<Animator>();
            anim.runtimeAnimatorController = ctrl;
            anim.applyRootMotion = false;   // 이동은 PlayerDrive가 담당 (In Place 클립 전제)
            player.AddComponent<PlayerDrive>();
            int springs = SpringBoneSetup.Attach(player);
            results.Add(("springs_attached", springs == 13, $"chains={springs} expected 13"));
            var cam = Camera.main;
            cam.transform.position = new Vector3(0, 2.2f, -4.5f);
            var fc = cam.gameObject.AddComponent<FollowCam>();
            fc.target = player.transform;
            EditorSceneManager.SaveScene(scene, "Assets/Play/PlayScene.unity");
            results.Add(("scene_saved", File.Exists(Path.GetFullPath("Assets/Play/PlayScene.unity")), "Assets/Play/PlayScene.unity"));
        }
        catch (Exception ex) { results.Add(("run_exception", false, ex.ToString())); }

        bool allPass = results.Count > 0 && results.All(r => r.pass);
        string json = "{\"allPass\":" + (allPass ? "true" : "false") + ",\"assertions\":[" +
            string.Join(",", results.Select(r =>
                "{\"name\":\"" + r.name + "\",\"pass\":" + (r.pass ? "true" : "false") +
                ",\"detail\":\"" + r.detail.Replace("\\", "/").Replace("\"", "'") + "\"}")) + "]}";
        File.WriteAllText(Path.Combine(RepoRoot(), "unity", "AvatarCheck", "playscene_report.json"), json);
        Debug.Log("PLAYSCENE REPORT: " + json);
        EditorApplication.Exit(allPass ? 0 : 1);
    }
}
