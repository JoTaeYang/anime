using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// meta appendageBones(?⑥씪 ?먯쿇)???대쫫 洹쒖튃?쇰줈 泥댁씤??臾띠뼱 SpringBoneChain??// 遺李⑺븯???뺤쟻 ?좏떥. PlaySceneBuild(???앹꽦)? ClipCapture(罹≪쿂) ?묒そ?먯꽌 ?몄텧.
// 洹몃９ ?쒕떇媛믪? ?꾨옒 ?뚯씠釉붿씠 ?⑥씪 ?먯쿇 ???섎졃 猷⑦봽(?ㅽ럺 짠3)???섏젙 ???
public static class SpringBoneSetup
{
    // (泥댁씤 蹂??대쫫 ?섏뿴, stiffness, damping, gravity, maxAngleDeg, useLegColliders)
    static readonly (string[] chain, float st, float dp, float gr, float ang, bool col)[] Chains =
    {
        (new[]{"Tail1","Tail2","Tail3","Tail4","Tail5"}, 0.06f, 0.15f, 0.5f, 45f, false),
        (new[]{"SkirtF1a","SkirtF1b"}, 0.15f, 0.25f, 1.5f, 40f, true),
        (new[]{"SkirtFR1a","SkirtFR1b"}, 0.15f, 0.25f, 1.5f, 40f, true),
        (new[]{"SkirtR1a","SkirtR1b"}, 0.15f, 0.25f, 1.5f, 40f, true),
        (new[]{"SkirtBR1a","SkirtBR1b"}, 0.15f, 0.25f, 1.5f, 40f, true),
        (new[]{"SkirtB1a","SkirtB1b"}, 0.15f, 0.25f, 1.5f, 40f, true),
        (new[]{"SkirtBL1a","SkirtBL1b"}, 0.15f, 0.25f, 1.5f, 40f, true),
        (new[]{"SkirtL1a","SkirtL1b"}, 0.15f, 0.25f, 1.5f, 40f, true),
        (new[]{"SkirtFL1a","SkirtFL1b"}, 0.15f, 0.25f, 1.5f, 40f, true),
        (new[]{"ScarfL1a","ScarfL1b"},   0.08f, 0.20f, 2.0f, 50f, false),
        (new[]{"ScarfR1a","ScarfR1b"},   0.08f, 0.20f, 2.0f, 50f, false),
        (new[]{"HoodEarL"},              0.12f, 0.20f, 0.3f, 25f, false),
        (new[]{"HoodEarR"},              0.12f, 0.20f, 0.3f, 25f, false),
    };
    static readonly (string bone, float r)[] LegColliders =
    {
        ("LeftUpperLeg", 0.11f), ("RightUpperLeg", 0.11f),
        ("LeftLowerLeg", 0.07f), ("RightLowerLeg", 0.07f),
    };

    public static int Attach(GameObject player)
    {
        var byName = player.GetComponentsInChildren<Transform>(true)
            .GroupBy(t => t.name).ToDictionary(g => g.Key, g => g.First());

        var legCols = new List<SpringCollider>();
        foreach (var (bone, r) in LegColliders)
            if (byName.TryGetValue(bone, out var t))
            {
                var c = t.GetComponent<SpringCollider>() ?? t.gameObject.AddComponent<SpringCollider>();
                c.radius = r;
                legCols.Add(c);
            }

        int attached = 0;
        foreach (var (chain, st, dp, gr, ang, col) in Chains)
        {
            var bones = chain.Select(n => byName.TryGetValue(n, out var t) ? t : null).ToArray();
            if (bones.Any(b => b == null)) continue;   // 遺?띾Ъ ?녿뒗 紐⑤뜽(Y Bot ??? 議곗슜???ㅽ궢
            var sb = bones[0].gameObject.GetComponent<SpringBoneChain>()
                     ?? bones[0].gameObject.AddComponent<SpringBoneChain>();
            sb.bones = bones;
            sb.stiffness = st; sb.damping = dp; sb.gravity = gr; sb.maxAngleDeg = ang;
            sb.colliders = col ? legCols.ToArray() : new SpringCollider[0];
            sb.Init();
            attached++;
        }
        return attached;
    }
}
