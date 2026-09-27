using UnityEngine;

// 부속물 체인 1개의 스프링 물리 (스펙 §2). Humanoid가 구동하지 않는 비인간 본을
// 감쇠 진자로 흔든다: 각 본의 "끝점"을 관성 추적하고, 레스트 방향과의 차이를
// 본 회전으로 환산한다. 시뮬 틱은 Step(dt)로 분리 — 런타임 LateUpdate와
// 에디트 모드 캡처(ClipCapture)가 같은 코드를 호출한다 (검증 재현성의 핵심 심).
public class SpringBoneChain : MonoBehaviour
{
    public Transform[] bones;          // 체인 순서 (루트→끝)
    public float stiffness = 0.1f;     // 레스트 방향 복원 계수 (틱당 lerp)
    public float damping = 0.2f;       // 관성 감쇠 (0=관성 유지, 1=즉시 정지)
    public float gravity = 1.0f;       // 하방 가속 (m/s^2)
    public float maxAngleDeg = 40f;    // 레스트 방향 대비 최대 흔들림 각
    public SpringCollider[] colliders; // 밀어낼 구체들 (없으면 빈 배열)
    // true: 매 틱 애니메이션이 쓴 로컬 회전을 기준 자세로 쓴다 (클립이 체인 본을 매 프레임 키로 가질 때만;
    // 플레이어 치마의 허벅지 따라가기가 여기에 해당). false(기본): Init 시점 레스트로 복귀 — Phase 2a 동작.
    public bool followAnimation = false;

    Quaternion[] restLocal;   // 레스트 로컬 회전 (Init 시점)
    Vector3[] restDirLocal;   // 본 로컬 공간에서 끝점 방향
    float[] len;              // 본 길이
    Vector3[] tip, prevTip;   // 끝점 verlet 상태 (world)
    bool ready;

    public void Init()
    {
        int n = bones.Length;
        restLocal = new Quaternion[n];
        restDirLocal = new Vector3[n];
        len = new float[n];
        tip = new Vector3[n];
        prevTip = new Vector3[n];
        for (int i = 0; i < n; i++)
        {
            restLocal[i] = bones[i].localRotation;
            // 끝점: 다음 본이 있으면 그 위치, 마지막 본은 자기 길이만큼 연장
            Vector3 childPos = i + 1 < n
                ? bones[i + 1].position
                : bones[i].position + (bones[i].position - (i > 0 ? bones[i - 1].position : bones[i].parent.position));
            len[i] = Mathf.Max(0.01f, Vector3.Distance(bones[i].position, childPos));
            restDirLocal[i] = bones[i].InverseTransformDirection((childPos - bones[i].position).normalized);
            tip[i] = childPos;
            prevTip[i] = childPos;
        }
        ready = true;
    }

    void LateUpdate()
    {
        if (Application.isPlaying && ready) Step(Time.deltaTime);
    }

    public void Step(float dt)
    {
        if (!ready || dt <= 0f) return;
        for (int i = 0; i < bones.Length; i++)
        {
            var b = bones[i];
            if (!followAnimation) b.localRotation = restLocal[i];   // 부모 애니메이션 위에 레스트로 복귀
            Vector3 restDirW = b.TransformDirection(restDirLocal[i]);
            Vector3 target = b.position + restDirW * len[i];

            Vector3 vel = (tip[i] - prevTip[i]) * (1f - damping);
            Vector3 next = tip[i] + vel
                         + Vector3.down * (gravity * dt * dt)
                         + (target - tip[i]) * Mathf.Clamp01(stiffness);

            next = b.position + (next - b.position).normalized * len[i];   // 길이 유지

            if (colliders != null)
                foreach (var c in colliders)
                {
                    if (c == null) continue;
                    Vector3 d = next - c.transform.position;
                    if (d.magnitude < c.radius)
                        next = c.transform.position + d.normalized * c.radius;
                }

            Vector3 dir = (next - b.position).normalized;
            float ang = Vector3.Angle(restDirW, dir);
            if (ang > maxAngleDeg)
            {
                dir = Vector3.Slerp(restDirW, dir, maxAngleDeg / ang).normalized;
                next = b.position + dir * len[i];
            }

            b.rotation = Quaternion.FromToRotation(restDirW, dir) * b.rotation;
            prevTip[i] = tip[i];
            tip[i] = next;
        }
    }
}
