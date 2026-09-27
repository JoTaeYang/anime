using UnityEngine;

// 구체 충돌 마커: 반경만 갖는다. SpringBoneChain.Step이 추적점을 구 밖으로 밀어낸다.
// 허벅지·종아리 본에 부착해 치마 관통을 "완화"한다 (완전 방지는 스펙상 비목표).
public class SpringCollider : MonoBehaviour
{
    public float radius = 0.08f;
}
