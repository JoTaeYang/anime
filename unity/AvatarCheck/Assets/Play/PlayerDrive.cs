using UnityEngine;

public class PlayerDrive : MonoBehaviour
{
    Animator anim;
    float speed;
    void Start() { anim = GetComponent<Animator>(); }
    void Update()
    {
        float h = Input.GetAxis("Horizontal"), v = Input.GetAxis("Vertical");
        var dir = new Vector3(h, 0, v);
        float target = dir.magnitude < 0.1f ? 0f : (Input.GetKey(KeyCode.LeftShift) ? 1f : 0.5f);
        speed = Mathf.MoveTowards(speed, target, Time.deltaTime * 3f);
        anim.SetFloat("Speed", speed);
        if (dir.magnitude > 0.1f)
        {
            transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(dir), Time.deltaTime * 10f);
            // 이동 속도는 클립의 실측 접지 속도와 일치해야 발이 미끄러지지 않는다
            // (RetargetProbe: Walk 접지 발 0.85m/0.52s = 1.65m/s, Run = ~2.9m/s).
            // 이전 코드(1.6*speed=0.8m/s 걷기)는 절반 속도라 스케이팅이 발생했다.
            float ground = speed <= 0.5f
                ? Mathf.Lerp(0f, 1.65f, speed / 0.5f)
                : Mathf.Lerp(1.65f, 2.9f, (speed - 0.5f) / 0.5f);
            transform.position += dir.normalized * ground * Time.deltaTime;
        }
        if (Input.GetMouseButtonDown(0)) anim.SetTrigger("Attack");
        if (Input.GetKeyDown(KeyCode.Space)) anim.SetTrigger("Roll");
        if (Input.GetKeyDown(KeyCode.H)) anim.SetTrigger("Hit");
        if (Input.GetKeyDown(KeyCode.K)) anim.SetTrigger("Die");
    }
}

public class FollowCam : MonoBehaviour
{
    public Transform target; Vector3 offset;
    void Start() { offset = transform.position - target.position; }
    void LateUpdate() { transform.position = target.position + offset; transform.LookAt(target.position + Vector3.up * 1.0f); }
}
