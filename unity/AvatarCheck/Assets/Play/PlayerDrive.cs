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
            transform.position += dir.normalized * (speed < 0.75f ? 1.6f : 4.0f) * speed * Time.deltaTime;
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
