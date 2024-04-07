using UnityEngine;

public class VirtualWeapon : MonoBehaviour
{
    [SerializeField] LayerMask targetMask = 0;
    [SerializeField] float forceMagnitude = 10;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == gameObject.tag)
        {
            Debug.Log($"Weapon collided with {other.gameObject.name}");
            ApplyForceToOther(gameObject);
            ApplyForceToOther(other.gameObject);
        }
        else if (isLayerInMask(other.gameObject.layer))
        {
            Debug.Log($"Weapon collided with {other.gameObject.name} in layer");
            ApplyForceToOther(other.gameObject);
        }
    }

    private void ApplyForceToOther(GameObject other)
    {

        Vector3 direction = other.transform.up.normalized;
        Vector3 force = Quaternion.Euler(-45, 0, 0) * direction * forceMagnitude;
        Rigidbody body = ObjectUtils.GetComponentInTree<Rigidbody>(gameObject);
        if (body != null)
        {
            body.AddForce(force, ForceMode.Impulse);
            return;
        }
        else
        {
            ArticulationBody artBody = ObjectUtils.GetComponentInTree<ArticulationBody>(other);
            if (artBody != null)
            {
                artBody.AddForce(force, ForceMode.Impulse);
                return;
            }
        }
        Debug.Log($"No rigidbody or articulation body found in tree for {other.name}");
    }

    private bool isLayerInMask(int layer)
    {
        return targetMask == (targetMask | (1 << layer));
    }
}
