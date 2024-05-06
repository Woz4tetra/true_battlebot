using UnityEngine;

public class VirtualWeapon : MonoBehaviour
{
    [SerializeField] float forceMagnitude = 10;
    [SerializeField] float reactionForceScale = 0.25f;
    [SerializeField] string[] filterTags = new string[] { };

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
            Debug.Log($"Weapon collided with another weapon {other.gameObject.name}");
            ApplyForceToOther(gameObject, -gameObject.transform.forward.normalized);
            ApplyForceToOther(other.gameObject, -other.gameObject.transform.forward.normalized);
        }
        else if (isTagInFilter(other.gameObject.tag))
        {
            Debug.Log($"Weapon collided with a target {other.gameObject.name}");
            ApplyForceToOther(gameObject, -transform.up.normalized);
            ApplyForceToOther(other.gameObject, other.transform.up.normalized);
        }
        else
        {
            Debug.Log($"Ignoring weapon collision with {other.gameObject.tag} {other.gameObject.name}");
        }
    }

    private void ApplyForceToOther(GameObject other, Vector3 direction)
    {
        Vector3 force = direction * forceMagnitude;
        Rigidbody body = ObjectUtils.GetComponentInTree<Rigidbody>(gameObject);
        if (body != null)
        {
            force *= body.mass;
            body.AddForce(force, ForceMode.Impulse);
            return;
        }
        else
        {
            ArticulationBody artBody = ObjectUtils.GetComponentInTree<ArticulationBody>(other);
            if (artBody != null)
            {
                force *= artBody.mass;
                artBody.AddForce(force, ForceMode.Impulse);
                return;
            }
        }
        Debug.Log($"No rigidbody or articulation body found in tree for {other.name}");
    }

    private bool isTagInFilter(string tag)
    {
        foreach (string filterTag in filterTags)
        {
            if (tag == filterTag)
            {
                return true;
            }
        }
        return false;
    }
}
