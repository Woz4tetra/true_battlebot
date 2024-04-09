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
        Debug.Log($"Weapon collided with {other.gameObject.tag} {other.gameObject.name}");
        if (other.gameObject.tag == gameObject.tag)
        {
            Debug.Log($"Weapon collided with another weapon {other.gameObject.name}");
            ApplyForceToOther(gameObject);
            ApplyForceToOther(other.gameObject);
        }
        else if (isTagInFilter(other.gameObject.tag))
        {
            Debug.Log($"Weapon collided with a target {other.gameObject.name}");
            ApplyForceToOther(gameObject, -reactionForceScale);
            ApplyForceToOther(other.gameObject);
        }
    }

    private void ApplyForceToOther(GameObject other, float scale = 1.0f)
    {

        Vector3 direction = other.transform.up.normalized;
        Vector3 force = direction * forceMagnitude * scale;
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
