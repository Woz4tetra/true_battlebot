using UnityEngine;

public class VirtualWeapon : MonoBehaviour
{
    [SerializeField] float forceMagnitude = 10;
    [SerializeField] float collisionCooldown = 0.25f;
    [SerializeField] string[] filterTags = new string[] { };
    float collisionCooldownTimer = 0.0f;

    // Start is called before the first frame update
    void Start()
    {
        collisionCooldownTimer = 0.0f;
    }

    // Update is called once per frame
    void Update()
    {

    }

    void OnTriggerEnter(Collider other)
    {
        float timerDelta = Time.realtimeSinceStartup - collisionCooldownTimer;
        if (timerDelta < collisionCooldown)
        {
            Debug.Log(
                "Ignoring weapon collision with " +
                $"{other.gameObject.tag} {other.gameObject.name} due to cooldown. " +
                $"Time left: {timerDelta}"
            );
            return;
        }
        if (other.gameObject.tag == gameObject.tag)
        {
            Debug.Log($"Weapon collided with another weapon {other.gameObject.name}");
            Vector3 this_backwards = -1 * Vector3.Normalize(gameObject.transform.forward + gameObject.transform.right);
            Vector3 other_backwards = -1 * Vector3.Normalize(other.gameObject.transform.forward + other.gameObject.transform.right);
            ApplyForceToOther(gameObject, 2 * this_backwards);
            ApplyForceToOther(other.gameObject, 2 * other_backwards);
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

    private void ApplyForceToOther(GameObject obj, Vector3 direction)
    {
        collisionCooldownTimer = Time.realtimeSinceStartup;
        Vector3 force = direction * forceMagnitude;
        Rigidbody body = ObjectUtils.GetComponentInTree<Rigidbody>(obj);
        if (body != null)
        {
            force *= body.mass;
            body.AddForce(force, ForceMode.Impulse);
            return;
        }
        else
        {
            ArticulationBody artBody = ObjectUtils.GetComponentInTree<ArticulationBody>(obj);
            if (artBody != null)
            {
                force *= artBody.mass;
                artBody.AddForce(force, ForceMode.Impulse);
                return;
            }
        }
        Debug.Log($"No rigidbody or articulation body found in tree for {obj.name}");
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
