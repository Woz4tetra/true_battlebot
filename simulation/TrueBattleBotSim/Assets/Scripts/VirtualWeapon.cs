using UnityEngine;

public class VirtualWeapon : MonoBehaviour
{
    [SerializeField] float forceMagnitude = 10;
    [SerializeField] float torqueMagnitude = 0.25f;
    [SerializeField] float collisionCooldown = 0.25f;
    float collisionCooldownTimer = 0.0f;
    GameObject topLevelObject;

    // Start is called before the first frame update
    void Start()
    {
        collisionCooldownTimer = 0.0f;
        topLevelObject = ObjectUtils.GetTopLevelObject(gameObject);
    }

    // Update is called once per frame
    void Update()
    {

    }

    void OnTriggerEnter(Collider other)
    {
        if (topLevelObject == ObjectUtils.GetTopLevelObject(other.gameObject))
        {
            // Ignore collisions with self
            return;
        }
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
            ApplyForceToOther(gameObject, 2 * this_backwards, applyTorque: false);
            ApplyForceToOther(other.gameObject, 2 * other_backwards, applyTorque: false);
        }
        else
        {
            Debug.Log($"Weapon collided with a target {other.gameObject.name}");
            Vector3 this_backwards = -1 * Vector3.Normalize(gameObject.transform.forward + gameObject.transform.right);
            ApplyForceToOther(gameObject, this_backwards, applyTorque: false);
            ApplyForceToOther(other.gameObject, transform.up.normalized);
        }
    }

    private void ApplyForceToOther(GameObject obj, Vector3 direction, bool applyTorque = true)
    {
        collisionCooldownTimer = Time.realtimeSinceStartup;
        Vector3 force = direction * forceMagnitude;
        Rigidbody body = ObjectUtils.GetComponentInTree<Rigidbody>(obj);
        if (body != null)
        {
            force *= body.mass;
            body.AddForce(force, ForceMode.Impulse);
            if (applyTorque)
            {
                body.AddTorque(Random.onUnitSphere * torqueMagnitude, ForceMode.Impulse);
            }
            return;
        }
        else
        {
            ArticulationBody artBody = ObjectUtils.GetComponentInTree<ArticulationBody>(obj);
            if (artBody != null)
            {
                force *= artBody.mass;
                artBody.AddForce(force, ForceMode.Impulse);
                if (applyTorque)
                {
                    artBody.AddTorque(Random.onUnitSphere * torqueMagnitude, ForceMode.Impulse);
                }
                return;
            }
        }
        Debug.Log($"No rigidbody or articulation body found in tree for {obj.name}");
    }
}
