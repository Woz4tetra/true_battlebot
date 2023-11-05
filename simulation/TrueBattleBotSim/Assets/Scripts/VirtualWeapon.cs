using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using Unity.VisualScripting;
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

    void OnTriggerEnter(Collider other) {
        if (other.gameObject.tag == gameObject.tag) {
            ApplyForceToOther(gameObject);
            ApplyForceToOther(other.gameObject);
            Debug.Log("weapons collided");
        }
        else if (isLayerInMask(other.gameObject.layer)) {
            ApplyForceToOther(other.gameObject);
        }
    }

    private void ApplyForceToOther(GameObject other) {

        UnityEngine.Vector3 direction = other.transform.up.normalized;
        UnityEngine.Vector3 force = UnityEngine.Quaternion.Euler(-45, 0, 0) * direction * forceMagnitude;
        Rigidbody body = GetComponentInTree<Rigidbody>(gameObject);
        if (body != null) {
            body.AddForce(force, ForceMode.Impulse);
        }
        else {
            ArticulationBody artBody = GetComponentInTree<ArticulationBody>(other);
            if (artBody != null) {
                artBody.AddForce(force, ForceMode.Impulse);
            }
        }
    }

    private T GetComponentInTree<T>(GameObject obj) {
        Transform tf = obj.transform;
        T component = obj.GetComponent<T>();
        while (component is not null)  // ???somehow this only works in reverse???
        {
            if (tf.parent is null)
            {
                break;
            }
            tf = tf.parent;
            obj = tf.gameObject;
            component = obj.GetComponent<T>();
        }
        return component;
    }

    private bool isLayerInMask(int layer) {
        return targetMask == (targetMask | (1 << layer));
    }
}
