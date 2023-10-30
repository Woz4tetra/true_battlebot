using UnityEngine;

class Wheel : MonoBehaviour
{
    [SerializeField] private float maxWheelSpeed = 1.0f;
    [SerializeField] private float wheelRadius = 1.0f;
    private ArticulationBody body;
    private Vector3 torque = Vector3.zero;

    void Start()
    {
        body = GetComponent<ArticulationBody>();
    }

    public void setVelocity(float groundVelocity)
    {
        if (Mathf.Abs(groundVelocity) > maxWheelSpeed)
        {
            groundVelocity = Mathf.Sign(groundVelocity) * maxWheelSpeed;
        }
        float angularVelocity = groundVelocity / wheelRadius;
        torque = new Vector3(0.0f, 0.0f, -angularVelocity);
    }

    void Update()
    {
        body.AddRelativeTorque(torque, ForceMode.VelocityChange);
    }
}