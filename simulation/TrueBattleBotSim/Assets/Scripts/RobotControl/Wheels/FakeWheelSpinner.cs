using UnityEngine;

class FakeWheelSpinner : MonoBehaviour
{
    [SerializeField] WheelProperties wheelProperties;
    [SerializeField] Vector3 rotationAxis = Vector3.up;
    private float angularVelocityRadPerSec = 0.0f;
    private float previousAngularVelocity = 0.0f;

    void Start()
    {
        rotationAxis.Normalize();
    }

    public void SetVelocity(float groundVelocity)
    {
        float maxWheelSpeedRadPerSec = wheelProperties.MaxWheelSpeedRpm * 2.0f * Mathf.PI / 60.0f;
        angularVelocityRadPerSec = groundVelocity / wheelProperties.WheelRadius;
        if (Mathf.Abs(angularVelocityRadPerSec) > maxWheelSpeedRadPerSec)
        {
            angularVelocityRadPerSec = Mathf.Sign(angularVelocityRadPerSec) * maxWheelSpeedRadPerSec;
        }
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        float targetAngularVelocity = WheelRateLimiter.Limit(
            angularVelocityRadPerSec, previousAngularVelocity, dt, wheelProperties
        );

        // Rotate the wheel visually
        transform.Rotate(rotationAxis, targetAngularVelocity * Mathf.Rad2Deg * dt, Space.Self);
        previousAngularVelocity = targetAngularVelocity;
    }
}