using UnityEngine;

class OmniWheel : MonoBehaviour
{
    [SerializeField] WheelProperties wheelProperties;
    private ArticulationBody body;
    private float angularXVelocityRadPerSec = 0.0f;
    private float angularYVelocityRadPerSec = 0.0f;

    void Start()
    {
        body = GetComponent<ArticulationBody>();
    }

    private float GetLimitedVelocity(float groundVelocity)
    {
        float maxWheelSpeedRadPerSec = wheelProperties.MaxWheelSpeedRpm * 2.0f * Mathf.PI / 60.0f;
        float angularVelocityRadPerSec = groundVelocity / wheelProperties.WheelRadius;
        if (Mathf.Abs(angularVelocityRadPerSec) > maxWheelSpeedRadPerSec)
        {
            angularVelocityRadPerSec = Mathf.Sign(angularVelocityRadPerSec) * maxWheelSpeedRadPerSec;
        }
        return angularVelocityRadPerSec;
    }

    public void SetVelocity(Vector2 groundVelocity)
    {
        angularXVelocityRadPerSec = GetLimitedVelocity(groundVelocity.x);
        angularYVelocityRadPerSec = GetLimitedVelocity(groundVelocity.y);
    }

    void SetDriveVelocity(float velocity, ArticulationDrive drive, float dt)
    {
        float targetAngularVelocity = WheelRateLimiter.Limit(
            velocity, drive.targetVelocity, dt, wheelProperties
        );
        drive.targetVelocity = targetAngularVelocity;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        // SetDriveVelocity(angularXVelocityRadPerSec, body.xDrive, dt);
        // SetDriveVelocity(angularYVelocityRadPerSec, body.yDrive, dt);
    }
}