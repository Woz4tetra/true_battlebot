using UnityEngine;

class Wheel : MonoBehaviour
{
    [SerializeField] WheelProperties wheelProperties;
    private ArticulationBody body;
    private float angularVelocityRadPerSec = 0.0f;

    void Start()
    {
        body = GetComponent<ArticulationBody>();
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
        float targetAngularVelocity = WheelRateLimiter.Limit(
            angularVelocityRadPerSec, body.xDrive.targetVelocity, Time.fixedDeltaTime, wheelProperties
        );
        ArticulationDrive drive = body.xDrive;
        drive.targetVelocity = targetAngularVelocity;
        body.xDrive = drive;
    }
}