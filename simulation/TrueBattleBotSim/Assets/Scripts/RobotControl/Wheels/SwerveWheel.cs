using UnityEngine;

class SwerveWheel : MonoBehaviour
{
    [SerializeField] WheelProperties wheelProperties;
    [SerializeField] private ArticulationBody swerveBody;
    [SerializeField] private ArticulationBody driveBody;
    private float angularXVelocityRadPerSec = 0.0f;
    private float angularYVelocityRadPerSec = 0.0f;

    void Start()
    {
        if (swerveBody == null)
        {
            Debug.LogError($"SwerveWheel {gameObject.name} has no swerveBody assigned!");
        }
        if (driveBody == null)
        {
            Debug.LogError($"SwerveWheel {gameObject.name} has no driveBody assigned!");
        }
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

    void SetDriveVelocity(float velocity, float dt)
    {
        ArticulationDrive drive = driveBody.xDrive;
        float targetAngularVelocity = WheelRateLimiter.Limit(
            velocity, drive.targetVelocity, dt, wheelProperties
        );
        drive.targetVelocity = targetAngularVelocity;
        driveBody.xDrive = drive;
    }

    void SetDriveDirection(float angleDegrees)
    {
        ArticulationDrive drive = swerveBody.xDrive;
        drive.target = angleDegrees;
        swerveBody.xDrive = drive;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        float angle = Mathf.Atan2(angularYVelocityRadPerSec, angularXVelocityRadPerSec) * Mathf.Rad2Deg;
        if (float.IsNaN(angle))
        {
            angle = 0.0f;
        }
        float speed = Mathf.Sqrt(angularXVelocityRadPerSec * angularXVelocityRadPerSec + angularYVelocityRadPerSec * angularYVelocityRadPerSec);
        SetDriveVelocity(speed, dt);
        SetDriveDirection(angle);
    }
}