using UnityEngine;

class Wheel : MonoBehaviour
{
    [SerializeField] private float maxWheelSpeedRpm = 1000.0f;
    [SerializeField] private float wheelRadius = 1.0f;
    private ArticulationBody body;
    private float angularVelocityDegPerSec = 0.0f;
    private float maxWheelSpeedRadPerSec;

    void Start()
    {
        maxWheelSpeedRadPerSec = maxWheelSpeedRpm * 2.0f * Mathf.PI / 60.0f;
        body = GetComponent<ArticulationBody>();
    }

    public void setVelocity(float groundVelocity)
    {
        float angularVelocityRadPerSec = groundVelocity / wheelRadius;
        if (Mathf.Abs(angularVelocityRadPerSec) > maxWheelSpeedRadPerSec)
        {
            angularVelocityRadPerSec = Mathf.Sign(angularVelocityRadPerSec) * maxWheelSpeedRadPerSec;
        }
        angularVelocityDegPerSec = Mathf.Rad2Deg * angularVelocityRadPerSec;
    }

    void FixedUpdate()
    {
        ArticulationDrive drive = body.xDrive;
        drive.targetVelocity = angularVelocityDegPerSec;
        body.xDrive = drive;
    }
}