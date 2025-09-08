using System;
using UnityEngine;

class Wheel : MonoBehaviour
{
    [SerializeField] private float maxWheelSpeedRpm = 1000.0f;
    [SerializeField] private float wheelRadius = 1.0f;
    [SerializeField] private float accelerationRpmPerSec = 100.0f;
    [SerializeField] private float decelerationRpmPerSec = 10000.0f;
    private ArticulationBody body;
    private float angularVelocityDegPerSec = 0.0f;

    void Start()
    {
        body = GetComponent<ArticulationBody>();
    }

    public void setVelocity(float groundVelocity)
    {
        float maxWheelSpeedRadPerSec = maxWheelSpeedRpm * 2.0f * Mathf.PI / 60.0f;
        float angularVelocityRadPerSec = groundVelocity / wheelRadius;
        if (Mathf.Abs(angularVelocityRadPerSec) > maxWheelSpeedRadPerSec)
        {
            angularVelocityRadPerSec = Mathf.Sign(angularVelocityRadPerSec) * maxWheelSpeedRadPerSec;
        }
        angularVelocityDegPerSec = Mathf.Rad2Deg * angularVelocityRadPerSec;
    }

    void FixedUpdate()
    {
        float accelerationDegPerSec2 = Mathf.Abs(accelerationRpmPerSec) * 2.0f * 360.0f / 60.0f;
        float decelerationDegPerSec2 = Mathf.Abs(decelerationRpmPerSec) * 2.0f * 360.0f / 60.0f;
        ArticulationDrive drive = body.xDrive;
        float previousAngularVelocity = drive.targetVelocity;
        float currentAcceleration = (angularVelocityDegPerSec - previousAngularVelocity) / Time.fixedDeltaTime;
        if (currentAcceleration > 0)
        {
            if (previousAngularVelocity > 0) // speeding up
            {
                currentAcceleration = Mathf.Min(currentAcceleration, accelerationDegPerSec2);
            }
            else // slowing down
            {
                currentAcceleration = Mathf.Min(currentAcceleration, decelerationDegPerSec2);
            }
        }
        else
        {
            if (previousAngularVelocity < 0) // speeding up (in negative direction)
            {
                currentAcceleration = Mathf.Max(currentAcceleration, -accelerationDegPerSec2);
            }
            else // slowing down
            {
                currentAcceleration = Mathf.Max(currentAcceleration, -decelerationDegPerSec2);
            }
        }
        float targetAngularVelocity = previousAngularVelocity + currentAcceleration * Time.fixedDeltaTime;
        drive.targetVelocity = targetAngularVelocity;
        body.xDrive = drive;
    }
}