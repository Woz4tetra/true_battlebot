using UnityEngine;

public static class WheelRateLimiter
{
    public static float Limit(float angularVelocityRadPerSec, float previousAngularVelocity, float dt, WheelProperties wheelProperties)
    {
        float angularVelocityDegPerSec = Mathf.Rad2Deg * angularVelocityRadPerSec;
        float accelerationDegPerSec2 = Mathf.Abs(wheelProperties.AccelerationRpmPerSec) * 2.0f * 360.0f / 60.0f;
        float decelerationDegPerSec2 = Mathf.Abs(wheelProperties.DecelerationRpmPerSec) * 2.0f * 360.0f / 60.0f;
        float currentAcceleration = (angularVelocityDegPerSec - previousAngularVelocity) / dt;
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
        float targetAngularVelocity = previousAngularVelocity + currentAcceleration * dt;
        return targetAngularVelocity;
    }
}