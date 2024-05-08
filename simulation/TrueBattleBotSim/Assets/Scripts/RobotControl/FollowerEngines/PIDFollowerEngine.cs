using MathExtensions;
using RosMessageTypes.Geometry;
using UnityEngine;


public class PIDFollowerEngine : BaseFollowerEngine
{
    [SerializeField] PID linearPID = new PID(2.0f, 0.0f, 0.0f, 1.0f);
    [SerializeField] PID angularPID = new PID(10.0f, 0.1f, 1.0f, 1.0f);

    public override void Reset()
    {
        linearPID.Reset();
        angularPID.Reset();
    }

    public override TwistMsg ComputeVelocity(Matrix4x4 currentPose, Matrix4x4 goalPose, Vector3 currentVelocity, Vector3 goalVelocity)
    {
        Matrix4x4 relativePose = currentPose.inverse * goalPose;
        Vector3 relativePosition = relativePose.GetT();
        float relativeAngle = relativePose.GetR().eulerAngles.z * Mathf.Deg2Rad;
        relativeAngle = relativeAngle % (2 * Mathf.PI);
        if (relativeAngle > Mathf.PI)
        {
            relativeAngle -= 2 * Mathf.PI;
        }

        float linearVelocity = linearPID.Update(relativePosition.x, 0.0f, Time.deltaTime);
        float angularVelocity = angularPID.Update(relativeAngle, 0.0f, Time.deltaTime);

        float maxLinearSpeedMagnitude = Mathf.Abs(goalVelocity.x);
        linearVelocity = Mathf.Clamp(linearVelocity, -maxLinearSpeedMagnitude, maxLinearSpeedMagnitude);
        if (goalVelocity.x < 0)
        {
            linearVelocity *= -1;
        }
        float maxAngularSpeedMagnitude = Mathf.Abs(goalVelocity.z);
        angularVelocity = Mathf.Clamp(angularVelocity, -maxAngularSpeedMagnitude, maxAngularSpeedMagnitude);
        if (goalVelocity.z < 0)
        {
            angularVelocity *= -1;
        }
        return new TwistMsg
        {
            linear = new Vector3Msg { x = linearVelocity },
            angular = new Vector3Msg { z = angularVelocity }
        };
    }
}