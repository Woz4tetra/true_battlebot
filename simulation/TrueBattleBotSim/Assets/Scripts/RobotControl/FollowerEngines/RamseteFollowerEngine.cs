using MathExtensions;
using RosMessageTypes.Geometry;
using UnityEngine;


public class RamseteFollowerEngine : BaseFollowerEngine
{
    [SerializeField] RamseteConfig ramseteConfig = new RamseteConfig(2.0f, 0.7f);
    public override void Reset()
    {

    }

    public void SetRamseteConfig(RamseteConfig config)
    {
        ramseteConfig = config;
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

        float vxRef = goalVelocity.x;  // m/s
        float vtRef = goalVelocity.z;  // rad/s
        float k = (
            2.0f
            * ramseteConfig.zeta
            * Mathf.Sqrt(vtRef * vtRef + ramseteConfig.b * vxRef * vxRef)
        );

        float vxInput = vxRef * Mathf.Cos(relativeAngle) + k * relativePosition.x;
        float vtInput = (
            vtRef
            + k * relativeAngle
            + ramseteConfig.b * vxRef * Mathf.Sin(relativeAngle) * relativePosition.y
        );

        return new TwistMsg
        {
            linear = new Vector3Msg { x = vxInput },
            angular = new Vector3Msg { z = vtInput }
        };
    }
}