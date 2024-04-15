using MathExtensions;
using RosMessageTypes.Geometry;
using UnityEngine;


public class RamseteFollowerEngine : BaseFollowerEngine
{
    [SerializeField] float zeta = 4.0f;
    [SerializeField] float b = 8.0f;
    public override void Reset()
    {

    }

    public override TwistMsg ComputeVelocity(Matrix4x4 currentPose, Matrix4x4 goalPose, Vector3 currentVelocity, Vector3 goalVelocity)
    {
        Matrix4x4 relativePose = currentPose.inverse * goalPose;
        Vector3 relativePosition = relativePose.GetT();
        float relativeAngle = relativePose.GetR().eulerAngles.z * Mathf.Deg2Rad;
        relativeAngle = (relativeAngle % (2 * Mathf.PI)) - Mathf.PI;

        float vxRef = goalVelocity.x;
        float vtRef = goalVelocity.z;
        float k = (
            2.0f
            * zeta
            * Mathf.Sqrt(vtRef * vtRef + b * vxRef * vxRef)
        );

        float vxInput = vxRef * Mathf.Cos(relativeAngle) + k * relativePosition.x;
        float vtInput = (
            vtRef
            + k * relativeAngle
            + b * vxRef * Mathf.Sin(relativeAngle) * relativePosition.y
        );

        return new TwistMsg
        {
            linear = new Vector3Msg { x = vxInput },
            angular = new Vector3Msg { z = vtInput }
        };
    }
}