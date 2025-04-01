using RosMessageTypes.Geometry;
using UnityEngine;

public abstract class BaseFollowerEngine : MonoBehaviour
{
    public abstract void Reset();
    public abstract TwistMsg ComputeVelocity(Matrix4x4 currentPose, Matrix4x4 goalPose, Velocity2d currentVelocity, Velocity2d goalVelocity);
}