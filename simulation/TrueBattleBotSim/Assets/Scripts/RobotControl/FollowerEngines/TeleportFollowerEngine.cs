using MathExtensions;
using RosMessageTypes.Geometry;
using UnityEngine;


public class TeleportFollowerEngine : BaseFollowerEngine
{
    protected ControllerInterface controller;

    void Awake()
    {
        controller = GetComponent<ControllerInterface>();
        if (controller == null)
        {
            Debug.LogError("TeleportFollowerEngine requires a ControllerInterface component.");
        }
    }

    private void TeleportRobot(Matrix4x4 goalPose)
    {
        Vector3 position = goalPose.GetT();
        Quaternion rotation = goalPose.GetR();
        PointMsg positionMsg = new PointMsg(position.x, position.y, position.z);
        QuaternionMsg rotationMsg = new QuaternionMsg(rotation.x, rotation.y, rotation.z, rotation.w);
        controller.Teleport(positionMsg, rotationMsg);
    }

    public override TwistMsg ComputeVelocity(Matrix4x4 currentPose, Matrix4x4 goalPose, Vector3 currentVelocity, Vector3 goalVelocity)
    {
        TeleportRobot(goalPose);
        return new TwistMsg();
    }

    public override void Reset()
    {

    }
}