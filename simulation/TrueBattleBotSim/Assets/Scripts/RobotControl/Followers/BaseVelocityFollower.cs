using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using UnityEngine;

abstract class BaseVelocityFollower : BaseFollower
{
    BaseFollowerEngine followerEngine;
    public BaseVelocityFollower() : base()
    {
        if (followerEngine != null)
        {
            followerEngine.Reset();
        }
    }

    public override void Awake()
    {
        base.Awake();
        SetShowArrow(true);
    }

    public void SetFollowerEngine(BaseFollowerEngine engine)
    {
        followerEngine = engine;
    }

    TwistMsg ComputeVelocity(SequenceElementConfig currentElement)
    {
        OdometryMsg odom = controller.GetGroundTruth();
        Matrix4x4 currentPose = GetOdomPose(odom);
        Matrix4x4 goalPose = GetElementPose(currentElement);
        Vector3 currentVelocity = new Vector3(
            (float)odom.twist.twist.linear.x,
            (float)odom.twist.twist.linear.y,
            (float)odom.twist.twist.angular.z
        );
        Vector3 goalVelocity = new Vector3(currentElement.vx, currentElement.vy, currentElement.vyaw * Mathf.Deg2Rad);

        return followerEngine.ComputeVelocity(currentPose, goalPose, currentVelocity, goalVelocity);
    }

    protected override void UpdateRobotState(SequenceElementConfig next)
    {
        TwistMsg twist = ComputeVelocity(next);
        controller.SetCommand(twist);
    }
}
