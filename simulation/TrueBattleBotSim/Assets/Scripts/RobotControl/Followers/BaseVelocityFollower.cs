using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using UnityEngine;

abstract class BaseVelocityFollower : BaseFollower
{
    protected BaseFollowerEngine followerEngine;
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

    virtual public void SetFollowerEngine(BaseFollowerEngine engine)
    {
        followerEngine = engine;
    }

    override protected void OnResetSequence()
    {
        if (followerEngine != null)
        {
            followerEngine.Reset();
        }
    }

    TwistMsg ComputeVelocity(SequenceElementConfig currentElement)
    {
        OdometryMsg odom = controller.GetGroundTruth();
        Matrix4x4 currentPose = GetOdomPose(odom);
        Matrix4x4 goalPose = GetElementPose(currentElement);
        Velocity2d currentVelocity = new Velocity2d(
            (float)odom.twist.twist.linear.x,
            (float)odom.twist.twist.linear.y,
            (float)odom.twist.twist.angular.z
        );
        Velocity2d goalVelocity = new Velocity2d(currentElement.vx, currentElement.vy, currentElement.vyaw * Mathf.Deg2Rad);

        return followerEngine.ComputeVelocity(currentPose, goalPose, currentVelocity, goalVelocity);
    }

    protected override void UpdateRobotState(SequenceElementConfig next)
    {
        TwistMsg twist = ComputeVelocity(next);
        controller.SetCommand(twist);
    }
}
