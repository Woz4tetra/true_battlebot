using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
interface ControllerInterface
{
    void SetCommand(TwistMsg twist);
    OdometryMsg getGroundTruth();
    void Reset();
}