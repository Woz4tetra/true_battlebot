using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
interface ControllerInterface
{
    void setCommand(TwistMsg twist);
    OdometryMsg getGroundTruth();
}