using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
interface ControllerInterface
{
    void SetCommand(TwistMsg twist);
    OdometryMsg GetGroundTruth();
    void Reset();
    bool IsUpsideDown();
}