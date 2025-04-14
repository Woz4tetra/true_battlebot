using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

public interface ControllerInterface
{
    void SetCommand(TwistMsg twist);
    void Teleport(PointMsg position, QuaternionMsg rotation);
    OdometryMsg GetGroundTruth();
    void Reset();
    bool IsUpsideDown();
    string GetName();
}