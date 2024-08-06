using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

public interface ControllerInterface
{
    void SetCommand(TwistMsg twist);
    void Teleport(UnityEngine.Vector3 position, UnityEngine.Quaternion rotation);
    OdometryMsg GetGroundTruth();
    void Reset();
    bool IsUpsideDown();
}