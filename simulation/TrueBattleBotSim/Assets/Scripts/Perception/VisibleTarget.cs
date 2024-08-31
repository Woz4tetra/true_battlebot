using RosMessageTypes.Std;
using UnityEngine;

public class VisibleTarget
{
    public HeaderMsg header;
    public int objectId = 0;
    public TransformFrame frame;
    public string label;
    public Vector3 dimensions = Vector3.zero;
    public Matrix4x4 cameraRelativePose = Matrix4x4.identity;
    public ConfigurableKeypoint[] keypoints = new ConfigurableKeypoint[0];
}
