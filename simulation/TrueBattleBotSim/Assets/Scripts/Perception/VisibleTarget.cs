using RosMessageTypes.Std;
using UnityEngine;

public class VisibleTarget {
    public HeaderMsg header;
    public int tagId = 0;
    public string tagName = "";
    public Vector3 dimensions = Vector3.zero;
    public Matrix4x4 cameraRelativePose = Matrix4x4.identity;
}
