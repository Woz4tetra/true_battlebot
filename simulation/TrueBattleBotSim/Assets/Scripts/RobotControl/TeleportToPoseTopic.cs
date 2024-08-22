using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;

class TeleportToPoseTopic : MonoBehaviour

{
    [SerializeField] private string topic = "";
    private ROSConnection ros;
    [SerializeField] private GameObject referenceObject;

    public void Start()
    {
        if (referenceObject == null)
        {
            referenceObject = GameObject.Find("Coordinate Frame");
        }
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PoseMsg>(topic, poseCallback);
    }

    private void poseCallback(PoseMsg pose)
    {
        Matrix4x4 objectPose = Matrix4x4.TRS(pose.position.From<FLU>(), pose.orientation.From<FLU>(), Vector3.one);
        if (referenceObject != null)
        {
            objectPose = referenceObject.transform.worldToLocalMatrix * objectPose;
        }
        transform.position = objectPose.GetT();
        transform.rotation = objectPose.GetR();
    }

    public void OnDestroy()
    {
        if (ros != null)
        {
            ros.Unsubscribe(topic);
        }
    }
}
