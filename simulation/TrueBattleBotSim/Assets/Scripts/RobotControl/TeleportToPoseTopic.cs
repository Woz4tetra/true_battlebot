using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;


class TeleportToPoseTopic : MonoBehaviour
{
    [SerializeField] private string topic = "";
    [SerializeField] private GameObject referenceObject;

    RosPollSubscriber<PoseMsg> setPoseTopic;

    public void Start()
    {
        if (referenceObject == null)
        {
            referenceObject = GameObject.Find("Coordinate Frame");
        }

        setPoseTopic = new RosPollSubscriber<PoseMsg>(topic);
    }

    void Update()
    {
        if (!setPoseTopic.Receive().TryGet(out PoseMsg pose))
        {
            return;
        }
        Debug.Log($"Received pose: {pose.position} {pose.orientation}");
        Matrix4x4 objectPose = Matrix4x4.TRS(pose.position.From<FLU>(), pose.orientation.From<FLU>(), Vector3.one);
        if (referenceObject != null)
        {
            objectPose = referenceObject.transform.worldToLocalMatrix * objectPose;
        }
        objectPose = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0, 90, 0), Vector3.one) * objectPose;
        transform.position = objectPose.GetT();
        transform.rotation = objectPose.GetR();
    }
}
