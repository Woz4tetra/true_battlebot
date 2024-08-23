using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;

class SetPoseTopic
{
    private ROSConnection ros;
    private Optional<PoseMsg> pose = Optional<PoseMsg>.CreateEmpty();
    public SetPoseTopic(string topic)
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PoseMsg>(topic, PoseCallback);
    }

    private void PoseCallback(PoseMsg pose)
    {
        this.pose = Optional<PoseMsg>.Create(pose);
    }

    public Optional<PoseMsg> Receive()
    {
        Optional<PoseMsg> result = pose;
        pose = Optional<PoseMsg>.CreateEmpty();
        return result;
    }
}

class TeleportToPoseTopic : MonoBehaviour
{
    [SerializeField] private string topic = "";
    [SerializeField] private GameObject referenceObject;

    static SetPoseTopic setPoseTopic = null;

    public void Start()
    {
        if (referenceObject == null)
        {
            referenceObject = GameObject.Find("Coordinate Frame");
        }
        if (setPoseTopic == null)
        {
            setPoseTopic = new SetPoseTopic(topic);
        }
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
