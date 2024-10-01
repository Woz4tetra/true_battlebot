using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;
using UnityEngine;
public class StaticGroundTruthPublisher : MonoBehaviour
{
    private ROSConnection ros;
    [SerializeField] private string topic = "ground_truth/pose";
    [SerializeField] private string frame_id = "map";
    [SerializeField] private GameObject referenceObject = null;
    private uint messageCount = 0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        RosTopicState topicState = ros.GetTopic(topic);
        if (topicState == null || !topicState.IsPublisher)
        {
            ros.RegisterPublisher<PoseStampedMsg>(topic);
        }
        if (referenceObject == null)
        {
            referenceObject = GameObject.Find("Coordinate Frame");
        }
    }

    void Update()
    {
        Matrix4x4 pose = transform.localToWorldMatrix;
        if (referenceObject != null)
        {
            pose = referenceObject.transform.worldToLocalMatrix * pose;
        }
        pose = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0, -90, 0), Vector3.one) * pose * Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0, 90, 0), Vector3.one);
        PoseStampedMsg msg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = frame_id,
                stamp = RosUtil.GetTimeMsg(),
                seq = messageCount++
            },
            pose = new PoseMsg
            {
                position = pose.GetT().To<FLU>(),
                orientation = pose.GetR().To<FLU>()
            }
        };
        ros.Publish(topic, msg);
    }
}
