using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.VisualScripting;
using UnityEngine;
public class GroundTruthPosePublisher : MonoBehaviour {
    private ROSConnection ros;
    [SerializeField] private string topic = "ground_truth_pose";
    [SerializeField] private string frame_id = "map";
    [SerializeField] private GameObject relativeTo = null;
    private uint messageCount = 0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topic);
    }

    void Update() {
        Matrix4x4 pose;
        if (relativeTo == null) {
            pose = transform.localToWorldMatrix;
        }
        else {
            pose = relativeTo.transform.worldToLocalMatrix * transform.localToWorldMatrix;
        }
        PoseStampedMsg msg = new PoseStampedMsg {
            header = new HeaderMsg {
                frame_id = frame_id,
                stamp = RosUtil.GetTimeMsg(),
                seq = messageCount++
            },
            pose = new PoseMsg {
                position = pose.GetT().To<FLU>(),
                orientation = pose.GetR().To<FLU>()
            }
        };
        ros.Publish(topic, msg);
    }
}
