using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

class GroundTruthPublisher : MonoBehaviour

{
    [SerializeField] private string baseTopic = "";
    private ROSConnection ros;
    private ControllerInterface controller;
    RosTopicState groundTruthTopic;
    RosTopicState groundTruthPoseTopic;
    public void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        groundTruthTopic = ros.RegisterPublisher<OdometryMsg>(baseTopic + "/ground_truth");
        groundTruthPoseTopic = ros.RegisterPublisher<PoseStampedMsg>(baseTopic + "/ground_truth/pose");
        controller = GetComponent<ControllerInterface>();
    }

    public void Update()
    {
        updateOdometry();
    }

    private void updateOdometry()
    {
        OdometryMsg odometry = controller.GetGroundTruth();
        groundTruthTopic.Publish(odometry);
        PoseStampedMsg pose = new PoseStampedMsg()
        {
            header = odometry.header,
            pose = odometry.pose.pose
        };
        groundTruthPoseTopic.Publish(pose);
    }
}