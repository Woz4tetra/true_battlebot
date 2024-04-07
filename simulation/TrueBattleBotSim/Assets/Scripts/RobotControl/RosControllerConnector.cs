using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;

class RosControllerConnector : MonoBehaviour

{
    [SerializeField] private string baseTopic = "";
    private ROSConnection ros;
    private ControllerInterface controller;
    private string cmdVelTopic;
    RosTopicState groundTruthTopic;
    public void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        cmdVelTopic = baseTopic + "/cmd_vel/relative";
        ros.Subscribe<TwistMsg>(cmdVelTopic, cmdVelCallback);
        groundTruthTopic = ros.RegisterPublisher<OdometryMsg>(baseTopic + "/ground_truth");
        controller = GetComponent<ControllerInterface>();
    }

    public void Update()
    {
        updateOdometry();
    }

    private void updateOdometry()
    {
        groundTruthTopic.Publish(controller.getGroundTruth());
    }

    private void cmdVelCallback(TwistMsg twist)
    {
        controller.setCommand(twist);
    }

    public void OnDestroy()
    {
        if (ros != null)
        {
            ros.Unsubscribe(cmdVelTopic);
        }
    }
}