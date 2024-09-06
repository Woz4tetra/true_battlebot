using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;

class RosControllerConnector : MonoBehaviour

{
    [SerializeField] string baseTopic = "";
    [SerializeField] float commandTimeout = 0.1f;
    ROSConnection ros;
    ControllerInterface controller;
    RosTopicState groundTruthTopic;
    RosPollSubscriber<TwistMsg> cmdVelTopic;
    float lastCommandTime;

    public void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        cmdVelTopic = new RosPollSubscriber<TwistMsg>(baseTopic + "/cmd_vel");
        groundTruthTopic = ros.GetTopic(baseTopic + "/ground_truth");
        if (groundTruthTopic == null)
        {
            groundTruthTopic = ros.RegisterPublisher<OdometryMsg>(baseTopic + "/ground_truth");
        }
        controller = GetComponent<ControllerInterface>();
        lastCommandTime = Time.time;
    }

    public void Update()
    {
        updateOdometry();

        if (cmdVelTopic.Receive().TryGet(out TwistMsg command))
        {
            controller.SetCommand(command);
            lastCommandTime = Time.time;
        }
        if (Time.time - lastCommandTime > commandTimeout)
        {
            controller.SetCommand(new TwistMsg());
        }
    }

    private void updateOdometry()
    {
        groundTruthTopic.Publish(controller.GetGroundTruth());
    }
}
