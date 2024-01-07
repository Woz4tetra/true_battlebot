using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;

class RosControllerConnector : MonoBehaviour

{
    [SerializeField] private string baseTopic = "";
    private ROSConnection ros;
    private ControllerInterface controller;
    public void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(baseTopic + "/cmd_vel/relative", cmdVelCallback);
        ros.RegisterPublisher<OdometryMsg>(baseTopic + "/ground_truth");

        controller = GetComponent<ControllerInterface>();
    }
    public void Update()
    {
        updateOdometry();
    }

    private void updateOdometry()
    {
        ros.Publish(baseTopic + "/ground_truth", controller.getGroundTruth());
    }

    private void cmdVelCallback(TwistMsg twist)
    {
        controller.setCommand(twist);
    }
}