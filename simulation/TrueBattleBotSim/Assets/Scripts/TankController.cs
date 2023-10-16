using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

class TankController : MonoBehaviour
{
    [SerializeField] private string cmdVelTopic = "cmd_vel";
    [SerializeField] private float baseWidth = 1.0f;
    private ROSConnection ros;

    [SerializeField] private Wheel leftWheel;
    [SerializeField] private Wheel rightWheel;

    public void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(cmdVelTopic, cmdVelCallback);
    }

    private void cmdVelCallback(TwistMsg twist)
    {
        float linearVelocity = (float)twist.linear.x;  // m/s
        float angularVelocity = (float)twist.angular.z;  // rad/s
        float leftWheelSpeed = linearVelocity - angularVelocity * baseWidth / 2.0f;
        float rightWheelSpeed = linearVelocity + angularVelocity * baseWidth / 2.0f;
        leftWheel.setVelocity(leftWheelSpeed);
        rightWheel.setVelocity(rightWheelSpeed);
    }
}