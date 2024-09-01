using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

class ImuSensor : MonoBehaviour
{
    [SerializeField] private string FrameId = "imu";
    [SerializeField] private double[] orientationCovariance = { 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f };
    [SerializeField] private double[] angularVelocityCovariance = { 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f };
    [SerializeField] private double[] linearAccelerationCovariance = { 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f };

    private ImuMsg imuMsg;
    private Vector3 prevVelocity = new Vector3();

    [SerializeField] private double publishDelay;
    [SerializeField] private string topic;
    private double _prevPublishTime;
    private uint messageCount;

    private ROSConnection ros;
    private Quaternion startOrientation;
    private Rigidbody sensorBody;

    void Start()
    {
        sensorBody = GetComponent<Rigidbody>();
        imuMsg = new ImuMsg();

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topic);
        _prevPublishTime = Time.realtimeSinceStartup;

        imuMsg.header.frame_id = FrameId;

        imuMsg.orientation_covariance = orientationCovariance;
        imuMsg.angular_velocity_covariance = angularVelocityCovariance;
        imuMsg.linear_acceleration_covariance = linearAccelerationCovariance;

        startOrientation = Quaternion.Inverse(sensorBody.transform.rotation);
    }

    void FixedUpdate()
    {
        double now = Time.realtimeSinceStartup;
        if (now - _prevPublishTime > publishDelay)
        {
            ros.Publish(topic, imuMsg);
            _prevPublishTime = now;
        }

        imuMsg.header.stamp = RosUtil.GetTimeMsg();
        imuMsg.header.seq = messageCount;

        Vector3 velocity = transform.InverseTransformDirection(sensorBody.velocity);
        float dt = Time.fixedDeltaTime;
        Vector3 accel = new Vector3(
            (velocity.x - prevVelocity.x) / dt,
            (velocity.y - prevVelocity.y) / dt,
            (velocity.z - prevVelocity.z) / dt
        );
        prevVelocity = velocity;
        imuMsg.linear_acceleration = accel.To<FLU>();

        imuMsg.angular_velocity = -sensorBody.angularVelocity.To<FLU>();

        imuMsg.orientation = (sensorBody.transform.rotation * startOrientation).To<FLU>();
        messageCount++;
    }
}