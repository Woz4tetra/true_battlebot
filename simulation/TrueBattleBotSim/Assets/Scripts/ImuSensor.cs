using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

class ImuSensor : MonoBehaviour
{
    [SerializeField] private string FrameId = "imu";

    private ImuMsg imuMsg;
    private Vector3 prevVelocity = new Vector3();

    [SerializeField] private double publishDelay;
    [SerializeField] private string topic;
    private double _prevPublishTime;
    private uint messageCount;

    private ROSConnection _ros;
    private Quaternion startOrientation;
    private Rigidbody sensorBody;

    void Start()
    {
        sensorBody = GetComponent<Rigidbody>();
        imuMsg = new ImuMsg();

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<ImuMsg>(topic);
        _prevPublishTime = Time.realtimeSinceStartup;

        imuMsg.header.frame_id = FrameId;

        imuMsg.orientation_covariance = new double[] {
            1e-1, 0.0, 0.0,
            0.0, 1e-1, 0.0,
            0.0, 0.0, 1e-1
        };
        imuMsg.angular_velocity_covariance = new double[] {
            1e-1, 0.0, 0.0,
            0.0, 1e-1, 0.0,
            0.0, 0.0, 1e-1
        };
        imuMsg.linear_acceleration_covariance = new double[] {
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1e-2
        };

        startOrientation = Quaternion.Inverse(sensorBody.transform.rotation);
    }

    void FixedUpdate()
    {
        double now = Time.realtimeSinceStartup;
        if (now - _prevPublishTime > publishDelay)
        {
            _ros.Publish(topic, imuMsg);
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