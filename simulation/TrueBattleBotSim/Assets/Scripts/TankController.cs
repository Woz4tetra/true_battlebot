using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

class TankController : MonoBehaviour
{
    [SerializeField] private string baseTopic = "";
    [SerializeField] private float baseWidth = 1.0f;
    private ROSConnection ros;

    [SerializeField] private Wheel leftWheel;
    [SerializeField] private Wheel rightWheel;

    ArticulationBody body;

    public void Start()
    {
        body = GetComponent<ArticulationBody>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(baseTopic + "/cmd_vel", cmdVelCallback);
        ros.RegisterPublisher<OdometryMsg>(baseTopic + "/ground_truth");
    }

    public void Update() {
        OdometryMsg msg = new OdometryMsg {
            header = new HeaderMsg {
                seq = 0,
                stamp = RosUtil.GetTimeMsg(),
                frame_id = "map"
            },
            child_frame_id = baseTopic + "_base_link",
            pose = new PoseWithCovarianceMsg {
                pose = new PoseMsg {
                    position = transform.position.To<FLU>(),
                    orientation = transform.rotation.To<FLU>()
                }
            },
            twist = new TwistWithCovarianceMsg {
                twist = GetGroundTruthVelocity()
            }
        };
        ros.Publish(baseTopic + "/ground_truth", msg);
    }
    private Vector3 GetRelativeVelocity()
    {
        // Get the velocity of the body in world space
        Vector3 worldVelocity = body.velocity;

        // Convert the velocity to the body's local space
        Vector3 localVelocity = body.transform.InverseTransformDirection(worldVelocity);

        return localVelocity;
    }

    private TwistMsg GetGroundTruthVelocity() {
        Vector3 localVelocity = GetRelativeVelocity();
        Vector3 angularVelocity = -1 * body.angularVelocity;


        return new TwistMsg {
            linear = localVelocity.To<FLU>(),
            angular = angularVelocity.To<FLU>()
        };
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