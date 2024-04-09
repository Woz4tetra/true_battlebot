using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

class TankController : MonoBehaviour, ControllerInterface
{
    [SerializeField] private string parentFrame = "map";
    [SerializeField] private float baseWidth = 1.0f;

    [SerializeField] private Wheel leftWheel;
    [SerializeField] private Wheel rightWheel;

    [SerializeField] private Wheel[] followLeftWheels;
    [SerializeField] private Wheel[] followRightWheels;
    [SerializeField] private PID linearPid;
    [SerializeField] private PID angularPid;
    [SerializeField] private bool enablePid;
    [SerializeField] private bool reverseLeft;
    [SerializeField] private bool reverseRight;

    private ArticulationBody body;
    private TwistMsg setpoint = new TwistMsg();
    private TransformFrame frame;

    public void Start()
    {
        body = GetComponent<ArticulationBody>();
        frame = GetComponent<TransformFrame>();
    }

    public void FixedUpdate()
    {
        updateCommand();
    }

    private void updateCommand()
    {
        TwistMsg twist = ComputeControl(setpoint);
        float linearVelocity = (float)twist.linear.x;  // m/s
        float angularVelocity = (float)twist.angular.z;  // rad/s
        float leftWheelSpeed = linearVelocity - angularVelocity * baseWidth / 2.0f;
        float rightWheelSpeed = linearVelocity + angularVelocity * baseWidth / 2.0f;
        if (reverseLeft)
        {
            leftWheelSpeed *= -1;
        }
        if (reverseRight)
        {
            rightWheelSpeed *= -1;
        }
        leftWheel.setVelocity(leftWheelSpeed);
        rightWheel.setVelocity(rightWheelSpeed);
        foreach (Wheel wheel in followLeftWheels)
        {
            wheel.setVelocity(leftWheelSpeed);
        }
        foreach (Wheel wheel in followRightWheels)
        {
            wheel.setVelocity(rightWheelSpeed);
        }
    }

    public OdometryMsg getGroundTruth()
    {
        return new OdometryMsg
        {
            header = new HeaderMsg
            {
                seq = 0,
                stamp = RosUtil.GetTimeMsg(),
                frame_id = parentFrame
            },
            child_frame_id = frame.GetFrameId(),
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = transform.position.To<FLU>(),
                    orientation = transform.rotation.To<FLU>()
                }
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = GetGroundTruthVelocity()
            }
        };
    }

    private Vector3 GetRelativeVelocity()
    {
        // Get the velocity of the body in world space
        Vector3 worldVelocity = body.velocity;

        // Convert the velocity to the body's local space
        Vector3 localVelocity = body.transform.InverseTransformDirection(worldVelocity);

        return localVelocity;
    }

    private TwistMsg GetGroundTruthVelocity()
    {
        Vector3 localVelocity = GetRelativeVelocity();
        Vector3 angularVelocity = -1 * body.angularVelocity;


        return new TwistMsg
        {
            linear = localVelocity.To<FLU>(),
            angular = angularVelocity.To<FLU>()
        };
    }

    private TwistMsg ComputeControl(TwistMsg inputTwist)
    {
        TwistMsg groundTruth = GetGroundTruthVelocity();

        float linear_out;
        float angular_out;
        if (enablePid)
        {
            linear_out = linearPid.Update((float)inputTwist.linear.x, (float)groundTruth.linear.x, Time.deltaTime);
            angular_out = angularPid.Update((float)inputTwist.angular.z, (float)groundTruth.angular.z, Time.deltaTime);
        }
        else
        {
            linear_out = (float)inputTwist.linear.x;
            angular_out = (float)inputTwist.angular.z;
        }

        return new TwistMsg
        {
            linear = new Vector3Msg { x = linear_out },
            angular = new Vector3Msg { z = angular_out }
        };
    }

    public void SetCommand(TwistMsg twist)
    {
        setpoint = twist;
    }

    public void Reset()
    {
        linearPid.Reset();
        angularPid.Reset();
    }
}