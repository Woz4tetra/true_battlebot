using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;

class TankController : MonoBehaviour, ControllerInterface
{
    [SerializeField] private string parentFrame = "map";
    [SerializeField] private float baseWidth = 1.0f;

    [SerializeField] private Wheel leftWheel;
    [SerializeField] private Wheel rightWheel;

    [SerializeField] private Wheel[] followLeftWheels;
    [SerializeField] private Wheel[] followRightWheels;
    [SerializeField] private PidConfig linearPIDConfig;
    [SerializeField] private PidConfig angularPIDConfig;
    [SerializeField] private bool enablePid;
    [SerializeField] private bool reverseLeft;
    [SerializeField] private bool reverseRight;
    [SerializeField] private GameObject referenceObject;
    [SerializeField] private Transform spawnHere;

    private ArticulationBody body;
    private TwistMsg setpoint = new TwistMsg();
    private TransformFrame frame;
    private Matrix4x4 tf_spawnhere_from_body;
    private PID linearPid, angularPid;

    public void Start()
    {
        linearPid = new PID(linearPIDConfig);
        angularPid = new PID(angularPIDConfig);
        if (referenceObject == null)
        {
            referenceObject = GameObject.Find("Coordinate Frame");
        }
        if (spawnHere == null)
        {
            spawnHere = transform.Find("SpawnHere");
        }
        body = GetComponent<ArticulationBody>();
        frame = GetComponent<TransformFrame>();

        tf_spawnhere_from_body = Matrix4x4.TRS(spawnHere.localPosition, spawnHere.localRotation, Vector3.one);
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

    public OdometryMsg GetGroundTruth()
    {
        Matrix4x4 pose = Matrix4x4.TRS(body.transform.position, body.transform.rotation, Vector3.one);
        if (referenceObject != null)
        {
            pose = referenceObject.transform.worldToLocalMatrix * pose;
        }
        pose = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0, -90, 0), Vector3.one) * pose * Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0, 90, 0), Vector3.one);
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
                    position = pose.GetT().To<FLU>(),
                    orientation = pose.GetR().To<FLU>()
                }
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = GetGroundTruthVelocity()
            }
        };
    }

    public void Teleport(Vector3 position, Quaternion rotation)
    {
        Matrix4x4 tf_world_from_body = Matrix4x4.TRS(position, rotation, Vector3.one);
        Matrix4x4 tf_world_from_spawnhere = tf_world_from_body * tf_spawnhere_from_body.inverse;
        Vector3 spawnPosition = tf_world_from_spawnhere.GetT();
        Quaternion spawnRotation = tf_world_from_spawnhere.GetR();
        body.TeleportRoot(spawnPosition, spawnRotation);
    }

    private Vector3 GetRelativeVelocity()
    {
        // Get the velocity of the body in world space
        Vector3 worldVelocity = Quaternion.Euler(0, -90, 0) * body.velocity;

        // Convert the velocity to the body's local space
        Vector3 localVelocity = body.transform.InverseTransformDirection(worldVelocity);

        return localVelocity;
    }

    private TwistMsg GetGroundTruthVelocity()
    {
        Vector3 localVelocity = GetRelativeVelocity();  // m/s
        Vector3 angularVelocity = -1 * body.angularVelocity;  // rad/s

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
        if (IsUpsideDown())
        {
            twist.linear.x *= -1;
        }
        setpoint = twist;
    }

    public bool IsUpsideDown()
    {
        return transform.up.y < 0;
    }

    public void Reset()
    {
        linearPid.Reset();
        angularPid.Reset();
    }
}