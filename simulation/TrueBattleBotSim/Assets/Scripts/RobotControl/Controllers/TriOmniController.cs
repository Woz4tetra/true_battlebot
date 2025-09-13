using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MathExtensions;

class TriOmniController : MonoBehaviour, ControllerInterface
{
    [SerializeField] string robotName = "";
    [SerializeField] string parentFrame = "map";

    [SerializeField] FakeWheelSpinner fakeLeftWheel;
    [SerializeField] FakeWheelSpinner fakeRightWheel;
    [SerializeField] FakeWheelSpinner fakeBackWheel;

    [SerializeField] SwerveWheel leftWheel;
    [SerializeField] SwerveWheel rightWheel;
    [SerializeField] SwerveWheel backWheel;

    [SerializeField] PidConfig linearXPIDConfig;
    [SerializeField] PidConfig linearYPIDConfig;
    [SerializeField] PidConfig angularPIDConfig;
    [SerializeField] bool enablePid;
    [SerializeField] bool reverseLeft;
    [SerializeField] bool reverseRight;
    [SerializeField] bool reverseBack;
    [SerializeField] GameObject referenceObject;
    [SerializeField] Transform spawnHere;

    // Distance from robot center to wheel (set this to your robot's value)
    [SerializeField] float wheelBaseRadius = 0.1f; // meters

    // Wheel angles in radians (0°, 120°, 240°)
    [SerializeField] float[] wheelAngles = { 0.0f, 120.0f, 240.0f };

    ArticulationBody body;
    TwistMsg setpoint = new TwistMsg();
    TransformFrame frame;
    Matrix4x4 tf_spawnhere_from_body;
    PID linearXPid, linearYPid, angularPid;
    SwerveWheel[] swerveWheels;
    bool[] wheelReversals;

    public void Start()
    {
        swerveWheels = new SwerveWheel[] { leftWheel, rightWheel, backWheel };
        wheelReversals = new bool[] { reverseLeft, reverseRight, reverseBack };

        linearXPid = new PID(linearXPIDConfig);
        linearYPid = new PID(linearYPIDConfig);
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

    public string GetName()
    {
        return robotName;
    }

    public void FixedUpdate()
    {
        updateCommand();
    }

    // Apply the wheel speeds to the fake wheels as if they were omni wheels
    private void ApplyFakeWheelSpeeds(TwistMsg twist)
    {
        float linearXVelocity = (float)twist.linear.x;  // m/s
        float linearYVelocity = (float)twist.linear.y;  // m/s
        float angularVelocity = (float)twist.angular.z;  // rad/s

        float leftSpeed = -1 * linearXVelocity * Mathf.Sin(wheelAngles[0] * Mathf.Deg2Rad) + linearYVelocity * Mathf.Cos(wheelAngles[0] * Mathf.Deg2Rad) + -1 * angularVelocity * wheelBaseRadius;
        float rightSpeed = -1 * linearXVelocity * Mathf.Sin(wheelAngles[1] * Mathf.Deg2Rad) + linearYVelocity * Mathf.Cos(wheelAngles[1] * Mathf.Deg2Rad) + -1 * angularVelocity * wheelBaseRadius;
        float backSpeed = -1 * linearXVelocity * Mathf.Sin(wheelAngles[2] * Mathf.Deg2Rad) + linearYVelocity * Mathf.Cos(wheelAngles[2] * Mathf.Deg2Rad) + -1 * angularVelocity * wheelBaseRadius;
        if (reverseLeft)
            leftSpeed *= -1;
        if (reverseRight)
            rightSpeed *= -1;
        if (reverseBack)
            backSpeed *= -1;

        fakeLeftWheel.SetVelocity(leftSpeed);
        fakeRightWheel.SetVelocity(rightSpeed);
        fakeBackWheel.SetVelocity(backSpeed);
    }

    // Apply the wheel speeds to the real omni wheels. They are swerve wheels.
    private void ApplyRealWheelSpeeds(TwistMsg twist)
    {
        float linearXVelocity = (float)twist.linear.x;  // m/s
        float linearYVelocity = -1 * (float)twist.linear.y;  // m/s
        float angularVelocity = -1 * (float)twist.angular.z;  // rad/s

        for (int index = 0; index < wheelAngles.Length; index++)
        {
            float angleRad = wheelAngles[index] * Mathf.Deg2Rad;
            // Wheel position vector (assuming wheels are at equal radius from center)
            float wheelX = wheelBaseRadius * Mathf.Cos(angleRad);
            float wheelY = wheelBaseRadius * Mathf.Sin(angleRad);

            // The velocity at the wheel is the sum of the robot's linear velocity and the velocity from rotation
            float vx = linearXVelocity - angularVelocity * wheelY;
            float vy = linearYVelocity + angularVelocity * wheelX;

            if (wheelReversals[index])
            {
                vx *= -1;
                vy *= -1;
            }

            swerveWheels[index].SetVelocity(new Vector2(vx, vy));
        }
    }

    private void updateCommand()
    {
        TwistMsg twist = ComputeControl(setpoint);
        ApplyFakeWheelSpeeds(twist);
        ApplyRealWheelSpeeds(twist);
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

    public void Teleport(PointMsg position, QuaternionMsg rotation)
    {
        Matrix4x4 tf_world_from_body = Matrix4x4.TRS(position.From<FLU>(), rotation.From<FLU>(), Vector3.one);
        tf_world_from_body = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0, 90, 0), Vector3.one) * tf_world_from_body * Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(0, -90, 0), Vector3.one);
        Matrix4x4 tf_world_from_spawnhere = tf_world_from_body * tf_spawnhere_from_body.inverse;
        Vector3 spawnPosition = tf_world_from_spawnhere.GetT();
        Quaternion spawnRotation = tf_world_from_spawnhere.GetR();
        body.TeleportRoot(spawnPosition, spawnRotation);
    }

    private Vector3 GetRelativeVelocity()
    {
        // Get the velocity of the body in world space
        Vector3 worldVelocity = Quaternion.Euler(0, -90, 0) * body.linearVelocity;

        // Convert the velocity to the body's local space
        Vector3 localVelocity = body.transform.InverseTransformDirection(worldVelocity);

        return localVelocity;
    }

    private TwistMsg GetGroundTruthVelocity()
    {
        Vector3 localVelocity = GetRelativeVelocity();  // m/s
        Vector3 angularVelocity = body.angularVelocity;  // rad/s

        return new TwistMsg
        {
            linear = localVelocity.To<FLU>(),
            angular = angularVelocity.To<FLU>()
        };
    }

    private TwistMsg ComputeControl(TwistMsg inputTwist)
    {
        TwistMsg groundTruth = GetGroundTruthVelocity();

        float linear_x_out;
        float linear_y_out;
        float angular_out;
        if (enablePid)
        {
            linear_x_out = linearXPid.Update((float)inputTwist.linear.x, (float)groundTruth.linear.x, Time.deltaTime);
            linear_y_out = linearYPid.Update((float)inputTwist.linear.y, (float)groundTruth.linear.y, Time.deltaTime);
            angular_out = angularPid.Update((float)inputTwist.angular.z, (float)groundTruth.angular.z, Time.deltaTime);
        }
        else
        {
            linear_x_out = (float)inputTwist.linear.x;
            linear_y_out = (float)inputTwist.linear.y;
            angular_out = (float)inputTwist.angular.z;
        }

        return new TwistMsg
        {
            linear = new Vector3Msg { x = -1 * linear_x_out, y = -1 * linear_y_out },
            angular = new Vector3Msg { z = -1 * angular_out }
        };
    }

    public void SetCommand(TwistMsg twist)
    {
        if (!IsUpsideDown())
        {
            twist.linear.x *= -1;
            twist.linear.y *= -1;
        }
        setpoint = twist;
    }

    public bool IsUpsideDown()
    {
        return transform.up.y < 0;
    }

    public void Reset()
    {
        if (linearXPid != null)
        {
            linearXPid.Reset();
        }
        if (linearYPid != null)
        {
            linearYPid.Reset();
        }
        if (angularPid != null)
        {
            angularPid.Reset();
        }
    }
}