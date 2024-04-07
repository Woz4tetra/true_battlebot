using System;
using System.Collections.Generic;
using MathExtensions;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using UnityEngine;

class WaypointFollower : MonoBehaviour
{
    [SerializeField] PID linearPID = new PID(0.1f, 0.0f, 0.0f);
    [SerializeField] PID angularPID = new PID(0.1f, 0.0f, 0.0f);
    private ControllerInterface controller;
    List<SequenceElementConfig> sequence = new List<SequenceElementConfig>();
    float sequence_time = 0.0f;
    float prevRelativeAngle = 0.0f;

    public void Start()
    {
        controller = GetComponent<ControllerInterface>();
        Reset();
    }
    public void FixedUpdate()
    {
        updateCommand();
    }

    void Reset()
    {
        sequence_time = Time.time;
        linearPID.Reset();
        angularPID.Reset();
        prevRelativeAngle = 0.0f;
        if (controller != null)
        {
            controller.Reset();
        }
    }

    public void SetSequence(List<SequenceElementConfig> sequence)
    {
        this.sequence = sequence;
        Reset();
    }

    private TwistMsg GetCommand()
    {
        if (sequence.Count == 0)
        {
            return new TwistMsg();
        }

        float current_time = Time.time - sequence_time;
        if (current_time < 0)
        {
            return new TwistMsg();
        }

        int index = 0;
        while (index < sequence.Count - 1 && sequence[index + 1].timestamp < current_time)
        {
            index++;
        }

        return ComputeVelocity(sequence, index);
    }

    TwistMsg ComputeVelocity(List<SequenceElementConfig> sequence, int index)
    {
        OdometryMsg odom = controller.getGroundTruth();
        PointMsg position = odom.pose.pose.position;
        QuaternionMsg orientation = odom.pose.pose.orientation;
        Matrix4x4 currentPose = Matrix4x4.TRS(
            new Vector3((float)position.x, (float)position.y, (float)position.z),
            new Quaternion((float)orientation.x, (float)orientation.y, (float)orientation.z, (float)orientation.w),
            Vector3.one);
        SequenceElementConfig currentElement = sequence[index];
        Matrix4x4 goalPose = Matrix4x4.TRS(
            new Vector3(currentElement.x, currentElement.y, 0.0f),
            Quaternion.Euler(0, 0, currentElement.theta),
            Vector3.one);
        Matrix4x4 relativePose = currentPose.inverse * goalPose;
        Vector3 relativePosition = relativePose.GetT();
        float relativeAngle = Mathf.Atan2(relativePosition.y, relativePosition.x);
        float deltaAngle = relativeAngle - prevRelativeAngle;
        if (deltaAngle > Mathf.PI)
        {
            relativeAngle -= 2 * Mathf.PI;
        }
        else if (deltaAngle < -Mathf.PI)
        {
            relativeAngle += 2 * Mathf.PI;
        }
        prevRelativeAngle = relativeAngle;
        relativeAngle = Mathf.Clamp(relativeAngle, -Mathf.PI, Mathf.PI);
        float linearVelocity = linearPID.Update(relativePosition.x, 0.0f, Time.deltaTime);
        float angularVelocity = angularPID.Update(relativeAngle, 0.0f, Time.deltaTime);
        return new TwistMsg
        {
            linear = new Vector3Msg { x = linearVelocity },
            angular = new Vector3Msg { z = angularVelocity }
        };
    }

    private void updateCommand()
    {
        controller.SetCommand(GetCommand());
    }
}
