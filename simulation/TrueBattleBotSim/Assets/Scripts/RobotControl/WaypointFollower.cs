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
    ArrowIndicator arrow;

    public void Start()
    {
        ArrowIndicator[] arrows = Resources.LoadAll<ArrowIndicator>("Indicators");
        if (arrows.Length == 0)
        {
            Debug.LogError("No arrow prefabs found");
        }
        else
        {
            arrow = Instantiate(arrows[0]);
        }
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

        SequenceElementConfig current = sequence[index];
        int next_index = Math.Min(index + 1, sequence.Count - 1);
        SequenceElementConfig next = sequence[next_index];
        SequenceElementConfig interpolated;
        if (next.timestamp == current.timestamp)
        {
            interpolated = current;
        }
        else
        {
            float interpolation_value = (current_time - current.timestamp) / (next.timestamp - current.timestamp);
            interpolated = InterpolateSequenceElement(current, next, interpolation_value);
        }
        arrow.Set2D(interpolated.x, 0.1f, interpolated.y, interpolated.theta);
        return ComputeVelocity(interpolated);
    }

    SequenceElementConfig InterpolateSequenceElement(SequenceElementConfig current, SequenceElementConfig next, float interp_value)
    {
        return new SequenceElementConfig
        {
            timestamp = interp_value,
            x = Mathf.Lerp(current.x, next.x, interp_value),
            y = Mathf.Lerp(current.y, next.y, interp_value),
            theta = Mathf.Lerp(current.theta, next.theta, interp_value),
            vx = Mathf.Lerp(current.vx, next.vx, interp_value),
            vy = Mathf.Lerp(current.vy, next.vy, interp_value),
            vtheta = Mathf.Lerp(current.vtheta, next.vtheta, interp_value)
        };
    }

    TwistMsg ComputeVelocity(SequenceElementConfig currentElement)
    {
        OdometryMsg odom = controller.GetGroundTruth();
        PointMsg position = odom.pose.pose.position;
        QuaternionMsg orientation = odom.pose.pose.orientation;
        Matrix4x4 currentPose = Matrix4x4.TRS(
            new Vector3((float)position.x, (float)position.y, (float)position.z),
            new Quaternion((float)orientation.x, (float)orientation.y, (float)orientation.z, (float)orientation.w),
            Vector3.one);
        Matrix4x4 goalPose = Matrix4x4.TRS(
            new Vector3(currentElement.x, currentElement.y, 0.0f),
            Quaternion.Euler(0, 0, currentElement.theta + 90.0f),
            Vector3.one);
        Matrix4x4 relativePose = currentPose.inverse * goalPose;
        Vector3 relativePosition = relativePose.GetT();
        float relativeAngle = relativePose.GetR().eulerAngles.z * Mathf.Deg2Rad;
        relativeAngle = (relativeAngle % (2 * Mathf.PI)) - Mathf.PI;
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
