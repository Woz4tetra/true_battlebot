using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using UnityEngine;

class BaseFollower : MonoBehaviour
{
    protected ControllerInterface controller;
    List<SequenceElementConfig> sequence = new List<SequenceElementConfig>();
    float sequence_time = 0.0f;
    ArrowIndicator arrow;
    BaseFollowerEngine followerEngine;

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

        followerEngine = FindFollowerEngine();
        if (followerEngine == null)
        {
            Debug.LogError("No follower engine found");
        }
        else
        {
            Debug.Log($"Follower engine: {followerEngine.GetType().Name}");
        }

        Reset();
    }

    protected virtual BaseFollowerEngine FindFollowerEngine()
    {
        BaseFollowerEngine engine = null;
        Type[] types = new Type[] { typeof(RamseteFollowerEngine), typeof(PIDFollowerEngine) };
        foreach (Type type in types)
        {
            if (type.IsSubclassOf(typeof(BaseFollowerEngine)) && !type.IsAbstract)
            {
                engine = GetComponent(type) as BaseFollowerEngine;
                break;
            }
        }
        return engine;
    }

    public void FixedUpdate()
    {
        updateCommand();
    }

    void Reset()
    {
        sequence_time = Time.time;
        if (followerEngine != null)
        {
            followerEngine.Reset();
        }
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

        SequenceElementConfig next;
        if (!ComputeNextGoal(current_time, index, out next))
        {
            return new TwistMsg();
        }
        arrow.Set2D(next.x, 0.1f, next.y, next.theta - 90.0f);
        return ComputeVelocity(next);
    }

    protected virtual bool ComputeNextGoal(float current_time, int index, out SequenceElementConfig next)
    {
        next = GetElement(index);
        return true;
    }

    protected SequenceElementConfig GetElement(int index)
    {
        return sequence[Mathf.Clamp(index, 0, sequence.Count - 1)];
    }

    protected Matrix4x4 GetOdomPose(OdometryMsg odom)
    {
        PointMsg position = odom.pose.pose.position;
        QuaternionMsg orientation = odom.pose.pose.orientation;
        return Matrix4x4.TRS(
            new Vector3((float)position.x, (float)position.y, (float)position.z),
            new Quaternion((float)orientation.x, (float)orientation.y, (float)orientation.z, (float)orientation.w),
            Vector3.one);
    }

    protected Matrix4x4 GetElementPose(SequenceElementConfig element)
    {
        return Matrix4x4.TRS(
            new Vector3(element.x, element.y, 0.0f),
            Quaternion.Euler(0, 0, element.theta),
            Vector3.one);
    }

    TwistMsg ComputeVelocity(SequenceElementConfig currentElement)
    {
        OdometryMsg odom = controller.GetGroundTruth();
        Matrix4x4 currentPose = GetOdomPose(odom);
        Matrix4x4 goalPose = GetElementPose(currentElement);
        Vector3 currentVelocity = new Vector3(
            (float)odom.twist.twist.linear.x,
            (float)odom.twist.twist.linear.y,
            (float)odom.twist.twist.angular.z
        );
        Vector3 goalVelocity = new Vector3(currentElement.vx, currentElement.vy, currentElement.vtheta * Mathf.Deg2Rad);

        TwistMsg command = followerEngine.ComputeVelocity(currentPose, goalPose, currentVelocity, goalVelocity);
        Debug.Log($"{gameObject.name} | vt error: {command.angular.z - odom.twist.twist.angular.z} | v error: {command.linear.x - odom.twist.twist.linear.x}");
        return command;
    }

    private void updateCommand()
    {
        controller.SetCommand(GetCommand());
    }
}
