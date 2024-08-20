using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public abstract class BaseFollower : MonoBehaviour
{
    protected ControllerInterface controller;
    protected ActorSharedProperties properties;
    List<SequenceElementConfig> sequence = new List<SequenceElementConfig>();
    float sequence_time = 0.0f;
    ArrowIndicator[] arrowPrefabs;
    ArrowIndicator arrow;
    bool isArrowEnabled = true;
    ROSConnection ros;
    static RosTopicState sequenceProgressTopic = null;

    public virtual void Awake()
    {
        properties = FindObjectOfType<ActorSharedProperties>();
        arrowPrefabs = Resources.LoadAll<ArrowIndicator>("Indicators");
        controller = GetComponent<ControllerInterface>();

        ros = ROSConnection.GetOrCreateInstance();
        if (sequenceProgressTopic == null)
        {
            sequenceProgressTopic = ros.RegisterPublisher<Float64Msg>(properties.GetSequenceProgressTopicName());
        }
    }

    void OnDestroy()
    {
        if (arrow != null)
        {
            Destroy(arrow.gameObject);
        }
    }

    public void Update()
    {
        if (Time.timeScale == 0.0f)
        {
            return;
        }
        TickSequence();
    }

    void TickSequence()
    {
        if (GetNextGoal().TryGet(out SequenceElementConfig next))
        {
            UpdateRobotState(next);
        }
    }

    Optional<SequenceElementConfig> GetNextGoal()
    {
        if (sequence.Count == 0)
        {
            return Optional<SequenceElementConfig>.CreateEmpty();
        }

        float current_time = Time.time - sequence_time;
        if (current_time < 0)
        {
            return Optional<SequenceElementConfig>.CreateEmpty();
        }
        sequenceProgressTopic.Publish(new Float64Msg { data = current_time });

        int index = 0;
        while (index < sequence.Count - 1 && sequence[index + 1].timestamp < current_time)
        {
            index++;
        }

        SequenceElementConfig next;
        if (!ComputeNextGoal(current_time, index, out next))
        {
            return Optional<SequenceElementConfig>.CreateEmpty();
        }
        arrow.Set2D(next.x, 0.1f, next.y, -next.yaw);
        if (next == null)
        {
            return Optional<SequenceElementConfig>.CreateEmpty();
        }
        return Optional<SequenceElementConfig>.Create(next);
    }

    public void SetShowArrow(bool show)
    {
        isArrowEnabled = show;
    }

    public void SetSequence(List<SequenceElementConfig> sequence)
    {
        this.sequence = sequence;
        sequence_time = Time.time;
        controller.Reset();
        if (arrowPrefabs.Length == 0)
        {
            Debug.LogError("No arrow prefabs found");
        }
        else if (isArrowEnabled)
        {
            arrow = Instantiate(arrowPrefabs[0]);
        }
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
            Quaternion.Euler(0, 0, element.yaw),
            Vector3.one);
    }

    protected abstract bool ComputeNextGoal(float current_time, int index, out SequenceElementConfig next);
    protected abstract void UpdateRobotState(SequenceElementConfig next);
}
