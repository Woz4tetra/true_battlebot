using RosMessageTypes.BwInterfaces;
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
    List<SequenceElementConfig> objectiveSequence = new List<SequenceElementConfig>();
    float sequenceTime = 0.0f;
    ArrowIndicator[] arrowPrefabs;
    ArrowIndicator arrow;
    bool isArrowEnabled = true;
    ROSConnection ros;
    int objectiveIndex = 0;
    string objectiveName = "";
    string actorName = "";
    RosTopicState sequenceProgressTopic;
    string objectiveProgressTopic = "simulation/objective_progress";

    public virtual void Awake()
    {
        properties = FindObjectOfType<ActorSharedProperties>();
        arrowPrefabs = Resources.LoadAll<ArrowIndicator>("Indicators");
        controller = GetComponent<ControllerInterface>();

        ros = ROSConnection.GetOrCreateInstance();
        sequenceProgressTopic = ros.GetTopic(objectiveProgressTopic);
        if (sequenceProgressTopic == null)
        {
            sequenceProgressTopic = ros.RegisterPublisher<SimulationObjectiveProgressMsg>(objectiveProgressTopic, queue_size: 100);
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
        if (objectiveSequence.Count == 0)
        {
            return Optional<SequenceElementConfig>.CreateEmpty();
        }

        float current_time = Time.time - sequenceTime;
        if (current_time < 0)
        {
            return Optional<SequenceElementConfig>.CreateEmpty();
        }

        int nextIndex = objectiveIndex + 1;
        while (nextIndex < objectiveSequence.Count && objectiveSequence[nextIndex].timestamp < current_time)
        {
            objectiveIndex++;
            nextIndex++;
        }
        if (objectiveIndex >= objectiveSequence.Count)
        {
            objectiveIndex = objectiveSequence.Count - 1;
        }

        if (objectiveSequence[objectiveIndex].reset)
        {
            ResetSequenceTime();
            Debug.Log($"Looping sequence for {actorName}");
            return Optional<SequenceElementConfig>.CreateEmpty();
        }

        sequenceProgressTopic.Publish(new SimulationObjectiveProgressMsg
        {
            header = new HeaderMsg
            {
                frame_id = actorName,
                stamp = RosUtil.GetTimeMsg((double)current_time),
            },
            duration = RosUtil.GetTimeMsg(objectiveSequence[objectiveSequence.Count - 1].timestamp),
            objective_index = (uint)objectiveIndex,
            sequence_length = (uint)objectiveSequence.Count,
            objective_name = objectiveName
        });

        SequenceElementConfig next;
        if (!ComputeNextGoal(current_time, objectiveIndex, out next))
        {
            return Optional<SequenceElementConfig>.CreateEmpty();
        }
        if (arrow != null)
        {
            arrow.Set2D(next.x, 0.1f, next.y, -next.yaw);
        }
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

    abstract protected void OnResetSequence();

    private void ResetSequenceTime()
    {
        sequenceTime = Time.time;
        objectiveIndex = 0;
        OnResetSequence();
    }

    public void SetSequence(string actorName, string objectiveName, List<SequenceElementConfig> objectiveSequence)
    {
        this.objectiveSequence = objectiveSequence;
        this.actorName = actorName;
        this.objectiveName = objectiveName;
        ResetSequenceTime();
        if (controller != null)
        {
            controller.Reset();
        }
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
        return objectiveSequence[Mathf.Clamp(index, 0, objectiveSequence.Count - 1)];
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
