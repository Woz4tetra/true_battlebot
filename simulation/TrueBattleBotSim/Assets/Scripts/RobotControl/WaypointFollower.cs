using System.Collections.Generic;
using RosMessageTypes.Geometry;
using UnityEngine;

class WaypointFollower : MonoBehaviour
{

    private ControllerInterface controller;
    List<SequenceElementConfig> sequence = new List<SequenceElementConfig>();
    float sequence_time = 0.0f;

    public void Start()
    {
        controller = GetComponent<ControllerInterface>();
        sequence_time = Time.time;
    }
    public void FixedUpdate()
    {
        updateCommand();
    }

    public void SetSequence(List<SequenceElementConfig> sequence)
    {
        this.sequence = sequence;
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

        int i = 0;
        while (i < sequence.Count - 1 && sequence[i + 1].timestamp < current_time)
        {
            i++;
        }

        return ComputeVelocity(sequence[i]);
    }

    TwistMsg ComputeVelocity(SequenceElementConfig goal)
    {
        return new TwistMsg
        {
            linear = new Vector3Msg { x = 1.0f },
            angular = new Vector3Msg { z = 0.0f }
        };
    }

    private void updateCommand()
    {
        controller.setCommand(GetCommand());
    }
}
