using System.Collections.Generic;
using UnityEngine;

class RelativeToFollower : BaseFollower
{
    Dictionary<string, GameObject> active_actors = new Dictionary<string, GameObject>();

    public override void Awake()
    {
        base.Awake();
        SetShowArrow(false);
    }

    public void SetActiveActors(Dictionary<string, GameObject> active_actors)
    {
        this.active_actors = active_actors;
    }

    protected override bool ComputeNextGoal(float current_time, int index, out SequenceElementConfig next)
    {
        SequenceElementConfig current = GetElement(index);
        string target_name = current.target_name;
        next = current;
        if (target_name.Length == 0)
        {
            return false;
        }

        transform.parent = active_actors[target_name].transform;

        return true;
    }

    protected override void UpdateRobotState(SequenceElementConfig next)
    {

    }
}
