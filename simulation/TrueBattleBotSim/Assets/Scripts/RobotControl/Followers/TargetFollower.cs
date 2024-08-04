using System.Collections.Generic;
using MathExtensions;
using UnityEngine;

class TargetFollower : BaseFollower
{
    [SerializeField] float maxLinearSpeed = 10.0f;
    [SerializeField] float maxAngularSpeed = 1000.0f;
    [SerializeField] bool reverseWhenUpsideDown = true;

    Dictionary<string, GameObject> active_actors = new Dictionary<string, GameObject>();

    public void SetActiveActors(Dictionary<string, GameObject> active_actors)
    {
        this.active_actors = active_actors;
    }
    protected override BaseFollowerEngine FindFollowerEngine()
    {
        return GetComponent<PIDFollowerEngine>();
    }

    protected override bool ComputeNextGoal(float current_time, int index, out SequenceElementConfig next)
    {
        SequenceElementConfig current = GetElement(index);
        string target_name = current.target_name;
        if (target_name.Length == 0)
        {
            next = current;
            return false;
        }

        Matrix4x4 currentPose = GetOdomPose(controller.GetGroundTruth());

        float vx_limit;
        if (reverseWhenUpsideDown)
        {
            vx_limit = controller.IsUpsideDown() ? -maxLinearSpeed : maxLinearSpeed;
        }
        else
        {
            vx_limit = maxLinearSpeed;
        }

        foreach (string name in active_actors.Keys)
        {
            if (name == target_name)
            {
                GameObject actor = active_actors[name];
                ControllerInterface other_controller = actor.GetComponent<ControllerInterface>();

                Matrix4x4 goalPose = GetOdomPose(other_controller.GetGroundTruth());
                Vector3 delta = goalPose.GetT() - currentPose.GetT();
                float heading = Mathf.Rad2Deg * Mathf.Atan2(delta.y, delta.x);
                next = new SequenceElementConfig
                {
                    timestamp = current.timestamp,
                    x = actor.transform.position.x,
                    y = actor.transform.position.z,
                    yaw = heading,
                    vx = vx_limit,
                    vy = 0.0f,
                    vyaw = maxAngularSpeed,
                };
                return true;
            }
        }

        Debug.LogError("Target not found: " + target_name);
        next = current;
        return false;
    }
}
