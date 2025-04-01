using System.Collections.Generic;
using MathExtensions;
using UnityEngine;

class Target180Follower : BaseVelocityFollower
{
    [SerializeField] float maxLinearSpeed = 10.0f;
    [SerializeField] float maxAngularSpeed = 1000.0f;
    [SerializeField] bool reverseWhenUpsideDown = true;

    Dictionary<string, GameObject> active_actors = new Dictionary<string, GameObject>();

    public void SetActiveActors(Dictionary<string, GameObject> active_actors)
    {
        Debug.Log($"Setting active actors: {string.Join(", ", active_actors.Keys)}");
        this.active_actors = active_actors;
    }

    protected override bool ComputeNextGoal(float current_time, int index, out SequenceElementConfig next)
    {
        SequenceElementConfig current = GetElement(index);
        string target_name = current.target_name; // Opponent in the middle
        if (target_name.Length == 0)
        {
            next = current;
            return false;
        }
        string secondary_target_name = current.secondary_target_name;  // Friendly robot
        if (secondary_target_name.Length == 0)
        {
            next = current;
            return false;
        }

        float vxLimit;
        if (reverseWhenUpsideDown)
        {
            vxLimit = controller.IsUpsideDown() ? -maxLinearSpeed : maxLinearSpeed;
        }
        else
        {
            vxLimit = maxLinearSpeed;
        }

        GameObject target = null, secondary_target = null;
        foreach (string name in active_actors.Keys)
        {
            if (name == target_name)
                target = active_actors[name];
            if (name == secondary_target_name)
                secondary_target = active_actors[name];
            if (target != null && secondary_target != null)
                break;
        }
        if (target == null && secondary_target == null)
        {
            next = current;
            return false;
        }
        ControllerInterface targetController = target.GetComponent<ControllerInterface>();
        ControllerInterface secondaryTargetController = secondary_target.GetComponent<ControllerInterface>();

        Matrix4x4 tfMapFromTarget = GetOdomPose(targetController.GetGroundTruth());
        Matrix4x4 tfMapFromSecondaryTarget = GetOdomPose(secondaryTargetController.GetGroundTruth());
        // Matrix4x4 tfSecondaryTargetFromTarget = tfMapFromSecondaryTarget.inverse * tfMapFromTarget;
        Vector3 delta = tfMapFromSecondaryTarget.GetT() - tfMapFromTarget.GetT();
        float heading = Mathf.Atan2(delta.y, delta.x);
        float radius = delta.magnitude;
        heading += Mathf.PI;
        float angle = Mathf.Deg2Rad * tfMapFromSecondaryTarget.GetR().eulerAngles.z;
        angle += Mathf.PI;

        Vector3 newDelta = new Vector3(radius * Mathf.Cos(heading), radius * Mathf.Sin(heading));
        Vector3 goalPos = newDelta + tfMapFromTarget.GetT();

        next = new SequenceElementConfig
        {
            timestamp = current.timestamp,
            x = goalPos.x,
            y = goalPos.y,
            yaw = Mathf.Rad2Deg * angle,
            vx = vxLimit,
            vy = 0.0f,
            vyaw = maxAngularSpeed,
        };

        return true;
    }
}
