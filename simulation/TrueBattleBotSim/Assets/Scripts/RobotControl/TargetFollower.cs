using System.Collections.Generic;
using MathExtensions;
using RosMessageTypes.Nav;
using UnityEngine;

class TargetFollower : BaseFollower
{
    Dictionary<string, GameObject> active_robots = new Dictionary<string, GameObject>();

    public void SetActiveRobots(Dictionary<string, GameObject> active_robots)
    {
        this.active_robots = active_robots;
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

        foreach (string name in active_robots.Keys)
        {
            if (name == target_name)
            {
                GameObject robot = active_robots[name];
                ControllerInterface other_controller = robot.GetComponent<ControllerInterface>();

                Matrix4x4 goalPose = GetOdomPose(other_controller.GetGroundTruth());
                Vector3 delta = goalPose.GetT() - currentPose.GetT();
                float heading = Mathf.Rad2Deg * Mathf.Atan2(delta.y, delta.x);
                heading += 180.0f;
                next = new SequenceElementConfig
                {
                    timestamp = current.timestamp,
                    x = robot.transform.position.x,
                    y = robot.transform.position.z,
                    theta = heading,
                };
                return true;
            }
        }

        Debug.LogError("Target not found: " + target_name);
        next = current;
        return false;
    }
}
