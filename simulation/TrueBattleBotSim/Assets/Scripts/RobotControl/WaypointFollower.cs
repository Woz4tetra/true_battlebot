using UnityEngine;

class WaypointFollower : BaseFollower
{
    protected override bool ComputeNextGoal(float current_time, int index, out SequenceElementConfig interpolated)
    {
        SequenceElementConfig current = GetElement(index);
        SequenceElementConfig next = GetElement(index + 1);
        if (next.timestamp == current.timestamp)
        {
            interpolated = current;
        }
        else
        {
            float interpolation_value = (current_time - current.timestamp) / (next.timestamp - current.timestamp);
            interpolated = InterpolateSequenceElement(current, next, interpolation_value);
        }
        return true;
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
}
