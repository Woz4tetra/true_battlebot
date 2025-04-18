using UnityEngine;

class TeleportFollower : BaseFollower
{
    [SerializeField] private bool smooth = false;
    TeleportFollowerEngine followerEngine;

    public override void Awake()
    {
        base.Awake();
        SetShowArrow(false);
    }

    override protected void OnResetSequence()
    {

    }

    public void SetFollowerEngine(TeleportFollowerEngine engine)
    {
        followerEngine = engine;
    }

    protected override bool ComputeNextGoal(float current_time, int index, out SequenceElementConfig next)
    {
        if (smooth)
        {
            next = ComputeNextGoalSmooth(current_time, index);
        }
        else
        {
            next = ComputeNextGoalJumpy(index);
        }
        return true;
    }

    public void SetComputeMethod(bool smooth)
    {
        this.smooth = smooth;
    }

    SequenceElementConfig ComputeNextGoalJumpy(int index)
    {
        return GetElement(index);
    }

    SequenceElementConfig ComputeNextGoalSmooth(float current_time, int index)
    {
        SequenceElementConfig next;
        SequenceElementConfig step1 = GetElement(index);
        SequenceElementConfig step2 = GetElement(index + 1);

        if (step2.timestamp == step1.timestamp)
        {
            next = step1;
        }
        else
        {
            float interpolation_value = (current_time - step1.timestamp) / (step2.timestamp - step1.timestamp);
            interpolation_value = Mathf.Clamp(interpolation_value, 0.0f, 1.0f);
            next = InterpolateSequenceElement(step1, step2, interpolation_value);
        }
        return next;
    }

    SequenceElementConfig InterpolateSequenceElement(SequenceElementConfig step1, SequenceElementConfig step2, float interp_value)
    {
        Vector3 position = Vector3.Lerp(new Vector3(step1.x, step1.y, step1.z), new Vector3(step2.x, step2.y, step2.z), interp_value);
        Quaternion rotation = Quaternion.Slerp(Quaternion.Euler(step1.roll, step1.pitch, step1.yaw), Quaternion.Euler(step2.roll, step2.pitch, step2.yaw), interp_value);
        Vector3 euler = rotation.eulerAngles;
        return new SequenceElementConfig
        {
            timestamp = interp_value,
            x = position.x,
            y = position.y,
            z = position.z,
            roll = euler.x,
            pitch = euler.y,
            yaw = euler.z,
        };
    }

    protected override void UpdateRobotState(SequenceElementConfig currentElement)
    {
        Matrix4x4 goalPose = GetElementPose(currentElement);
        followerEngine.ComputeVelocity(Matrix4x4.identity, goalPose, new Velocity2d(), new Velocity2d());
    }
}
