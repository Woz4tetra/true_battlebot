using UnityEngine;
[System.Serializable]
public class PID
{
    [SerializeField] private float kp, ki = 0.0f, kd = 0.0f, kf = 0.0f;
    [SerializeField] private float i_zone = 0.0f, i_max = 0.0f;
    [SerializeField] private float epsilon = 1e-9f, tolerance = 0.0f;
    private float i_accum = 0.0f;
    private float prev_error = float.NaN;

    public PID(float kp, float ki = 0.0f, float kd = 0.0f, float kf = 0.0f, float i_zone = 0.0f, float i_max = 0.0f, float epsilon = 1e-9f, float tolerance = 0.0f)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.i_zone = i_zone;
        this.i_max = i_max;
        this.epsilon = epsilon;
        this.tolerance = tolerance;
    }

    public void Reset()
    {
        i_accum = 0.0f;
        prev_error = float.NaN;
    }

    public float Update(float setpoint, float measurement, float dt)
    {
        float error = setpoint - measurement;
        if (Mathf.Abs(error) < tolerance)
        {
            return 0.0f;
        }
        if (dt <= 0.0f)
        {
            Debug.LogWarning("dt must be positive");
            return 0.0f;
        }
        float output = 0.0f;
        output += CalculateP(error);
        output += CalculateI(error, dt);
        output += CalculateD(error, dt);
        output += CalculateF(setpoint);
        return output;
    }

    private float CalculateP(float error)
    {
        if (Mathf.Abs(kp) < epsilon)
        {
            return 0.0f;
        }
        return kp * error;
    }
    private float CalculateI(float error, float dt)
    {
        if (Mathf.Abs(ki) < epsilon)
            return 0.0f;
        if (i_zone <= epsilon || Mathf.Abs(error) < i_zone)
            i_accum += error;
        else
            i_accum = 0.0f;

        if (i_max > epsilon)
            if (i_accum > 0.0f)
                i_accum = Mathf.Min(i_accum, i_max / ki);
            else
                i_accum = Mathf.Max(i_accum, -i_max / ki);

        return ki * i_accum * dt;
    }

    private float CalculateD(float error, float dt)
    {
        if (Mathf.Abs(kd) < epsilon)
            return 0.0f;
        if (float.IsNaN(prev_error))
            prev_error = error;
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        return kd * derivative;
    }

    private float CalculateF(float setpoint)
    {
        if (Mathf.Abs(kf) < epsilon)
            return 0.0f;
        return kf * setpoint;
    }
}
