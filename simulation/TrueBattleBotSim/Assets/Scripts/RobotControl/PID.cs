using UnityEngine;

public class PID
{
    private float kp, ki = 0.0f, kd = 0.0f, kf = 0.0f;
    private float i_zone = 0.0f, i_max = 0.0f;
    private float epsilon = 1e-9f, tolerance = 0.0f;
    private float i_accum = 0.0f;
    private float prev_error = float.NaN;

    public PID(PidConfig config)
    {
        SetConfig(config);
    }

    public void SetConfig(PidConfig config)
    {
        kp = config.kp;
        ki = config.ki;
        kd = config.kd;
        kf = config.kf;
        i_zone = config.i_zone;
        i_max = config.i_max;
        epsilon = config.epsilon;
        tolerance = config.tolerance;

        Reset();
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
