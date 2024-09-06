using System;

[Serializable]
public class PidConfig
{
    public PidConfig(float kp, float ki, float kd, float kf)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public float kp = 1.0f;
    public float ki = 0.0f;
    public float kd = 0.0f;
    public float kf = 0.0f;
    public float i_zone = 0.0f;
    public float i_max = 0.0f;
    public float epsilon = 1e-9f;
    public float tolerance = 0.0f;
}