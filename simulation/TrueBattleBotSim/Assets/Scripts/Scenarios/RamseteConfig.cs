using System;

[Serializable]
public class RamseteConfig
{
    public RamseteConfig(float zeta, float b)
    {
        this.zeta = zeta;
        this.b = b;
    }

    public float zeta = 2.0f;
    public float b = 0.7f;
}