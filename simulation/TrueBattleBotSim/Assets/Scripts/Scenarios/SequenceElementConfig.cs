using System;

[Serializable]
public class SequenceElementConfig
{
    public float timestamp = 0.0f;
    public float x = 0.0f;  // meters
    public float y = 0.0f;  // meters
    public float theta = 0.0f;  // degrees
    public float vx = 0.0f;  // meters/sec
    public float vy = 0.0f;  // meters/sec
    public float vtheta = 0.0f;  // degrees/sec
}
