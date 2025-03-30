using System;

[Serializable]
public class SequenceElementConfig : ICloneable
{
    public float timestamp = 0.0f;

    // Waypoint follower fields
    public float x = 0.0f;  // meters
    public float y = 0.0f;  // meters
    public float z = 0.0f;  // meters
    public float roll = 0.0f;  // degrees
    public float pitch = 0.0f;  // degrees
    public float yaw = 0.0f;  // degrees
    public float vx = 0.0f;  // meters/sec
    public float vy = 0.0f;  // meters/sec
    public float vz = 0.0f;  // meters/sec
    public float vroll = 0.0f;  // degrees/sec
    public float vpitch = 0.0f;  // degrees/sec
    public float vyaw = 0.0f;  // degrees/sec

    // Target follower fields
    public string target_name = "";
    public string secondary_target_name = "";

    // Loop back to the top
    public bool reset = false;

    public object Clone()
    {
        return MemberwiseClone();
    }
}
