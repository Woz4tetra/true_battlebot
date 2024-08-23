using System;

[Serializable]
public class ScenarioInitConfig
{
    // absolute (cage coords)
    // relative (scaled cage coords)
    // world (sim coords)
    // flu (front-left-up in absolute coords)
    public string type = "absolute";
    public float x = 0.0f;  // meters
    public float y = 0.0f;  // meters
    public float z = 0.0f;  // meters
    public float roll = 0.0f;  // degrees
    public float pitch = 0.0f;  // degrees
    public float yaw = 0.0f;  // degrees
    public float x_buffer = 0.0f;
    public float y_buffer = 0.0f;
    public float z_buffer = 0.0f;
}