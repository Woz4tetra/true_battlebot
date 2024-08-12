using System;

[Serializable]
public class ScenarioInitConfig
{
    public string type = "absolute";  // absolute (cage coords), relative (scaled cage coords), or world (sim coords).
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