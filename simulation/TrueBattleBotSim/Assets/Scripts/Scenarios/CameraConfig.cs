using System;

[Serializable]
public class CameraConfig
{
    public bool follow_mouse = false;
    public PoseConfig pose = new PoseConfig();
}