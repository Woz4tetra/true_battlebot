using System;

[Serializable]
public class CageConfig
{
    public DimsConfig dims = new DimsConfig();
    public PoseConfig slow_cam = new PoseConfig();
    public PoseConfig tracking_cam = new PoseConfig();
}