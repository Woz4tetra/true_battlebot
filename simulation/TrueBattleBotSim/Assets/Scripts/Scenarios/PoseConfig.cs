using System;

[Serializable]
public class PositionConfig
{
    public float x = 0.0f;
    public float y = 0.0f;
    public float z = 0.0f;
}

[Serializable]
public class RotationConfig
{
    public float x = 0.0f;
    public float y = 0.0f;
    public float z = 0.0f;

}

[Serializable]
public class PoseConfig
{
    public PositionConfig position = new PositionConfig();
    public RotationConfig rotation = new RotationConfig();
}