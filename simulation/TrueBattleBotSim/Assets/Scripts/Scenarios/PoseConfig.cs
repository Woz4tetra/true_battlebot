using System;

[Serializable]
public class PositionConfig
{
    public float x;
    public float y;
    public float z;
}

[Serializable]
public class RotationConfig
{
    public float x;
    public float y;
    public float z;

}

[Serializable]
public class PoseConfig
{
    public PositionConfig position;
    public RotationConfig rotation;
}