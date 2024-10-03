using System;

[Serializable]
public class PhysicsMaterialsConfig
{
    public string name = "default";
    public float static_friction = 0.0f;
    public float dynamic_friction = 0.0f;
    public float bounciness = 0.0f;
    public int friction_combine = 0;  // See PhysicMaterialCombine enum
    public int bounce_combine = 0;  // See PhysicMaterialCombine enum
}