using System.Collections.Generic;
using UnityEngine;

public class PhysicsMaterialConfigurator : MonoBehaviour
{
    [SerializeField] List<PhysicsMaterial> materials;
    List<PhysicsMaterialsConfig> defaultConfigs = new List<PhysicsMaterialsConfig>();

    void Start()
    {
        foreach (PhysicsMaterial material in materials)
        {
            PhysicsMaterialsConfig config = new PhysicsMaterialsConfig
            {
                name = material.name,
                dynamic_friction = material.dynamicFriction,
                static_friction = material.staticFriction,
                bounciness = material.bounciness,
                friction_combine = (int)material.frictionCombine,
                bounce_combine = (int)material.bounceCombine
            };
            defaultConfigs.Add(config);
        }
    }

    public void ResetMaterials()
    {
        foreach (PhysicsMaterialsConfig config in defaultConfigs)
        {
            ConfigureMaterial(config);
        }
    }

    public void ConfigureMaterial(PhysicsMaterialsConfig config)
    {
        PhysicsMaterial material = materials.Find(m => m.name == config.name);
        if (material == null)
        {
            Debug.LogError("Material not found: " + config.name);
            return;
        }
        material.dynamicFriction = config.dynamic_friction;
        material.staticFriction = config.static_friction;
        material.bounciness = config.bounciness;
        material.frictionCombine = (PhysicsMaterialCombine)config.friction_combine;
        material.bounceCombine = (PhysicsMaterialCombine)config.bounce_combine;
    }
}