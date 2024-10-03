using System.Collections.Generic;
using UnityEngine;

public class PhysicsMaterialConfigurator : MonoBehaviour
{
    [SerializeField] List<PhysicMaterial> materials;
    List<PhysicsMaterialsConfig> defaultConfigs = new List<PhysicsMaterialsConfig>();

    void Start()
    {
        foreach (PhysicMaterial material in materials)
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
        PhysicMaterial material = materials.Find(m => m.name == config.name);
        if (material == null)
        {
            Debug.LogError("Material not found: " + config.name);
            return;
        }
        material.dynamicFriction = config.dynamic_friction;
        material.staticFriction = config.static_friction;
        material.bounciness = config.bounciness;
        material.frictionCombine = (PhysicMaterialCombine)config.friction_combine;
        material.bounceCombine = (PhysicMaterialCombine)config.bounce_combine;
    }
}