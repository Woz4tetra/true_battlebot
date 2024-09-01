using UnityEngine;
public class FixturesManager : MonoBehaviour
{
    [SerializeField] Light spotlight;
    [SerializeField] Light evenLighting;

    public void UpdateFromConfig(FixturesConfig config)
    {
        UpdateSpotlight(config.spotlight);
        UpdateEvenLighting(config.even_lighting);
    }

    void UpdateSpotlight(SpotlightConfig config)
    {
        spotlight.enabled = config.enabled;
        spotlight.range = config.range;
        spotlight.spotAngle = config.spot_angle;
        spotlight.intensity = config.intensity;
        spotlight.shadowStrength = config.shadow_strength;
    }

    void UpdateEvenLighting(EvenLightingConfig config)
    {
        evenLighting.enabled = config.enabled;
        evenLighting.intensity = config.intensity;
        evenLighting.range = config.size;
    }
}