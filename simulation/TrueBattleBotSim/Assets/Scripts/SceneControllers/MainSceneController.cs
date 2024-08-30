using UnityEngine;

public class MainSceneController : MonoBehaviour
{
    [SerializeField] string skyImage;
    void Start()
    {
        Material skyboxMaterial = Resources.Load<Material>($"Skyboxes/{skyImage}");
        if (skyboxMaterial == null)
        {
            Debug.LogError($"Skybox material not found: {skyImage}");
            return;
        }
        RenderSettings.skybox = skyboxMaterial;
    }
}
