using UnityEngine;

public class PanoramaSceneController : MonoBehaviour
{
    BackgroundConfig backgroundConfig = new BackgroundConfig();
    void Start()
    {
        GameObject mainSceneManager = GameObject.Find("MainSceneManager");
        backgroundConfig = mainSceneManager.GetComponent<MainSceneManager>().GetLoadedBackgroundConfig();
        Debug.Log($"Background name: {backgroundConfig.name}");
        Material skyboxMaterial = Resources.Load<Material>($"Skyboxes/{backgroundConfig.skyImage}");
        if (skyboxMaterial == null)
        {
            Debug.LogError($"Skybox material not found: {backgroundConfig.skyImage}");
            return;
        }
        RenderSettings.skybox = skyboxMaterial;
    }
}
