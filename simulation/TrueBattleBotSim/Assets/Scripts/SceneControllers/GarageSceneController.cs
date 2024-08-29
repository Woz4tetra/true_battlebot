using UnityEngine;

public class GarageSceneController : MonoBehaviour
{
    BackgroundConfig backgroundConfig = new BackgroundConfig();
    void Start()
    {
        GameObject mainSceneManager = GameObject.Find("MainSceneManager");
        backgroundConfig = mainSceneManager.GetComponent<MainSceneManager>().GetLoadedBackgroundConfig();
        Debug.Log($"Background name: {backgroundConfig.name}");
    }
}
