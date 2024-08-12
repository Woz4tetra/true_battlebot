using UnityEngine;
using UnityEngine.UI;

public class RestartButton : MonoBehaviour
{
    [SerializeField] FieldManager sceneManager;
    Button button;

    void Start()
    {
        button = GetComponent<Button>();
        button.onClick.AddListener(OnClick);
    }

    public void RestartScenario()
    {
        sceneManager.ReloadScenario();
    }

    public void OnClick()
    {
        RestartScenario();
    }
}