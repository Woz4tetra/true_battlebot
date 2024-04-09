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

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            RestartScenario();
        }
    }

    void RestartScenario()
    {
        sceneManager.ReloadScenario();
    }

    public void OnClick()
    {
        RestartScenario();
    }
}