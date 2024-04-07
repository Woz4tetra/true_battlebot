using UnityEngine;
using UnityEngine.UI;

public class PlayPauseButton : MonoBehaviour
{
    [SerializeField] PauseManager pauseManager;
    [SerializeField] Sprite playIcon;
    [SerializeField] Sprite pauseIcon;

    Image icon;
    Button button;

    void Start()
    {
        icon = transform.Find("Image").GetComponent<Image>();
        button = GetComponent<Button>();
        button.onClick.AddListener(OnClick);
    }

    private void SetPause(bool pausedState)
    {
        icon.sprite = pausedState ? playIcon : pauseIcon;
    }

    void Update()
    {
        SetPause(pauseManager.IsPaused());
    }

    public void OnClick()
    {
        bool pauseState = !pauseManager.IsPaused();
        Debug.Log($"Setting pause icon to {pauseState}");
        pauseManager.SetPause(pauseState);
    }
}